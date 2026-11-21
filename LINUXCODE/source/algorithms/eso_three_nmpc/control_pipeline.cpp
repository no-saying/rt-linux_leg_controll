#include "control_pipeline.h"
#include <algorithm>
#include <cmath>

namespace p112 {
namespace {
constexpr double pi = 3.14159265358979323846;
constexpr Torque6 limits{8.,8.,3.,8.,8.,3.};
constexpr Torque6 rates{250.,250.,80.,250.,250.,80.};
bool finite(const Vec3& x) noexcept { for (double v:x) if (!std::isfinite(v)) return false; return true; }
template<std::size_t N> bool finite_a(const std::array<double,N>& x) noexcept { for (double v:x) if (!std::isfinite(v)) return false; return true; }
double norm(const Vec3& x) noexcept { return std::hypot(std::hypot(x[0],x[1]),x[2]); }
Vec3 cross(const Vec3& a, const Vec3& b) noexcept {
    return {a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]};
}
Vec3 rotate_minus_gyro(const Vec3& v, const Vec3& w, double dt) noexcept {
    // Rodrigues form of Rotation.from_rotvec(-w*dt).apply(v), matching the
    // SciPy reference without a quaternion or heap allocation.
    Vec3 axis{-w[0]*dt,-w[1]*dt,-w[2]*dt}; double a=norm(axis);
    if (a<1e-12) return v;
    Vec3 k{axis[0]/a,axis[1]/a,axis[2]/a}; Vec3 kv=cross(k,v);
    double c=std::cos(a), s=std::sin(a), kd=k[0]*v[0]+k[1]*v[1]+k[2]*v[2];
    return {v[0]*c+kv[0]*s+k[0]*kd*(1.-c),v[1]*c+kv[1]*s+k[1]*kd*(1.-c),v[2]*c+kv[2]*s+k[2]*kd*(1.-c)};
}
}

GroundSensorPipeline::GroundSensorPipeline(std::size_t samples):imu_(samples) { reset(); }
void GroundSensorPipeline::reset() noexcept {
    imu_.reset(); ready_=false; yaw_zero_=0.; filtered_force_={}; motor_={}; rate_={};
}
bool GroundSensorPipeline::update(const SensorPacket& p, CanonicalSensor& canonical, Vec6& attitude) noexcept {
    if (!finite(p.gyro) || !finite(p.specific_force) || !finite_a(p.motor_position) ||
        !finite_a(p.motor_velocity) || !std::isfinite(p.dt) || p.dt<=0.) return false;
    motor_=p.motor_position; rate_=p.motor_velocity;
    // Canonical order is gyro, force, then [position,velocity] in
    // [LF,LR,RF,RR,LW,RW] encoder order, matching the Python sensor bus.
    canonical={p.gyro[0],p.gyro[1],p.gyro[2],p.specific_force[0],p.specific_force[1],p.specific_force[2],
               motor_[0],rate_[0],motor_[1],rate_[1],motor_[3],rate_[3],motor_[4],rate_[4],motor_[2],rate_[2],motor_[5],rate_[5]};
    bool estimator_ready=false;
    if (!ready_) {
        if (!imu_.step({canonical[0],canonical[1],canonical[2],canonical[3],canonical[4],canonical[5]},p.dt,estimator_ready,attitude)) return false;
        if (estimator_ready) { ready_=true; yaw_zero_=attitude[2]; filtered_force_=p.specific_force; }
    } else {
        double a=1.-std::exp(-2.*pi*50.*p.dt);
        for (unsigned i=0;i<3;++i) filtered_force_[i]+=a*(p.specific_force[i]-filtered_force_[i]);
        if (!imu_.step({canonical[0],canonical[1],canonical[2],filtered_force_[0],filtered_force_[1],filtered_force_[2]},p.dt,estimator_ready,attitude)) return false;
    }
    if (ready_) {
        double yaw=std::remainder(attitude[2]-yaw_zero_,2.*pi); attitude[2]=yaw;
    }
    return ready_;
}

void CentroidalSuspensionObserver::reset() noexcept {
    force_={0.,0.,9.81}; previous_relative_velocity_={}; has_previous_=false;
    com_site_offset_={}; low_=high_=age_=0.; transitions_=0; confidence_=1.;
    mode_=JumpSupportEstimate::Mode::ground;
}
bool CentroidalSuspensionObserver::update(const Vec3& specific_force, const Vec3& gyro,
    const Vec3& offset, const std::array<double,18>& jacobian, const Torque6& motor_velocity,
    double dt, JumpSupportEstimate& out) noexcept {
    if (!finite(specific_force)||!finite(gyro)||!finite(offset)||!finite_a(jacobian)||
        !finite_a(motor_velocity)||!std::isfinite(dt)||dt<=0.||dt>.02) return false;
    Vec3 relative{0.,0.,0.};
    for (unsigned r=0;r<3;++r) for (unsigned c=0;c<6;++c) relative[r]+=jacobian[r*6+c]*motor_velocity[c];
    Vec3 wcross=cross(gyro,offset), u{wcross[0]+relative[0],wcross[1]+relative[1],wcross[2]+relative[2]};
    Vec3 correction{};
    if (has_previous_) {
        Vec3 transported=rotate_minus_gyro(previous_relative_velocity_,gyro,dt);
        for (unsigned i=0;i<3;++i) correction[i]=(u[i]-transported[i])/dt;
    }
    Vec3 estimated{specific_force[0]+correction[0],specific_force[1]+correction[1],specific_force[2]+correction[2]};
    double a=1.-std::exp(-dt/.008); for(unsigned i=0;i<3;++i) force_[i]+=a*(estimated[i]-force_[i]);
    previous_relative_velocity_=u;has_previous_=true;com_site_offset_=offset;age_+=dt;
    double magnitude=norm(force_); bool recontact=false;
    low_=magnitude<.30*9.81?low_+dt:0.; high_=magnitude>.75*9.81?high_+dt:0.;
    auto before=mode_;
    using M=JumpSupportEstimate::Mode;
    if(mode_==M::ground && low_>=.006-1e-12) mode_=M::unloading;
    else if(mode_==M::unloading) { if(low_>=.012-1e-12) mode_=M::flight; else if(high_>=.010-1e-12) mode_=M::recover; }
    else if(mode_==M::flight && high_>=.006-1e-12) { mode_=M::recover; recontact=true; }
    else if(mode_==M::recover) { if(low_>=.006-1e-12) mode_=M::unloading; else if(age_>=.080-1e-12 && high_>=.020-1e-12) mode_=M::ground; }
    if(before!=mode_) { ++transitions_; age_=0.; }
    bool airborne=mode_==M::unloading||mode_==M::flight;
    double desired=airborne?0.:std::clamp((magnitude/9.81-.30)/.45,0.,1.);
    double tau=desired<confidence_?.003:.15; confidence_+= (1.-std::exp(-dt/tau))*(desired-confidence_);
    out={mode_,mode_==M::flight,airborne,recontact,std::clamp(confidence_,0.,1.),magnitude,transitions_};
    return true;
}

bool protected_motor_torque(const Torque6& action,const Torque6& previous,double dt,Torque6& output) noexcept {
    if(!finite_a(action)||!finite_a(previous)||!std::isfinite(dt)||dt<=0.||dt>.02) return false;
    for(unsigned i=0;i<6;++i) if(std::abs(previous[i])>limits[i]+1e-8) return false;
    for(unsigned i=0;i<6;++i) { double target=std::clamp(action[i],-1.,1.)*limits[i]; output[i]=std::clamp(previous[i]+std::clamp(target-previous[i],-rates[i]*dt,rates[i]*dt),-limits[i],limits[i]); }
    return true;
}
double skill_gate(double forward,double yaw,unsigned skill) noexcept {
    if(!std::isfinite(forward)||!std::isfinite(yaw)||skill>3) return 0.;
    double move=std::clamp(std::abs(forward)/.02,0.,1.), spin=(1.-move)*std::clamp(std::abs(yaw)/.1,0.,1.);
    return skill==0?1.:skill==1||skill==3?move:spin;
}
double command_profile(double t,double target,double duration) noexcept {
    if(!std::isfinite(t)||!std::isfinite(target)||!std::isfinite(duration)||duration<=0.) return 0.;
    double end=duration-6.; if(t<2.)return 0.; if(t<6.)return target*(t-2.)/4.; if(t<end)return target; if(t<duration-2.)return target*(duration-2.-t)/4.; return 0.;
}
bool mix_residual(const Torque6& nominal,const Torque6& previous,const Torque6& residual,double dt,Torque6& output) noexcept {
    if(!finite_a(nominal)||!finite_a(previous)||!finite_a(residual)||!std::isfinite(dt)||dt<=0.)return false;
    for(unsigned i=0;i<6;++i) if(std::abs(previous[i])>limits[i]+1e-8)return false;
    bool zero=true;for(double v:residual)zero&=v==0.; if(zero){output=nominal;return true;}
    constexpr Torque6 scale{.6,.6,.2,.6,.6,.2};
    for(unsigned i=0;i<6;++i) { double lo=std::max(-limits[i],previous[i]-rates[i]*dt),hi=std::min(limits[i],previous[i]+rates[i]*dt); output[i]=std::clamp(nominal[i]+scale[i]*std::clamp(residual[i],-1.,1.),lo,hi); }
    return true;
}
}
