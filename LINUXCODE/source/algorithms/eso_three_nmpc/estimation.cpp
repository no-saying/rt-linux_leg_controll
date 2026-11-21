#include "estimation.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace p112 {
namespace {
constexpr double pi=3.14159265358979323846;
template<std::size_t N> bool finite(const std::array<double,N>& a) noexcept { for(double x:a) if(!std::isfinite(x)) return false; return true; }
double norm3(const Vec3& x) noexcept { return std::hypot(std::hypot(x[0],x[1]),x[2]); }
}
MotionImu::MotionImu(std::size_t samples):alignment_samples_(samples) { if(!samples) throw std::invalid_argument("alignment_samples must be positive"); reset(); }
void MotionImu::reset() noexcept { count_=0; sum_={}; attitude_={}; bias_={}; accel_={0,0,9.81}; q_={1,0,0,0}; ready_=false; }
bool MotionImu::step(const Vec6& y,double dt,bool& ready,Vec6& attitude) noexcept {
    if(!finite(y) || !std::isfinite(dt) || dt<=0) return false;
    if(!ready_) {
        for(unsigned i=0;i<6;++i) sum_[i]+=y[i];
        if(++count_<alignment_samples_) { ready=false; attitude=attitude_; return true; }
        for(unsigned i=0;i<3;++i) { bias_[i]=sum_[i]/count_; accel_[i]=sum_[i+3]/count_; }
        double roll=std::atan2(accel_[1],accel_[2]), pitch=std::atan2(-accel_[0],std::hypot(accel_[1],accel_[2]));
        double cr=std::cos(roll/2),sr=std::sin(roll/2),cp=std::cos(pitch/2),sp=std::sin(pitch/2);
        q_={cr*cp,sr*cp,cr*sp,-sr*sp}; ready_=true;
    }
    Vec3 gyro{}, corr{}, force{y[3],y[4],y[5]};
    for(unsigned i=0;i<3;++i) gyro[i]=y[i]-bias_[i];
    if(norm3(force)>1e-6) {
        double a=1-std::exp(-2*pi*2.5*dt);
        for(unsigned i=0;i<3;++i) accel_[i]+=a*(force[i]-accel_[i]);
        double mag=norm3(accel_);
        if(std::abs(mag-9.81)<.8) {
            double w=q_[0],x=q_[1],z=q_[3],v=q_[2];
            Vec3 pred{2*(x*z-w*v),2*(v*z+w*x),1-2*(x*x+v*v)};
            Vec3 m{accel_[0]/mag,accel_[1]/mag,accel_[2]/mag};
            Vec3 err{m[1]*pred[2]-m[2]*pred[1],m[2]*pred[0]-m[0]*pred[2],m[0]*pred[1]-m[1]*pred[0]};
            if(norm3(err)<.15) for(unsigned i=0;i<3;++i) {corr[i]=.3*err[i];bias_[i]-=.003*dt*err[i];}
        }
    }
    Vec3 rv{};for(unsigned i=0;i<3;++i)rv[i]=(gyro[i]+corr[i])*dt;
    double angle=norm3(rv),factor=angle<1e-12?.5:std::sin(angle/2)/angle;
    double a=std::cos(angle/2),b=rv[0]*factor,c=rv[1]*factor,d=rv[2]*factor;
    double w=q_[0],x=q_[1],yq=q_[2],z=q_[3];
    q_={w*a-x*b-yq*c-z*d,w*b+x*a+yq*d-z*c,w*c-x*d+yq*a+z*b,w*d+x*c-yq*b+z*a};
    double n=std::hypot(std::hypot(q_[0],q_[1]),std::hypot(q_[2],q_[3])); for(double& v:q_)v/=n;
    w=q_[0];x=q_[1];yq=q_[2];z=q_[3];
    double s=std::clamp(2*(w*yq-z*x),-1.,1.);
    attitude_[1]=std::asin(s);
    if(std::abs(std::cos(attitude_[1]))>1e-7) {
        attitude_[0]=std::atan2(2*(w*x+yq*z),1-2*(x*x+yq*yq));
        attitude_[2]=std::atan2(2*(w*z+x*yq),1-2*(yq*yq+z*z));
    } else { attitude_[0]=std::atan2(2*(w*x-yq*z),1-2*(x*x+z*z)); attitude_[2]=0.; }
    for(unsigned i=0;i<3;++i)attitude_[i+3]=gyro[i];
    ready=true;attitude=attitude_;return true;
}
TurnOdometryHold::TurnOdometryHold(double dt,double gain,double speed,double acceleration,double deadband):dt_(dt),gain_(gain),max_speed_(speed),max_acceleration_(acceleration),deadband_(deadband) {
    if(!finite(std::array<double,5>{dt,gain,speed,acceleration,deadband}) || dt<=0 || gain<0 || speed<=0 || acceleration<=0 || deadband<0) throw std::invalid_argument("invalid odometry settings");
}
void TurnOdometryHold::reset() noexcept {position_={};previous_={};correction_=0.;has_previous_=false;}
bool TurnOdometryHold::step(const Vec2& v,double heading,bool enabled,double& output) noexcept {
    if(!finite(v) || !std::isfinite(heading)) return false;
    if(!enabled) {reset();output=0;return true;}
    if(has_previous_)for(unsigned i=0;i<2;++i)position_[i]+=.5*dt_*(previous_[i]+v[i]);
    previous_=v;has_previous_=true;
    double radius=std::hypot(position_[0],position_[1]);
    double along=position_[0]*std::cos(heading)+position_[1]*std::sin(heading);
    double excess=std::max(0.,1.-deadband_/std::max(radius,1e-12));
    double desired=std::clamp(-gain_*excess*along,-max_speed_,max_speed_);
    double delta=max_acceleration_*dt_;
    correction_=gain_==0.?0.:correction_+std::clamp(desired-correction_,-delta,delta);
    output=correction_;return true;
}
}
