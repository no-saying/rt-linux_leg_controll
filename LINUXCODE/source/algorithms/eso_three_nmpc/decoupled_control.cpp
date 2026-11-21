#include "decoupled_control.h"
#include <algorithm>
#include <cmath>
#include <limits>
#if defined(__linux__)
#include <pthread.h>
#include <sched.h>
#endif

namespace p112 {
namespace {
constexpr double pi=3.14159265358979323846;
template<std::size_t N> bool finite(const std::array<double,N>& a) noexcept {for(double v:a)if(!std::isfinite(v))return false;return true;}
double wrap(double x) noexcept {return std::remainder(x,2.*pi);}
double fal(double e,double alpha,double delta) noexcept {const double a=std::abs(e);return a<=delta?e/std::pow(delta,1.-alpha):std::pow(a,alpha)*(e<0.?-1.:1.);}
bool valid_esoconfig(const EsoConfig& c) noexcept {
    if(!finite(c.bandwidth)||!finite(c.disturbance_limit)||!std::isfinite(c.nonlinear_alpha)||!std::isfinite(c.nonlinear_delta)||c.nonlinear_alpha<=0.||c.nonlinear_alpha>=1.||c.nonlinear_delta<=0.)return false;
    for(unsigned i=0;i<3;++i) {
        if(c.bandwidth[i]<=0.||c.disturbance_limit[i]<=0.) return false;
    }
    return true;
}
bool valid_config(const DecoupledConfig& c) noexcept {
    if(!valid_esoconfig(c.eso))return false;
    bool used[6]={};for(const auto& a:c.axis){for(unsigned j=0;j<2;++j){if(a.motor_index[j]>=6||used[a.motor_index[j]]||!std::isfinite(a.input_gain[j])||std::abs(a.input_gain[j])<1e-8||!std::isfinite(a.torque_limit[j])||a.torque_limit[j]<=0.||!std::isfinite(a.torque_rate[j])||a.torque_rate[j]<=0.)return false;used[a.motor_index[j]]=true;}}
    return true;
}
}
GlobalEso::GlobalEso(const EsoConfig& c):config_(c){if(!configure(c))config_=EsoConfig{};reset();}
bool GlobalEso::configure(const EsoConfig& c) noexcept {if(!valid_esoconfig(c))return false;config_=c;return true;}
void GlobalEso::reset(const Vec3& a,const Vec3& r) noexcept {state_.angle=a;state_.rate=r;state_.disturbance={};initialized_=false;}
bool GlobalEso::update(const EsoMeasurement& m,double dt,EsoState& out) noexcept {
    if(!finite(m.angle)||!finite(m.rate)||!finite(m.nominal_acceleration)||!std::isfinite(dt)||dt<=0.||dt>.02)return false;
    if(!initialized_) {
        // A reset pose is not a zero pose.  Prime the observer from the first
        // valid measurement so the initial pose does not become fake d_hat.
        state_.angle=m.angle; state_.rate=m.rate; state_.disturbance={};
        initialized_=true; out=state_; return true;
    }
    // The fal correction has a steep slope around zero. Integrate the
    // 500 Hz sample hold with bounded internal steps to avoid Euler
    // instability in the observer states.
    const unsigned substeps=static_cast<unsigned>(std::ceil(dt/.0002));
    const double h=dt/static_cast<double>(substeps);
    EsoState next=state_;
    for(unsigned step=0;step<substeps;++step) {
        const EsoState old=next;
        for(unsigned i=0;i<3;++i) {
            const double w=config_.bandwidth[i];
            const double innovation=wrap(m.angle[i]-old.angle[i])+
                                    (m.rate[i]-old.rate[i])/std::max(w,1e-6);
            const double correction=fal(innovation,config_.nonlinear_alpha,
                                        config_.nonlinear_delta);
            const double b1=3.*w,b2=3.*w*w,b3=w*w*w;
            next.angle[i]=wrap(old.angle[i]+h*(old.rate[i]+b1*innovation));
            next.rate[i]=old.rate[i]+h*(m.nominal_acceleration[i]+
                                        old.disturbance[i]+b2*correction);
            next.disturbance[i]=std::clamp(old.disturbance[i]+h*b3*correction,
                -config_.disturbance_limit[i],config_.disturbance_limit[i]);
        }
    }
    if(!finite(next.angle)||!finite(next.rate)||!finite(next.disturbance)) return false;
    state_=next; out=state_; return true;
}
DecoupledConfig DecoupledConfig::defaults() noexcept {DecoupledConfig c{};c.axis[0].motor_index={0,3};c.axis[1].motor_index={1,4};c.axis[2].motor_index={2,5};for(auto& a:c.axis){a.input_gain={1.,1.};a.torque_limit={8.,8.};a.torque_rate={250.,250.};}c.axis[2].torque_limit={3.,3.};c.axis[2].torque_rate={80.,80.};return c;}
DecoupledController::DecoupledController(void* ctx,BlockNmpcCallback cb,const DecoupledConfig& c):context_(ctx),callback_(cb),config_(c){if(!callback_||!configure(c))callback_=nullptr;reset();}
DecoupledController::~DecoupledController() {
    if(parallel_started_) {
        parallel_stop_.store(true,std::memory_order_release);
        parallel_generation_.fetch_add(1,std::memory_order_release);
        parallel_start_cv_.notify_all();
        for(auto& worker:workers_) if(worker.joinable()) worker.join();
        parallel_started_=false;
    }
}
bool DecoupledController::configure(const DecoupledConfig& c) noexcept {if(parallel_started_||!callback_||!valid_config(c))return false;config_=c;return true;}
void DecoupledController::reset() noexcept {previous_actual_={};initialized_=false;}
bool DecoupledController::acknowledge(const Torque6& actual) noexcept {if(!finite(actual))return false;for(const auto& a:config_.axis)for(unsigned j=0;j<2;++j)if(std::abs(actual[a.motor_index[j]])>a.torque_limit[j]+1e-8)return false;previous_actual_=actual;initialized_=true;return true;}
bool DecoupledController::step(const DecoupledInput& in,DecoupledOutput& out) noexcept {
    if(!callback_||!std::isfinite(in.dt)||in.dt<=0.||in.dt>.02) return false;
    const double control_dt=in.nmpc_dt>0.?in.nmpc_dt:in.dt;
    if(!std::isfinite(control_dt)||control_dt<=0.||control_dt>.02) return false;
    const EsoState& estimate=in.observed_eso;
    if(!finite(estimate.angle)||!finite(estimate.rate)||!finite(estimate.disturbance)) return false;
    DecoupledOutput next{}; next.sequence=in.sequence; next.eso=estimate;
    if(!in.run_nmpc) {
        next.torque=previous_actual_;
        for(unsigned axis=0;axis<3;++axis) for(unsigned j=0;j<2;++j)
            next.axis_torque[axis].value[j]=previous_actual_[config_.axis[axis].motor_index[j]];
        out=next;
        return true;
    }
    for(unsigned axis=0;axis<3;++axis) {
        AxisTorque2 block{};
        if(callback_(context_,static_cast<DecoupledAxis>(axis),in.axis_state[axis],
                     in.reference[axis],estimate.disturbance[axis],control_dt,block)!=0 ||
           !finite(block.value)) return false;
        const auto& cfg=config_.axis[axis];
        for(unsigned j=0;j<2;++j) {
            double previous=previous_actual_[cfg.motor_index[j]];
            double lo=std::max(-cfg.torque_limit[j],previous-cfg.torque_rate[j]*in.dt);
            double hi=std::min(cfg.torque_limit[j],previous+cfg.torque_rate[j]*control_dt);
            next.axis_torque[axis].value[j]=std::clamp(block.value[j],lo,hi);
            next.torque[cfg.motor_index[j]]=next.axis_torque[axis].value[j];
        }
    }
    out=next; return true;
}
void DecoupledController::worker_loop(unsigned axis) noexcept {
#if defined(__linux__)
    if(cpu_affinity_[axis]>=0) {
        cpu_set_t set;CPU_ZERO(&set);
        if(cpu_affinity_[axis]>=CPU_SETSIZE) {
            parallel_affinity_ok_.store(false,std::memory_order_release);
        } else {
            CPU_SET(cpu_affinity_[axis],&set);
            if(pthread_setaffinity_np(pthread_self(),sizeof(set),&set)!=0)
                parallel_affinity_ok_.store(false,std::memory_order_release);
        }
    }
#else
    if(cpu_affinity_[axis]>=0) parallel_affinity_ok_.store(false,std::memory_order_release);
#endif
    parallel_ready_.fetch_add(1,std::memory_order_release);
    std::uint64_t seen=0;
    while(!parallel_stop_.load(std::memory_order_acquire)) {
        std::uint64_t generation=0;
        {
            std::unique_lock<std::mutex> lock(parallel_start_mutex_);
            parallel_start_cv_.wait(lock,[this,seen]{return parallel_stop_.load(std::memory_order_acquire)||parallel_generation_.load(std::memory_order_acquire)!=seen;});
            if(parallel_stop_.load(std::memory_order_acquire)) break;
            generation=parallel_generation_.load(std::memory_order_acquire);
        }
        seen=generation;
        AxisTorque2 block{};
        int status=-1;
        if(callback_) {
            status=callback_(context_,static_cast<DecoupledAxis>(axis),parallel_job_.axis_state[axis],
                              parallel_job_.reference[axis],parallel_job_.disturbance[axis],
                              parallel_job_.dt,block);
            if(status==0&&!finite(block.value)) status=-2;
        }
        parallel_job_.output[axis]=block;
        parallel_job_.status[axis]=status;
        if(parallel_completed_.fetch_add(1,std::memory_order_release)+1==3) parallel_cv_.notify_one();
    }
}
bool DecoupledController::start_parallel(const std::array<int,3>& cpu_affinity) noexcept {
    if(parallel_started_) return true;
    if(!callback_) return false;
    for(int cpu:cpu_affinity) if(cpu<-1) return false;
    cpu_affinity_=cpu_affinity;
    parallel_stop_.store(false,std::memory_order_relaxed);
    parallel_generation_.store(0,std::memory_order_relaxed);
    parallel_completed_.store(0,std::memory_order_relaxed);
    parallel_ready_.store(0,std::memory_order_relaxed);
    parallel_affinity_ok_.store(true,std::memory_order_relaxed);
    unsigned started=0;
    try {
        for(;started<3;++started) workers_[started]=std::thread(&DecoupledController::worker_loop,this,started);
    } catch(...) {
        parallel_stop_.store(true,std::memory_order_release);
        parallel_generation_.fetch_add(1,std::memory_order_release);
        parallel_start_cv_.notify_all();
        for(unsigned i=0;i<started;++i) if(workers_[i].joinable()) workers_[i].join();
        return false;
    }
    while(parallel_ready_.load(std::memory_order_acquire)!=3) std::this_thread::yield();
    if(!parallel_affinity_ok_.load(std::memory_order_acquire)) {
        parallel_stop_.store(true,std::memory_order_release);
        parallel_generation_.fetch_add(1,std::memory_order_release);
        parallel_start_cv_.notify_all();
        for(auto& worker:workers_) if(worker.joinable()) worker.join();
        return false;
    }
    parallel_started_=true;
    return true;
}
bool DecoupledController::finish_parallel(const DecoupledInput& input,const EsoState& estimate,DecoupledOutput& output) noexcept {
    const double control_dt=input.nmpc_dt>0.?input.nmpc_dt:input.dt;
    for(unsigned axis=0;axis<3;++axis) {
        parallel_job_.axis_state[axis]=input.axis_state[axis];
        parallel_job_.reference[axis]=input.reference[axis];
        parallel_job_.disturbance[axis]=estimate.disturbance[axis];
        parallel_job_.output[axis]={};
        parallel_job_.status[axis]=-3;
    }
    parallel_job_.dt=control_dt;
    parallel_job_.sequence=input.sequence;
    parallel_completed_.store(0,std::memory_order_relaxed);
    parallel_generation_.fetch_add(1,std::memory_order_release);
    parallel_start_cv_.notify_all();
    {
        std::unique_lock<std::mutex> lock(parallel_mutex_);
        parallel_cv_.wait(lock,[this]{return parallel_completed_.load(std::memory_order_acquire)==3;});
    }
    DecoupledOutput next{};next.sequence=parallel_job_.sequence;next.eso=estimate;
    for(unsigned axis=0;axis<3;++axis) {
        if(parallel_job_.status[axis]!=0) return false;
        const auto& cfg=config_.axis[axis];
        for(unsigned j=0;j<2;++j) {
            double previous=previous_actual_[cfg.motor_index[j]];
            double lo=std::max(-cfg.torque_limit[j],previous-cfg.torque_rate[j]*input.dt);
            double hi=std::min(cfg.torque_limit[j],previous+cfg.torque_rate[j]*control_dt);
            next.axis_torque[axis].value[j]=std::clamp(parallel_job_.output[axis].value[j],lo,hi);
            next.torque[cfg.motor_index[j]]=next.axis_torque[axis].value[j];
        }
    }
    output=next;return true;
}
bool DecoupledController::step_parallel(const DecoupledInput& input,DecoupledOutput& output) noexcept {
    if(!parallel_started_||!callback_||!std::isfinite(input.dt)||input.dt<=0.||input.dt>.02)return false;
    const double control_dt=input.nmpc_dt>0.?input.nmpc_dt:input.dt;
    if(!std::isfinite(control_dt)||control_dt<=0.||control_dt>.02)return false;
    const EsoState& estimate=input.observed_eso;
    if(!finite(estimate.angle)||!finite(estimate.rate)||!finite(estimate.disturbance))return false;
    if(!input.run_nmpc) {
        DecoupledOutput next{}; next.sequence=input.sequence; next.eso=estimate; next.torque=previous_actual_;
        for(unsigned axis=0;axis<3;++axis) for(unsigned j=0;j<2;++j)
            next.axis_torque[axis].value[j]=previous_actual_[config_.axis[axis].motor_index[j]];
        output=next;
        return true;
    }
    return finish_parallel(input,estimate,output);
}
}
