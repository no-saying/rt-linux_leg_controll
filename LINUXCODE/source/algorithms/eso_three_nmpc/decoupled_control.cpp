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
    if(!finite(c.axis_input_gain)||!finite(c.motor_effectiveness)||
       !finite(c.priority_weight)||!finite(c.motor_torque_limit)||
       !finite(c.motor_torque_rate)||!std::isfinite(c.allocation_regularization)||
       c.allocation_regularization<=0.)return false;
    for(unsigned axis=0;axis<3;++axis) {
        if(c.priority_weight[axis]<=0.)return false;
        double row_norm=0.;
        for(unsigned motor=0;motor<6;++motor) {
            const double value=c.motor_effectiveness[axis*6+motor];
            row_norm+=value*value;
            if(c.motor_torque_limit[motor]<=0.||c.motor_torque_rate[motor]<=0.)return false;
        }
        if(row_norm<1e-12||std::abs(c.axis_input_gain[axis*2])+std::abs(c.axis_input_gain[axis*2+1])<1e-8)return false;
    }
    return true;
}
bool solve_linear(double matrix[6][6],double rhs[6],unsigned n) noexcept {
    for(unsigned column=0;column<n;++column) {
        unsigned pivot=column;
        for(unsigned row=column+1;row<n;++row)
            if(std::abs(matrix[row][column])>std::abs(matrix[pivot][column]))pivot=row;
        if(std::abs(matrix[pivot][column])<1e-14)return false;
        if(pivot!=column) {
            for(unsigned j=column;j<n;++j)std::swap(matrix[pivot][j],matrix[column][j]);
            std::swap(rhs[pivot],rhs[column]);
        }
        const double scale=matrix[column][column];
        for(unsigned j=column;j<n;++j)matrix[column][j]/=scale;
        rhs[column]/=scale;
        for(unsigned row=0;row<n;++row)if(row!=column) {
            const double factor=matrix[row][column];
            for(unsigned j=column;j<n;++j)matrix[row][j]-=factor*matrix[column][j];
            rhs[row]-=factor*rhs[column];
        }
    }
    return true;
}
bool allocate_motor_torque(const DecoupledConfig& config,
                           const std::array<AxisTorque2,3>& axis_input,
                           const Torque6& previous,double dt,
                           Torque6& output) noexcept {
    if(!finite(previous)||!std::isfinite(dt)||dt<=0.||dt>.02)return false;
    constexpr unsigned motors=6;
    double h[motors][motors]{};
    double g[motors]{};
    for(unsigned axis=0;axis<3;++axis) {
        const double desired=config.axis_input_gain[axis*2]*axis_input[axis].value[0]+
                             config.axis_input_gain[axis*2+1]*axis_input[axis].value[1];
        const double weight=config.priority_weight[axis];
        if(!std::isfinite(desired))return false;
        for(unsigned i=0;i<motors;++i) {
            const double bi=config.motor_effectiveness[axis*motors+i];
            g[i]+=weight*bi*desired;
            for(unsigned j=0;j<motors;++j)
                h[i][j]+=weight*bi*config.motor_effectiveness[axis*motors+j];
        }
    }
    const double regularization=config.allocation_regularization;
    double lower[motors]{},upper[motors]{};
    for(unsigned i=0;i<motors;++i) {
        h[i][i]+=regularization;
        g[i]+=regularization*previous[i];
        lower[i]=std::max(-config.motor_torque_limit[i],
                          previous[i]-config.motor_torque_rate[i]*dt);
        upper[i]=std::min(config.motor_torque_limit[i],
                          previous[i]+config.motor_torque_rate[i]*dt);
    }

    // Most cycles do not hit actuator bounds. Solve the unconstrained QP once
    // and return immediately when it is feasible; only saturated cycles need
    // the exhaustive active-set fallback below.
    {
        double unconstrained[6][6]{},rhs[6]{};
        for(unsigned i=0;i<motors;++i) {
            rhs[i]=g[i];
            for(unsigned j=0;j<motors;++j)unconstrained[i][j]=h[i][j];
        }
        if(solve_linear(unconstrained,rhs,motors)) {
            bool feasible=true;
            for(unsigned i=0;i<motors;++i)
                if(rhs[i]<lower[i]-1e-8||rhs[i]>upper[i]+1e-8)feasible=false;
            if(feasible) {
                for(unsigned i=0;i<motors;++i)output[i]=std::clamp(rhs[i],lower[i],upper[i]);
                return finite(output);
            }
        }
    }

    // Six actuators make exhaustive lower/free/upper active-set enumeration
    // small (3^6 cases) and deterministic, while satisfying the box QP KKT
    // conditions exactly up to floating-point tolerance.
    constexpr unsigned cases=729;
    double best_cost=std::numeric_limits<double>::infinity();
    bool found=false;
    for(unsigned code=1;code<cases;++code) {
        unsigned state_code=code;
        unsigned free_index[motors]{},free_count=0;
        int state[motors]{}; // 0 free, 1 lower, 2 upper
        Torque6 candidate{};
        for(unsigned i=0;i<motors;++i) {
            state[i]=static_cast<int>(state_code%3);state_code/=3;
            if(state[i]==0)free_index[free_count++]=i;
            else candidate[i]=state[i]==1?lower[i]:upper[i];
        }
        double reduced[6][6]{},rhs[6]{};
        for(unsigned row=0;row<free_count;++row) {
            const unsigned i=free_index[row];rhs[row]=g[i];
            for(unsigned j=0;j<motors;++j)if(state[j]!=0)rhs[row]-=h[i][j]*candidate[j];
            for(unsigned column=0;column<free_count;++column)
                reduced[row][column]=h[i][free_index[column]];
        }
        if(free_count&&!solve_linear(reduced,rhs,free_count))continue;
        bool feasible=true;
        for(unsigned row=0;row<free_count;++row) {
            const unsigned i=free_index[row];candidate[i]=rhs[row];
            if(candidate[i]<lower[i]-1e-8||candidate[i]>upper[i]+1e-8)feasible=false;
            candidate[i]=std::clamp(candidate[i],lower[i],upper[i]);
        }
        if(!feasible)continue;
        for(unsigned i=0;i<motors;++i)if(state[i]!=0) {
            double gradient=-g[i];
            for(unsigned j=0;j<motors;++j)gradient+=h[i][j]*candidate[j];
            if((state[i]==1&&gradient < -1e-7)||(state[i]==2&&gradient > 1e-7))feasible=false;
        }
        if(!feasible)continue;
        double cost=0.;
        for(unsigned i=0;i<motors;++i) {
            cost-=g[i]*candidate[i];
            for(unsigned j=0;j<motors;++j)cost+=.5*candidate[i]*h[i][j]*candidate[j];
        }
        if(cost<best_cost) {best_cost=cost;output=candidate;found=true;}
    }
    return found&&finite(output);
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
DecoupledConfig DecoupledConfig::defaults() noexcept {
    DecoupledConfig c{};
    c.axis_input_gain={1.8,-1.8, 1.5,-1.5, 1.2,-1.2};
    // Initial symmetric effectiveness model in motor order LF, LR, left wheel,
    // RF, RR, right wheel. Replace these coefficients with the robot model.
    c.motor_effectiveness={
         1.8, 1.8, 0.,-1.8,-1.8, 0.,
         1.5,-1.5, 0., 1.5,-1.5, 0.,
         0.,  0., -1.2,0.,  0.,  1.2
    };
    return c;
}
DecoupledController::DecoupledController(void* ctx,BlockNmpcCallback cb,const DecoupledConfig& c):context_(ctx),callback_(cb),config_(c){if(!callback_||!configure(c))callback_=nullptr;reset();}
DecoupledController::~DecoupledController() {
    if(parallel_started_) {
        {
            std::lock_guard<std::mutex> lock(parallel_start_mutex_);
            parallel_stop_.store(true,std::memory_order_release);
            parallel_generation_.fetch_add(1,std::memory_order_release);
        }
        parallel_start_cv_.notify_all();
        for(auto& worker:workers_) if(worker.joinable()) worker.join();
        parallel_started_=false;
    }
}
bool DecoupledController::configure(const DecoupledConfig& c) noexcept {if(parallel_started_||!callback_||!valid_config(c))return false;config_=c;return true;}
void DecoupledController::reset() noexcept {previous_actual_={};previous_axis_request_={};initialized_=false;}
bool DecoupledController::acknowledge(const Torque6& actual) noexcept {if(!finite(actual))return false;for(unsigned i=0;i<6;++i)if(std::abs(actual[i])>config_.motor_torque_limit[i]+1e-8)return false;previous_actual_=actual;initialized_=true;return true;}
bool DecoupledController::step(const DecoupledInput& in,DecoupledOutput& out) noexcept {
    if(!callback_||!std::isfinite(in.dt)||in.dt<=0.||in.dt>.02) return false;
    const double control_dt=in.nmpc_dt>0.?in.nmpc_dt:in.dt;
    if(!std::isfinite(control_dt)||control_dt<=0.||control_dt>.02) return false;
    const EsoState& estimate=in.observed_eso;
    if(!finite(estimate.angle)||!finite(estimate.rate)||!finite(estimate.disturbance)) return false;
    DecoupledOutput next{}; next.sequence=in.sequence; next.eso=estimate;
    if(!in.run_nmpc) {
        next.torque=previous_actual_;
        next.axis_torque=previous_axis_request_;
        out=next;
        return true;
    }
    std::array<AxisTorque2,3> axis_input{};
    for(unsigned axis=0;axis<3;++axis) {
        AxisTorque2 block{};
        if(callback_(context_,static_cast<DecoupledAxis>(axis),in.axis_state[axis],
                     in.reference[axis],estimate.disturbance[axis],control_dt,block)!=0 ||
           !finite(block.value)) return false;
        axis_input[axis]=block;
    }
    if(!allocate_motor_torque(config_,axis_input,previous_actual_,control_dt,next.torque))return false;
    next.axis_torque=axis_input;
    previous_axis_request_=axis_input;
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
        bool all_completed=false;
        {
            std::lock_guard<std::mutex> lock(parallel_mutex_);
            all_completed=parallel_completed_.fetch_add(1,std::memory_order_release)+1==3;
        }
        if(all_completed) parallel_cv_.notify_one();
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
        {
            std::lock_guard<std::mutex> lock(parallel_start_mutex_);
            parallel_stop_.store(true,std::memory_order_release);
            parallel_generation_.fetch_add(1,std::memory_order_release);
        }
        parallel_start_cv_.notify_all();
        for(unsigned i=0;i<started;++i) if(workers_[i].joinable()) workers_[i].join();
        return false;
    }
    while(parallel_ready_.load(std::memory_order_acquire)!=3) std::this_thread::yield();
    if(!parallel_affinity_ok_.load(std::memory_order_acquire)) {
        {
            std::lock_guard<std::mutex> lock(parallel_start_mutex_);
            parallel_stop_.store(true,std::memory_order_release);
            parallel_generation_.fetch_add(1,std::memory_order_release);
        }
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
    {
        std::lock_guard<std::mutex> lock(parallel_mutex_);
        parallel_completed_.store(0,std::memory_order_relaxed);
    }
    {
        std::lock_guard<std::mutex> lock(parallel_start_mutex_);
        parallel_generation_.fetch_add(1,std::memory_order_release);
    }
    parallel_start_cv_.notify_all();
    {
        std::unique_lock<std::mutex> lock(parallel_mutex_);
        parallel_cv_.wait(lock,[this]{return parallel_completed_.load(std::memory_order_acquire)==3;});
    }
    DecoupledOutput next{};next.sequence=parallel_job_.sequence;next.eso=estimate;
    for(unsigned axis=0;axis<3;++axis) {
        if(parallel_job_.status[axis]!=0) return false;
        next.axis_torque[axis]=parallel_job_.output[axis];
    }
    if(!allocate_motor_torque(config_,next.axis_torque,previous_actual_,control_dt,next.torque))return false;
    previous_axis_request_=next.axis_torque;
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
        next.axis_torque=previous_axis_request_;
        output=next;
        return true;
    }
    return finish_parallel(input,estimate,output);
}
}
