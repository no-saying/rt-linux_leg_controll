#include "scheduler.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace p112 {
namespace { constexpr Torque6 limits{8.,8.,3.,8.,8.,3.},rates{250.,250.,80.,250.,250.,80.};
bool finite(const Torque6& x){for(double v:x)if(!std::isfinite(v))return false;return true;}
bool finite(const State15& x){for(double v:x)if(!std::isfinite(v))return false;return true;}
bool actor_mode(SchedulerMode m){return m==SchedulerMode::jump_rl||m==SchedulerMode::locomotion_rl||m==SchedulerMode::spin_rl||m==SchedulerMode::recovery_rl;}
}
AlgorithmScheduler::AlgorithmScheduler(void* pc,PlanSolveCallback p,void* ac,ActorCallback a,ResidualAllocateCallback residual):plan_context_(pc),plan_(p),actor_context_(ac),actor_(a),residual_(residual){if(!plan_||!actor_)throw std::invalid_argument("scheduler callbacks required");reset();}
void AlgorithmScheduler::reset() noexcept{previous_actual_={};last_time_=0.;has_time_=false;}
bool AlgorithmScheduler::acknowledge(const Torque6& actual) noexcept{if(!finite(actual))return false;for(unsigned i=0;i<6;++i)if(std::abs(actual[i])>limits[i]+1e-8)return false;previous_actual_=actual;return true;}
bool AlgorithmScheduler::step(const SchedulerRequest& r,const State15& state,const std::array<float,42>& obs,double dt,SchedulerOutput& out) noexcept {
    if(!finite(state)||!std::isfinite(r.now)||!std::isfinite(r.speed)||!std::isfinite(r.yaw_rate)||!std::isfinite(r.confidence)||r.confidence<0.||r.confidence>1.||!std::isfinite(dt)||dt<=0.||dt>.02)return false;
    if(has_time_&&r.now<=last_time_)return false;
    if(actor_mode(r.mode)){for(float v:obs)if(!std::isfinite(v)||std::abs(v)>5.000001f)return false;}
    Torque6 requested{};int status=0;bool from_actor=actor_mode(r.mode);
    if(r.mode==SchedulerMode::ground_mpc||r.mode==SchedulerMode::nmpc) status=plan_(plan_context_,state,previous_actual_,r.speed,r.yaw_rate,r.confidence,r.now,requested);
    else if(from_actor) {
        Torque6 normalized{}; status=actor_(actor_context_,r.mode,obs,normalized);
        if(!status && (!finite(normalized) || [&](){for(double v:normalized)if(std::abs(v)>1.000001)return true;return false;}())) status=-1;
        if(!status && r.mode==SchedulerMode::jump_rl) status=protected_motor_torque(normalized,previous_actual_,dt,requested)?0:-1;
        else if(!status && residual_) status=residual_(actor_context_,r.mode,state,previous_actual_,normalized,dt,requested);
        else if(!status) status=-1;
    }
    else if(r.mode==SchedulerMode::hold) requested={}; else return false;
    if(status||!finite(requested))return false;
    Torque6 limited{};for(unsigned i=0;i<6;++i){double lo=std::max(-limits[i],previous_actual_[i]-rates[i]*dt),hi=std::min(limits[i],previous_actual_[i]+rates[i]*dt);limited[i]=std::clamp(requested[i],lo,hi);}
    out={limited,r.mode,from_actor};last_time_=r.now;has_time_=true;return true;
}
}
