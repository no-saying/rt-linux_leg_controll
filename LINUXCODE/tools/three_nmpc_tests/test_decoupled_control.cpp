#include "decoupled_control.h"
#include <cassert>
#include <cmath>
#include <iostream>
using namespace p112;
static int block(void*,DecoupledAxis axis,const AxisState4&,const AxisReference4&,double disturbance,double,AxisTorque2& out) noexcept {out.value={axis==DecoupledAxis::yaw?2.:1.,1.}; if(axis==DecoupledAxis::pitch)out.value[0]+=disturbance;return 0;}
int main(){
    auto cfg=DecoupledConfig::defaults(); cfg.eso.bandwidth={20.,20.,20.}; cfg.eso.disturbance_limit={10.,10.,10.};
    DecoupledController c(nullptr,block,cfg); DecoupledInput in{}; in.sequence=7; in.dt=.002;
    in.observed_eso.angle={}; in.observed_eso.rate={}; in.observed_eso.disturbance={}; DecoupledOutput out{};
    assert(c.step(in,out)); assert(out.sequence==7); assert(out.torque[0]==.5&&out.torque[3]==.5&&out.torque[2]==.16&&out.torque[5]==.16); assert(c.acknowledge(out.torque));
    in.sequence=8; in.observed_eso.angle={.01,-.02,.03};
    in.observed_eso.disturbance={.2,-.1,.05};
    assert(c.step(in,out));
    assert(out.sequence==8&&out.eso.disturbance==in.observed_eso.disturbance);
    assert(out.torque[1] < out.torque[4]);
    assert(!c.acknowledge({9.,0.,0.,0.,0.,0.})); assert(c.acknowledge(out.torque));
    const auto nmpc_torque=out.torque; const auto held=out.torque; in.run_nmpc=false; in.sequence=9; assert(c.step(in,out)); assert(out.torque==held);
    DecoupledController parallel(nullptr,block,cfg); assert(parallel.start_parallel()); DecoupledOutput parallel_out{}; DecoupledInput parallel_initial{}; parallel_initial.sequence=7; parallel_initial.dt=.002;
    parallel_initial.observed_eso=in.observed_eso;
    assert(parallel.step_parallel(parallel_initial,parallel_out)); assert(parallel_out.sequence==7); assert(parallel.acknowledge(parallel_out.torque)); in.run_nmpc=true; in.sequence=8;
    assert(parallel.step_parallel(in,parallel_out)); assert(parallel_out.sequence==8); for(unsigned i=0;i<6;++i) assert(std::abs(parallel_out.torque[i]-nmpc_torque[i])<1e-12);
    in.run_nmpc=false; in.sequence=9; assert(parallel.step_parallel(in,parallel_out)); assert(parallel_out.torque==parallel.previous_actual());
    std::cout<<"decoupled control tests passed\n";
}
