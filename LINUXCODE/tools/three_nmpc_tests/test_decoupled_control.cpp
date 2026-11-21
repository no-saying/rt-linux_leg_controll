#include "decoupled_control.h"
#include <cassert>
#include <cmath>
#include <iostream>
using namespace p112;
static int block(void*,DecoupledAxis axis,const AxisState4&,const AxisReference4&,double disturbance,double,AxisTorque2& out) noexcept {out.value={axis==DecoupledAxis::yaw?2.:1.,1.}; if(axis==DecoupledAxis::pitch)out.value[0]+=disturbance;return 0;}
static double allocated_effort(const DecoupledConfig& config,const Torque6& torque,unsigned axis) {
    double value=0.;for(unsigned motor=0;motor<6;++motor)value+=config.motor_effectiveness[axis*6+motor]*torque[motor];return value;
}
static int priority_block(void* context,DecoupledAxis axis,const AxisState4&,const AxisReference4&,double,double,AxisTorque2& out) noexcept {
    out.value={(*static_cast<std::array<double,3>*>(context))[static_cast<unsigned>(axis)],0.};return 0;
}
int main(){
    auto cfg=DecoupledConfig::defaults(); cfg.eso.bandwidth={20.,20.,20.}; cfg.eso.disturbance_limit={10.,10.,10.};
    DecoupledController c(nullptr,block,cfg); DecoupledInput in{}; in.sequence=7; in.dt=.002;
    in.observed_eso.angle={}; in.observed_eso.rate={}; in.observed_eso.disturbance={}; DecoupledOutput out{};
    assert(c.step(in,out)); assert(out.sequence==7); assert(std::abs(out.torque[2]+.16)<1e-9&&std::abs(out.torque[5]-.16)<1e-9); assert(std::abs(allocated_effort(cfg,out.torque,0))<1e-8&&std::abs(allocated_effort(cfg,out.torque,1))<1e-8); assert(c.acknowledge(out.torque));
    in.sequence=8; in.observed_eso.angle={.01,-.02,.03};
    in.observed_eso.disturbance={.2,-.1,.05};
    assert(c.step(in,out));
    assert(out.sequence==8&&out.eso.disturbance==in.observed_eso.disturbance);
    assert(allocated_effort(cfg,out.torque,1)<0.);
    assert(!c.acknowledge({9.,0.,0.,0.,0.,0.})); assert(c.acknowledge(out.torque));
    const auto nmpc_torque=out.torque; const auto held=out.torque; in.run_nmpc=false; in.sequence=9; assert(c.step(in,out)); assert(out.torque==held);
    DecoupledController parallel(nullptr,block,cfg); assert(parallel.start_parallel()); DecoupledOutput parallel_out{}; DecoupledInput parallel_initial{}; parallel_initial.sequence=7; parallel_initial.dt=.002;
    parallel_initial.observed_eso.angle={}; parallel_initial.observed_eso.rate={}; parallel_initial.observed_eso.disturbance={};
    assert(parallel.step_parallel(parallel_initial,parallel_out)); assert(parallel_out.sequence==7); assert(parallel.acknowledge(parallel_out.torque)); in.run_nmpc=true; in.sequence=8;
    assert(parallel.step_parallel(in,parallel_out)); assert(parallel_out.sequence==8); for(unsigned i=0;i<6;++i) assert(std::abs(parallel_out.torque[i]-nmpc_torque[i])<1e-12);
    in.run_nmpc=false; in.sequence=9; assert(parallel.step_parallel(in,parallel_out)); assert(parallel_out.torque==parallel.previous_actual());
    in.run_nmpc=true;
    for(std::uint64_t sequence=10;sequence<2010;++sequence) {
        in.sequence=sequence;
        assert(parallel.step_parallel(in,parallel_out));
        assert(parallel_out.sequence==sequence);
    }
    auto priority_cfg=DecoupledConfig::defaults();
    priority_cfg.axis_input_gain={1.,0.,1.,0.,1.,0.};
    priority_cfg.motor_effectiveness={1.,0.,0.,0.,0.,0., 1.,0.,0.,0.,0.,0., 0.,0.,1.,0.,0.,0.};
    std::array<double,3> priority_request{0.,1.,0.};
    DecoupledController priority_controller(&priority_request,priority_block,priority_cfg);
    DecoupledInput priority_input{};priority_input.sequence=1;priority_input.dt=.01;
    DecoupledOutput priority_output{};
    assert(priority_controller.step(priority_input,priority_output));
    assert(std::abs(priority_output.torque[0]-.8888)<1e-3);
    assert(std::abs(allocated_effort(priority_cfg,priority_output.torque,1)-priority_request[1])<.12);
    std::cout<<"decoupled control tests passed\n";
}
