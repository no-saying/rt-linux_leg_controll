#include "control_stack/application/control_runtime.hpp"
#include "control_stack/message_center/message_center.hpp"
#include "control_stack/platform/logging/static_log_buffer.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>

using namespace control_stack;

namespace {
struct Checks { int passed{0}; int failed{0}; };
void check(Checks& c, bool ok, const char* name) {
  if (ok) { ++c.passed; std::printf("PASS %s\n", name); }
  else { ++c.failed; std::printf("FAIL %s\n", name); }
}

class TestControl final : public IControlAlgorithm {
 public:
  void reset(ControlSelection s) override { selection=s; }
  ControlOutput update(const SensorFrame& s, const Command& c) override {
    ++calls;
    saw_perception = s.eso_valid && s.eso_disturbance[0] > 0.0 && s.wheel_slip[1];
    last_command=c;
    ControlOutput o{};o.valid=true;o.confidence=1.0;o.torque.fill(0.1);return o;
  }
  ControlSelection selection{}; Command last_command{}; int calls{0}; bool saw_perception{false};
};
class TestPerception final : public IPerceptionAlgorithm {
 public:
  void reset() override {}
  PerceptionState update(const SensorFrame&) override {
    PerceptionState p{};p.wheel_slip[1]=true;p.wheel_torque_scale[1]=0.6;return p;
  }
};
class TestGreenWave final : public IGreenWaveAlgorithm {
 public:
  void reset() override {}
  GreenWavePlan update(const SensorFrame&,const Command& c) override { return {c,0,false}; }
};
class TestLog final : public ILogSink {
 public: void write(const LogRecord&) override { ++records; } int records{0};
};
}

int main() {
  Checks checks{};
  TestControl control;TestPerception perception;TestGreenWave greenwave;TestLog log;
  ControlApplication app({control,perception,greenwave,&log});
  ControlRuntime runtime(app,&log);
  Command input{};input.x_speed_m_s=0.35;input.z_speed_m_s=8.0;input.leg_height_m=0.24;input.vehicle_state=1;input.motion_state=1;
  SensorFrame sensor{};sensor.body_vx_m_s=0.2;sensor.body_yaw_rad=0.01;
  sensor.eso_disturbance={0.2,-0.1,0.05};sensor.eso_valid=true;
  std::uint8_t wire[kAmpMaxFrameSize]{};std::size_t wire_size=0;
  std::uint8_t sensor_wire[kAmpMaxFrameSize]{};std::size_t sensor_wire_size=0;
  auto send_sensor=[&](std::uint16_t seq,std::uint64_t now) {
    sensor.timestamp_us=now;
    if(!AmpFrameCodec::encodeSensorSnapshot(sensor,seq,static_cast<std::uint32_t>(now/1000),sensor_wire,sizeof(sensor_wire),sensor_wire_size)) return false;
    return runtime.ingestAmpBytes(sensor_wire,sensor_wire_size,now)==AmpDecodeResult::Ok;
  };
  check(checks,AmpFrameCodec::encodeCommand(input,7,100,wire,sizeof(wire),wire_size),"encode AMP setpoint");
  check(checks,runtime.ingestAmpBytes(wire,wire_size,100000)==AmpDecodeResult::Ok,"AMP decode and accept");
  check(checks,!runtime.tick().valid,"hold output until AMP sensor snapshot arrives");
  wire[wire_size-1]^=0x01;
  check(checks,runtime.ingestAmpBytes(wire,wire_size,100500)==AmpDecodeResult::BadCrc,"reject CRC-corrupted AMP frame");
  wire[wire_size-1]^=0x01;
  AmpFrameCodec::encodeSensorSnapshot(sensor,1,101,sensor_wire,sizeof(sensor_wire),sensor_wire_size);
  sensor_wire[sensor_wire_size-2]^=0x40;
  check(checks,runtime.ingestAmpBytes(sensor_wire,sensor_wire_size,100800)==AmpDecodeResult::BadCrc,"reject corrupted sensor frame");
  sensor_wire[sensor_wire_size-2]^=0x40;
  check(checks,send_sensor(1,101000),"AMP sensor snapshot decode");
  auto output=runtime.tick();
  check(checks,output.valid&&control.calls==1,"application to controller cycle");
  check(checks,std::abs(runtime.command().x_speed_m_s-0.35)<0.001&&std::abs(runtime.command().z_speed_m_s-8.0)<0.01,"binary16 command values decoded");
  check(checks,control.saw_perception,"perception state reaches controller input");
  check(checks,runtime.status().state==RobotState::Spin,"state selection from high yaw command");
  check(checks,control.selection.profile==MpcProfile::Locomotion,"application selects locomotion controller profile");
  check(checks,runtime.ingestAmpBytes(wire,wire_size,102000)==AmpDecodeResult::StaleSequence,"reject duplicate sequence");

  input.vehicle_state=2;
  AmpFrameCodec::encodeCommand(input,8,102,wire,sizeof(wire),wire_size);
  runtime.ingestAmpBytes(wire,wire_size,102000);
  check(checks,send_sensor(2,103000),"refresh AMP sensor before ESTOP");
  output=runtime.tick();
  check(checks,!output.valid&&runtime.status().state==RobotState::Fault,"ESTOP suppresses control output");

  input.vehicle_state=1;
  AmpFrameCodec::encodeCommand(input,9,104,wire,sizeof(wire),wire_size);
  runtime.ingestAmpBytes(wire,wire_size,104000);
  send_sensor(3,104500);
  runtime.tick();
  check(checks,runtime.status().state==RobotState::Fault,"fault remains latched after enable command");
  runtime.clearFault();
  input.vehicle_state=1;input.motion_state=4;
  AmpFrameCodec::encodeCommand(input,10,105,wire,sizeof(wire),wire_size);
  runtime.ingestAmpBytes(wire,wire_size,105000);
  sensor.body_parallel=true;
  send_sensor(4,106000);runtime.tick();
  send_sensor(5,107000);runtime.tick();
  check(checks,runtime.status().recovery_stage==RecoveryStage::FoldLegs,"recovery stage: body parallel to fold");
  sensor.leg_height_m=0.07;send_sensor(6,108000);runtime.tick();
  check(checks,runtime.status().recovery_stage==RecoveryStage::MoveLegsToCenter,"recovery stage: fold to center");
  sensor.legs_centered=true;send_sensor(7,109000);runtime.tick();
  check(checks,runtime.status().recovery_stage==RecoveryStage::BalanceHandoff,"recovery stage: center to balance handoff");

  send_sensor(8,200000);
  output=runtime.tick();
  check(checks,!output.valid&&runtime.status().state==RobotState::Disabled,"command timeout enters disabled state");
  MessageCenter messages{};FaultEvent fault{201000,0x55,2},read_fault{};
  check(checks,messages.publishFault(fault)&&messages.pollFault(read_fault)&&read_fault.code==0x55,"static message center fault topic");
  StaticLogBuffer<4> logs{};LogRecord record{LogLevel::Info,201000,"test","static ring"};
  StaticLogBuffer<4>::Record read_record{};
  logs.write(record);
  check(checks,logs.pop(read_record)&&std::strcmp(read_record.event.data(),"test")==0,"static log ring");
  std::printf("RESULT passed=%d failed=%d\n",checks.passed,checks.failed);
  return checks.failed==0?0:1;
}
