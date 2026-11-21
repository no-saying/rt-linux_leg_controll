#include "control_stack/algorithms/control/three_nmpc_adapter.hpp"
#include "control_stack/algorithms/greenwave/greenwave_algorithm.hpp"
#include "control_stack/algorithms/perception/basic_robot_perception.hpp"
#include "control_stack/application/control_runtime.hpp"
#include "control_stack/platform/amp/amp_frame.hpp"
#include "decoupled_nmpc.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>
#include <vector>

namespace {
using Clock = std::chrono::steady_clock;
constexpr std::uint64_t kSamplePeriodUs = 2000;
constexpr std::uint64_t kSamplesPerSecond = 500;
constexpr std::uint64_t kWarmupSamples = 1000;

class BenchGreenWave final : public control_stack::IGreenWaveAlgorithm {
 public:
  void reset() override {}
  control_stack::GreenWavePlan update(const control_stack::SensorFrame&,
                                      const control_stack::Command& command) override {
    ++updates;
    return {command, 0, false};
  }
  std::uint64_t updates{};
};

struct BenchNmpcContext {
  p112::DecoupledNmpc* nmpc{};
  std::atomic<std::uint64_t> axis_solves{0};
};

int bench_nmpc_callback(void* context,p112::DecoupledAxis axis,
                        const p112::AxisState4& state,
                        const p112::AxisReference4& reference,
                        double disturbance,double dt,
                        p112::AxisTorque2& output) noexcept {
  auto& bench=*static_cast<BenchNmpcContext*>(context);
  bench.axis_solves.fetch_add(1,std::memory_order_relaxed);
  return p112::decoupled_nmpc_callback(bench.nmpc,axis,state,reference,
                                       disturbance,dt,output);
}

bool map_input(void*, const control_stack::SensorFrame& sensors,
               const control_stack::Command&, control_stack::ControlSelection,
               p112::DecoupledInput& input) noexcept {
  input.axis_state[0].value = {sensors.state[2], sensors.state[3],
                               sensors.state[13], sensors.state[14]};
  input.axis_state[1].value = {sensors.state[0], sensors.state[1],
                               sensors.state[4], sensors.state[5]};
  input.axis_state[2].value = {sensors.body_yaw_rad, sensors.state[12],
                               sensors.state[10], sensors.state[11]};
  for (auto& reference : input.reference) reference.value = {};
  return true;
}

double percentile(const std::vector<double>& sorted, double p) {
  if (sorted.empty()) return 0.0;
  const auto index = static_cast<std::size_t>(std::ceil(p * sorted.size())) - 1;
  return sorted[std::min(index, sorted.size() - 1)];
}

std::uint64_t monotonic_us() {
  return static_cast<std::uint64_t>(std::chrono::duration_cast<
      std::chrono::microseconds>(Clock::now().time_since_epoch()).count());
}

bool parse_seconds(int argc, char** argv, double& seconds) {
  if (argc == 1) return true;
  if (argc != 3 || std::string(argv[1]) != "--seconds") return false;
  char* end = nullptr;
  seconds = std::strtod(argv[2], &end);
  return end != argv[2] && *end == '\0' && std::isfinite(seconds) &&
         seconds >= 1.0 && seconds <= 86400.0;
}
}  // namespace

int main(int argc, char** argv) {
  double duration_s = 3600.0;
  if (!parse_seconds(argc, argv, duration_s)) {
    std::fprintf(stderr, "usage: %s [--seconds 1..86400]\n", argv[0]);
    return 2;
  }

  p112::DecoupledNmpc nmpc{};
  if (!p112::initialize_decoupled_nmpc(nmpc)) {
    std::fprintf(stderr, "NMPC initialization failed\n");
    return 3;
  }
  BenchNmpcContext bench_nmpc{&nmpc};
  p112::DecoupledController controller(&bench_nmpc,bench_nmpc_callback);
  if (!controller.start_parallel({0, 1, 2})) {
    std::fprintf(stderr, "failed to start three-axis workers on CPU0/1/2\n");
    p112::destroy_decoupled_nmpc(nmpc);
    return 4;
  }
  control_stack::ThreeNmpcControlAlgorithm algorithm(
      controller, nullptr, map_input, 0.01);
  control_stack::BasicRobotPerception perception{};
  BenchGreenWave greenwave{};
  control_stack::ControlApplication app({algorithm, perception, greenwave, nullptr});
  control_stack::ControlRuntime runtime(app);

  const auto total_samples = static_cast<std::uint64_t>(
      std::ceil(duration_s * kSamplesPerSecond));
  const auto warmup_samples = std::min(kWarmupSamples, total_samples / 10);
  std::vector<double> eso_us;
  std::vector<double> control_us;
  eso_us.reserve(total_samples - warmup_samples);
  control_us.reserve(static_cast<std::size_t>(duration_s * 100.0));

  p112::GlobalEso eso{};
  p112::EsoMeasurement measurement{};
  measurement.angle = {0.03, -0.02, 0.01};
  measurement.rate = {0.01, -0.015, 0.02};
  p112::EsoState estimate{};
  control_stack::SensorFrame sensors{};
  sensors.state = {-0.02, -0.015, 0.03, 0.01, 0.02, -0.01,
                   0.25, 0.0, 0.0, 0.0, 0.15, 0.14, 0.02, 0.01, -0.01};
  sensors.body_yaw_rad = 0.01;
  sensors.attitude_valid = true;
  sensors.imu_valid = true;
  sensors.kinematic_state_valid = true;
  sensors.joint_state_valid = true;
  sensors.wheel_speed_valid = true;
  sensors.body_velocity_valid = true;
  sensors.specific_force = {0.0, 0.0, 9.80665};
  sensors.eso_valid = true;
  control_stack::Command command{};
  command.vehicle_state = 1;

  std::uint64_t eso_failures = 0;
  std::uint64_t invalid_outputs = 0;
  std::uint64_t control_cycles = 0;
  std::uint64_t perception_updates = 0;
  std::uint64_t cycles_over_5ms = 0;
  std::uint64_t cycles_over_8ms = 0;
  std::uint64_t cycles_over_10ms = 0;
  std::uint64_t missed_sample_periods = 0;
  std::uint64_t late_eso_deadlines = 0;
  std::uint64_t eso_updates = 0;
  std::uint64_t eso_over_1ms = 0;
  std::uint64_t eso_over_2ms = 0;
  std::uint64_t dispatch_age_over_5ms = 0;
  std::uint64_t output_age_over_5ms = 0;
  std::vector<double> output_age_us;
  output_age_us.reserve(total_samples-warmup_samples);
  std::vector<double> control_period_us;
  control_period_us.reserve(static_cast<std::size_t>(duration_s*100.0));
  std::uint8_t sensor_wire[control_stack::kAmpMaxFrameSize]{};
  std::uint8_t command_wire[control_stack::kAmpMaxFrameSize]{};
  std::size_t sensor_wire_size=0,command_wire_size=0;
  std::uint16_t command_sequence=0;
  std::uint64_t last_control_trigger_us=0;
  const auto begin = Clock::now();
  auto deadline = begin;

  for (std::uint64_t sample = 0; sample < total_samples; ++sample) {
    deadline += std::chrono::microseconds(kSamplePeriodUs);
    std::this_thread::sleep_until(deadline);
    const auto sample_start = Clock::now();
    const auto late_us = std::chrono::duration_cast<std::chrono::microseconds>(
        sample_start - deadline).count();
    if (late_us > static_cast<std::int64_t>(kSamplePeriodUs))
      missed_sample_periods += static_cast<std::uint64_t>(late_us / kSamplePeriodUs);

    const auto full_cycle_start = sample_start;
    const auto eso_start = Clock::now();
    if (!eso.update(measurement, 0.002, estimate)) {
      ++eso_failures;
      break;
    }
    const auto eso_end = Clock::now();
    ++eso_updates;
    const double observer_us =
        std::chrono::duration<double, std::micro>(eso_end - eso_start).count();
    eso_over_1ms += observer_us > 1000.0;
    eso_over_2ms += observer_us > 2000.0;
    if (eso_end > deadline + std::chrono::microseconds(kSamplePeriodUs))
      ++late_eso_deadlines;
    if (sample >= warmup_samples) {
      eso_us.push_back(observer_us);
    }

    sensors.eso_disturbance = estimate.disturbance;
    sensors.eso_valid = true;
    const std::uint64_t received_us=monotonic_us();
    if(sample%25==0) {
      if(!control_stack::AmpFrameCodec::encodeCommand(
             command,++command_sequence,static_cast<std::uint32_t>(received_us/1000),
             command_wire,sizeof(command_wire),command_wire_size) ||
         runtime.ingestAmpBytes(command_wire,command_wire_size,received_us)!=
             control_stack::AmpDecodeResult::Ok) {
        ++invalid_outputs;
        continue;
      }
    }
    if(!control_stack::AmpFrameCodec::encodeSensorSnapshot(
           sensors,static_cast<std::uint16_t>(sample),
           static_cast<std::uint32_t>(received_us/1000),sensor_wire,
           sizeof(sensor_wire),sensor_wire_size) ||
       runtime.ingestAmpBytes(sensor_wire,sensor_wire_size,received_us)!=
           control_stack::AmpDecodeResult::Ok) {
      ++invalid_outputs;
      continue;
    }
    ++perception_updates;
    const std::uint64_t tick_us=monotonic_us();
    const double dispatch_age=static_cast<double>(tick_us-received_us);
    dispatch_age_over_5ms+=dispatch_age>5000.0;
    const auto solves_before=bench_nmpc.axis_solves.load(std::memory_order_relaxed);
    const auto output = runtime.tick(tick_us);
    const auto solves_after=bench_nmpc.axis_solves.load(std::memory_order_relaxed);
    const bool ran_nmpc=solves_after!=solves_before;
    if(ran_nmpc&&solves_after-solves_before!=3)++invalid_outputs;
    const double output_age=static_cast<double>(std::chrono::duration<double,
        std::micro>(Clock::now()-full_cycle_start).count());
    if (!output.valid) ++invalid_outputs;
    else algorithm.acknowledgeAppliedTorque(output.torque);
    if(sample>=warmup_samples) {
      output_age_us.push_back(output_age);
      output_age_over_5ms+=output_age>5000.0;
    }

    if (ran_nmpc) {
      ++control_cycles;
      if (sample >= warmup_samples) {
        control_us.push_back(output_age);
        cycles_over_5ms += output_age > 5000.0;
        cycles_over_8ms += output_age > 8000.0;
        cycles_over_10ms += output_age > 10000.0;
        if(last_control_trigger_us!=0) {
          const double period=static_cast<double>(tick_us-last_control_trigger_us);
          control_period_us.push_back(period);
          missed_sample_periods+=period>15000.0;
        }
        last_control_trigger_us=tick_us;
      }
    }
  }

  const auto finish = Clock::now();
  const double elapsed_s = std::chrono::duration<double>(finish - begin).count();
  std::sort(eso_us.begin(), eso_us.end());
  std::sort(control_us.begin(), control_us.end());
  std::sort(control_period_us.begin(),control_period_us.end());
  std::sort(output_age_us.begin(),output_age_us.end());
  std::printf(
      "{\"scope\":\"host synthetic full pipeline: ESO, AMP codec, perception, planner, NMPC, allocator; no transport or actuator\","
      "\"elapsed_s\":%.3f,\"samples\":%llu,\"sample_hz\":%.3f,"
      "\"eso_updates\":%llu,\"eso_hz\":%.3f,"
      "\"eso_us_p50\":%.3f,\"eso_us_p99\":%.3f,\"eso_us_p999\":%.3f,"
      "\"eso_us_max\":%.3f,\"eso_over_1ms\":%llu,"
      "\"eso_over_2ms\":%llu,"
      "\"control_cycles\":%llu,\"control_hz\":%.3f,"
      "\"control_us_p50\":%.3f,\"control_us_p99\":%.3f,"
      "\"control_us_p999\":%.3f,\"control_us_max\":%.3f,"
      "\"control_period_us_p50\":%.3f,\"control_period_us_p99\":%.3f,"
      "\"control_period_us_max\":%.3f,"
      "\"control_over_5ms\":%llu,\"control_over_8ms\":%llu,"
      "\"control_over_10ms\":%llu,\"missed_sample_periods\":%llu,"
      "\"sensor_to_output_us_p50\":%.3f,\"sensor_to_output_us_p99\":%.3f,"
      "\"sensor_to_output_us_p999\":%.3f,\"sensor_to_output_us_max\":%.3f,"
      "\"sensor_to_output_over_5ms\":%llu,\"dispatch_age_over_5ms\":%llu,"
      "\"perception_updates\":%llu,\"planner_updates\":%llu,"
      "\"nmpc_axis_solves\":%llu,"
      "\"late_eso_deadlines\":%llu,\"solver_or_output_failures\":%llu}\n",
      elapsed_s, static_cast<unsigned long long>(total_samples),
      elapsed_s > 0.0 ? total_samples / elapsed_s : 0.0,
      static_cast<unsigned long long>(eso_updates),
      elapsed_s > 0.0 ? eso_updates / elapsed_s : 0.0,
      percentile(eso_us, 0.50), percentile(eso_us, 0.99),
      percentile(eso_us, 0.999), eso_us.empty() ? 0.0 : eso_us.back(),
      static_cast<unsigned long long>(eso_over_1ms),
      static_cast<unsigned long long>(eso_over_2ms),
      static_cast<unsigned long long>(control_cycles),
      elapsed_s > 0.0 ? control_cycles / elapsed_s : 0.0,
      percentile(control_us, 0.50), percentile(control_us, 0.99),
      percentile(control_us, 0.999), control_us.empty() ? 0.0 : control_us.back(),
      percentile(control_period_us,0.50),percentile(control_period_us,0.99),
      control_period_us.empty()?0.0:control_period_us.back(),
      static_cast<unsigned long long>(cycles_over_5ms),
      static_cast<unsigned long long>(cycles_over_8ms),
      static_cast<unsigned long long>(cycles_over_10ms),
      static_cast<unsigned long long>(missed_sample_periods),
      percentile(output_age_us,0.50),percentile(output_age_us,0.99),
      percentile(output_age_us,0.999),output_age_us.empty()?0.0:output_age_us.back(),
      static_cast<unsigned long long>(output_age_over_5ms),
      static_cast<unsigned long long>(dispatch_age_over_5ms),
      static_cast<unsigned long long>(perception_updates),
      static_cast<unsigned long long>(greenwave.updates),
      static_cast<unsigned long long>(bench_nmpc.axis_solves.load(std::memory_order_relaxed)),
      static_cast<unsigned long long>(late_eso_deadlines),
      static_cast<unsigned long long>(eso_failures + invalid_outputs));

  p112::destroy_decoupled_nmpc(nmpc);
  const bool thresholds_met =
      eso_failures == 0 && invalid_outputs == 0 &&
      percentile(eso_us, 0.99) <= 1000.0 && late_eso_deadlines == 0 &&
      percentile(control_us, 0.99) <= 5000.0 &&
      percentile(control_us, 0.999) <= 8000.0 &&
      (control_us.empty() || control_us.back() <= 10000.0) &&
      cycles_over_10ms == 0 && missed_sample_periods == 0 &&
      control_cycles >= static_cast<std::uint64_t>(duration_s*99.0);
  return thresholds_met ? 0 : 1;
}
