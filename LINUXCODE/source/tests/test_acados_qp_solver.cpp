#include "../algorithms/eso_three_nmpc/acados_adapter.h"
#include "../algorithms/eso_three_nmpc/decoupled_nmpc.h"

#include <array>
#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <vector>

int main() {
  p112::GlobalEso eso;
  p112::EsoMeasurement stationary{};
  stationary.angle = {0.1, -0.2, 0.3};
  p112::EsoState eso_state{};
  assert(eso.update(stationary, 0.002, eso_state));
  for (unsigned sample = 0; sample < 500; ++sample) {
    assert(eso.update(stationary, 0.002, eso_state));
  }
  for (double estimate : eso_state.disturbance) assert(std::abs(estimate) < 0.01);

  constexpr int n = 2;
  void *solver = nmpc_acados_create(n);
  assert(solver != nullptr);

  const std::array<double, n * n> h{2.0, 0.0, 0.0, 4.0};
  const std::array<double, n> g{-4.0, 8.0};
  std::array<double, n * n> a{1.0, 0.0, 0.0, 1.0};
  const std::array<double, n> lower{-5.0, -5.0};
  const std::array<double, n> upper{5.0, 5.0};
  std::array<double, n> x{};

  assert(nmpc_acados_solve(solver, n, h.data(), g.data(), a.data(),
                           lower.data(), upper.data(), x.data()) == 0);
  assert(std::abs(x[0] - 2.0) < 1e-7);
  assert(std::abs(x[1] + 2.0) < 1e-7);

  const std::array<double, n * n> coupled_h{4.0, 1.0, 1.0, 2.0};
  const std::array<double, n> coupled_g{-2.0, 3.0};
  assert(nmpc_acados_solve(solver, n, coupled_h.data(), coupled_g.data(),
                           a.data(), lower.data(), upper.data(), x.data()) == 0);
  assert(std::abs(x[0] - 1.0) < 1e-7);
  assert(std::abs(x[1] + 2.0) < 1e-7);

  const std::array<double, n> clipped_lower{-5.0, -0.5};
  assert(nmpc_acados_solve(solver, n, h.data(), g.data(), a.data(),
                           clipped_lower.data(), upper.data(), x.data()) == 0);
  assert(std::abs(x[0] - 2.0) < 1e-7);
  assert(x[1] >= -0.5 - 1e-7 && x[1] <= -0.5 + 1e-7);

  a[1] = 1.0;
  assert(nmpc_acados_solve(solver, n, h.data(), g.data(), a.data(),
                           lower.data(), upper.data(), x.data()) == -1);

  nmpc_acados_destroy(solver);

  p112::DecoupledNmpc controller{};
  assert(p112::initialize_decoupled_nmpc(controller));
  const p112::AxisState4 state{{0.1, -0.05, 0.02, 0.0}};
  const p112::AxisReference4 reference{};
  for (unsigned axis = 0; axis < 3; ++axis) {
    p112::AxisTorque2 torque{};
    const int result = p112::decoupled_nmpc_callback(
        &controller, static_cast<p112::DecoupledAxis>(axis), state,
        reference, 0.0, 0.002, torque);
    assert(result == 0);
    assert(std::isfinite(torque.value[0]));
    assert(std::isfinite(torque.value[1]));
  }
  std::vector<double> solve_us;
  solve_us.reserve(600);
  for (unsigned repeat = 0; repeat < 200; ++repeat) {
    for (unsigned axis = 0; axis < 3; ++axis) {
      p112::AxisTorque2 torque{};
      const auto begin = std::chrono::steady_clock::now();
      const int result = p112::decoupled_nmpc_callback(
          &controller, static_cast<p112::DecoupledAxis>(axis), state,
          reference, 0.0, 0.002, torque);
      const auto end = std::chrono::steady_clock::now();
      assert(result == 0);
      solve_us.push_back(
          std::chrono::duration<double, std::micro>(end - begin).count());
    }
  }
  std::sort(solve_us.begin(), solve_us.end());
  const auto percentile = [&solve_us](double p) {
    const auto index = static_cast<std::size_t>(p * (solve_us.size() - 1));
    return solve_us[index];
  };
  std::fprintf(stderr,
               "acados dense-QP host solve latency (us): p50=%.2f p95=%.2f "
               "p99=%.2f max=%.2f, calls=%zu\n",
               percentile(0.50), percentile(0.95), percentile(0.99),
               solve_us.back(), solve_us.size());
  p112::destroy_decoupled_nmpc(controller);
  return 0;
}
