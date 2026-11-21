#include "acados_adapter.h"

#include <cmath>
#include <cstdint>
#include <new>
#include <vector>

#include "acados/dense_qp/dense_qp_common.h"
#include "acados/dense_qp/dense_qp_hpipm.h"

struct AdapterContext {
  int variables{};
  int rows{};
  qp_solver_config *config{};
  dense_qp_dims *dimensions{};
  dense_qp_in *qp{};
  dense_qp_out *solution{};
  void *options{};
  void *memory{};
  std::vector<std::uint8_t> config_memory;
  std::vector<std::uint8_t> dimensions_memory;
  std::vector<std::uint8_t> qp_memory;
  std::vector<std::uint8_t> solution_memory;
  std::vector<std::uint8_t> options_memory;
  std::vector<std::uint8_t> memory_memory;
  std::vector<int> box_indices;
  std::vector<double> lower;
  std::vector<double> upper;
};

extern "C" void *nmpc_acados_create(int rows) {
  if (rows <= 0) return nullptr;
  auto *context = new (std::nothrow) AdapterContext{};
  if (context == nullptr) return nullptr;
  try {
    context->variables = rows;
    context->rows = rows;
    context->config_memory.resize(dense_qp_solver_config_calculate_size());
    context->config = dense_qp_solver_config_assign(context->config_memory.data());
    dense_qp_hpipm_config_initialize_default(context->config);

    context->dimensions_memory.resize(dense_qp_dims_calculate_size());
    context->dimensions = dense_qp_dims_assign(context->dimensions_memory.data());
    const int nv = rows;
    const int ne = 0;
    const int nb = rows;
    const int ng = 0;
    const int ns = 0;
    context->config->dims_set(context->config, context->dimensions, "nv", &nv);
    context->config->dims_set(context->config, context->dimensions, "ne", &ne);
    context->config->dims_set(context->config, context->dimensions, "nb", &nb);
    context->config->dims_set(context->config, context->dimensions, "ng", &ng);
    context->config->dims_set(context->config, context->dimensions, "ns", &ns);

    context->qp_memory.resize(dense_qp_in_calculate_size(context->dimensions));
    context->qp = dense_qp_in_assign(context->dimensions, context->qp_memory.data());
    context->solution_memory.resize(dense_qp_out_calculate_size(context->dimensions));
    context->solution = dense_qp_out_assign(context->dimensions,
                                             context->solution_memory.data());
    context->options_memory.resize(context->config->opts_calculate_size(
        context->config, context->dimensions));
    context->options = context->config->opts_assign(
        context->config, context->dimensions, context->options_memory.data());
    context->config->opts_initialize_default(context->config, context->dimensions,
                                              context->options);
    char hpipm_mode[] = "SPEED";
    context->config->opts_set(context->config, context->options, "hpipm_mode",
                              hpipm_mode);
    context->memory_memory.resize(context->config->memory_calculate_size(
        context->config, context->dimensions, context->options));
    context->memory = context->config->memory_assign(
        context->config, context->dimensions, context->options,
        context->memory_memory.data());
    context->box_indices.resize(rows);
    context->lower.resize(rows);
    context->upper.resize(rows);
    for (int i = 0; i < rows; ++i) context->box_indices[i] = i;
    return context;
  } catch (...) {
    delete context;
    return nullptr;
  }
}

extern "C" void nmpc_acados_destroy(void *opaque) {
  auto *context = static_cast<AdapterContext *>(opaque);
  if (context == nullptr) return;
  if (context->config != nullptr && context->config->terminate != nullptr)
    context->config->terminate(context->config, context->memory, nullptr);
  delete context;
}

extern "C" int nmpc_acados_solve(void *opaque, int rows, const double *h,
                                 const double *g, const double *a,
                                 const double *l, const double *u,
                                 double *x) {
  auto *context = static_cast<AdapterContext *>(opaque);
  if (context == nullptr || rows != context->rows || h == nullptr ||
      g == nullptr || a == nullptr || l == nullptr || u == nullptr ||
      x == nullptr)
    return -1;

  const int n = context->variables;
  for (int r = 0; r < rows; ++r) {
    if (!std::isfinite(l[r]) || !std::isfinite(u[r]) || l[r] > u[r]) return -1;
    for (int c = 0; c < n; ++c) {
      const double expected = r == c ? 1.0 : 0.0;
      if (!std::isfinite(a[r * n + c]) || a[r * n + c] != expected) return -1;
    }
    context->lower[r] = l[r];
    context->upper[r] = u[r];
  }
  for (int r = 0; r < n; ++r) {
    if (!std::isfinite(g[r])) return -1;
    for (int c = 0; c < n; ++c) {
      const double hij = h[r * n + c];
      const double hji = h[c * n + r];
      if (!std::isfinite(hij) || std::abs(hij - hji) > 1e-10) return -1;
    }
  }

  // The condensed NMPC QP uses identity box constraints. Its Hessian is
  // symmetric, so the row-major caller buffer is also valid column-major data.
  d_dense_qp_set_H(const_cast<double *>(h), context->qp);
  d_dense_qp_set_g(const_cast<double *>(g), context->qp);
  d_dense_qp_set_idxb(context->box_indices.data(), context->qp);
  d_dense_qp_set_lb(context->lower.data(), context->qp);
  d_dense_qp_set_ub(context->upper.data(), context->qp);
  const int status = context->config->evaluate(
      context->config, context->qp, context->solution, context->options,
      context->memory, nullptr);
  if (status != ACADOS_SUCCESS) return status;
  d_dense_qp_sol_get_v(context->solution, x);
  for (int i = 0; i < n; ++i) {
    if (!std::isfinite(x[i]) || x[i] < l[i] - 1e-7 || x[i] > u[i] + 1e-7)
      return -2;
  }
  return 0;
}
