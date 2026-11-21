#ifndef NMPC_ACADOS_ADAPTER_H
#define NMPC_ACADOS_ADAPTER_H
#ifdef __cplusplus
extern "C" {
#endif
/* Re-entrant acados dense-QP/HPIPM context. Create it before the control loop;
 * each worker owns an independent context and preallocated workspace. */
void *nmpc_acados_create(int rows);
void nmpc_acados_destroy(void *ctx);
int nmpc_acados_solve(void *ctx, int rows, const double *h, const double *g,
                      const double *a, const double *l, const double *u,
                      double *x);
#ifdef __cplusplus
}
#endif
#endif
