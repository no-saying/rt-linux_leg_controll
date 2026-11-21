# Optimizer Dependencies

The control stack uses the official acados dense-QP HPIPM adapter for the
condensed three-axis NMPC QPs. HPIPM performs the dense QP solve and BLASFEO
provides its linear algebra backend. OSQP is not used.

Pinned upstream revisions:

- acados `77aebe195a686c736bf769fd06ea66e4694bf86a` (2026-11-20). This bundle
  contains only the dense-QP adapter and the utility files it directly needs.
- HPIPM `0a5afbdb88394075d2474c72e25a795b5ba009a5` (2026-11-20).
- BLASFEO `e5d1cbbb2f458acd7adb7d9e8a6522416b7e336f` (2026-11-20).

Build the framework with CMake. The default `GENERIC` BLASFEO target supports
portable host builds. Set `-DCONTROL_STACK_BLASFEO_TARGET=ARMV8A_ARM_CORTEX_A53`
or the exact BLASFEO target matching the board when cross-compiling. The board
target and toolchain must be validated together before using timing results for
control-loop acceptance.

Each axis context, QP input/output, options, and HPIPM workspace is created
during initialization. The current adapter accepts only the identity box
constraints produced by the NMPC QP builder and rejects other constraint
matrices. Third-party license texts are included beside each dependency.
