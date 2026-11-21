# 固定 MPC HPIPM 适配调试记录

> 历史记录：以下板端对比对应 2026-11-18 的旧适配实现，不能代表当前框架状态。
> 2026-11-21 的框架已集成 acados 官方 dense-QP/HPIPM adapter、HPIPM 与 BLASFEO，
> 并通过主机解析 QP 与三轴调用测试；见同目录
> `ACADOS_DENSE_QP_INTEGRATION_20261121.md`。旧板端 `MIN_STEP` 与 2.3 Nm 结果仍是
> 风险记录，尚未由本次主机测试消除，也尚未完成新 adapter 的板端复测。
> 本文件后续内容保留当时的 OSQP 对照和“默认后端仍保持 OSQP”等历史结论；
> 当前交付已使用 acados dense-QP/HPIPM，未包含或调用 OSQP。

## 结论

固定 MPC 使用 acados 提供的 HPIPM dense-QP adapter，不直接在控制器里拼接
`d_dense_qp` 的 raw API。adapter 已统一处理 HPIPM 的矩阵布局、上界符号、无限边界
mask、内部工作区和 solver 参数。

当前 raw HPIPM 实验后端保留为诊断代码，不作为默认控制后端。

## 语义对照

OSQP 和 HPIPM 理论上都求解：

```text
min  1/2 x' H x + g' x
s.t. lower <= A x <= upper
```

对应关系为 `H=P`、`g=q`、`A=C`、`lower=lg`、`upper=ug`。HPIPM 内部把上界
转换为 `-C x >= -upper`，不能在调用方提前取负。

## 已完成实验

- 原始 Packed-LDL OSQP：固定 MPC 三个 profile 的 Python 对照最大误差约
  `4e-7 Nm`。
- raw HPIPM dense API：板端部分用例达到 `MAX_ITER`，成功用例和 OSQP 最大偏差
 约 `2.3 Nm`。HPIPM 自身残差很小，但不能证明加载的问题与 OSQP 相同。
- 调整 `ROBUST/BALANCE`、`warm_start`、`reg_prim/reg_dual`、容差和迭代上限，
  没有消除偏差，因此问题不是单纯收敛参数。

## acados adapter

adapter 源码：

```text
nmpc/cpp_experimental/acados_qp_static.c
nmpc/benchmark_history/20261117-acados/acados_qp.c
```

板端已有共享库：

```text
/home/kickpi/benchmarks/nmpc-20261117/experimental/acados_lib/libacados_qp.so
```

导出的接口为 `aq_create(n,m)`、`aq_solve(...)`、`aq_destroy(...)`。下一步固定 MPC
后端通过这个接口传递完整 `H/g/A/l/u`，再用同一批 1800 条用例复核力矩误差、失败状态
和板端耗时。

当前 C++ 适配已经切换到 `aq_create/aq_solve/aq_destroy`。首次板端复测返回 HPIPM
状态 `2 (MIN_STEP)`，说明还需要校准 adapter 的构建版本和依赖；这次切换已完成编译，
但尚未通过数值验收，默认后端仍保持 OSQP。

## 验收条件

只有满足以下条件才切换默认后端：

1. 1800 条用例最大力矩误差不超过 `2e-5 Nm`；
2. HPIPM 返回 `SUCCESS`，`MAX_ITER` 视为失败；
3. 非法输入和不可行边界行为与 OSQP 一致；
4. 板端报告平均、P50、P95、P99、最大耗时和超时次数。

