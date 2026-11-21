# acados Dense-QP 接入记录

## 接入内容

当前 `source/control_stack` 的三轴 NMPC 将构造出的 36 变量凝聚 QP 交给 acados
官方 dense-QP/HPIPM adapter。HPIPM 使用 SPEED 模式，BLASFEO 提供矩阵运算。没有
OSQP 路径。Roll、pitch、yaw 各自持有独立的 solver 配置、QP 输入输出与工作区；这些
内存均在 `initialize_decoupled_nmpc()` 前创建，控制周期只更新 QP 并调用求解器。

约束适配当前限定为 NMPC 生成的 36 行单位矩阵盒约束，输入上下界直接映射到 HPIPM
变量边界。非单位约束矩阵、非对称 Hessian、非法边界会在进入求解器前拒绝。

第三方源码固定版本及许可证记录在
`source/control_stack/third_party/optimizer/README.md`。

## 主机验证

- CMake Release 构建通过，静态库和安装导出通过。
- 解析 QP `min 1/2 x'Hx + g'x` 返回解析解 `[2, -2]`，边界夹紧用例返回 `[2, -0.5]`。
- 非单位约束矩阵被拒绝。
- 三个轴均完成 `initialize_decoupled_nmpc()` 后的实际 acados dense-QP 调用。
- 额外连续执行 600 次单轴控制回调，状态全部成功。最近一轮 Generic BLASFEO
  主机数据：P50 约 61.70 us、P95 约 61.92 us、P99 约 64.73 us、最大约 89.86 us。
  不同轮次的尾延迟受主机负载影响，不能替代目标板测量。

## 未完成验收

上述延迟是当前主机和 Generic BLASFEO 配置的结果，不是目标板实时性能结论。目标板需
设置与 SoC 匹配的 BLASFEO 架构并交叉编译，然后复测单轴及三轴并行耗时、求解状态、
最大 torque 偏差和长时失败次数。2026-11-18 旧 HPIPM 适配的 `MIN_STEP` 和约 2.3 Nm
偏差仍保留为历史风险，不能由本次解析 QP 测试代替板端 NMPC 数值验收。
