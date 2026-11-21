# NMPC + ESO 主机验证（2026-11-21）

## 结论

当前框架的 acados dense-QP/HPIPM 求解路径和三轴调用回归通过。后续已按既有 CAD `GroundModel.f + RK4` 方法，通过框架外仿真工具完成框架 C++ ESO/NMPC 的 0.1 s 与 0.2 s 闭环复测；运动稳定性未通过，详见 `FRAMEWORK_GROUNDMODEL_RETEST_20261121.md`。本文件记录的主机求解器结果仍不代表目标板实时性。

## 执行环境与结果

- 日期：2026-11-21，Linux 主机，Generic BLASFEO。
- `ctest --test-dir build/control_stack_hpipm --output-on-failure`：2/2 通过。包含 acados dense-QP 解析解、边界限幅、非法约束拒绝和三轴 NMPC 求解调用。
- `ctest --test-dir build/deliverable_control_stack_test --output-on-failure`：4/4 通过。覆盖板级应用路径、三轴控制/模型测试和应用绑定测试；这些测试不构成机器人闭环验证。
- `ctest --test-dir build/final_three_nmpc --output-on-failure`：2/2 通过，控制和模型测试。
- 直接运行 `build/control_stack_hpipm/acados_qp_solver_test`：600 次单轴求解全部成功；P50 58.95 us、P95 61.93 us、P99 62.34 us、最大 146.04 us。该测量是主机单轴求解延迟，不是三轴并行周期或目标板实时性。
- 后续同一 CAD/RK4 框架闭环复测的三轴 NMPC 单步主机延迟与运动结果，见 `FRAMEWORK_GROUNDMODEL_RETEST_20261121.md` 及 `logs/nmpc_eso_framework_retest_20261121/report.json`。

## 阻止闭环验证的问题

NMPC 默认模型见 `source/control_stack/algorithms/eso_three_nmpc/decoupled_nmpc.cpp` 的 `DecoupledNmpcConfig::defaults()`，仿真参数副本见本目录 `decoupled_nmpc_defaults_simulation.json`。状态方程中机身角加速度为 `f[1]`，其两个输入增益来自 `input_gain[2]`、`input_gain[3]`；目前 roll、pitch、yaw 默认值在这两个位置都是零，非零值位于 `input_gain[6]`、`input_gain[7]`，只作用于第四个状态的加速度 `f[3]`。因此控制输入不能通过当前模型改变预测机身角加速度，ESO 给出的机身轴扰动也没有可用的机身轴力矩通道去抵消。需用已辨识/CAD 闭链模型确认状态定义、关节力矩变换和增益矩阵后，才可运行有意义的平衡、扰动阶跃、步态/转弯和长时仿真。

此外，框架文档约定实际部署时 Global ESO 在 RT-Thread 运行，经 AMP `0x12` 输入 `observed_eso`；本次主机 CTest 没有运行 RT-Thread，也没有从传感器/仿真观测产生并校验 ESO 扰动估计。因此本次结果只证明求解器和接口回归通过，不证明 ESO 估计质量或闭环抗扰能力。

## 下一步验收

1. 依据机器人运动学/动力学或辨识结果修正三轴 `input_gain`、向量场和四状态映射，并确认电机索引及正负号。
2. 在框架外的 `tools/` 仿真 runner 中调用本框架三轴 NMPC；使用真实机器人 plant、500 Hz 传感器/ESO 更新、100 Hz NMPC、2 ms 步长和每拍实际扭矩回填。
3. 先做零参考平衡和已知扰动阶跃，再测行进/转弯/腿长变化；记录姿态峰值/RMS、扰动估计误差、求解失败、饱和、扭矩和延迟分位数。
4. 闭环通过后再做长时和地形测试。当前不得将旧控制树的 Isaac 视频或历史烟雾测试归到本框架结果下。

部署验收状态：`not accepted`。无 OSQP 路径；本次求解测试使用 acados dense-QP/HPIPM。
