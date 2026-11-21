# 框架 NMPC + ESO 按既有 CAD/RK4 测法复测（2026-11-21）

## 测法与调用链

沿用 `NONLINEAR_TUNING_20261118.md` 的 CAD `GroundModel.f + RK4` 测法，传感器和 ESO 为 500 Hz、三轴 NMPC 为 100 Hz、每个 NMPC 周期 10 ms。仿真工具在顶层 `tools/control_stack_framework_sim/`，通过 C ABI 桥直接调用框架 C++ `GlobalEso`、`DecoupledController`、`decoupled_nmpc_callback` 和 acados dense-QP/HPIPM；没有调用历史 Python NMPC 控制器。每个 2 ms 步进均把上一实际/模拟执行扭矩回填给框架。

使用 2026-11-18 CAD 初始关节位置和静态配平。初始姿态施加 pitch 0.02 rad、yaw 0.01 rad 偏差。0.1 s 案例不加外部扰动；0.2 s 案例在 0.04–0.08 s 叠加 pitch 1.0、yaw 0.5 rad/s² 的诊断加速度扰动。后者是 reduced plant 的直接加速度注入，用于检查 ESO/控制链响应，不等价于轮地冲击或真实外力。

## 结果

| 案例 | QP 调用 | 峰值 pitch 偏差 | 峰值 yaw 偏差 | 峰值 yaw 角速度 | ESO 扰动估计峰值 pitch/yaw | 结论 |
|---|---:|---:|---:|---:|---:|---|
| 0.1 s，零外扰 smoke | 10 | 1.646° | 3.644° | 1.722 rad/s | 14.50 / 18.78 rad/s² | 求解完成，闭环未通过 |
| 0.2 s，扰动阶跃 | 20 | 1.966° | 16.766° | 3.363 rad/s | 24.38 / 20.00 rad/s² | 求解完成，yaw ESO 达限幅，闭环未通过 |

两次运行都没有 ESO 更新错误或 QP 求解失败；该结果说明 acados 后端能持续返回解，但闭环运动表现不合格。即使零外扰，yaw ESO 估计也接近配置上限；加入扰动后 yaw 估计达到 20 rad/s² 上限，机身 yaw 偏差继续扩大。不得把 `execution_status=completed` 解释为控制稳定通过。

测试还暴露了 ESO 离散实现差异：框架 `GlobalEso` 原先对 2 ms 样本只做一次显式 Euler 更新，而历史 Python ESO 每个采样周期内部以最多 0.2 ms 子步积分。本轮已把框架 observer 改成同样的子步积分，并在 `test_acados_qp_solver` 增加了静止非零姿态运行 1 秒的无假扰动回归。该修正消除了离散实现与历史测法的差异，但动态 CAD 闭环仍未通过，说明还存在输入增益/状态模型或观测输入模型不匹配。

## 结果文件与复现

- 原始数据：`logs/nmpc_eso_framework_retest_20261121/100ms/telemetry.csv`、`200ms/telemetry.csv`。
- 指标：`logs/nmpc_eso_framework_retest_20261121/report.json`。
- 复现命令和工具边界：`tools/control_stack_framework_sim/README.md`。
- 求解回归：`ctest --test-dir build/framework_ground_sim/control_stack --output-on-failure`，2/2 通过；新增静态 ESO 长更新检查包含在 `acados_qp_solver_test` 中。

## 限制与后续

`GroundModel` 是 CAD 闭链、理想双轮接触约束下的简化动力学，没有 roll 自由度、PhysX 接触、弹丸、坡面或真实传感器噪声/总线延迟。本次只复现旧 0.1 s smoke 并扩展 0.2 s 扰动案例，不代表 Isaac 实物级视频或整车长时验证。

下一步先核对三轴四状态定义、关节力矩到轴加速度的输入增益，以及 RT-Thread 计算 ESO `nominal_acceleration` 的来源；随后重跑同样基线并做扰动估计误差与力矩方向审计。当前部署验收状态：`not accepted`。
