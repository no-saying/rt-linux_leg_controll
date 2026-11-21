# 当前控制算法基线

当前仿真、调参和后续控制验收统一使用：

```text
Global ESO + nonlinear NMPC
```

## 数据流

```text
传感器/编码器/IMU → RT-Thread Global ESO
        ↓ 同步打包到 AMP 0x12
控制框架接收传感器快照 + 三轴 ESO 扰动状态
        ↓
三块 decoupled nonlinear NMPC（roll / pitch / yaw-wheel）
        ↓
力矩速率限制和物理限幅
        ↓
六个执行器
```

ESO 运行位置是 RT-Thread。板端运行保留的 `GlobalEso`，把 roll/pitch/yaw 三轴集总
扰动估计与同一采样时刻的传感器数据放入 AMP `0x12`；bit3 标记估计有效。三个 NMPC
消费该观测值并将对应扰动作为加速度项放入预测滚动；控制框架不运行第二个 ESO，
求解后也不额外叠加 ESO 力矩。

`arm_mpc_rl_p112_nmpc_20261118/cpp_runtime/decoupled_control.*` 保留 `GlobalEso` 实现
供 RT-Thread 移植；控制器从 AMP 映射到 `DecoupledInput::observed_eso`，不会二次观测。
2026-11-21 已把扰动接入 NMPC 预测滚动并通过模型测试；仍需目标板 solver adapter
和真实运行周期验证。

当前三个 NMPC block 是同一个 NMPC 控制器的三个固定上下文，不是三个恢复阶段。
恢复阶段由应用层状态机管理；进入恢复后，恢复动作完成才交给 ESO+NMPC 平衡接管。

## 当前仿真默认参数

```text
ESO scale:              0.5
ESO bandwidth:          20, 20, 15
ESO disturbance limit:  15, 15, 10
NMPC wheel soft limit:  80% of physical wheel limit
physical wheel limit:   ±3 Nm
```

仿真命令必须在输出目录、summary 和 telemetry 中记录上述参数。任何使用 PID、
普通 MPC、单独 ESO 补偿或 RL 的结果都必须标注为对照，不得混入当前主线基线。
