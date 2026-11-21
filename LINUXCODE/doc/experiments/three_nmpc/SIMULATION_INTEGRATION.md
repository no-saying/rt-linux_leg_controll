# ESO + 三块 NMPC 仿真接入说明

当前仿真入口是 `GlobalEso`、`DecoupledController` 和 `DecoupledNmpc`。仿真器拥有传感器模型、机器人状态、执行器模型和参数文件；算法包只处理一拍状态快照，不打开设备、不访问 CAN、不依赖 RT-Thread。

## 一拍时序

仿真时间步长固定为 `dt_sensor = 0.002 s`，每拍产生一个递增的 `sequence`：

```text
sequence k
  │
  ├─ 仿真器推进/采样：IMU、腿长、摆角、轮速
  ├─ 组装 DecoupledInput{sequence=k, dt=0.002}
  ├─ RT-Thread GlobalEso.update() / simulator equivalent  500 Hz
  ├─ 将 ESO 结果与本拍传感器字段写入同一 AMP 0x12 样本
  │
  ├─ k % 5 != 4：run_nmpc=false
  │    └─ 不唤醒 NMPC，发布上一拍已确认力矩
  │
  └─ k % 5 == 4：run_nmpc=true, nmpc_dt=0.010
       ├─ 发布同一 sequence 给 Roll/Pitch/Yaw 三个 worker
       ├─ 三个 4 状态/2 输入 QP 并行求解
       ├─ 等待 completed == 3
       ├─ ESO 扰动进入 NMPC 预测，输出执行限幅和速率限制
       └─ 仿真器接收该 sequence 的六路力矩
```

`dt` 只表示传感器/ESO 采样周期；`nmpc_dt` 表示 NMPC 离散周期。不能把 2 ms 直接当成 NMPC 步长。每次成功执行后的实际力矩必须通过 `acknowledge()` 回填，下一拍的速率限制以这个实际值为准。

## 初始化和调用骨架

```cpp
p112::DecoupledNmpc nmpc;
auto nmpc_cfg = p112::DecoupledNmpcConfig::defaults();
// 从 decoupled_nmpc_defaults.json 读取并覆盖 nmpc_cfg.axis[]。
if (!p112::initialize_decoupled_nmpc(nmpc, nmpc_cfg)) abort();

auto cfg = p112::DecoupledConfig::defaults();
p112::DecoupledController control(&nmpc, p112::decoupled_nmpc_callback, cfg);
if (!control.start_parallel({-1,-1,-1})) abort();

for (uint64_t k = 0; simulation_running; ++k) {
    p112::DecoupledInput in{};
    in.sequence = k;
    in.dt = 0.002;
    in.nmpc_dt = 0.010;
    in.run_nmpc = (k % 5) == 4;
    in.observed_eso = amp_snapshot.eso_state();
    in.axis_state = state_adapter.axis_state();
    in.reference = reference_generator.axis_reference();

    p112::DecoupledOutput out{};
    if (!control.step_parallel(in, out) || out.sequence != k) fail_safe();
    plant.apply_torque(out.torque, in.dt);
    if (in.run_nmpc) control.acknowledge(plant.applied_torque());
}
```

销毁顺序必须是先停止仿真循环和 worker，再调用 `destroy_decoupled_nmpc()`。QP context 只在初始化期创建，实时/仿真周期内不得重新创建。

## 仿真必须覆盖的情况

- 观测噪声、零偏、丢样本、时间戳倒退和 `sequence` 不连续；
- Roll→Pitch/Yaw、Pitch→Yaw、Yaw→Pitch 的交叉扰动，以及扰动估计饱和；
- 输入限幅、速率限幅、上一拍实际力矩与请求力矩不一致；
- NMPC worker 超时/返回非零状态时的保持策略；
- 5 拍 ESO 与 1 拍 NMPC 的相位关系，不能每个 2 ms 样本都重新求解 NMPC；
- 传感器延迟、执行器延迟、模型参数失配和轮胎打滑；
- 初始配平、恢复、起步、停止、急转和跳跃/落地边界。

每个 CSV/回放样本至少保存 `sequence`、仿真时间、ESO 状态/扰动、三个 block 状态、NMPC 原始输出、实际执行力矩、QP status 和限幅标志。只看最终姿态误差无法定位时序或饱和问题。

## 当前包边界

默认路径包含 AMP 提供的 ESO 状态 + 三块 NMPC；Global ESO 由 RT-Thread 更新。RL actor、传感器驱动和 CAN 接口由各自集成层管理，不属于 NMPC 求解周期。
