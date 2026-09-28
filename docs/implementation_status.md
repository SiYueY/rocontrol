# RoControl V1 实施状态

更新日期：2026-09-28。本文件记录当前提交的实现和验收状态；设计契约以
[rocontrol_design.md](rocontrol_design.md) 为准。

## 已完成

| 阶段 | 已实现并验证的内容 |
| --- | --- |
| Phase 1 | `Result`、基础类型、ControlCycle、URDF/Pinocchio RobotModel、canonical DoF 顺序、mimic/continuous joint、effective limits、JointGroup、Frame、FK/Jacobian；模型更新的稳态无分配测试通过。 |
| Phase 2 | JointPositionTask、默认启用的位置与速度硬约束、预分配 dense velocity QP、Controller、JointVelocityCommand；关节闭环、实际周期限幅、冲突任务和不可行约束测试通过。 |
| Phase 3 | CartesianPoseTask、SO(3) 姿态误差、位置/姿态增益与权重、速度前馈；FR3 URDF 运动学闭环及 RoMuJoCo Joint/Cartesian 闭环示例通过。 |
| Phase 4 | Joint/Cartesian quintic trajectory、SLERP、轻量 FSM、周期边界 transition、外部请求优先级与失败传播；FR3 Duo 双臂及 Mobile FR3 Duo 双臂加 spine 闭环测试通过。 |
| Phase 5 已完成部分 | Runtime 时间契约示例、周期抖动与新鲜度测试、稳态分配审计、300,000 周期计算和 10 分钟墙钟基准、性能诊断报告。 |

库使用 C++17。Core 仅依赖 Eigen3 和 Pinocchio；RoMuJoCo 只在可选示例中链接。
Public API 的单位、顺序、坐标系、生命周期及错误语义已在头文件注释中说明。

## 验证结果

| 验证 | 结果 |
| --- | --- |
| 全新 Release 构建，含本机 FR3、FR3 Duo、Mobile FR3 Duo URDF | CTest 11/11 通过 |
| Debug 构建，同样的真实描述文件 | CTest 11/11 通过 |
| 可选 RoMuJoCo 构建 | CTest 10/10 通过，含 Cartesian 闭环 |
| 安装后的独立 CMake consumer | 编译并运行通过 |
| 15 DoF、3 Task、2 硬约束、300,000 周期计算基准 | Core p99 0.008639 ms、max 0.158353 ms、C++ `new` 分配 0 |
| CPU 17 忙等调度的 500 Hz、10 分钟墙钟基准 | 300,000 周期通过；Core p99 0.014120 ms、max 0.056046 ms、C++ `new` 分配 0 |

完整环境、分阶段延迟和命令见 [benchmark_report.md](benchmark_report.md)。真实描述文件
来自本机外部项目；默认构建不要求这些文件，需按 [README](../README.md) 配置可选测试。

## 尚未完成或未验证

1. **真实机器人命令语义与适配器验收：未完成。** 当前没有已确认可连接的
   FR3/FR3 Duo 实体机器人及其控制接口。尚未在硬件上验证外部 joint 顺序到
   canonical DoF 顺序的映射、原生速度或位置命令转换、位置模式积分/插值/限速、
   命令超时和故障停机。Core 按设计不内置硬件驱动；此项必须在具体 Runtime/
   Adapter 接入后执行。
2. **普通休眠调度的 500 Hz 运行：本机未通过。** `sleep_until` 模式在第 58
   周期测得 5.22807 ms，超过默认 4 ms 上限；Controller 正确拒绝该周期命令。
   忙等调度已通过 10 分钟基准，但本机不能设置 `SCHED_FIFO`，不能据此保证
   其他主机或实际 I/O 负载下的硬实时行为。
3. **真实 FR3 的 MuJoCo 动力学闭环：未验证。** FR3 的 Joint/Cartesian
   测试使用真实 URDF 与运动学状态积分；RoMuJoCo Joint/Cartesian 示例使用与其
   URDF 匹配的轻量 MJCF 测试模型。真实 FR3 MuJoCo 模型与设备适配器接入后
   仍应另做整机集成测试。

因此，此提交是 **V1 Core 与本地仿真/性能验证检查点**，尚不能声明设计文档中
“真实机器人命令语义已验证”的最终验收条件成立。

## V1 范围之外

HQP/严格优先级、动力学和接触控制、G1、插件系统、GUI、通用 mailbox 及
Safety Manager 均未纳入 V1 Core；这些不是本次待完成项。
