# RoControl V1 设计文档

> **项目名称：** RoControl  
> **仓库名称：** `rocontrol`  
> **C++ Namespace：** `rocontrol`  
> **基线语言：** C++17  
> **基线平台：** Ubuntu 22.04  
> **默认控制频率：** 500 Hz  
> **默认控制周期：** 2 ms  
> **控制周期：** 可配置  
> **V1 控制类型：** Fixed-base Kinematic Velocity QP  
> **V1 输出类型：** `JointVelocityCommand`  
> **文档状态：** V1 Architecture Baseline

---

# 1. 项目定位、系统边界与设计依据

RoControl 是一个**轻量、面向任务的机器人控制框架**。它位于 Application / Agent 与真实机器人或仿真 Runtime 之间，负责把上层的运动意图转换成结构化的关节速度命令。

V1 重点支持：

- Franka3；
- Franka3 Duo；
- Mobile FR3 Duo；
- 固定基座机器人；
- 单臂、双臂和 spine / lift 等附加关节；
- Joint Space 位置目标；
- Cartesian Space 末端 Pose 目标；
- 多 Task 加权协同；
- 关节位置与速度硬约束；
- FSM 行为阶段组织；
- 简单轨迹 reference；
- 默认 500 Hz、可配置周期的控制 Runtime。

V1 不追求完整覆盖未来所有 Whole-Body Control 场景。设计的主要目标是先把当前机械臂和复合机器人需要的控制主线做完整，同时确保未来加入 HQP、floating base、contact、inverse dynamics、Unitree G1 时不需要推翻核心架构。

## 1.1 项目名称与工程关系

项目正式命名为：

```text
Project:      RoControl
Repository:   rocontrol
Namespace:    rocontrol
CMake target: rocontrol::rocontrol
```

选择 `rocontrol` 的主要原因是与现有 `romujoco` 保持一致：

```text
romujoco   → robot simulation
rocontrol  → robot control
```

RoControl 与其他项目的关系：

```text
                   Application / Agent
                           │
                           ▼
                       RoControl
                           │
              ┌────────────┴────────────┐
              ▼                         ▼
         robohardware                romujoco
              │                         │
              ▼                         ▼
          Real Robot                 Simulation
```

RoControl Core 不依赖：

```text
robohardware
romujoco
ROS 2
Agent runtime
GUI
```

这些系统只通过稳定的数据契约与 RoControl 对接。

## 1.2 Core 的职责

RoControl Core 只有六个长期稳定的核心概念：

```text
RobotModel
Task
Constraint
Solver
Controller
FSM
```

它们分别回答：

```text
RobotModel   机器人模型是什么，当前运动学状态是什么
Task         希望机器人做什么
Constraint   求解结果不能违反什么
Solver       当前控制问题如何构造和求解
Controller   一个控制周期如何组织
FSM          当前应该处于哪个行为阶段
```

此外还需要少量基础数据与辅助类型：

```text
Pose
RobotState
ControlCycle
JointVelocityCommand
JointIndex
JointGroup
FrameIndex
Trajectory helper
Result<T,E>
```

这些是数据契约，不应演变为额外的复杂子系统。

## 1.3 明确不属于 Core 的能力

以下能力不属于 RoControl Core：

```text
CAN / EtherCAT / Serial
hardware driver
motor servo
MuJoCo runtime
ROS 2 middleware
navigation
motion planning
perception
SLAM
GUI
Agent runtime
network transport
state estimator
E-stop
hardware watchdog
driver fault recovery
realtime scheduler
```

对应关系：

```text
Hardware / MuJoCo
       │
       ▼
   RobotState
       │
       ▼
    RoControl
       │
       ▼
JointVelocityCommand
       │
       ▼
Robot Adapter / Runtime
       │
       ▼
Hardware / MuJoCo
```

如果某个机器人原生接收关节位置而不是关节速度，应由 Adapter 完成积分、插值、限速和 native command 转换，而不是让 RoControl 的 Velocity QP 输出语义变得混乱。

## 1.4 核心设计原则

V1 冻结以下原则：

1. **核心尽量小。**
2. **Task 表达控制目标。**
3. **Constraint 表达硬限制。**
4. **Solver 同时负责 QP 构建和数值求解。**
5. **Controller 只负责控制生命周期、校验和 Solver 调用，不直接操作 H/g。**
6. **FSM 管理行为状态，不直接执行 QP。**
7. **Runtime 管理时序、I/O、状态新鲜度和系统级安全。**
8. **默认 500 Hz，但控制周期可配置。**
9. **Task gain 使用连续时间语义，单位为 `s^-1`。**
10. **V1 Solver 决策变量是 `qdot`。**
11. **V1 输出是 `JointVelocityCommand`。**
12. **Task weight 不是 priority。**
13. **V1 不伪造 HQP。**
14. **Core 不依赖 ROS 2、robohardware 或 romujoco。**
15. **控制开始后 Task / Constraint 拓扑固定。**
16. **名称查找和模型解析发生在初始化阶段。**
17. **500 Hz 路径不进行常规 heap allocation。**
18. **失败时不返回半有效 Command。**
19. **坐标系、关节顺序、时间、线程模型都作为正式契约。**
20. **真实机器人所需的基础 Joint position / velocity limits 默认启用。**
21. **跨线程输入不能直接修改 Task 或 FSM。**
22. **Public API 使用 C++17 能力，不引入 C++20 类型。**
23. **不为了未来需求提前建立 backend/plugin 体系。**

## 1.5 设计来源

RoControl 不复制任何单一项目，而是按职责参考成熟框架。

| 参考项目 | 主要参考内容 | RoControl 取舍 |
|---|---|---|
| mc_rtc | Controller Runtime、FSM 生命周期、Task/Constraint 组织 | 保留职责，不复制 GUI、DataStore、复杂插件体系 |
| ARC-OPT/wbc | RobotModel / Task / Constraint / Solver 分层 | 保留边界，不复制 Scene/Plugin/多 backend 复杂度 |
| OpenSoT | Task / Constraint / Priority / HQP 数学语义 | 作为未来 HQP 参考，V1 不提前承诺 |
| PlaCo | 易用 Task API、Kinematics/Dynamics 分离 | 学习 API 简洁性，不复制 solver/state 耦合 |
| libfranka | Joint / Cartesian 命名、实际周期概念 | 用于命名和时间模型 |
| romujoco | 仿真边界 | 仿真保持在 Core 外 |
| robohardware | 硬件与 Runtime 边界 | 驱动、实时线程保持在 Core 外 |

总体原则：

> 借鉴成熟项目已经验证的职责划分和数学语义，但不复制为解决多年历史问题积累出的全部复杂度。

---

# 2. 运行模型、时间契约与单周期语义

RoControl 的执行结构为：

```text
Application / Agent
        │
        ▼
    Command Mailbox
        │
        ▼
       FSM
        │
        ▼
   Controller
        │
        ▼
Task / Constraint
        │
        ▼
     Solver
        │
        ▼
      qdot
        │
        ▼
JointVelocityCommand
```

外部 Runtime：

```text
Robot / MuJoCo
      │
      ▼
 RobotState
      │
      ▼
   Runtime
      │
      ├── validate time/state
      ├── RobotModel::update()
      ├── consume external commands
      ├── FSM::update()
      ├── Controller::update()
      └── write command
```

## 2.1 500 Hz 默认，可配置周期

默认配置：

```text
frequency = 500 Hz
period    = 2 ms
```

但 500 Hz 不能写死在 Task、Constraint 或 Solver 中。

配置：

```cpp
struct ControllerConfig
{
    std::chrono::duration<double> nominal_period{0.002};
    std::chrono::duration<double> max_period{0.004};
    std::chrono::duration<double> max_state_age{0.004};

    bool enforce_position_limits{true};
    bool enforce_velocity_limits{true};

    double joint_limit_tolerance{1e-6};
    double qp_regularization{1e-6};
};
```

配置文件可以允许：

```yaml
frequency: 500
```

但内部只保留 canonical：

```text
nominal_period
```

支持：

```text
100 Hz
250 Hz
500 Hz
1 kHz
```

而不修改控制公式。

## 2.2 Nominal Period 与 Actual Period

Nominal period 是期望周期。

Actual period 是 Runtime 实际测得的相邻周期时间差。

定义：

```cpp
struct ControlCycle
{
    std::chrono::steady_clock::time_point time;
    std::chrono::duration<double> period;
    std::uint64_t sequence{0};
};
```

语义：

```text
time
    当前周期时间戳，使用 steady/monotonic clock

period
    当前周期相对前一正常控制周期的实际时间差

sequence
    单调递增周期号
```

正常 500 Hz：

```text
nominal = 2.000 ms

actual:
1.998 ms
2.003 ms
2.001 ms
...
```

出现 missed cycle 时：

```text
4 ms
```

不能假设实际周期永远等于 nominal period。

## 2.3 Actual Period 合法范围

V1 要求：

```text
finite(period)
period > 0
period <= config.max_period
```

默认：

```text
nominal_period = 2 ms
max_period     = 4 ms
```

即默认允许一次轻微 missed cycle，但不允许一次 10 ms、20 ms 的调度暂停后继续用同样控制律产生大步长。

超过 `max_period`：

```text
ControlError::InvalidCyclePeriod
```

并且当前周期不输出有效 Command。

`max_period` 必须可配置，因为不同 Runtime 的实时能力不同。

## 2.4 Startup / Synchronization Cycle

第一周期没有可靠的 actual period，因此不能伪造：

```text
period = nominal_period
```

V1 明确定义一个启动同步阶段：

```text
1. Runtime read initial RobotState

2. validate state

3. RobotModel::update(initial_state)

4. Controller::start(initial_state)

5. StateMachine::start(initial StateContext)

6. record initial cycle time / state stamp

7. 不输出 RoControl command
```

下一次 Runtime 读取到新的 RobotState 后：

```text
actual period =
current_cycle_time - previous_cycle_time
```

才进入正常控制循环。

这样第一周期不会使用虚构的 `dt`。

## 2.5 RobotState 与 ControlCycle 的时间关系

`RobotState.stamp` 和 `ControlCycle.time` 必须使用相同的：

```text
steady / monotonic clock domain
```

Runtime 检查：

\[
0 \le cycle.time - state.stamp \le max\_state\_age
\]

禁止：

```text
future state
stale state
duplicate state timestamp
non-increasing state timestamp
```

默认 V1 语义：

> 每个正常 control cycle 使用一份新的 RobotState。

如果底层状态采样频率低于 RoControl 控制频率，由 Adapter 明确负责重采样、插值或不同步策略；不能在 Runtime 中悄悄重复旧状态而不标记。

## 2.6 为什么 Solver 求 qdot

V1 正式采用：

```text
decision variable = qdot
```

而不是：

```text
delta_q
```

原因是 `delta_q` 会把 Task gain 与调用频率耦合。

Joint Task：

\[
S\dot q = K_q(q_d-q)
\]

Cartesian Task：

\[
J\dot q = v_d + K_xe
\]

Gain 单位：

```text
s^-1
```

因此从 250 Hz 改为 500 Hz，不会改变 gain 的连续时间反馈意义。

## 2.7 V1 输出 JointVelocityCommand

V1 不再输出：

```text
JointPositionCommand
```

而是：

```cpp
struct JointVelocityCommand
{
    std::uint64_t sequence{0};
    std::chrono::steady_clock::time_point stamp;
    Eigen::VectorXd velocity;
};
```

其中：

```text
velocity = qdot solution
sequence = ControlCycle.sequence
stamp    = ControlCycle.time
```

这样 Solver 求什么，RoControl 就输出什么：

```text
Kinematic Velocity QP
        ↓
      qdot
        ↓
JointVelocityCommand
```

避免：

```text
q_command = q_measured + qdot * dt
```

造成相邻 position command 因 measured-state lag 而跳变。

如果机器人底层只能接受 Position Command：

```text
JointVelocityCommand
        ↓
Robot Adapter
        ├── integrate
        ├── interpolation
        ├── output rate limit
        └── native command conversion
        ↓
JointPositionCommand
```

命令连续性由 Adapter 保证。

## 2.8 单周期契约

每个正常控制周期严格按以下顺序执行：

```text
┌─────────────────────────────────────────────────────────┐
│ 1. Runtime acquire fresh RobotState                     │
│                                                         │
│ 2. Runtime validate                                     │
│    - timestamp                                           │
│    - state age                                           │
│    - actual period                                       │
│    - finite values                                       │
│    - vector dimensions                                   │
│                                                         │
│ 3. RobotModel::update(state)                            │
│                                                         │
│ 4. Control thread consume external command mailbox      │
│                                                         │
│ 5. StateMachine::update(context)                        │
│    - commit pending transition                          │
│    - exit old state                                     │
│    - enter new state                                    │
│    - update current state                               │
│    - update Task target / enabled                       │
│    - sample trajectory                                  │
│                                                         │
│ 6. Controller::update(state, cycle, output)             │
│    - validate controller/task state                     │
│    - invoke QPSolver                                    │
│                                                         │
│ 7. QPSolver                                             │
│    - rebuild fixed-size H/g numerically                 │
│    - update fixed hard constraint bounds                │
│    - solve qdot                                         │
│                                                         │
│ 8. Controller writes JointVelocityCommand fields        │
│                                                         │
│ 9. Runtime validates / writes command                   │
│                                                         │
│10. Runtime diagnostics / watchdog                       │
└─────────────────────────────────────────────────────────┘
```

FSM 必须在 RobotModel 更新后运行，因为 State 的 `enter()` / `update()` 可能读取当前 Frame Pose 来初始化轨迹。

## 2.9 Model 与当前 RobotState 的关联

Controller 的前置条件是：

> RobotModel 已使用同一个 RobotState 更新。

如果要提供：

```text
ModelNotUpdated
```

错误，就必须能够验证。

RobotModel 保存：

```cpp
std::chrono::steady_clock::time_point
last_state_stamp() const noexcept;
```

`RobotModel::update(state)` 成功后：

```text
last_state_stamp = state.stamp
```

Controller 检查：

```cpp
if (model.last_state_stamp() != state.stamp)
{
    return ControlError::ModelNotUpdated;
}
```

因为 Runtime 已保证 state timestamp 单调且唯一，所以无需额外引入 ModelRevision/GenerationId。

## 2.10 FSM transition 的周期边界语义

状态 A 在周期 N：

```text
A.update()
    ↓
Succeeded
```

本周期只产生：

```text
pending transition A → B
```

周期 N 仍由 A 的最终 Task 状态完成一次求解。

周期 N+1：

```text
A.exit()
B.enter()
B.update()
Controller::update()
```

即：

```text
cycle N:
    A.update
    mark transition
    solve A final cycle

cycle N+1:
    A.exit
    B.enter
    B.update
    solve B first cycle
```

默认 500 Hz 下最多增加 2 ms 切换延迟，但生命周期完全确定。

## 2.11 External transition 竞争规则

跨线程 Application / Agent 不直接调用 FSM setter。

它们将请求写入 Application-owned mailbox。

控制线程在周期开始阶段读取 mailbox。

优先规则：

```text
1. 周期开始消费外部 transition request
2. 如果存在，则优先应用
3. 当前 State update
4. State result 产生 automatic pending transition
5. automatic transition 在下一周期应用
```

本周期执行过程中后来到达的外部请求，一律等下一周期处理。

重复请求：

```text
同一周期只接受一个最终 request
```

Mailbox 的覆盖/队列策略由 Application Runtime 决定，但提交给 FSM 时必须已经归一化成最多一个 request。

## 2.12 FSM 无匹配 transition 时的行为

`Succeeded` 无 transition：

```text
exit current state
→ FSM Completed
```

`Failed` 无 transition：

```text
exit current state
→ FSM Failed
```

不是隐式停留，也不是自动重启。

Runtime / Application 根据 Completed/Failed 决定后续行为。

## 2.13 线程模型

`Controller::start()` 以后：

> 所有 Task setter、Trajectory 修改、FSM update 和 transition commit 必须发生在同一个 control thread。

禁止 Agent/UI/network thread 直接调用：

```text
task.set_target()
task.set_gain()
task.set_weight()
task.set_enabled()
fsm.request_transition()
```

跨线程输入必须先进入 Application-owned：

```text
command mailbox
```

由 control thread 在周期边界读取。

RoControl V1 不实现通用 mailbox；它只规定线程契约。

## 2.14 失败交接

Controller：

```cpp
Result<void, ControlError>
Controller::update(
    const RobotState& state,
    const ControlCycle& cycle,
    JointVelocityCommand& output);
```

语义：

```text
success:
    output 当前周期有效

failure:
    output 不得视为有效命令
```

失败时 RoControl 不：

```text
返回 last command
自动 hold
自动 relax constraint
直接调用 hardware stop
```

Runtime 根据系统策略决定：

```text
hold
controlled stop
native robot stop
fault
```

---

# 3. 数据模型、RobotModel 与坐标契约

V1 必须把“顺序”和“坐标系”视为 API 契约，否则非常容易出现：

> 维度全部正确，但控制的是错误关节或错误方向。

## 3.1 Pose

统一：

```cpp
struct Pose
{
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
};
```

不创建：

```text
PoseTarget
PoseReference
TargetPose
```

Quaternion 必须是 finite 且归一化。

Public API 不使用 Euler angle 作为基础姿态表示。

## 3.2 RobotState

```cpp
struct RobotState
{
    std::chrono::steady_clock::time_point stamp;

    Eigen::VectorXd position;
    Eigen::VectorXd velocity;
};
```

V1 不包含：

```text
effort
acceleration
IMU
contact
floating base
```

这些属于后续 Dynamics/G1 扩展。

## 3.3 裸向量与 Joint 顺序验证边界

RoControl 内部所有 RobotState 向量都使用：

```text
RobotModel canonical DoF order
```

但因为 RobotState 只携带数值向量，Controller 无法检查外部来源是否偷偷交换了两个关节。

因此正式契约是：

> 外部 Adapter 在初始化阶段建立“外部 Joint Order → RoControl Canonical DoF Order”映射，并验证一次。运行阶段进入 RoControl 的 RobotState 已经必须是 canonical order。

Controller 运行时只能验证：

```text
dimension
finite values
timestamp
```

不能承诺自动识别 semantic joint order 错误。

## 3.4 Canonical DoF Order

RobotModel 提供：

```cpp
const std::vector<std::string>&
dof_names() const noexcept;
```

不用 `std::span`，因为 V1 是 C++17。

例如 Mobile FR3 Duo：

```text
[
    left_joint1,
    left_joint2,
    ...
    left_joint7,

    right_joint1,
    ...
    right_joint7,

    spine_joint
]
```

以下向量全部使用同一顺序：

```text
RobotState.position
RobotState.velocity
Solver qdot
JointVelocityCommand.velocity
position limits
velocity limits
```

正式契约：

```text
vector[i]
↔
RobotModel::dof_names()[i]
```

## 3.5 Fixed Joint

Fixed joint：

```text
不进入 canonical DoF vector
不进入 Solver variable
不进入 JointVelocityCommand
```

它只存在于模型拓扑和 Frame kinematics 中。

## 3.6 Mimic Joint

Mimic joint：

```text
不作为独立 Solver DoF
```

其位置由 master joint 关系推导：

\[
q_m = a q + b
\]

Mimic joint 的物理 limit 不能忽略。

RobotModel 初始化时应把 mimic 限制转换成 master DoF 的有效限制：

\[
q_{m,min}
\le
a q+b
\le
q_{m,max}
\]

最终 Solver 只看到：

```text
effective independent-DoF limits
```

这样 Task / Constraint 不需要认识 mimic 逻辑。

## 3.7 Continuous Joint

Continuous revolute joint 的位置限制表示为：

```text
lower = -inf
upper = +inf
```

因此：

```text
没有 position constraint
```

但仍可以有：

```text
velocity limit
```

RobotModel 的 effective limit vector 必须支持无限边界。

## 3.8 JointLimit Tolerance

配置：

```cpp
double joint_limit_tolerance{1e-6};
```

单位采用每个 Joint 的 native coordinate：

```text
revolute  → rad
prismatic → m
```

该 tolerance 仅用于：

```text
state validation
floating-point numerical tolerance
```

不扩大 QP 允许运动范围。

如果：

```text
q > q_max + tolerance
```

或：

```text
q < q_min - tolerance
```

则：

```text
StateOutOfBounds
```

## 3.9 JointIndex 与 ModelId

```cpp
using JointIndex = std::uint32_t;
using FrameIndex = std::uint32_t;
using ModelId = std::uint64_t;
```

每个 RobotModel 实例有稳定 ModelId：

```cpp
ModelId id() const noexcept;
```

ModelId 用来防止：

```text
JointGroup from Model A
被用于 Controller(Model B)
```

## 3.10 JointGroup

C++17 下不使用 `std::span`。

建议：

```cpp
struct JointGroup
{
    ModelId model;
    std::vector<JointIndex> joints;
};
```

JointGroup 是初始化阶段创建的稳定 Value Object。

RobotModel 创建 JointGroup 时必须：

- 验证所有 JointIndex；
- 拒绝重复 Joint；
- 保持声明顺序；
- 记录 ModelId。

Controller 添加 JointPositionTask 时：

```text
group.model == controller.model.id()
```

否则返回错误。

JointGroup 自己拥有 index 集合，因此不会因为 RobotModel 内部容器重分配而悬挂。

## 3.11 JointGroup 顺序

`JointGroup.joints` 顺序就是 Task target 顺序。

例如：

```text
left_arm:
[
    left_joint1,
    left_joint2,
    ...
    left_joint7
]
```

则：

```text
JointPositionTask.target()[0]
↔
left_joint1
```

Task 内部不能重新排序。

## 3.12 Frame

Cartesian 控制对象统一使用：

```text
Frame
```

初始化：

```cpp
auto left_ee =
    model.frame("left_ee");
```

返回：

```cpp
Result<FrameIndex, ModelError>
```

运行路径保存 FrameIndex，不反复查字符串。

V1 不引入：

```text
ControlPoint
EndEffectorId
ReferenceFrame object
ControlFrame object
```

## 3.13 Cartesian Reference Frame

V1 fixed-base：

> 所有 Cartesian Pose、error、linear/angular velocity 和 Jacobian 都表达在 RobotModel reference frame。

V1 reference frame：

```text
robot root/base frame
```

Future floating-base 再扩展 world/reference frame policy。

V1 不要求每个 Task 都携带独立 reference frame 参数。

## 3.14 Jacobian 契约

`frame_jacobian(frame)` 返回：

```text
6 x ndof
```

列顺序：

```text
canonical DoF order
```

行顺序：

```text
[
    linear_x,
    linear_y,
    linear_z,
    angular_x,
    angular_y,
    angular_z
]
```

即：

\[
[v;\omega]
\]

reference point 位于 controlled Frame 原点。

线速度和角速度都表达在 RobotModel reference frame。

如果 Pinocchio 内部约定不同：

> RobotModel 负责转换，backend convention 不得泄漏到 Public API。

## 3.15 Orientation Error

姿态误差使用 SO(3) logarithm，不使用 Euler subtraction。

例如：

\[
e_R =
\mathrm{Log}
(
R_{target}R_{current}^{T}
)
\]

最终 Pose error：

\[
e =
\begin{bmatrix}
p_{target}-p_{current}\\
e_R
\end{bmatrix}
\]

仍然采用：

```text
linear
angular
```

顺序。

## 3.16 RobotModel 构造配置

V1 必须有可执行的模型创建接口。

建议：

```cpp
struct JointLimitOverrides
{
    // optional per-joint overrides
};

struct RobotModelConfig
{
    std::filesystem::path urdf;

    std::map<
        std::string,
        std::vector<std::string>>
        joint_groups;

    std::optional<JointLimitOverrides>
        limit_overrides;
};
```

创建：

```cpp
static Result<
    std::unique_ptr<RobotModel>,
    ModelError>
RobotModel::create(
    const RobotModelConfig& config);
```

V1 只支持：

```text
fixed-base URDF
Pinocchio internally
```

初始化时完成：

```text
load URDF
build Pinocchio model
resolve independent DoFs
resolve fixed joints
resolve mimic joints
build effective limits
build Joint lookup
build Frame lookup
validate JointGroups
assign ModelId
preallocate model caches
```

默认 limits 来自 URDF。

允许 optional explicit override。

## 3.17 RobotModel 查询 API

```cpp
class RobotModel
{
public:
    static Result<
        std::unique_ptr<RobotModel>,
        ModelError>
    create(const RobotModelConfig& config);

    Result<void, ModelError>
    update(const RobotState& state);

    ModelId id() const noexcept;

    std::size_t dof() const noexcept;

    const std::vector<std::string>&
    dof_names() const noexcept;

    Result<JointIndex, ModelError>
    joint(std::string_view name) const;

    Result<JointGroup, ModelError>
    joint_group(std::string_view name) const;

    Result<FrameIndex, ModelError>
    frame(std::string_view name) const;

    const Pose&
    frame_pose(FrameIndex frame) const;

    const Eigen::MatrixXd&
    frame_jacobian(FrameIndex frame) const;

    const Eigen::VectorXd&
    lower_position_limits() const noexcept;

    const Eigen::VectorXd&
    upper_position_limits() const noexcept;

    const Eigen::VectorXd&
    velocity_limits() const noexcept;

    std::chrono::steady_clock::time_point
    last_state_stamp() const noexcept;
};
```

名称查询是可预期失败，因此统一走 `Result`。

## 3.18 RobotModel 生命周期

RobotModel：

```text
create
    ↓
immutable topology
    ↓
update(state) every cycle
```

V1 不支持运行时：

```text
reload URDF
change DoF topology
add/remove Joint
rebuild Frame table
```

如果模型重建：

```text
旧 JointIndex / FrameIndex / JointGroup 全部失效
```

因此 Application 必须重新构造 Controller/FSM。

不支持“同一个 Controller 热替换 RobotModel”。

---

# 4. Task、Constraint、Solver 与 Controller

V1 控制核心采用：

```text
Weighted Kinematic Velocity QP
```

结构：

```text
Task        → objective
Constraint  → hard feasible region
QPSolver    → build + solve
Controller  → lifecycle + validation + call Solver
```

## 4.1 Task 命名

V1：

```text
Task
├── JointPositionTask
└── CartesianPoseTask
```

命名遵循：

```text
Control Space + Controlled Quantity
```

因此：

```text
JointPositionTask
CartesianPoseTask
```

而不是：

```text
JointTask
PoseTask
```

未来可自然扩展：

```text
JointVelocityTask
CartesianPositionTask
CartesianVelocityTask
```

## 4.2 Task 基类

```cpp
class Task
{
public:
    virtual ~Task() = default;

    bool enabled() const noexcept;

    void set_enabled(bool enabled) noexcept;

    double weight() const noexcept;

    Result<void, TaskError>
    set_weight(double weight);
};
```

所有 Task 默认：

```text
enabled = false
weight  = 1.0
```

### Weight 合法域

```text
finite
> 0
```

不允许：

```text
0
negative
NaN
Inf
```

如果 Task 不参与当前求解：

```cpp
task.set_enabled(false);
```

而不是把 weight 设成 0。

## 4.3 Task target 初始化规则

所有 Task 必须保证：

> enabled 时 target 一定有效。

Controller::start() 时初始化所有尚未显式设置 target 的 Task。

### JointPositionTask

默认 target：

```text
current joint position
```

### CartesianPoseTask

默认 target：

```text
current Frame Pose
```

Task 默认 disabled，因此初始化不会立即产生控制目标变化。

如果用户在 `start()` 前已经显式 `set_target()`，保留用户 target。

## 4.4 Setter 的事务语义

所有可能失败的 setter 都采用：

```text
validate first
commit second
```

失败时内部原值保持不变。

例如：

```cpp
Result<void, TaskError>
CartesianPoseTask::set_target(
    const Pose& target);
```

验证：

```text
position finite
quaternion finite
quaternion norm valid
```

成功后才更新。

同样适用于：

```text
set_weight
set_gain
set_position_weight
set_orientation_weight
trajectory.reset
```

这样 State 操作失败时不会留下半更新状态。

## 4.5 JointPositionTask

```cpp
class JointPositionTask final
    : public Task
{
public:
    const JointGroup&
    group() const noexcept;

    Result<void, TaskError>
    set_target(
        const Eigen::VectorXd& position);

    const Eigen::VectorXd&
    target() const noexcept;

    Result<void, TaskError>
    set_gain(double gain);

    double gain() const noexcept;

    Result<void, TaskError>
    set_reference_velocity(
        const Eigen::VectorXd& velocity);

    void clear_reference_velocity() noexcept;
};
```

Gain：

```text
finite
>= 0
default = 1.0 s^-1
```

Reference velocity 默认：

```text
0
```

数学目标：

\[
S\dot q
\approx
\dot q_d
+
K_q(q_d-q)
\]

其中：

```text
S       JointGroup selection
q       current joint position
q_d     target position
qdot_d  trajectory/feedforward velocity
K_q     feedback gain [s^-1]
```

这样 trajectory 可以提供前馈，不必只靠位置误差追踪。

## 4.6 CartesianPoseTask

```cpp
class CartesianPoseTask final
    : public Task
{
public:
    FrameIndex frame() const noexcept;

    Result<void, TaskError>
    set_target(const Pose& pose);

    const Pose&
    target() const noexcept;

    Result<void, TaskError>
    set_position_gain(double gain);

    Result<void, TaskError>
    set_orientation_gain(double gain);

    double position_gain() const noexcept;

    double orientation_gain() const noexcept;

    Result<void, TaskError>
    set_position_weight(double weight);

    Result<void, TaskError>
    set_orientation_weight(double weight);

    Result<void, TaskError>
    set_reference_velocity(
        const Eigen::Vector3d& linear,
        const Eigen::Vector3d& angular);

    void clear_reference_velocity() noexcept;
};
```

### Gain 合法域

```text
position gain:
    finite
    >= 0
    default 1.0 s^-1

orientation gain:
    finite
    >= 0
    default 1.0 s^-1
```

### Cartesian metric

Position residual：

```text
m/s
```

Orientation residual：

```text
rad/s
```

二者单位不同，因此不能只靠一个总 Task weight 假装定义 metric。

增加：

```text
position_weight
orientation_weight
```

默认：

```text
1.0
1.0
```

合法域：

```text
finite
> 0
```

总 Task weight 控制该 Cartesian Task 与其他 Task 的相对权重。

内部 component weight 控制：

```text
translation
vs
rotation
```

的相对尺度。

数学目标：

\[
v_{ref}
=
v_d
+
K_p(p_d-p)
\]

\[
\omega_{ref}
=
\omega_d
+
K_R e_R
\]

构成：

\[
b =
\begin{bmatrix}
v_{ref}\\
\omega_{ref}
\end{bmatrix}
\]

优化：

\[
J\dot q \approx b
\]

并在 cost 中分别施加 position/orientation component weights。

## 4.7 为什么 V1 没有 Priority

V1 不提供：

```cpp
task.set_priority(...)
```

真正 Priority 表示：

> 低层任务不得破坏高层任务最优解。

这是：

```text
Hierarchical QP
Lexicographic optimization
```

而不是：

```text
weight = 1e6
```

V1：

```text
Weighted QP
```

未来真正实现 HQP 后再加入：

```text
Task level
HQPSolver
```

语义：

```text
level   strict hierarchy
weight  trade-off inside same level
```

## 4.8 Constraint

V1：

```text
Constraint
├── JointPositionLimitConstraint
└── JointVelocityLimitConstraint
```

Constraint 默认：

```text
hard
always active
```

V1 没有：

```text
constraint weight
constraint priority
soft constraint
constraint enabled
```

## 4.9 基础 Limits 默认启用

真实机器人 V1 不应依赖用户记得调用：

```text
add_joint_position_limit_constraint()
add_joint_velocity_limit_constraint()
```

因此 Controller 默认根据 Config 自动创建：

```text
JointPositionLimitConstraint
JointVelocityLimitConstraint
```

配置：

```cpp
bool enforce_position_limits{true};
bool enforce_velocity_limits{true};
```

默认均为：

```text
true
```

只有用户显式关闭才不启用。

未来：

```text
CollisionConstraint
WorkspaceConstraint
```

才需要显式添加 API。

## 4.10 JointVelocityLimitConstraint

直接约束：

\[
-\dot q_{max}
\le
\dot q
\le
\dot q_{max}
\]

不依赖 dt。

## 4.11 JointPositionLimitConstraint

V1 fixed-base independent revolute/prismatic DoF：

\[
q_{min}
\le
q+\dot q\,dt
\le
q_{max}
\]

转换：

\[
\frac{q_{min}-q}{dt}
\le
\dot q
\le
\frac{q_{max}-q}{dt}
\]

使用：

```text
actual period
```

不是 nominal period。

Continuous joint：

```text
position lower/upper = ±inf
```

因此不产生 position bound。

## 4.12 已越界状态

如果：

```text
q > q_max + tolerance
or
q < q_min - tolerance
```

Controller 返回：

```text
ControlError::StateOutOfBounds
```

不进入正常 QP。

V1 不自动：

```text
relax hard limit
disable velocity limit
generate recovery overspeed
```

Limit recovery 以后单独设计。

## 4.13 Solver 职责

QP 构建职责全部属于 Solver。

Controller 不直接构造：

```text
H
g
C
lower
upper
```

QPSolver：

```text
read Task
read Constraint
build QP
solve QP
```

这避免 Controller 与数学 backend 耦合。

## 4.14 Solver API

为避免控制周期内分配，使用 caller-owned output buffer。

```cpp
class Solver
{
public:
    virtual ~Solver() = default;

    virtual Result<void, SolverError>
    solve(
        const RobotModel& model,
        const std::vector<std::unique_ptr<Task>>& tasks,
        const std::vector<std::unique_ptr<Constraint>>& constraints,
        const ControlCycle& cycle,
        Eigen::Ref<Eigen::VectorXd> output_velocity)
        = 0;
};
```

Controller 在 `start()` 之前确定 Task/Constraint containers，并 reserve/finalize。

500 Hz 期间不创建新的临时 Task pointer vector。

具体实现可以把只读视图进一步封装，但 V1 不必增加复杂 TaskSet 类型。

## 4.15 QPSolver

V1：

```cpp
class QPSolver final : public Solver
{
    ...
};
```

QPSolver 同时负责：

```text
preallocated matrix reset
Task objective accumulation
Constraint bound update
QP backend invocation
solution validation
```

QP backend 是内部实现细节。

Public API 不提供：

```cpp
set_backend()
```

只有未来出现第二个真实 backend 需求时再抽象。

## 4.16 V1 QP 数学模型

Decision variable：

\[
\dot q \in \mathbb{R}^{n}
\]

其中：

```text
n = canonical independent controllable DoF count
```

每个 enabled Task：

\[
A_i\dot q \approx b_i
\]

总体：

\[
\min_{\dot q}
\sum_i
w_i
\|W_i(A_i\dot q-b_i)\|^2
+
\lambda\|\dot q\|^2
\]

其中：

```text
w_i      Task overall weight
W_i      optional task-internal metric
lambda   QP regularization
```

展开：

\[
H
=
\sum_i
w_i A_i^T W_i^T W_i A_i
+
\lambda I
\]

\[
g
=
-\sum_i
w_i A_i^T W_i^T W_i b_i
\]

Hard constraints：

\[
l
\le
C\dot q
\le
u
\]

得到：

\[
\dot q^*
\]

直接写入：

```text
JointVelocityCommand.velocity
```

## 4.17 QP Regularization

Config：

```cpp
double qp_regularization{1e-6};
```

合法域：

```text
finite
> 0
```

不允许：

```text
0
negative
NaN
Inf
```

这样 Hessian 至少具有基础正则化，避免冗余自由度下退化。

## 4.18 QP 固定结构

V1 明确采用：

> **预分配、固定尺寸的 Dense QP 结构。**

理由：

- Franka3 / Duo / spine 的 DoF 数量较小；
- V1 没必要为 large sparse problem 提前复杂化；
- 500 Hz 更容易通过固定 dense buffer 做稳定性能控制。

`start()` 后：

```text
decision variable count fixed
constraint row count fixed
dense matrix dimensions fixed
```

Task enable/disable：

```text
只改变 H/g 数值
```

不改变 storage shape。

Constraint：

```text
固定存在
只更新 bound 数值
```

因此“QP topology fixed”在 V1 的真实含义是：

```text
dense matrix dimensions fixed
variable count fixed
constraint row count fixed
```

不是承诺 sparse non-zero pattern 永远相同。

## 4.19 SolverError

```cpp
enum class SolverError
{
    Infeasible,
    NumericalFailure,
    InvalidProblem
};
```

Solver 不处理：

```text
hold
stop
fault recovery
```

## 4.20 Controller API

```cpp
class Controller
{
public:
    Controller(
        RobotModel& model,
        Solver& solver,
        ControllerConfig config = {});

    Result<
        std::reference_wrapper<JointPositionTask>,
        ControlError>
    add_joint_position_task(
        const JointGroup& group);

    Result<
        std::reference_wrapper<CartesianPoseTask>,
        ControlError>
    add_cartesian_pose_task(
        FrameIndex frame);

    Result<void, ControlError>
    start(const RobotState& state);

    Result<void, ControlError>
    update(
        const RobotState& state,
        const ControlCycle& cycle,
        JointVelocityCommand& output);

    void stop() noexcept;
};
```

### 为什么 add API 返回 Result

可能失败：

```text
controller already started
wrong ModelId
invalid group
invalid FrameIndex
duplicate internal name/policy if future added
```

所以不能直接返回引用并假定永远成功。

## 4.21 Controller 生命周期

```text
Construct
    ↓
Add Tasks
    ↓
start(initial_state)
    ↓
runtime updates @ configured rate
    ↓
stop()
```

`start()` 后：

### 允许

```text
Task target
Task gain
Task weight
Task enabled
Trajectory runtime state
FSM state transition
```

### 禁止

```text
add/remove Task
add/remove Constraint
change RobotModel topology
change Solver dimension
```

## 4.22 Controller::start

`start(initial_state)` 前置条件：

```text
RobotModel already updated with initial_state
```

Controller start：

1. 检查 ModelId / DoF；
2. 创建/确认 built-in limits；
3. 初始化尚未设 target 的 JointPositionTask 为当前 joint position；
4. 初始化尚未设 target 的 CartesianPoseTask 为当前 Frame Pose；
5. finalize Task/Constraint containers；
6. reserve / allocate Solver buffers；
7. resize caller-owned command storage requirement；
8. 标记 started。

`start()` 不输出控制命令。

## 4.23 Controller::update

前置条件：

```text
Controller started
RobotModel.last_state_stamp == RobotState.stamp
ControlCycle valid
Task modifications already completed by control thread
```

内部：

```text
1. validate state dimensions / finite values

2. validate cycle

3. validate model-current-state association

4. validate current hard position limits

5. invoke Solver::solve(...)

6. validate qdot finite / dimension

7. set output.sequence

8. set output.stamp

9. copy solution into preallocated output.velocity

10. return success
```

Controller 不构建 H/g。

## 4.24 ControlError

至少：

```cpp
enum class ControlError
{
    InvalidCyclePeriod,
    InvalidState,
    StateOutOfBounds,
    ModelNotUpdated,
    InvalidModelReference,
    InvalidTaskConfiguration,
    AlreadyStarted,
    NotStarted,
    SolverInfeasible,
    SolverFailure,
    NumericalFailure
};
```

Runtime freshness error 可以保持在 Runtime 层，不必重复放进 ControlError。

---

# 5. FSM、Trajectory、线程与错误传播

FSM 是 V1 Core，因为 Task + Solver 只能解决“当前怎么运动”，不能解决“当前应该做什么”。

## 5.1 StateContext

```cpp
struct StateContext
{
    const RobotState& state;
    const RobotModel& model;
    const ControlCycle& cycle;
};
```

它只是轻量数据聚合，不是新的架构层。

## 5.2 StateStatus

```cpp
enum class StateStatus
{
    Running,
    Succeeded,
    Failed
};
```

如果需要区分：

```text
Timeout
Cancelled
```

可以由 State 自己映射为 Failed 并带具体 FsmError / diagnostics；V1 不扩大 StateStatus 枚举。

## 5.3 State 生命周期全部可失败

```cpp
class State
{
public:
    virtual ~State() = default;

    virtual Result<void, FsmError>
    enter(const StateContext& context) = 0;

    virtual Result<StateStatus, FsmError>
    update(const StateContext& context) = 0;

    virtual Result<void, FsmError>
    exit(const StateContext& context) = 0;
};
```

这样：

```text
trajectory.reset failure
invalid Task target
invalid transition setup
State-specific validation
```

都能可靠传播。

## 5.4 FSM 失败后的原子性

State setter / trajectory operations 必须遵循：

```text
validate first
commit second
```

因此单个操作失败不会破坏之前有效状态。

对于 transition：

```text
A.exit()
B.enter()
```

需要定义失败语义。

V1 建议：

### A.exit() 失败

```text
FSM → Failed
不进入 B
```

### A.exit() 成功，但 B.enter() 失败

```text
FSM → Failed
B 不视为 active
```

此时 Runtime 收到 FsmError 后进入 safety policy。

V1 不尝试自动 rollback A，因为通用 rollback 很难保证安全和一致。

因此设计原则是：

> transition failure is terminal for current FSM execution.

## 5.5 State 与 Task

初始化阶段创建 Task。

State 只保存非 owning reference。

例如：

```cpp
class ApproachState final
    : public State
{
public:
    ApproachState(
        CartesianPoseTask& hand_task,
        CartesianPoseTrajectory& trajectory);

private:
    CartesianPoseTask& hand_task_;
    CartesianPoseTrajectory& trajectory_;
};
```

`enter()`：

```text
read current frame pose
trajectory.reset(...)
task.set_enabled(true)
```

`update()`：

```text
sample trajectory
set Task reference
check tracking error
check timeout
```

`exit()`：

```text
task.set_enabled(false)
```

## 5.6 Trajectory 不是 Manager

V1 只有：

```text
JointPositionTrajectory
CartesianPoseTrajectory
```

不建立：

```text
TrajectoryManager
TrajectoryServer
TrajectoryExecutor
TrajectoryGraph
```

它们只是 State 使用的 reference helper。

## 5.7 JointPositionReference

```cpp
struct JointPositionReference
{
    Eigen::VectorXd position;
    Eigen::VectorXd velocity;
};
```

Trajectory sample 返回：

```text
position reference
velocity feedforward
```

不是只有 position。

JointPositionTask：

\[
S\dot q
\approx
\dot q_d + K(q_d-q)
\]

避免移动 reference 只能靠误差追踪而产生明显相位滞后。

## 5.8 CartesianPoseReference

```cpp
struct CartesianPoseReference
{
    Pose pose;

    Eigen::Vector3d linear_velocity;
    Eigen::Vector3d angular_velocity;
};
```

Task：

\[
J\dot q
\approx
\begin{bmatrix}
v_d\\
\omega_d
\end{bmatrix}
+
K e
\]

这样 quintic / SLERP trajectory 的速度信息真正进入 Velocity QP。

## 5.9 Trajectory 时间语义

Trajectory 使用：

```text
monotonic absolute time
```

而不是：

```text
step count
nominal dt accumulation
```

例如：

```cpp
trajectory.reset(
    current,
    target,
    duration,
    context.cycle.time);

auto reference =
    trajectory.sample(
        context.cycle.time);
```

因此 1.99 ms / 2.01 ms jitter 不改变总体 trajectory duration。

## 5.10 JointPositionTrajectory

推荐：

```text
quintic time scaling
```

至少满足：

```text
start velocity      = 0
end velocity        = 0
start acceleration  = 0
end acceleration    = 0
```

输出：

```text
q_ref
qdot_ref
```

## 5.11 CartesianPoseTrajectory

Translation：

```text
quintic scalar progress
```

Orientation：

```text
quaternion SLERP
+
same scalar progress
```

并计算：

```text
linear velocity feedforward
angular velocity feedforward
```

输出：

```text
Pose
linear_velocity
angular_velocity
```

## 5.12 Trajectory duration 的真实语义

`duration` 只表示：

> reference 从 start 生成到 target 的时间。

它不表示：

> 机器人保证在 duration 内到位。

因此 State success 一般定义为：

```text
trajectory finished
AND
tracking error <= tolerance
```

并需要额外：

```text
settling timeout
```

例如：

```text
trajectory duration = 2 s
settling timeout    = 1 s
```

3 秒后仍未满足 error threshold：

```text
StateStatus::Failed
```

具体 threshold / timeout 属于 State 配置，不放进通用 Trajectory。

## 5.13 FSM Transition 无匹配项

当前 State：

### `Succeeded`

如果存在 transition：

```text
pending → target
```

如果不存在：

```text
FSM Completed
```

### `Failed`

如果存在 failure transition：

```text
pending → recovery/error state
```

如果不存在：

```text
FSM Failed
```

不会隐式停留在当前 State。

## 5.14 External Request 与 State Result 冲突

确定规则：

```text
cycle boundary:
    external request first

during current cycle:
    State result produces automatic pending transition

next cycle:
    new external request again has higher precedence
```

如果下一周期既有：

```text
old automatic pending transition
new external transition request
```

则：

```text
external request wins
automatic pending transition discarded
```

这是为了让上层显式控制指令具有更高优先级。

Emergency stop 不使用普通 FSM request，而由 Runtime Safety 直接处理。

## 5.15 Command Mailbox

RoControl 不实现通用 mailbox，但要求 Runtime 提供：

```text
single-writer to RoControl control thread
```

的跨线程交接。

Agent/UI/network thread：

```text
publish desired action/request
```

control thread：

```text
consume request at cycle boundary
apply Task/FSM changes
```

这样避免：

```text
Task setter
```

和 Solver 同时访问导致 data race。

## 5.16 FsmError

至少：

```cpp
enum class FsmError
{
    InvalidState,
    InvalidTransition,
    EnterFailed,
    UpdateFailed,
    ExitFailed,
    TaskUpdateFailed,
    TrajectoryFailed
};
```

更具体业务错误可以通过 State-specific diagnostics 暴露，不需要把所有业务枚举塞进通用 FsmError。

---

# 6. 工程组织、Result、性能、测试与分阶段实施

## 6.1 推荐目录

```text
rocontrol/
├── CMakeLists.txt
├── cmake/
│
├── include/
│   └── rocontrol/
│       ├── result.hpp
│       ├── types.hpp
│       ├── cycle.hpp
│       │
│       ├── model/
│       │   ├── robot_model.hpp
│       │   └── config.hpp
│       │
│       ├── task/
│       │   ├── task.hpp
│       │   ├── joint_position_task.hpp
│       │   └── cartesian_pose_task.hpp
│       │
│       ├── constraint/
│       │   ├── constraint.hpp
│       │   ├── joint_position_limit.hpp
│       │   └── joint_velocity_limit.hpp
│       │
│       ├── solver/
│       │   ├── solver.hpp
│       │   └── qp_solver.hpp
│       │
│       ├── controller.hpp
│       │
│       ├── fsm/
│       │   ├── state.hpp
│       │   └── state_machine.hpp
│       │
│       └── trajectory/
│           ├── joint_position_trajectory.hpp
│           └── cartesian_pose_trajectory.hpp
│
├── src/
│   ├── model/
│   ├── task/
│   ├── constraint/
│   ├── solver/
│   ├── fsm/
│   └── trajectory/
│
├── tests/
├── benchmarks/
├── examples/
└── docs/
    └── rocontrol_design.md
```

V1 只提供：

```text
rocontrol::rocontrol
```

一个 CMake target。

暂时不拆：

```text
rocontrol::model
rocontrol::solver
rocontrol::fsm
```

## 6.2 依赖

V1：

```text
C++17
Eigen3
Pinocchio
one QP backend
```

Core 不依赖：

```text
ROS 2
MuJoCo
robohardware
Agent
Web
Tauri
```

QP backend 选择重点：

```text
dense small-to-medium QP performance
warm start
preallocation
500 Hz suitability
stable API
```

具体 backend 属于实现阶段选择。

## 6.3 Result<T,E>

C++17 没有 `std::expected`。

RoControl 自己提供：

```text
include/rocontrol/result.hpp
```

API：

```cpp
template<typename T, typename E>
class Result;

template<typename E>
class Result<void, E>;
```

要求：

- 正常成功/失败路径不进行 heap allocation；
- 正常错误路径不抛 exception；
- move-aware；
- Public API 不暴露 `std::variant`；
- value/error 状态明确；
- `Result<void,E>` 有专门实现；
- 可在 500 Hz 路径安全使用。

实现内部可以：

```text
union
or
std::variant
```

只要满足上述契约。

RoControl 不依赖 robohardware 的 `Result`，但可以沿用其设计经验。

## 6.4 Error Handling

典型：

```text
ModelError
TaskError
SolverError
ControlError
FsmError
TrajectoryError
```

都使用：

```cpp
enum class
```

不设计二级 error code 体系。

可预期运行错误：

```text
Result
```

内部不变量 / programming error：

```text
assert / contract
```

模型解析等初始化阶段如果底层第三方库抛异常，应在 RoControl 边界捕获并转换成 `Result`。

## 6.5 Ownership

```text
Application
├── owns RobotModel
├── owns Solver
├── owns Controller
└── owns StateMachine
```

Controller：

```text
references RobotModel
references Solver
owns Tasks
owns Constraints
```

StateMachine：

```text
owns States
```

State：

```text
non-owning references to Tasks
non-owning references to Trajectory
```

Owning pointer：

```cpp
std::unique_ptr
```

不使用 raw owning pointer。

## 6.6 500 Hz 内存契约

控制周期内目标：

```text
steady-state heap allocations = 0
```

因此：

### 初始化阶段

```text
resize RobotState buffers
resize command buffer
reserve Task container
reserve Constraint container
allocate dense QP matrices
allocate solver workspaces
allocate model caches
```

### 运行阶段

禁止：

```text
Eigen::VectorXd resize
std::vector growth
new/delete
Task creation
Constraint creation
string allocation
```

输出采用 caller-owned buffer：

```cpp
JointVelocityCommand command;
command.velocity.resize(model.dof());
```

之后每周期复用。

## 6.7 Public API 质量要求

Public API 必须：

- C++17；
- Doxygen；
- const-correctness；
- ownership 明确；
- 单位明确；
- DoF order 明确；
- Cartesian frame convention 明确；
- lifecycle 明确；
- threading 明确；
- failure semantics 明确；
- 不暴露 backend-specific 类型；
- 不暴露 QP H/g plumbing；
- 避免不必要模板；
- 避免 plugin factory；
- 避免运行路径字符串查找。

## 6.8 Command 与周期关联

`JointVelocityCommand` 自带：

```text
sequence
stamp
```

与：

```text
ControlCycle.sequence
ControlCycle.time
```

完全对应。

Adapter 可以检测：

```text
stale command
out-of-order command
queued old command
```

V1 Runtime 推荐同步消费：

> 当前周期命令应在当前周期写出，不允许无限排队。

## 6.9 RobotModel 测试

至少：

```text
URDF load
invalid URDF
canonical DoF order
fixed joint exclusion
mimic resolution
mimic-derived limits
continuous joint
JointGroup lookup
duplicate joint rejection
cross-model group rejection
Frame lookup
FK
Jacobian
Jacobian row convention
Jacobian column order
effective limits
state dimension
NaN
last_state_stamp
```

## 6.10 Task 测试

JointPositionTask：

```text
default target on start
target dimension
JointGroup order
gain validation
weight validation
enabled
reference velocity
validate-before-commit
```

CartesianPoseTask：

```text
default current Pose target
finite target
quaternion validation
SO(3) error
frame convention
position gain
orientation gain
position weight
orientation weight
velocity feedforward
validate-before-commit
```

## 6.11 Constraint 测试

```text
velocity bounds
position-derived velocity bounds
actual dt
near-zero dt
max-period rejection
limit edge
continuous joint
unbounded joint
out-of-bound state
mimic-derived effective limit
```

## 6.12 Solver 测试

```text
single JointPositionTask
single CartesianPoseTask
dual Task
dual-arm simultaneous Task
conflicting weighted Tasks
disabled Task
Task metric
velocity feedforward
fixed matrix dimension
regularization
redundant DoF
near singular Jacobian
infeasible constraints
numerical failure
warm start
no steady-state allocation
```

## 6.13 Controller 测试

```text
start before Model update
start success
update before start
ModelNotUpdated
actual period validation
state dimension
StateOutOfBounds
solver infeasible
solver failure
output sequence/stamp
output buffer reuse
stop lifecycle
add Task after start rejected
```

## 6.14 FSM 测试

```text
enter success/failure
update Running/Succeeded/Failed
exit success/failure
cycle-boundary transition
external transition precedence
automatic transition
missing success transition → Completed
missing failure transition → Failed
transition enter failure terminal behavior
Task enable/disable
control-thread-only contract
```

## 6.15 Trajectory 测试

```text
start reference
end reference
position feedforward velocity
Cartesian linear velocity
Cartesian angular velocity
duration
zero/negative duration rejection
monotonic time
jittered sample interval
quintic boundary velocity
quintic boundary acceleration
quaternion shortest-path SLERP
tracking completion semantics
settling timeout integration in State
```

## 6.16 Integration 测试

至少：

```text
Franka3 joint-space closed loop

Franka3 Cartesian closed loop

Franka3 Duo:
    left Cartesian Task
    right Cartesian Task
    simultaneous dual-arm

Mobile FR3 Duo:
    dual arm
    spine
    mixed Joint + Cartesian Task

FSM + trajectory

romujoco closed loop

500 Hz long-running stability

250 Hz smoke test

1 kHz smoke test

actual-dt jitter

one missed cycle

stale state rejection
```

## 6.17 500 Hz 性能验收

参考场景：

```text
Mobile FR3 Duo / similar
~15 independent DoF

2 x CartesianPoseTask
1 x JointPositionTask

position limits
velocity limits
```

运行：

```text
500 Hz
10 minutes
300,000 cycles
Release build
CPU affinity documented
reference machine documented
```

RoControl Core 通过条件建议：

```text
p99 <= 1.0 ms
max <= 1.5 ms
```

留下至少：

```text
0.5 ms
```

给 Runtime / I/O。

同时要求：

```text
steady-state heap allocations = 0
solver failure = 0
NaN = 0
RoControl-caused deadline miss = 0
```

Benchmark report 必须记录：

```text
CPU model
kernel
compiler
compiler version
build flags
QP backend
Task count
DoF count
Constraint count
model update timing
FSM timing
Solver timing
Controller total timing
p50/p95/p99/max
```

这样“500 Hz 达标”才是可判定结果，而不是只有统计数字。

## 6.18 V1 实现阶段

设计文档可以完整，但实现应按纵向切片推进。

### Phase 1 — Foundation + RobotModel

实现：

```text
Result<T,E>
core types
ControlCycle
RobotModelConfig
URDF / Pinocchio model
canonical DoF order
JointGroup
Frame
FK
Jacobian
effective limits
```

验收：

```text
模型能正确加载
DoF / JointGroup / Frame 契约稳定
500 Hz RobotModel update 空循环可运行
无 steady-state allocation
```

### Phase 2 — Joint Velocity QP + JointPositionTask

实现：

```text
Task base
JointPositionTask
built-in limits
dense QPSolver
Controller
JointVelocityCommand
```

验收：

```text
单臂 JointPosition 闭环
velocity limit 生效
position limit 生效
250/500/1000 Hz 行为语义一致
```

### Phase 3 — CartesianPoseTask + romujoco

实现：

```text
CartesianPoseTask
SO(3) error
Cartesian metric
romujoco adapter/example
```

验收：

```text
Franka3 Cartesian pose 闭环
Cartesian frame convention 验证
near singular behavior
```

### Phase 4 — Trajectory + FSM + Dual Arm

实现：

```text
JointPositionTrajectory
CartesianPoseTrajectory
velocity feedforward
State
StateMachine
cycle-boundary transition
dual-arm / spine examples
```

验收：

```text
Franka3 Duo simultaneous Cartesian control
Mobile FR3 Duo dual arm + spine
FSM + trajectory full behavior
timeout / failure propagation
```

### Phase 5 — 500 Hz Engineering Convergence

实现/收敛：

```text
allocation audit
solver buffer reuse
benchmark
runtime jitter tests
real robot adapter validation
diagnostics
```

验收：

```text
p99/max performance threshold
0 steady-state allocations
500 Hz long-running stability
real robot command semantics verified
```

## 6.19 V1 必须实现

### Core

```text
RobotModel

Task
    JointPositionTask
    CartesianPoseTask

Constraint
    JointPositionLimitConstraint
    JointVelocityLimitConstraint

Solver
    QPSolver

Controller

FSM
    State
    StateMachine
```

### Types

```text
Result<T,E>
Pose
RobotState
ControlCycle
JointVelocityCommand

JointIndex
FrameIndex
ModelId
JointGroup

JointPositionReference
CartesianPoseReference
```

### Utility

```text
JointPositionTrajectory
CartesianPoseTrajectory
```

### Examples

```text
single-arm joint
single-arm Cartesian
dual-arm
dual-arm + spine
FSM
trajectory
romujoco
500 Hz runtime
```

## 6.20 V1 明确不实现

```text
HQP
Strict Priority

Soft Constraint
Scaled Task
continuous activation blending

Cartesian Force Task
Force Control
Contact

Torque Control
Inverse Dynamics
Acceleration WBC

Floating Base WBC

State Estimator
Observer Pipeline

Behavior Tree

Navigation
Motion Planning

Plugin Loader
Dynamic State Plugin

ROS 2 Wrapper

GUI

Control Loop Scheduler

Hardware Driver
```

## 6.21 V1.x 自然扩展

按真实需求增加：

```text
JointVelocityTask
CartesianPositionTask
CartesianVelocityTask

SelfCollisionConstraint
WorkspaceConstraint
```

不修改总体架构。

## 6.22 HQP 演进

真正出现严格优先级需求后：

```text
Task level
HierarchicalQP
HQPSolver
```

语义：

```text
level   strict hierarchy
weight  same-level trade-off
```

重点参考 OpenSoT。

## 6.23 G1 / Dynamics 演进

进入 Unitree G1 / floating-base WBC 后再增加：

```text
FloatingBaseState

Contact
ContactConstraint
FrictionCone

CoMTask

TorqueLimitConstraint

AccelerationSolver
InverseDynamicsSolver

JointTorqueCommand

StateEstimator integration
```

核心仍建立在：

```text
RobotModel
Task
Constraint
Solver
Controller
FSM
```

之上。

## 6.24 最终架构冻结

```text
                    Application / Agent
                           │
                    Command Mailbox
                           │
                           ▼
                     StateMachine
                           │
                   target / reference
                           │
                           ▼
                       Controller
                           │
              ┌────────────┴────────────┐
              ▼                         ▼
             Task                   Constraint
              │                         │
              └────────────┬────────────┘
                           ▼
                       QPSolver
                    build + solve QP
                           │
                           ▼
                          qdot
                           │
                           ▼
                JointVelocityCommand
                           │
                           ▼
                    Robot Adapter
                           │
                 ┌─────────┴─────────┐
                 ▼                   ▼
          native velocity      native position
                               integrate/interpolate
```

正常单周期：

```text
Runtime read fresh RobotState
        ↓
validate state/time
        ↓
RobotModel::update
        ↓
consume external requests
        ↓
FSM boundary transition
        ↓
FSM update
        ↓
trajectory sample
        ↓
Task reference update
        ↓
Controller
        ↓
QPSolver build + solve
        ↓
qdot
        ↓
JointVelocityCommand
        ↓
Runtime / Robot Adapter
```

V1 默认：

```text
500 Hz
2 ms nominal period
configurable period
actual period validated each cycle
```

V1 Solver：

```text
Kinematic Velocity QP

decision variable:
    qdot

objective:
    weighted Task objectives
    Cartesian component metric
    regularization

hard constraints:
    joint position limits
    joint velocity limits
```

V1 的最终目标不是功能数量最多，而是：

> **让 Franka3、Franka3 Duo、Mobile FR3 Duo 所需要的 Joint / Cartesian、多 Task、FSM、Trajectory 和 500 Hz 控制路径拥有完整、明确、可执行、可测试的契约；同时避免 HQP、Dynamics、Contact、G1 等未来能力过早侵入核心。**
