# RoControl

RoControl is a C++17 fixed-base kinematic velocity controller. It turns joint
position and Cartesian pose tasks into a canonical-order joint velocity command.
The core has no hardware, ROS 2, or MuJoCo dependency. An application owns the
control thread, state mapping, command transport, and system safety policy.

The V1 design and its control contracts are in [docs/rocontrol_design.md](docs/rocontrol_design.md).
Current implementation and acceptance status is in
[docs/implementation_status.md](docs/implementation_status.md).

## Build and test

The build needs Eigen3 and Pinocchio. On a ROS Humble installation with
Pinocchio under `/opt/ros/humble`:

```sh
cmake -S . -B build-release -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH=/opt/ros/humble
cmake --build build-release -j2
ctest --test-dir build-release --output-on-failure
```

To install the library, add `-DCMAKE_INSTALL_PREFIX=/your/prefix` during
configuration, then run `cmake --install build-release`. Consumers can use
`find_package(rocontrol CONFIG REQUIRED)` and link `rocontrol::rocontrol`.

## Runtime contract

Before the first command, create a fixed-base `RobotModel`, map external joint
names to `RobotModel::dof_names()` order, update it with a fresh initial state,
configure tasks, and call `Controller::start()`. The first cycle is a
synchronization cycle and emits no command. On each later cycle, validate the
state timestamp and measured period, update the model, update the FSM, then call
`Controller::update()` with a preallocated command vector. Only the control
thread may modify tasks or the FSM. A failed update produces no valid command;
the application must choose its safety response.

The command is joint velocity. An adapter for position-only hardware must
provide integration, interpolation, output rate limits, and a command timeout.

## Optional simulation and benchmark

Set `-DROCONTROL_BUILD_ROMUJOCO_EXAMPLE=ON` and include an installed
`romujoco` package in `CMAKE_PREFIX_PATH` to build a closed-loop integration
example. This dependency stays outside the core target.

`rocontrol_benchmark` measures a 15-DoF model with two Cartesian tasks and one
joint task for 300,000 cycles. Its default mode measures compute time using a
2 ms simulated period. `--realtime` also schedules those cycles at 500 Hz for
10 minutes using `sleep_until`; `--spin` busy waits for each wall-clock cycle
to measure another scheduling strategy. The benchmark prints p50, p95, p99,
maximum duration, and C++
allocation counts for the model, FSM, QP, Controller, and full core cycle.
The measured results and host limitations are in
[docs/benchmark_report.md](docs/benchmark_report.md).

`rocontrol_runtime_example` demonstrates the startup synchronization cycle,
fresh-state checks, measured period use, and 250/500/1000 Hz control semantics.
To run the optional Mobile FR3 Duo description test, configure CMake with
`-DROCONTROL_INTEGRATION_URDF=/path/to/mfr3duo.urdf`.
For the optional single FR3 and FR3 Duo tests, generate URDFs from
`franka_description` with `xacro` (using `hand:=false`), then set
`ROCONTROL_FR3_URDF` and `ROCONTROL_FR3_DUO_URDF` respectively.

## Runnable control examples

The test executables are also small closed-loop usage examples. CTest runs
each when its model description is configured:

| Scenario | Executable |
| --- | --- |
| Single-arm Joint, 250/500/1000 Hz Runtime contract | `rocontrol_runtime_example` |
| Single FR3 Joint and Cartesian | `rocontrol_fr3_description_test` |
| Two simultaneous Cartesian arms | `rocontrol_fr3_duo_description_test` |
| Mobile dual arm plus spine | `rocontrol_real_description_test` |
| FSM with trajectory and settling timeout | `rocontrol_fsm_trajectory_test` |
| RoMuJoCo closed loop | `rocontrol_romujoco_example` |
| RoMuJoCo Cartesian pose closed loop | `rocontrol_romujoco_cartesian_example` |

`rocontrol_dual_arm_test` provides a self-contained 15-DoF synthetic model when
the external robot descriptions are unavailable.
