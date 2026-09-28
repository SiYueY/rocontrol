# RoControl V1 performance report

## Reference setup

| Item | Value |
| --- | --- |
| CPU | Intel Core Ultra 5 125H |
| Kernel | Linux 6.8.0-138-generic, Ubuntu 22.04, PREEMPT_DYNAMIC |
| Compiler | GCC 11.4.0, C++17 |
| Build | CMake Release, `-O3 -DNDEBUG`; library also `-Wall -Wextra -Wpedantic` |
| QP backend | RoControl dense box-QP active-set solver |
| Model | Synthetic fixed-base dual arm plus spine, 15 DoF |
| Tasks | 2 CartesianPoseTask, 1 JointPositionTask |
| Hard constraints | Joint position and velocity limits |
| Workload | 300,000 cycles, 500 Hz actual-period input in compute mode |
| Affinity | Real-time attempt pinned to CPU 17 with `taskset` |

## Compute-path result

Command: `./build-release/rocontrol_benchmark`. Durations are milliseconds.

| Stage | p50 | p95 | p99 | max |
| --- | ---: | ---: | ---: | ---: |
| RobotModel update | 0.003143 | 0.003253 | 0.004221 | 0.057904 |
| FSM | 0.000033 | 0.000037 | 0.000040 | 0.036761 |
| QP build and solve | 0.003236 | 0.003785 | 0.004527 | 0.154409 |
| Controller update | 0.003332 | 0.003881 | 0.004662 | 0.154581 |
| Core cycle | 0.006514 | 0.007091 | 0.008639 | 0.158353 |

The 300,000-cycle compute run had zero C++ `new` allocations, solver failures, and
non-finite outputs. Its core cycle p99 and maximum are below the design's 1.0 ms
and 1.5 ms thresholds. This mode advances timestamps by 2 ms without sleeping;
it measures Core compute cost, not host scheduling quality or I/O.

## Ten-minute wall-clock run

Command: `taskset -c 17 ./build-release/rocontrol_benchmark --spin`.
The benchmark busy-waited for each 2 ms deadline on CPU 17 and completed all
300,000 cycles over ten minutes. Durations are milliseconds.

| Stage | p50 | p95 | p99 | max |
| --- | ---: | ---: | ---: | ---: |
| RobotModel update | 0.005958 | 0.006113 | 0.006702 | 0.044312 |
| FSM | 0.000081 | 0.000092 | 0.000105 | 0.001465 |
| QP build and solve | 0.006826 | 0.006930 | 0.007194 | 0.049134 |
| Controller update | 0.007007 | 0.007118 | 0.007446 | 0.049482 |
| Core cycle | 0.013052 | 0.013282 | 0.014120 | 0.056046 |

The run exited successfully with zero C++ `new` allocations, solver failures,
non-finite outputs, and Core-caused deadline misses. Core p99 and maximum are
below 1.0 ms and 1.5 ms. Busy waiting occupied one CPU for the entire run;
this result establishes the tested host and scheduling strategy, not a general
hard real-time guarantee.

## Sleep-based scheduling attempt

Command: `taskset -c 17 ./build-release/rocontrol_benchmark --realtime`.
The run stopped at cycle 58: measured period 5.22807 ms exceeded the default
4 ms `max_period`, so Controller returned `InvalidCyclePeriod` and emitted no
valid command. `chrt -f 50 true` failed with “Operation not permitted” in this
environment. Ordinary sleep-based scheduling did not pass the 500 Hz period
contract on this host. A configured real-time host and command watchdog are
still needed for an independently scheduled robot Runtime.

The benchmark uses a synthetic 15-DoF model for repeatability. Optional
real-description integration tests load local FR3, FR3 Duo, and Mobile FR3 Duo
URDFs. They exercise single-arm Joint and Cartesian closure, simultaneous
dual-arm Cartesian closure, and the mobile spine Joint task. No physical robot
adapter or hardware command path was available to validate native command
semantics.

The optional RoMuJoCo integration suite also runs a joint-velocity closed
loop and a Cartesian pose closed loop against matched URDF/MJCF fixtures.
