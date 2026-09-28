#include <rocontrol/controller.hpp>
#include <rocontrol/solver/qp_solver.hpp>
#include <romujoco/simulation.hpp>

#include <chrono>
#include <cmath>
#include <iostream>
#include <string>

int main() {
    rocontrol::RobotModelConfig model_config;
    model_config.urdf = TEST_URDF_PATH;
    auto created = rocontrol::RobotModel::create(model_config);
    if (!created) return 1;
    auto model = std::move(created).value();
    const auto tool = model->frame("tool");
    if (!tool || model->dof() != 2) return 1;

    rocontrol::RobotState state;
    state.position = Eigen::Vector2d(0.2, 0.15);
    state.velocity = Eigen::Vector2d::Zero();
    auto initial_time = std::chrono::steady_clock::now();
    state.stamp = initial_time;
    if (!model->update(state)) return 1;
    const auto target = model->frame_pose(tool.value());
    state.position.setZero();
    if (!model->update(state)) return 1;

    romujoco::SimulationConfig simulation_config;
    simulation_config.model.model_path = TEST_MJCF_PATH;
    simulation_config.scheduler.physics_period = 0.001;
    simulation_config.viewer_enabled = false;
    for (std::size_t index = 0; index < 2; ++index) {
        romujoco::JointInfo joint;
        joint.id = index;
        joint.joint_name = "joint" + std::to_string(index + 1);
        joint.actuator_name = "motor" + std::to_string(index + 1);
        joint.default_mode = romujoco::JointMode::Velocity;
        joint.allowed_modes = {romujoco::JointMode::Velocity};
        joint.velocity.damping = 20;
        joint.period = 0.001;
        simulation_config.components.push_back(joint);
    }
    romujoco::Simulation simulation;
    if (!simulation.initialize(simulation_config)) return 1;

    rocontrol::QPSolver solver;
    rocontrol::Controller controller(*model, solver);
    auto task = controller.add_cartesian_pose_task(tool.value());
    if (!task || !task.value().get().set_target(target)) return 1;
    task.value().get().set_enabled(true);
    if (!controller.start(state)) return 1;
    rocontrol::JointVelocityCommand command;
    command.velocity.setZero(model->dof());
    rocontrol::ControlCycle cycle;
    cycle.period = std::chrono::duration<double>(0.002);
    for (int tick = 0; tick < 2500; ++tick) {
        for (std::size_t index = 0; index < 2; ++index) {
            romujoco::JointState measured;
            measured.id = index;
            if (!simulation.read_state(measured)) return 1;
            state.position[index] = measured.position;
            state.velocity[index] = measured.velocity;
        }
        state.stamp = initial_time + std::chrono::milliseconds(2 * tick + 2);
        cycle.time = state.stamp;
        cycle.sequence = tick + 1;
        if (!model->update(state) || !controller.update(state, cycle, command)) return 1;
        for (std::size_t index = 0; index < 2; ++index) {
            romujoco::JointCommand native;
            native.id = index;
            native.mode = static_cast<std::uint8_t>(romujoco::JointMode::Velocity);
            native.velocity = command.velocity[index];
            if (!simulation.write_command(native)) return 1;
        }
        if (!simulation.step(2)) return 1;
    }
    for (std::size_t index = 0; index < 2; ++index) {
        romujoco::JointState measured;
        measured.id = index;
        if (!simulation.read_state(measured)) return 1;
        state.position[index] = measured.position;
    }
    if (!model->update(state)) return 1;
    const auto& actual = model->frame_pose(tool.value());
    const Eigen::Quaterniond error = target.orientation * actual.orientation.conjugate();
    const double position_error = (actual.position - target.position).norm();
    const double orientation_error = Eigen::AngleAxisd(error).angle();
    std::cout << "Cartesian error: position=" << position_error
              << " orientation=" << orientation_error << '\n';
    return position_error < 0.05 && orientation_error < 0.05 ? 0 : 1;
}
