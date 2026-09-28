#include <rocontrol/controller.hpp>
#include <rocontrol/solver/qp_solver.hpp>
#include <romujoco/simulation.hpp>

#include <chrono>
#include <cmath>
#include <iostream>

int main() {
    rocontrol::RobotModelConfig model_config;
    model_config.urdf = TEST_URDF_PATH;
    model_config.joint_groups["arm"] = {"joint"};
    auto created = rocontrol::RobotModel::create(model_config);
    if (!created) return 1;
    auto model = std::move(created).value();

    romujoco::SimulationConfig simulation_config;
    simulation_config.model.model_path = TEST_MJCF_PATH;
    simulation_config.scheduler.physics_period = 0.001;
    simulation_config.viewer_enabled = false;
    romujoco::JointInfo joint;
    joint.id = 0;
    joint.joint_name = "joint";
    joint.actuator_name = "motor";
    joint.default_mode = romujoco::JointMode::Velocity;
    joint.allowed_modes = {romujoco::JointMode::Velocity};
    joint.velocity.damping = 20;
    joint.period = 0.001;
    simulation_config.components.push_back(joint);

    romujoco::Simulation simulation;
    if (!simulation.initialize(simulation_config)) {
        std::cerr << "RoMuJoCo initialization failed\n";
        return 1;
    }

    rocontrol::RobotState state;
    state.position.setZero(model->dof());
    state.velocity.setZero(model->dof());
    auto initial_time = std::chrono::steady_clock::now();
    state.stamp = initial_time;
    if (!model->update(state)) return 1;

    rocontrol::QPSolver solver;
    rocontrol::Controller controller(*model, solver);
    auto group = model->joint_group("arm");
    if (!group) return 1;
    auto added = controller.add_joint_position_task(group.value());
    if (!added) return 1;
    auto& task = added.value().get();
    Eigen::VectorXd goal(1);
    goal[0] = 0.5;
    if (!task.set_target(goal)) return 1;
    task.set_enabled(true);
    if (!controller.start(state)) return 1;

    rocontrol::JointVelocityCommand command;
    command.velocity.setZero(model->dof());
    rocontrol::ControlCycle cycle;
    cycle.period = std::chrono::duration<double>(0.002);
    for (int tick = 0; tick < 2500; ++tick) {
        romujoco::JointState measured;
        measured.id = joint.id;
        if (!simulation.read_state(measured)) return 1;
        state.position[0] = measured.position;
        state.velocity[0] = measured.velocity;
        state.stamp = initial_time + std::chrono::milliseconds(2 * tick + 2);
        cycle.time = state.stamp;
        cycle.sequence = tick + 1;
        if (!model->update(state) || !controller.update(state, cycle, command)) return 1;

        romujoco::JointCommand native;
        native.id = joint.id;
        native.mode = static_cast<std::uint8_t>(romujoco::JointMode::Velocity);
        native.velocity = command.velocity[0];
        if (!simulation.write_command(native) || !simulation.step(2)) return 1;
    }
    romujoco::JointState measured;
    measured.id = joint.id;
    if (!simulation.read_state(measured)) return 1;
    std::cout << "final joint position: " << measured.position << '\n';
    return std::abs(measured.position - 0.5) < 0.05 ? 0 : 1;
}
