#include <rocontrol/controller.hpp>
#include <rocontrol/solver/qp_solver.hpp>

#include <chrono>
#include <cmath>
#include <iostream>

namespace {

// Runtime-side validation. The Core assumes a fresh, synchronized state.
bool valid_sample(const rocontrol::RobotState& state,
                  const rocontrol::ControlCycle& cycle,
                  std::chrono::steady_clock::time_point previous_stamp,
                  const rocontrol::ControllerConfig& config,
                  Eigen::Index dof) {
    const auto age = cycle.time - state.stamp;
    const double period = cycle.period.count();
    return state.stamp > previous_stamp
        && age >= std::chrono::steady_clock::duration::zero()
        && age <= config.max_state_age
        && std::isfinite(period) && period > 0 && period <= config.max_period.count()
        && state.position.size() == dof && state.velocity.size() == dof
        && state.position.allFinite() && state.velocity.allFinite();
}

bool run_rate(double period_seconds) {
    rocontrol::RobotModelConfig model_config;
    model_config.urdf = TEST_URDF_PATH;
    model_config.joint_groups["arm"] = {"joint1", "spin"};
    auto created = rocontrol::RobotModel::create(model_config);
    if (!created) return false;
    auto model = std::move(created).value();

    rocontrol::ControllerConfig controller_config;
    controller_config.nominal_period = std::chrono::duration<double>(period_seconds);
    controller_config.max_period = 2 * controller_config.nominal_period;
    controller_config.max_state_age = controller_config.max_period;
    rocontrol::QPSolver solver;
    rocontrol::Controller controller(*model, solver, controller_config);
    auto added = controller.add_joint_position_task(model->joint_group("arm").value());
    if (!added) return false;
    Eigen::Vector2d target(0.1, 0.1);
    if (!added.value().get().set_target(target)) return false;
    added.value().get().set_enabled(true);

    rocontrol::RobotState state;
    state.position = Eigen::Vector2d::Zero();
    state.velocity = Eigen::Vector2d::Zero();
    state.stamp = std::chrono::steady_clock::time_point{} + std::chrono::seconds(1);
    if (!model->update(state) || !controller.start(state)) return false;

    // The synchronization sample above intentionally produced no command.
    rocontrol::JointVelocityCommand command;
    command.velocity.setZero(model->dof());
    auto previous_stamp = state.stamp;
    auto previous_cycle_time = state.stamp;
    const auto nominal = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        controller_config.nominal_period);
    const auto total_cycles = static_cast<std::uint64_t>(2.0 / period_seconds);
    for (std::uint64_t sequence = 1; sequence <= total_cycles; ++sequence) {
        const auto actual = sequence == 10 ? 2 * nominal : nominal;
        rocontrol::ControlCycle cycle{previous_cycle_time + actual, actual, sequence};
        state.stamp = cycle.time - std::chrono::microseconds(100);
        if (!valid_sample(state, cycle, previous_stamp, controller_config, model->dof())) {
            return false;
        }
        if (!model->update(state) || !controller.update(state, cycle, command)) return false;
        if (command.sequence != sequence || command.stamp != cycle.time) return false;
        state.position += command.velocity * cycle.period.count();
        state.velocity = command.velocity;
        previous_stamp = state.stamp;
        previous_cycle_time = cycle.time;
    }

    rocontrol::ControlCycle invalid{previous_cycle_time + nominal, nominal, total_cycles + 1};
    state.stamp = previous_stamp;
    if (valid_sample(state, invalid, previous_stamp, controller_config, model->dof())) return false;
    state.stamp = invalid.time + std::chrono::microseconds(1);
    if (valid_sample(state, invalid, previous_stamp, controller_config, model->dof())) return false;
    state.stamp = invalid.time
        - std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            controller_config.max_state_age)
        - std::chrono::microseconds(1);
    if (valid_sample(state, invalid, previous_stamp, controller_config, model->dof())) return false;
    invalid.period = controller_config.max_period + std::chrono::microseconds(1);
    state.stamp = invalid.time;
    if (valid_sample(state, invalid, previous_stamp, controller_config, model->dof())) return false;
    return (state.position - target).norm() < 0.03;
}

}  // namespace

int main() {
    for (const double period : {0.001, 0.002, 0.004}) {
        if (!run_rate(period)) return 1;
    }
    std::cout << "Runtime contract: 250, 500, and 1000 Hz passed\n";
}
