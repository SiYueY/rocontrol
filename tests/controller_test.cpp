#include <rocontrol/controller.hpp>
#include <rocontrol/solver/qp_solver.hpp>

#undef NDEBUG
#include <cassert>
#include <chrono>
#include <cmath>
#include <limits>

namespace {

class FailingSolver final : public rocontrol::Solver {
public:
    explicit FailingSolver(bool report_failure) : report_failure_(report_failure) {}

    rocontrol::Result<void, rocontrol::SolverError> prepare(
        const rocontrol::RobotModel&,
        const std::vector<std::unique_ptr<rocontrol::Task>>&,
        const std::vector<std::unique_ptr<rocontrol::Constraint>>&,
        double) override {
        return rocontrol::Result<void, rocontrol::SolverError>::success();
    }
    rocontrol::Result<void, rocontrol::SolverError> solve(
        const rocontrol::RobotModel&,
        const std::vector<std::unique_ptr<rocontrol::Task>>&,
        const std::vector<std::unique_ptr<rocontrol::Constraint>>&,
        const rocontrol::ControlCycle&,
        Eigen::Ref<Eigen::VectorXd> output) override {
        output.setConstant(report_failure_ ? 1.0 : std::numeric_limits<double>::quiet_NaN());
        return report_failure_
            ? rocontrol::Result<void, rocontrol::SolverError>::failure(
                  rocontrol::SolverError::NumericalFailure)
            : rocontrol::Result<void, rocontrol::SolverError>::success();
    }

private:
    bool report_failure_;
};

}  // namespace

int main() {
    rocontrol::RobotModelConfig config;
    config.urdf = TEST_URDF_PATH;
    config.joint_groups["arm"] = {"joint1", "spin"};
    auto created = rocontrol::RobotModel::create(config);
    assert(created);
    auto model = std::move(created).value();
    auto never_updated = rocontrol::RobotModel::create(config);
    assert(never_updated);
    rocontrol::QPSolver prestart_solver;
    rocontrol::Controller prestart_controller(*never_updated.value(), prestart_solver);
    rocontrol::RobotState epoch_state;
    epoch_state.stamp = std::chrono::steady_clock::time_point{};
    epoch_state.position = Eigen::Vector2d::Zero();
    epoch_state.velocity = Eigen::Vector2d::Zero();
    const auto premature = prestart_controller.start(epoch_state);
    assert(!premature && premature.error() == rocontrol::ControlError::ModelNotUpdated);
    rocontrol::QPSolver solver;
    rocontrol::Controller controller(*model, solver);

    const auto group = model->joint_group("arm");
    assert(group);
    rocontrol::JointGroup duplicate{model->id(), {0, 0}};
    assert(!controller.add_joint_position_task(duplicate));
    rocontrol::JointGroup foreign{never_updated.value()->id(), {0}};
    assert(!controller.add_joint_position_task(foreign));
    auto added = controller.add_joint_position_task(group.value());
    assert(added);
    auto& task = added.value().get();
    assert(!task.enabled());
    assert(!task.set_weight(0));
    assert(task.weight() == 1.0);
    assert(task.set_gain(2));
    assert(!task.set_gain(-1) && task.gain() == 2);
    assert(task.set_gain(1));

    rocontrol::RobotState state;
    state.stamp = std::chrono::steady_clock::now();
    state.position = Eigen::Vector2d::Zero();
    state.velocity = Eigen::Vector2d::Zero();
    assert(model->update(state));
    assert(controller.start(state));
    assert(task.has_target() && task.target().isZero());
    assert(!controller.add_joint_position_task(group.value()));

    rocontrol::JointVelocityCommand command;
    command.velocity.setZero(model->dof());
    rocontrol::ControlCycle cycle;
    cycle.time = state.stamp;
    cycle.period = std::chrono::duration<double>(0.002);
    cycle.sequence = 1;
    assert(controller.update(state, cycle, command));
    assert(command.velocity.isZero());

    Eigen::Vector2d target(0.5, 0.25);
    assert(task.set_target(target));
    task.set_enabled(true);
    assert(controller.update(state, cycle, command));
    assert(std::abs(command.velocity[0] - 0.5) < 1e-5);
    assert(std::abs(command.velocity[1] - 0.25) < 1e-5);
    assert(command.sequence == 1 && command.stamp == cycle.time);

    Eigen::VectorXd invalid_target(1);
    invalid_target[0] = 1;
    assert(!task.set_target(invalid_target));
    assert((task.target() - target).norm() == 0);

    state.position[0] = 0.999;
    state.stamp += std::chrono::milliseconds(2);
    cycle.time = state.stamp;
    cycle.sequence = 2;
    target[0] = 2;
    assert(task.set_target(target));
    assert(model->update(state));
    assert(controller.update(state, cycle, command));
    assert(command.velocity[0] <= 0.500001);
    assert(command.velocity[0] >= 0.499);
    cycle.period = std::chrono::duration<double>(0.001);
    assert(controller.update(state, cycle, command));
    assert(command.velocity[0] > 0.999 && command.velocity[0] <= 1.000001);
    cycle.period = std::chrono::duration<double>(0.004);
    assert(controller.update(state, cycle, command));
    assert(command.velocity[0] > 0.249 && command.velocity[0] <= 0.250001);

    cycle.period = std::chrono::duration<double>(0.01);
    assert(!controller.update(state, cycle, command));
    assert(command.sequence == 0 && command.velocity.isZero());
    cycle.period = std::chrono::duration<double>(0.002);
    state.position[0] = 1.1;
    state.stamp += std::chrono::milliseconds(2);
    assert(model->update(state));
    assert(!controller.update(state, cycle, command));

    state.position[0] = 0;
    state.stamp += std::chrono::milliseconds(2);
    assert(model->update(state));
    Eigen::internal::set_is_malloc_allowed(false);
    for (int index = 0; index < 100; ++index) {
        cycle.sequence++;
        cycle.time = state.stamp;
        assert(controller.update(state, cycle, command));
    }
    Eigen::internal::set_is_malloc_allowed(true);

    controller.stop();
    assert(controller.start(state));
    assert(controller.update(state, cycle, command));

    rocontrol::ControllerConfig invalid_config;
    invalid_config.nominal_period = std::chrono::duration<double>(0.01);
    rocontrol::QPSolver other_solver;
    rocontrol::Controller invalid_controller(*model, other_solver, invalid_config);
    assert(!invalid_controller.start(state));

    rocontrol::QPSolver weighted_solver;
    rocontrol::Controller weighted_controller(*model, weighted_solver);
    rocontrol::JointGroup first_joint{model->id(), {model->joint("joint1").value()}};
    auto positive = weighted_controller.add_joint_position_task(first_joint);
    auto negative = weighted_controller.add_joint_position_task(first_joint);
    assert(positive && negative);
    Eigen::VectorXd positive_target(1), negative_target(1);
    positive_target[0] = 0.2;
    negative_target[0] = -0.2;
    assert(positive.value().get().set_target(positive_target));
    assert(negative.value().get().set_target(negative_target));
    assert(negative.value().get().set_weight(3));
    positive.value().get().set_enabled(true);
    negative.value().get().set_enabled(true);
    assert(weighted_controller.start(state));
    assert(weighted_controller.update(state, cycle, command));
    assert(std::abs(command.velocity[0] + 0.1) < 1e-5);

    rocontrol::ControllerConfig tolerant_config;
    tolerant_config.joint_limit_tolerance = 2;
    rocontrol::QPSolver infeasible_solver;
    rocontrol::Controller infeasible_controller(*model, infeasible_solver, tolerant_config);
    assert(infeasible_controller.start(state));
    state.position[0] = 2;
    state.stamp += std::chrono::milliseconds(2);
    assert(model->update(state));
    assert(!infeasible_controller.update(state, cycle, command));
    assert(command.sequence == 0 && command.velocity.isZero());

    state.position.setZero();
    state.stamp += std::chrono::milliseconds(2);
    assert(model->update(state));
    FailingSolver failed_solver(true);
    rocontrol::Controller failed_controller(*model, failed_solver);
    assert(failed_controller.start(state));
    assert(!failed_controller.update(state, cycle, command));
    assert(command.sequence == 0 && command.velocity.isZero());
    FailingSolver nan_solver(false);
    rocontrol::Controller nan_controller(*model, nan_solver);
    assert(nan_controller.start(state));
    assert(!nan_controller.update(state, cycle, command));
    assert(command.sequence == 0 && command.velocity.isZero());
}
