#include <rocontrol/controller.hpp>
#include <rocontrol/fsm/state_machine.hpp>
#include <rocontrol/solver/qp_solver.hpp>
#include <rocontrol/trajectory/joint_position_trajectory.hpp>

#undef NDEBUG
#include <cassert>
#include <chrono>
#include <memory>

namespace {

class MoveState final : public rocontrol::State {
public:
    MoveState(rocontrol::JointPositionTask& task, Eigen::Vector2d destination)
        : task_(task), destination_(std::move(destination)), trajectory_(2) {
        reference_.position.setZero(2);
        reference_.velocity.setZero(2);
    }

    rocontrol::Result<void, rocontrol::FsmError> enter(
        const rocontrol::StateContext& context) override {
        started_ = context.cycle.time;
        if (!trajectory_.reset(context.state.position, destination_,
                std::chrono::seconds(2), started_)) {
            return rocontrol::Result<void, rocontrol::FsmError>::failure(
                rocontrol::FsmError::TrajectoryFailed);
        }
        task_.set_enabled(true);
        return rocontrol::Result<void, rocontrol::FsmError>::success();
    }

    rocontrol::Result<rocontrol::StateStatus, rocontrol::FsmError> update(
        const rocontrol::StateContext& context) override {
        if (!trajectory_.sample(context.cycle.time, reference_)) {
            return rocontrol::Result<rocontrol::StateStatus, rocontrol::FsmError>::failure(
                rocontrol::FsmError::TrajectoryFailed);
        }
        if (!task_.set_target(reference_.position)
            || !task_.set_reference_velocity(reference_.velocity)) {
            return rocontrol::Result<rocontrol::StateStatus, rocontrol::FsmError>::failure(
                rocontrol::FsmError::TaskUpdateFailed);
        }
        if (trajectory_.finished(context.cycle.time)
            && (context.state.position - destination_).norm() < 0.02) {
            return rocontrol::Result<rocontrol::StateStatus, rocontrol::FsmError>::success(
                rocontrol::StateStatus::Succeeded);
        }
        if (context.cycle.time > started_ + std::chrono::seconds(3)) {
            return rocontrol::Result<rocontrol::StateStatus, rocontrol::FsmError>::success(
                rocontrol::StateStatus::Failed);
        }
        return rocontrol::Result<rocontrol::StateStatus, rocontrol::FsmError>::success(
            rocontrol::StateStatus::Running);
    }

    rocontrol::Result<void, rocontrol::FsmError> exit(
        const rocontrol::StateContext&) override {
        task_.set_enabled(false);
        task_.clear_reference_velocity();
        return rocontrol::Result<void, rocontrol::FsmError>::success();
    }

private:
    rocontrol::JointPositionTask& task_;
    Eigen::Vector2d destination_;
    rocontrol::JointPositionTrajectory trajectory_;
    rocontrol::JointPositionReference reference_;
    std::chrono::steady_clock::time_point started_;
};

}  // namespace

int main() {
    rocontrol::RobotModelConfig config;
    config.urdf = TEST_URDF_PATH;
    config.joint_groups["arm"] = {"joint1", "spin"};
    auto created = rocontrol::RobotModel::create(config);
    assert(created);
    auto model = std::move(created).value();
    rocontrol::RobotState state;
    state.stamp = std::chrono::steady_clock::now();
    state.position = Eigen::Vector2d::Zero();
    state.velocity = Eigen::Vector2d::Zero();
    assert(model->update(state));

    rocontrol::QPSolver solver;
    rocontrol::Controller controller(*model, solver);
    auto added = controller.add_joint_position_task(model->joint_group("arm").value());
    assert(added);
    assert(controller.start(state));
    rocontrol::StateMachine fsm;
    auto move = fsm.add_state(std::make_unique<MoveState>(
        added.value().get(), Eigen::Vector2d(0.5, 0.2)));
    assert(move);
    rocontrol::ControlCycle cycle{state.stamp, std::chrono::duration<double>(0), 0};
    rocontrol::StateContext context{state, *model, cycle};
    assert(fsm.start(move.value(), context));
    rocontrol::JointVelocityCommand command;
    command.velocity.setZero(model->dof());
    cycle.period = std::chrono::duration<double>(0.002);
    Eigen::internal::set_is_malloc_allowed(false);
    for (int step = 0; step < 2000 && fsm.status() == rocontrol::FsmStatus::Running; ++step) {
        state.stamp += std::chrono::milliseconds(2);
        cycle.time = state.stamp;
        cycle.sequence = step + 1;
        assert(model->update(state));
        auto result = fsm.update(context);
        assert(result);
        if (result.value() != rocontrol::FsmStatus::Running) break;
        assert(controller.update(state, cycle, command));
        state.position += command.velocity * cycle.period.count();
        state.velocity = command.velocity;
    }
    Eigen::internal::set_is_malloc_allowed(true);
    assert(fsm.status() == rocontrol::FsmStatus::Completed);
    assert((state.position - Eigen::Vector2d(0.5, 0.2)).norm() < 0.02);
    assert(!added.value().get().enabled());
}
