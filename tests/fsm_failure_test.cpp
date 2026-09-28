#include <rocontrol/fsm/state_machine.hpp>

#undef NDEBUG
#include <cassert>
#include <chrono>
#include <memory>

namespace {

class TestState final : public rocontrol::State {
public:
    TestState(int& enters, int& updates, int& exits,
              rocontrol::StateStatus result = rocontrol::StateStatus::Running,
              bool fail_enter = false, bool fail_update = false, bool fail_exit = false)
        : enters_(enters), updates_(updates), exits_(exits), result_(result),
          fail_enter_(fail_enter), fail_update_(fail_update), fail_exit_(fail_exit) {}

    rocontrol::Result<void, rocontrol::FsmError>
    enter(const rocontrol::StateContext&) override {
        ++enters_;
        if (fail_enter_) {
            return rocontrol::Result<void, rocontrol::FsmError>::failure(
                rocontrol::FsmError::EnterFailed);
        }
        return rocontrol::Result<void, rocontrol::FsmError>::success();
    }
    rocontrol::Result<rocontrol::StateStatus, rocontrol::FsmError>
    update(const rocontrol::StateContext&) override {
        ++updates_;
        if (fail_update_) {
            return rocontrol::Result<rocontrol::StateStatus, rocontrol::FsmError>::failure(
                rocontrol::FsmError::UpdateFailed);
        }
        return rocontrol::Result<rocontrol::StateStatus, rocontrol::FsmError>::success(result_);
    }
    rocontrol::Result<void, rocontrol::FsmError>
    exit(const rocontrol::StateContext&) override {
        ++exits_;
        if (fail_exit_) {
            return rocontrol::Result<void, rocontrol::FsmError>::failure(
                rocontrol::FsmError::ExitFailed);
        }
        return rocontrol::Result<void, rocontrol::FsmError>::success();
    }

private:
    int& enters_;
    int& updates_;
    int& exits_;
    rocontrol::StateStatus result_;
    bool fail_enter_;
    bool fail_update_;
    bool fail_exit_;
};

}  // namespace

int main() {
    rocontrol::RobotModelConfig config;
    config.urdf = TEST_URDF_PATH;
    auto created = rocontrol::RobotModel::create(config);
    assert(created);
    auto model = std::move(created).value();
    rocontrol::RobotState state;
    state.stamp = std::chrono::steady_clock::now();
    state.position = Eigen::Vector2d::Zero();
    state.velocity = Eigen::Vector2d::Zero();
    assert(model->update(state));
    rocontrol::ControlCycle cycle{state.stamp, std::chrono::duration<double>(0), 0};
    rocontrol::StateContext context{state, *model, cycle};

    int a_enter = 0, a_update = 0, a_exit = 0;
    int b_enter = 0, b_update = 0, b_exit = 0;
    rocontrol::StateMachine enter_failure;
    auto a = enter_failure.add_state(std::make_unique<TestState>(
        a_enter, a_update, a_exit, rocontrol::StateStatus::Running, true));
    assert(a);
    auto started = enter_failure.start(a.value(), context);
    assert(!started && started.error() == rocontrol::FsmError::EnterFailed);
    assert(enter_failure.status() == rocontrol::FsmStatus::Failed);
    assert(!enter_failure.current_state() && a_enter == 1 && a_update == 0);

    rocontrol::StateMachine exit_failure;
    a = exit_failure.add_state(std::make_unique<TestState>(
        a_enter, a_update, a_exit, rocontrol::StateStatus::Succeeded,
        false, false, true));
    auto b = exit_failure.add_state(std::make_unique<TestState>(
        b_enter, b_update, b_exit));
    assert(a && b);
    assert(exit_failure.add_transition(a.value(), rocontrol::StateStatus::Succeeded, b.value()));
    assert(exit_failure.start(a.value(), context));
    assert(exit_failure.update(context));
    auto updated = exit_failure.update(context);
    assert(!updated && updated.error() == rocontrol::FsmError::ExitFailed);
    assert(exit_failure.status() == rocontrol::FsmStatus::Failed);
    assert(!exit_failure.current_state() && b_enter == 0);

    rocontrol::StateMachine next_enter_failure;
    a = next_enter_failure.add_state(std::make_unique<TestState>(
        a_enter, a_update, a_exit, rocontrol::StateStatus::Succeeded));
    b = next_enter_failure.add_state(std::make_unique<TestState>(
        b_enter, b_update, b_exit, rocontrol::StateStatus::Running, true));
    assert(a && b);
    assert(next_enter_failure.add_transition(
        a.value(), rocontrol::StateStatus::Succeeded, b.value()));
    assert(next_enter_failure.start(a.value(), context));
    assert(next_enter_failure.update(context));
    updated = next_enter_failure.update(context);
    assert(!updated && updated.error() == rocontrol::FsmError::EnterFailed);
    assert(next_enter_failure.status() == rocontrol::FsmStatus::Failed);
    assert(!next_enter_failure.current_state());

    rocontrol::StateMachine missing_failure_transition;
    a = missing_failure_transition.add_state(std::make_unique<TestState>(
        a_enter, a_update, a_exit, rocontrol::StateStatus::Failed));
    assert(a && missing_failure_transition.start(a.value(), context));
    assert(missing_failure_transition.update(context).value() == rocontrol::FsmStatus::Running);
    assert(missing_failure_transition.update(context).value() == rocontrol::FsmStatus::Failed);
    assert(!missing_failure_transition.current_state());

    rocontrol::StateMachine update_failure;
    a = update_failure.add_state(std::make_unique<TestState>(
        a_enter, a_update, a_exit, rocontrol::StateStatus::Running,
        false, true));
    assert(a && update_failure.start(a.value(), context));
    updated = update_failure.update(context);
    assert(!updated && updated.error() == rocontrol::FsmError::UpdateFailed);
    assert(update_failure.status() == rocontrol::FsmStatus::Failed);
}
