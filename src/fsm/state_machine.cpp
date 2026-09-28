#include <rocontrol/fsm/state_machine.hpp>

#include <limits>
#include <utility>

namespace rocontrol {

Result<StateId, FsmError> StateMachine::add_state(std::unique_ptr<State> state) {
    if (started_ || !state || states_.size() >= std::numeric_limits<StateId>::max()) {
        return Result<StateId, FsmError>::failure(FsmError::InvalidState);
    }
    const auto id = static_cast<StateId>(states_.size());
    states_.push_back({std::move(state), std::nullopt, std::nullopt});
    return Result<StateId, FsmError>::success(id);
}

Result<void, FsmError> StateMachine::add_transition(
    StateId from, StateStatus result, StateId to) {
    if (started_ || from >= states_.size() || to >= states_.size()
        || result == StateStatus::Running) {
        return Result<void, FsmError>::failure(FsmError::InvalidTransition);
    }
    auto& target = result == StateStatus::Succeeded
        ? states_[from].succeeded : states_[from].failed;
    if (target) return Result<void, FsmError>::failure(FsmError::InvalidTransition);
    target = to;
    return Result<void, FsmError>::success();
}

Result<void, FsmError> StateMachine::start(StateId initial, const StateContext& context) {
    if (started_ || initial >= states_.size()) {
        return Result<void, FsmError>::failure(FsmError::InvalidState);
    }
    started_ = true;
    current_ = initial;
    status_ = FsmStatus::Running;
    if (!states_[initial].state->enter(context)) {
        status_ = FsmStatus::Failed;
        current_.reset();
        return Result<void, FsmError>::failure(FsmError::EnterFailed);
    }
    return Result<void, FsmError>::success();
}

Result<void, FsmError> StateMachine::request_transition(StateId target) {
    if (!started_ || status_ != FsmStatus::Running || target >= states_.size()) {
        return Result<void, FsmError>::failure(FsmError::InvalidTransition);
    }
    pending_external_ = target;
    return Result<void, FsmError>::success();
}

Result<FsmStatus, FsmError> StateMachine::update(const StateContext& context) {
    if (!started_) return Result<FsmStatus, FsmError>::failure(FsmError::InvalidState);
    if (status_ != FsmStatus::Running) {
        return Result<FsmStatus, FsmError>::success(status_);
    }

    if (pending_external_ || pending_automatic_) {
        const StateId next = pending_external_ ? *pending_external_ : *pending_automatic_;
        pending_external_.reset();
        pending_automatic_.reset();
        pending_terminal_.reset();
        if (!states_[*current_].state->exit(context)) {
            status_ = FsmStatus::Failed;
            current_.reset();
            return Result<FsmStatus, FsmError>::failure(FsmError::ExitFailed);
        }
        current_ = next;
        if (!states_[next].state->enter(context)) {
            status_ = FsmStatus::Failed;
            current_.reset();
            return Result<FsmStatus, FsmError>::failure(FsmError::EnterFailed);
        }
    } else if (pending_terminal_) {
        if (!states_[*current_].state->exit(context)) {
            status_ = FsmStatus::Failed;
            current_.reset();
            return Result<FsmStatus, FsmError>::failure(FsmError::ExitFailed);
        }
        status_ = *pending_terminal_;
        pending_terminal_.reset();
        current_.reset();
        return Result<FsmStatus, FsmError>::success(status_);
    }

    const auto result = states_[*current_].state->update(context);
    if (!result) {
        status_ = FsmStatus::Failed;
        current_.reset();
        return Result<FsmStatus, FsmError>::failure(result.error());
    }
    if (result.value() != StateStatus::Running) {
        const auto& next = result.value() == StateStatus::Succeeded
            ? states_[*current_].succeeded : states_[*current_].failed;
        if (next) {
            pending_automatic_ = next;
        } else {
            pending_terminal_ = result.value() == StateStatus::Succeeded
                ? FsmStatus::Completed : FsmStatus::Failed;
        }
    }
    return Result<FsmStatus, FsmError>::success(FsmStatus::Running);
}

}  // namespace rocontrol
