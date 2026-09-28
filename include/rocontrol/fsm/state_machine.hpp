#pragma once

#include <rocontrol/fsm/state.hpp>

#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

namespace rocontrol {

/** Index assigned to a State when added before start(). */
using StateId = std::uint32_t;
/** Execution status of the complete state machine. */
enum class FsmStatus { Running, Completed, Failed };

/** Fixed-topology FSM. Transitions commit at the next control-cycle boundary. */
class StateMachine {
public:
    /** Transfer ownership of a State before start(). */
    Result<StateId, FsmError> add_state(std::unique_ptr<State> state);
    /** Map a terminal result to a next State before start(). */
    Result<void, FsmError> add_transition(
        StateId from, StateStatus result, StateId to);
    /** Enter initial State in the synchronization cycle without update(). */
    Result<void, FsmError> start(StateId initial, const StateContext& context);
    /** Commit a pending transition, then update the active State once. */
    Result<FsmStatus, FsmError> update(const StateContext& context);
    /** Control-thread-only request; overrides an automatic transition at the next boundary. */
    Result<void, FsmError> request_transition(StateId target);
    /** Currently active State, if execution is running. */
    std::optional<StateId> current_state() const noexcept { return current_; }
    /** Running, Completed, or Failed execution status. */
    FsmStatus status() const noexcept { return status_; }

private:
    struct Node {
        std::unique_ptr<State> state;
        std::optional<StateId> succeeded;
        std::optional<StateId> failed;
    };
    std::vector<Node> states_;
    std::optional<StateId> current_;
    std::optional<StateId> pending_automatic_;
    std::optional<StateId> pending_external_;
    std::optional<FsmStatus> pending_terminal_;
    FsmStatus status_{FsmStatus::Completed};
    bool started_{false};
};

}  // namespace rocontrol
