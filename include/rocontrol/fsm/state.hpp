#pragma once

#include <rocontrol/cycle.hpp>
#include <rocontrol/model/robot_model.hpp>
#include <rocontrol/result.hpp>
#include <rocontrol/types.hpp>

namespace rocontrol {

/** Result of one State update; terminal results commit at the next cycle. */
enum class StateStatus { Running, Succeeded, Failed };
/** Recoverable State and transition lifecycle failures. */
enum class FsmError {
    InvalidState, InvalidTransition, EnterFailed, UpdateFailed,
    ExitFailed, TaskUpdateFailed, TrajectoryFailed
};

/** Current state and model snapshot; model.update(state) precedes every call. */
struct StateContext {
    /** Current fresh measured state in canonical DoF order. */
    const RobotState& state;
    /** Model already updated from state. */
    const RobotModel& model;
    /** Current measured cycle, or startup synchronization cycle. */
    const ControlCycle& cycle;
};

/** A behavior stage. Each method runs on the control thread. */
class State {
public:
    /** Destroy a polymorphic behavior stage. */
    virtual ~State() = default;
    /** Initialize stage-local references when this state becomes active. */
    virtual Result<void, FsmError> enter(const StateContext& context) = 0;
    /** Update targets and return one stage status on the control thread. */
    virtual Result<StateStatus, FsmError> update(const StateContext& context) = 0;
    /** Disable owned tasks and release stage-local effects on transition. */
    virtual Result<void, FsmError> exit(const StateContext& context) = 0;
};

}  // namespace rocontrol
