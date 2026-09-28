#pragma once

#include <rocontrol/constraint/constraint.hpp>
#include <rocontrol/cycle.hpp>
#include <rocontrol/model/robot_model.hpp>
#include <rocontrol/result.hpp>
#include <rocontrol/solver/solver.hpp>
#include <rocontrol/task/joint_position_task.hpp>
#include <rocontrol/task/cartesian_pose_task.hpp>
#include <rocontrol/types.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include <vector>

namespace rocontrol {

/** Control timing and hard-limit policy; max_state_age is enforced by Runtime. */
struct ControllerConfig {
    /** Requested interval between cycles, in seconds; default 500 Hz. */
    std::chrono::duration<double> nominal_period{0.002};
    /** Largest accepted measured period, in seconds. */
    std::chrono::duration<double> max_period{0.004};
    /** Largest state age accepted by Runtime, in seconds. */
    std::chrono::duration<double> max_state_age{0.004};
    /** Enable one-step hard position bounds. */
    bool enforce_position_limits{true};
    /** Enable hard joint velocity bounds. */
    bool enforce_velocity_limits{true};
    /** Allowed measured position excess before StateOutOfBounds, in native joint units. */
    double joint_limit_tolerance{1e-6};
    /** Positive diagonal Hessian regularization. */
    double qp_regularization{1e-6};
};

/** Recoverable controller and QP orchestration failures. */
enum class ControlError {
    InvalidCyclePeriod,
    InvalidState,
    StateOutOfBounds,
    ModelNotUpdated,
    InvalidModelReference,
    InvalidTaskConfiguration,
    AlreadyStarted,
    NotStarted,
    SolverInfeasible,
    SolverFailure,
    NumericalFailure
};

/** One-cycle orchestration; RobotModel must be updated before update(). */
class Controller {
public:
    /** Borrow model and solver; both must outlive this controller. */
    Controller(RobotModel& model, Solver& solver, ControllerConfig config = {});
    /** Add a joint objective in group order before start(). */
    Result<std::reference_wrapper<JointPositionTask>, ControlError>
    add_joint_position_task(const JointGroup& group);
    /** Add a root-frame Cartesian objective for a valid model frame. */
    Result<std::reference_wrapper<CartesianPoseTask>, ControlError>
    add_cartesian_pose_task(FrameIndex frame);
    /** Finalize topology and initialize unset targets to current positions. */
    Result<void, ControlError> start(const RobotState& state);
    /** Solve one cycle using actual period; failure invalidates sequence and clears a correctly sized buffer. */
    Result<void, ControlError> update(
        const RobotState& state, const ControlCycle& cycle,
        JointVelocityCommand& output);
    /** Stop updates; a later start() reuses the same tasks and solver workspace. */
    void stop() noexcept { started_ = false; }

private:
    RobotModel& model_;
    Solver& solver_;
    ControllerConfig config_;
    std::vector<std::unique_ptr<Task>> tasks_;
    std::vector<std::unique_ptr<Constraint>> constraints_;
    bool built_in_constraints_added_{false};
    bool started_{false};
};

}  // namespace rocontrol
