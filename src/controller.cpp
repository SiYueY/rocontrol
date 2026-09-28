#include <rocontrol/controller.hpp>

#include <algorithm>
#include <cmath>

namespace rocontrol {

Controller::Controller(RobotModel& model, Solver& solver, ControllerConfig config)
    : model_(model), solver_(solver), config_(config) {}

Result<std::reference_wrapper<JointPositionTask>, ControlError>
Controller::add_joint_position_task(const JointGroup& group) {
    if (started_) {
        return Result<std::reference_wrapper<JointPositionTask>, ControlError>::failure(
            ControlError::AlreadyStarted);
    }
    if (group.model != model_.id() || group.joints.empty()) {
        return Result<std::reference_wrapper<JointPositionTask>, ControlError>::failure(
            ControlError::InvalidModelReference);
    }
    for (std::size_t local = 0; local < group.joints.size(); ++local) {
        const auto index = group.joints[local];
        if (index >= model_.dof()
            || std::find(group.joints.begin(), group.joints.begin() + local, index)
                != group.joints.begin() + local) {
            return Result<std::reference_wrapper<JointPositionTask>, ControlError>::failure(
                ControlError::InvalidModelReference);
        }
    }
    auto task = std::make_unique<JointPositionTask>(group);
    auto& reference = *task;
    tasks_.push_back(std::move(task));
    return Result<std::reference_wrapper<JointPositionTask>, ControlError>::success(
        std::ref(reference));
}

Result<std::reference_wrapper<CartesianPoseTask>, ControlError>
Controller::add_cartesian_pose_task(FrameIndex frame) {
    if (started_) {
        return Result<std::reference_wrapper<CartesianPoseTask>, ControlError>::failure(
            ControlError::AlreadyStarted);
    }
    if (!model_.valid_frame(frame)) {
        return Result<std::reference_wrapper<CartesianPoseTask>, ControlError>::failure(
            ControlError::InvalidModelReference);
    }
    auto task = std::make_unique<CartesianPoseTask>(frame);
    auto& reference = *task;
    tasks_.push_back(std::move(task));
    return Result<std::reference_wrapper<CartesianPoseTask>, ControlError>::success(
        std::ref(reference));
}

Result<void, ControlError> Controller::start(const RobotState& state) {
    if (started_) return Result<void, ControlError>::failure(ControlError::AlreadyStarted);
    if (!std::isfinite(config_.nominal_period.count())
        || config_.nominal_period.count() <= 0
        || !std::isfinite(config_.max_period.count())
        || config_.max_period < config_.nominal_period
        || !std::isfinite(config_.max_state_age.count())
        || config_.max_state_age.count() < 0
        || !std::isfinite(config_.joint_limit_tolerance)
        || config_.joint_limit_tolerance < 0
        || !std::isfinite(config_.qp_regularization)
        || config_.qp_regularization <= 0) {
        return Result<void, ControlError>::failure(ControlError::InvalidTaskConfiguration);
    }
    if (!model_.has_state() || model_.last_state_stamp() != state.stamp) {
        return Result<void, ControlError>::failure(ControlError::ModelNotUpdated);
    }
    if (state.position.size() != static_cast<Eigen::Index>(model_.dof())
        || state.velocity.size() != static_cast<Eigen::Index>(model_.dof())
        || !state.position.allFinite() || !state.velocity.allFinite()) {
        return Result<void, ControlError>::failure(ControlError::InvalidState);
    }
    if (!built_in_constraints_added_) {
        if (config_.enforce_position_limits) {
            constraints_.push_back(std::make_unique<JointPositionLimitConstraint>());
        }
        if (config_.enforce_velocity_limits) {
            constraints_.push_back(std::make_unique<JointVelocityLimitConstraint>());
        }
        built_in_constraints_added_ = true;
    }
    for (const auto& task : tasks_) {
        auto* joint = dynamic_cast<JointPositionTask*>(task.get());
        if (joint && !joint->has_target()) {
            Eigen::VectorXd target(joint->group().joints.size());
            for (Eigen::Index index = 0; index < target.size(); ++index) {
                target[index] = state.position[joint->group().joints[index]];
            }
            if (!joint->set_target(target)) {
                return Result<void, ControlError>::failure(ControlError::InvalidTaskConfiguration);
            }
        }
        auto* cartesian = dynamic_cast<CartesianPoseTask*>(task.get());
        if (cartesian && !cartesian->has_target()) {
            if (!cartesian->set_target(model_.frame_pose(cartesian->frame()))) {
                return Result<void, ControlError>::failure(ControlError::InvalidTaskConfiguration);
            }
        }
    }
    const auto prepared = solver_.prepare(
        model_, tasks_, constraints_, config_.qp_regularization);
    if (!prepared) return Result<void, ControlError>::failure(ControlError::SolverFailure);
    started_ = true;
    return Result<void, ControlError>::success();
}

Result<void, ControlError> Controller::update(
    const RobotState& state, const ControlCycle& cycle,
    JointVelocityCommand& output) {
    output.sequence = 0;
    output.stamp = {};
    if (output.velocity.size() == static_cast<Eigen::Index>(model_.dof())) {
        output.velocity.setZero();
    }
    if (!started_) return Result<void, ControlError>::failure(ControlError::NotStarted);
    const double dt = cycle.period.count();
    if (!std::isfinite(dt) || dt <= 0 || dt > config_.max_period.count()) {
        return Result<void, ControlError>::failure(ControlError::InvalidCyclePeriod);
    }
    if (state.position.size() != static_cast<Eigen::Index>(model_.dof())
        || state.velocity.size() != static_cast<Eigen::Index>(model_.dof())
        || !state.position.allFinite() || !state.velocity.allFinite()
        || output.velocity.size() != static_cast<Eigen::Index>(model_.dof())) {
        return Result<void, ControlError>::failure(ControlError::InvalidState);
    }
    if (!model_.has_state() || model_.last_state_stamp() != state.stamp) {
        return Result<void, ControlError>::failure(ControlError::ModelNotUpdated);
    }
    if (config_.enforce_position_limits) {
        for (Eigen::Index index = 0; index < state.position.size(); ++index) {
            if (state.position[index] < model_.lower_position_limits()[index]
                    - config_.joint_limit_tolerance
                || state.position[index] > model_.upper_position_limits()[index]
                    + config_.joint_limit_tolerance) {
                return Result<void, ControlError>::failure(ControlError::StateOutOfBounds);
            }
        }
    }
    const auto solved = solver_.solve(model_, tasks_, constraints_, cycle, output.velocity);
    if (!solved) {
        output.velocity.setZero();
        return Result<void, ControlError>::failure(
            solved.error() == SolverError::Infeasible
                ? ControlError::SolverInfeasible : ControlError::SolverFailure);
    }
    if (!output.velocity.allFinite()) {
        output.velocity.setZero();
        return Result<void, ControlError>::failure(ControlError::NumericalFailure);
    }
    output.sequence = cycle.sequence;
    output.stamp = cycle.time;
    return Result<void, ControlError>::success();
}

}  // namespace rocontrol
