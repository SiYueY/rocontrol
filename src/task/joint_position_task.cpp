#include <rocontrol/task/joint_position_task.hpp>

#include <cmath>
#include <utility>

namespace rocontrol {

Result<void, TaskError> Task::set_weight(double weight) {
    if (!std::isfinite(weight) || weight <= 0) {
        return Result<void, TaskError>::failure(TaskError::InvalidWeight);
    }
    weight_ = weight;
    return Result<void, TaskError>::success();
}

JointPositionTask::JointPositionTask(JointGroup group)
    : group_(std::move(group)),
      target_(Eigen::VectorXd::Zero(group_.joints.size())),
      reference_velocity_(Eigen::VectorXd::Zero(group_.joints.size())) {}

Result<void, TaskError> JointPositionTask::set_target(const Eigen::VectorXd& position) {
    if (position.size() != target_.size()) {
        return Result<void, TaskError>::failure(TaskError::DimensionMismatch);
    }
    if (!position.allFinite()) {
        return Result<void, TaskError>::failure(TaskError::InvalidTarget);
    }
    target_ = position;
    has_target_ = true;
    return Result<void, TaskError>::success();
}

Result<void, TaskError> JointPositionTask::set_gain(double gain) {
    if (!std::isfinite(gain) || gain < 0) {
        return Result<void, TaskError>::failure(TaskError::InvalidGain);
    }
    gain_ = gain;
    return Result<void, TaskError>::success();
}

Result<void, TaskError> JointPositionTask::set_reference_velocity(
    const Eigen::VectorXd& velocity) {
    if (velocity.size() != reference_velocity_.size()) {
        return Result<void, TaskError>::failure(TaskError::DimensionMismatch);
    }
    if (!velocity.allFinite()) {
        return Result<void, TaskError>::failure(TaskError::InvalidTarget);
    }
    reference_velocity_ = velocity;
    return Result<void, TaskError>::success();
}

}  // namespace rocontrol
