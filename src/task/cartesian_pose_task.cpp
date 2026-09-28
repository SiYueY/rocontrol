#include <rocontrol/task/cartesian_pose_task.hpp>

#include <cmath>

namespace rocontrol {
namespace {

bool valid_gain(double value) { return std::isfinite(value) && value >= 0; }
bool valid_weight(double value) { return std::isfinite(value) && value > 0; }

}  // namespace

Result<void, TaskError> CartesianPoseTask::set_target(const Pose& pose) {
    if (!pose.position.allFinite() || !pose.orientation.coeffs().allFinite()
        || std::abs(pose.orientation.norm() - 1.0) > 1e-6) {
        return Result<void, TaskError>::failure(TaskError::InvalidTarget);
    }
    target_ = pose;
    has_target_ = true;
    return Result<void, TaskError>::success();
}

Result<void, TaskError> CartesianPoseTask::set_position_gain(double gain) {
    if (!valid_gain(gain)) return Result<void, TaskError>::failure(TaskError::InvalidGain);
    position_gain_ = gain;
    return Result<void, TaskError>::success();
}

Result<void, TaskError> CartesianPoseTask::set_orientation_gain(double gain) {
    if (!valid_gain(gain)) return Result<void, TaskError>::failure(TaskError::InvalidGain);
    orientation_gain_ = gain;
    return Result<void, TaskError>::success();
}

Result<void, TaskError> CartesianPoseTask::set_position_weight(double weight) {
    if (!valid_weight(weight)) return Result<void, TaskError>::failure(TaskError::InvalidWeight);
    position_weight_ = weight;
    return Result<void, TaskError>::success();
}

Result<void, TaskError> CartesianPoseTask::set_orientation_weight(double weight) {
    if (!valid_weight(weight)) return Result<void, TaskError>::failure(TaskError::InvalidWeight);
    orientation_weight_ = weight;
    return Result<void, TaskError>::success();
}

Result<void, TaskError> CartesianPoseTask::set_reference_velocity(
    const Eigen::Vector3d& linear, const Eigen::Vector3d& angular) {
    if (!linear.allFinite() || !angular.allFinite()) {
        return Result<void, TaskError>::failure(TaskError::InvalidTarget);
    }
    linear_velocity_ = linear;
    angular_velocity_ = angular;
    return Result<void, TaskError>::success();
}

}  // namespace rocontrol
