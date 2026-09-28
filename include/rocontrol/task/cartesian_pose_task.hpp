#pragma once

#include <rocontrol/task/task.hpp>
#include <rocontrol/types.hpp>

namespace rocontrol {

/** Pose feedback at a controlled frame in the model root frame. */
class CartesianPoseTask final : public Task {
public:
    /** Bind a frame index resolved from the associated RobotModel. */
    explicit CartesianPoseTask(FrameIndex frame) : frame_(frame) {}
    /** Controlled frame index. */
    FrameIndex frame() const noexcept { return frame_; }
    /** Set finite root-frame position in metres and unit quaternion orientation. */
    Result<void, TaskError> set_target(const Pose& pose);
    /** Last valid target pose in the model root frame. */
    const Pose& target() const noexcept { return target_; }
    /** Whether start() or set_target() established a target. */
    bool has_target() const noexcept { return has_target_; }
    /** Set a finite nonnegative translation gain in s^-1. */
    Result<void, TaskError> set_position_gain(double gain);
    /** Set a finite nonnegative rotation gain in s^-1. */
    Result<void, TaskError> set_orientation_gain(double gain);
    /** Current translation feedback gain in s^-1. */
    double position_gain() const noexcept { return position_gain_; }
    /** Current rotation feedback gain in s^-1. */
    double orientation_gain() const noexcept { return orientation_gain_; }
    /** Set a finite positive translation metric weight. */
    Result<void, TaskError> set_position_weight(double weight);
    /** Set a finite positive rotation metric weight. */
    Result<void, TaskError> set_orientation_weight(double weight);
    /** Current translation metric weight. */
    double position_weight() const noexcept { return position_weight_; }
    /** Current rotation metric weight. */
    double orientation_weight() const noexcept { return orientation_weight_; }
    /** Set root-frame spatial feedforward, metres/s and rad/s. */
    Result<void, TaskError> set_reference_velocity(
        const Eigen::Vector3d& linear, const Eigen::Vector3d& angular);
    /** Set spatial feedforward to zero. */
    void clear_reference_velocity() noexcept {
        linear_velocity_.setZero();
        angular_velocity_.setZero();
    }
    /** Root-frame linear feedforward in metres/s. */
    const Eigen::Vector3d& linear_reference_velocity() const noexcept { return linear_velocity_; }
    /** Root-frame angular feedforward in rad/s. */
    const Eigen::Vector3d& angular_reference_velocity() const noexcept { return angular_velocity_; }

private:
    FrameIndex frame_;
    Pose target_;
    Eigen::Vector3d linear_velocity_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d angular_velocity_{Eigen::Vector3d::Zero()};
    double position_gain_{1};
    double orientation_gain_{1};
    double position_weight_{1};
    double orientation_weight_{1};
    bool has_target_{false};
};

}  // namespace rocontrol
