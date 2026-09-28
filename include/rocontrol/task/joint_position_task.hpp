#pragma once

#include <rocontrol/task/task.hpp>
#include <rocontrol/types.hpp>

namespace rocontrol {

/** Joint position feedback and optional velocity feedforward in group order. */
class JointPositionTask final : public Task {
public:
    /** Allocate group-sized target and reference buffers during setup. */
    explicit JointPositionTask(JointGroup group);
    /** Joint order used by target and velocity reference. */
    const JointGroup& group() const noexcept { return group_; }
    /** Set finite target positions in group order, rad or m. */
    Result<void, TaskError> set_target(const Eigen::VectorXd& position);
    /** Last valid position target in group order. */
    const Eigen::VectorXd& target() const noexcept { return target_; }
    /** Whether start() or set_target() established a target. */
    bool has_target() const noexcept { return has_target_; }
    /** Set a finite nonnegative feedback gain in s^-1. */
    Result<void, TaskError> set_gain(double gain);
    /** Current continuous-time feedback gain in s^-1. */
    double gain() const noexcept { return gain_; }
    /** Set finite feedforward speed in group order, rad/s or m/s. */
    Result<void, TaskError> set_reference_velocity(const Eigen::VectorXd& velocity);
    /** Set all feedforward speeds to zero. */
    void clear_reference_velocity() noexcept { reference_velocity_.setZero(); }
    /** Current feedforward speed in group order. */
    const Eigen::VectorXd& reference_velocity() const noexcept { return reference_velocity_; }

private:
    JointGroup group_;
    Eigen::VectorXd target_;
    Eigen::VectorXd reference_velocity_;
    double gain_{1.0};
    bool has_target_{false};
};

}  // namespace rocontrol
