#pragma once

#include <rocontrol/model/config.hpp>
#include <rocontrol/result.hpp>
#include <rocontrol/types.hpp>

#include <Eigen/Core>
#include <chrono>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace rocontrol {

/** Model construction, lookup, and kinematic-update failures. */
enum class ModelError {
    InvalidConfig,
    InvalidUrdf,
    UnsupportedJoint,
    InvalidLimits,
    InvalidGroup,
    InvalidJoint,
    InvalidFrame,
    InvalidState,
    DimensionMismatch,
    NumericalFailure
};

/** Fixed-base Pinocchio model. All vectors use dof_names() order. */
class RobotModel {
public:
    /** Parse URDF and build immutable topology and effective joint limits. */
    static Result<std::unique_ptr<RobotModel>, ModelError> create(const RobotModelConfig& config);
    /** Release the internal Pinocchio model and preallocated caches. */
    ~RobotModel();
    RobotModel(const RobotModel&) = delete;
    RobotModel& operator=(const RobotModel&) = delete;

    /** Refresh kinematics once for this monotonic-stamped state. */
    Result<void, ModelError> update(const RobotState& state);

    /** Identifier used to reject groups from another model. */
    ModelId id() const noexcept;
    /** Number of independent controllable scalar DoFs. */
    std::size_t dof() const noexcept;
    /** Canonical order for every joint vector and limit array. */
    const std::vector<std::string>& dof_names() const noexcept;
    /** Resolve an independent joint to its canonical DoF index. */
    Result<JointIndex, ModelError> joint(std::string_view name) const;
    /** Resolve a configured group, retaining its configured order. */
    Result<JointGroup, ModelError> joint_group(std::string_view name) const;
    /** Resolve a URDF frame once, before entering the control loop. */
    Result<FrameIndex, ModelError> frame(std::string_view name) const;
    /** Check whether an index is a frame of this model. */
    bool valid_frame(FrameIndex frame) const noexcept;
    /** Pose relative to the fixed root frame, valid after update(). */
    const Pose& frame_pose(FrameIndex frame) const;
    /** 6 x DoF Jacobian: linear then angular rows, root frame, frame origin. */
    const Eigen::MatrixXd& frame_jacobian(FrameIndex frame) const;
    /** Effective lower limits in canonical order, rad or m; may be -infinity. */
    const Eigen::VectorXd& lower_position_limits() const noexcept;
    /** Effective upper limits in canonical order, rad or m; may be +infinity. */
    const Eigen::VectorXd& upper_position_limits() const noexcept;
    /** Effective positive speed magnitudes in canonical order, rad/s or m/s. */
    const Eigen::VectorXd& velocity_limits() const noexcept;
    /** Latest measured joint positions in canonical DoF order. */
    const Eigen::VectorXd& joint_positions() const noexcept;
    /** Whether this DoF is a continuous revolute joint. */
    bool is_continuous(JointIndex index) const noexcept;
    /** Stamp of the last successfully applied RobotState. */
    std::chrono::steady_clock::time_point last_state_stamp() const noexcept;
    /** Whether update() has succeeded at least once. */
    bool has_state() const noexcept;

private:
    struct Impl;
    explicit RobotModel(std::unique_ptr<Impl> impl);
    std::unique_ptr<Impl> impl_;
};

}  // namespace rocontrol
