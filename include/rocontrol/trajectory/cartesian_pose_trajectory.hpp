#pragma once

#include <rocontrol/result.hpp>
#include <rocontrol/trajectory/joint_position_trajectory.hpp>
#include <rocontrol/types.hpp>

#include <chrono>

namespace rocontrol {

/** Pose plus spatial linear and angular velocity in the model root frame. */
struct CartesianPoseReference {
    /** Pose relative to the fixed model root frame. */
    Pose pose;
    /** Root-frame linear velocity in metres/s. */
    Eigen::Vector3d linear_velocity{Eigen::Vector3d::Zero()};
    /** Root-frame angular velocity in rad/s. */
    Eigen::Vector3d angular_velocity{Eigen::Vector3d::Zero()};
};

/** Quintic translation and shortest-path quaternion SLERP reference. */
class CartesianPoseTrajectory {
public:
    /** Set valid root-frame endpoints and a positive reference duration. */
    Result<void, TrajectoryError> reset(
        const Pose& start, const Pose& target,
        std::chrono::duration<double> duration,
        std::chrono::steady_clock::time_point start_time);
    /** Sample at nondecreasing monotonic time without allocating. */
    Result<void, TrajectoryError> sample(
        std::chrono::steady_clock::time_point time,
        CartesianPoseReference& output);
    /** Whether reference generation has reached its final time. */
    bool finished(std::chrono::steady_clock::time_point time) const noexcept;

private:
    Pose start_;
    Pose target_;
    Eigen::Vector3d angular_path_{Eigen::Vector3d::Zero()};
    std::chrono::steady_clock::time_point start_time_;
    std::chrono::steady_clock::time_point last_sample_time_;
    double duration_{0};
    bool initialized_{false};
};

}  // namespace rocontrol
