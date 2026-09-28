#pragma once

#include <rocontrol/result.hpp>

#include <Eigen/Core>
#include <chrono>

namespace rocontrol {

/** Invalid reference generation inputs or sampling order. */
enum class TrajectoryError { InvalidDuration, InvalidTarget, DimensionMismatch, InvalidTime, NotInitialized };

/** Caller-owned, preallocated joint reference in JointGroup order. */
struct JointPositionReference {
    /** Group-ordered target positions, rad or m. */
    Eigen::VectorXd position;
    /** Group-ordered feedforward speeds, rad/s or m/s. */
    Eigen::VectorXd velocity;
};

/** Quintic joint reference sampled at absolute monotonic time. */
class JointPositionTrajectory {
public:
    /** Allocate reference storage for a fixed number of joints. */
    explicit JointPositionTrajectory(Eigen::Index size);
    /** Set endpoints and a positive reference duration at an absolute start time. */
    Result<void, TrajectoryError> reset(
        const Eigen::VectorXd& start, const Eigen::VectorXd& target,
        std::chrono::duration<double> duration,
        std::chrono::steady_clock::time_point start_time);
    /** Sample at nondecreasing monotonic time into preallocated output. */
    Result<void, TrajectoryError> sample(
        std::chrono::steady_clock::time_point time,
        JointPositionReference& output);
    /** Whether reference generation has reached its final time. */
    bool finished(std::chrono::steady_clock::time_point time) const noexcept;

private:
    Eigen::VectorXd start_;
    Eigen::VectorXd delta_;
    std::chrono::steady_clock::time_point start_time_;
    std::chrono::steady_clock::time_point last_sample_time_;
    double duration_{0};
    bool initialized_{false};
};

}  // namespace rocontrol
