#include <rocontrol/trajectory/joint_position_trajectory.hpp>

#include <algorithm>
#include <cmath>

namespace rocontrol {
namespace {

double progress(double u) { return u * u * u * (10 + u * (-15 + 6 * u)); }
double progress_rate(double u) { return 30 * u * u * (1 - u) * (1 - u); }

}  // namespace

JointPositionTrajectory::JointPositionTrajectory(Eigen::Index size)
    : start_(Eigen::VectorXd::Zero(size)), delta_(Eigen::VectorXd::Zero(size)) {}

Result<void, TrajectoryError> JointPositionTrajectory::reset(
    const Eigen::VectorXd& start, const Eigen::VectorXd& target,
    std::chrono::duration<double> duration,
    std::chrono::steady_clock::time_point start_time) {
    if (start.size() != start_.size() || target.size() != start_.size()) {
        return Result<void, TrajectoryError>::failure(TrajectoryError::DimensionMismatch);
    }
    if (!start.allFinite() || !target.allFinite()) {
        return Result<void, TrajectoryError>::failure(TrajectoryError::InvalidTarget);
    }
    if (!std::isfinite(duration.count()) || duration.count() <= 0) {
        return Result<void, TrajectoryError>::failure(TrajectoryError::InvalidDuration);
    }
    start_ = start;
    delta_ = target - start;
    duration_ = duration.count();
    start_time_ = start_time;
    last_sample_time_ = start_time;
    initialized_ = true;
    return Result<void, TrajectoryError>::success();
}

Result<void, TrajectoryError> JointPositionTrajectory::sample(
    std::chrono::steady_clock::time_point time, JointPositionReference& output) {
    if (!initialized_) return Result<void, TrajectoryError>::failure(TrajectoryError::NotInitialized);
    if (time < last_sample_time_) {
        return Result<void, TrajectoryError>::failure(TrajectoryError::InvalidTime);
    }
    if (output.position.size() != start_.size() || output.velocity.size() != start_.size()) {
        return Result<void, TrajectoryError>::failure(TrajectoryError::DimensionMismatch);
    }
    const double elapsed = std::chrono::duration<double>(time - start_time_).count();
    const double u = std::clamp(elapsed / duration_, 0.0, 1.0);
    output.position = start_ + progress(u) * delta_;
    output.velocity = (progress_rate(u) / duration_) * delta_;
    last_sample_time_ = time;
    return Result<void, TrajectoryError>::success();
}

bool JointPositionTrajectory::finished(std::chrono::steady_clock::time_point time) const noexcept {
    return initialized_ && std::chrono::duration<double>(time - start_time_).count() >= duration_;
}

}  // namespace rocontrol
