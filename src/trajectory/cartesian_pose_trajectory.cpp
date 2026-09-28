#include <rocontrol/trajectory/cartesian_pose_trajectory.hpp>

#include <algorithm>
#include <cmath>

namespace rocontrol {
namespace {

bool valid_pose(const Pose& pose) {
    return pose.position.allFinite() && pose.orientation.coeffs().allFinite()
        && std::abs(pose.orientation.norm() - 1.0) <= 1e-6;
}
double progress(double u) { return u * u * u * (10 + u * (-15 + 6 * u)); }
double progress_rate(double u) { return 30 * u * u * (1 - u) * (1 - u); }

}  // namespace

Result<void, TrajectoryError> CartesianPoseTrajectory::reset(
    const Pose& start, const Pose& target, std::chrono::duration<double> duration,
    std::chrono::steady_clock::time_point start_time) {
    if (!valid_pose(start) || !valid_pose(target)) {
        return Result<void, TrajectoryError>::failure(TrajectoryError::InvalidTarget);
    }
    if (!std::isfinite(duration.count()) || duration.count() <= 0) {
        return Result<void, TrajectoryError>::failure(TrajectoryError::InvalidDuration);
    }
    Pose end = target;
    if (start.orientation.dot(end.orientation) < 0) end.orientation.coeffs() *= -1;
    const Eigen::Quaterniond difference = end.orientation * start.orientation.conjugate();
    const Eigen::AngleAxisd angle_axis(difference);
    start_ = start;
    target_ = end;
    angular_path_ = angle_axis.axis() * angle_axis.angle();
    duration_ = duration.count();
    start_time_ = start_time;
    last_sample_time_ = start_time;
    initialized_ = true;
    return Result<void, TrajectoryError>::success();
}

Result<void, TrajectoryError> CartesianPoseTrajectory::sample(
    std::chrono::steady_clock::time_point time, CartesianPoseReference& output) {
    if (!initialized_) return Result<void, TrajectoryError>::failure(TrajectoryError::NotInitialized);
    if (time < last_sample_time_) {
        return Result<void, TrajectoryError>::failure(TrajectoryError::InvalidTime);
    }
    const double elapsed = std::chrono::duration<double>(time - start_time_).count();
    const double u = std::clamp(elapsed / duration_, 0.0, 1.0);
    const double s = progress(u);
    const double ds = progress_rate(u) / duration_;
    output.pose.position = start_.position + s * (target_.position - start_.position);
    output.pose.orientation = start_.orientation.slerp(s, target_.orientation);
    output.linear_velocity = ds * (target_.position - start_.position);
    output.angular_velocity = ds * angular_path_;
    last_sample_time_ = time;
    return Result<void, TrajectoryError>::success();
}

bool CartesianPoseTrajectory::finished(std::chrono::steady_clock::time_point time) const noexcept {
    return initialized_ && std::chrono::duration<double>(time - start_time_).count() >= duration_;
}

}  // namespace rocontrol
