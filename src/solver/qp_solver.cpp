#include <rocontrol/solver/qp_solver.hpp>

#include <rocontrol/task/joint_position_task.hpp>
#include <rocontrol/task/cartesian_pose_task.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace rocontrol {

Result<void, SolverError> QPSolver::prepare(
    const RobotModel& model, const std::vector<std::unique_ptr<Task>>&,
    const std::vector<std::unique_ptr<Constraint>>&, double regularization) {
    if (!std::isfinite(regularization) || regularization <= 0 || model.dof() == 0) {
        return Result<void, SolverError>::failure(SolverError::InvalidProblem);
    }
    const auto size = static_cast<Eigen::Index>(model.dof());
    hessian_.setZero(size, size);
    gradient_.setZero(size);
    lower_.setZero(size);
    upper_.setZero(size);
    solution_.setZero(size);
    derivative_.setZero(size);
    direction_.setZero(size);
    right_hand_side_.setZero(size);
    factor_.setZero(size, size);
    free_indices_.reserve(size);
    regularization_ = regularization;
    prepared_ = true;
    return Result<void, SolverError>::success();
}

Result<void, SolverError> QPSolver::solve(
    const RobotModel& model, const std::vector<std::unique_ptr<Task>>& tasks,
    const std::vector<std::unique_ptr<Constraint>>& constraints,
    const ControlCycle& cycle, Eigen::Ref<Eigen::VectorXd> output_velocity) {
    struct TimingGuard {
        std::chrono::steady_clock::time_point start;
        std::chrono::nanoseconds& output;
        ~TimingGuard() {
            output = std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now() - start);
        }
    } timing{std::chrono::steady_clock::now(), last_solve_duration_};
    const auto size = static_cast<Eigen::Index>(model.dof());
    const double dt = cycle.period.count();
    if (!prepared_ || output_velocity.size() != size || !std::isfinite(dt) || dt <= 0) {
        return Result<void, SolverError>::failure(SolverError::InvalidProblem);
    }
    hessian_.setZero();
    hessian_.diagonal().setConstant(regularization_);
    gradient_.setZero();
    lower_.setConstant(-std::numeric_limits<double>::infinity());
    upper_.setConstant(std::numeric_limits<double>::infinity());

    for (const auto& task : tasks) {
        if (!task->enabled()) continue;
        const auto* joint = dynamic_cast<const JointPositionTask*>(task.get());
        if (joint) {
            if (!joint->has_target()) {
                return Result<void, SolverError>::failure(SolverError::InvalidProblem);
            }
            for (Eigen::Index local = 0; local < joint->target().size(); ++local) {
                const auto global = joint->group().joints[local];
                double error = joint->target()[local] - model.joint_positions()[global];
                if (model.is_continuous(global)) {
                    error = std::remainder(error, 2 * std::acos(-1.0));
                }
                const double desired = joint->reference_velocity()[local] + joint->gain() * error;
                hessian_(global, global) += joint->weight();
                gradient_[global] -= joint->weight() * desired;
            }
            continue;
        }
        const auto* cartesian = dynamic_cast<const CartesianPoseTask*>(task.get());
        if (!cartesian || !cartesian->has_target()) {
            return Result<void, SolverError>::failure(SolverError::InvalidProblem);
        }
        const auto& current = model.frame_pose(cartesian->frame());
        const auto& jacobian = model.frame_jacobian(cartesian->frame());
        const Eigen::Vector3d position_error = cartesian->target().position - current.position;
        Eigen::Quaterniond rotation_error =
            cartesian->target().orientation * current.orientation.conjugate();
        if (rotation_error.w() < 0) rotation_error.coeffs() *= -1;
        const Eigen::AngleAxisd angle_axis(rotation_error);
        const Eigen::Vector3d orientation_error = angle_axis.axis() * angle_axis.angle();
        for (Eigen::Index row = 0; row < 6; ++row) {
            const double component_weight = row < 3
                ? cartesian->position_weight() : cartesian->orientation_weight();
            const double desired = row < 3
                ? cartesian->linear_reference_velocity()[row]
                    + cartesian->position_gain() * position_error[row]
                : cartesian->angular_reference_velocity()[row - 3]
                    + cartesian->orientation_gain() * orientation_error[row - 3];
            const double weight = cartesian->weight() * component_weight;
            for (Eigen::Index column = 0; column < size; ++column) {
                const double entry = jacobian(row, column);
                gradient_[column] -= weight * entry * desired;
                for (Eigen::Index second = 0; second < size; ++second) {
                    hessian_(column, second) +=
                        weight * entry * jacobian(row, second);
                }
            }
        }
    }
    for (const auto& constraint : constraints) {
        if (dynamic_cast<const JointVelocityLimitConstraint*>(constraint.get())) {
            for (Eigen::Index index = 0; index < size; ++index) {
                const double limit = model.velocity_limits()[index];
                lower_[index] = std::max(lower_[index], -limit);
                upper_[index] = std::min(upper_[index], limit);
            }
        } else if (dynamic_cast<const JointPositionLimitConstraint*>(constraint.get())) {
            for (Eigen::Index index = 0; index < size; ++index) {
                const double lower = model.lower_position_limits()[index];
                const double upper = model.upper_position_limits()[index];
                if (std::isfinite(lower)) {
                    lower_[index] = std::max(lower_[index],
                        (lower - model.joint_positions()[index]) / dt);
                }
                if (std::isfinite(upper)) {
                    upper_[index] = std::min(upper_[index],
                        (upper - model.joint_positions()[index]) / dt);
                }
            }
        } else {
            return Result<void, SolverError>::failure(SolverError::InvalidProblem);
        }
    }
    for (Eigen::Index index = 0; index < size; ++index) {
        if (lower_[index] > upper_[index]) {
            return Result<void, SolverError>::failure(SolverError::Infeasible);
        }
        solution_[index] = std::clamp(solution_[index], lower_[index], upper_[index]);
    }

    constexpr double tolerance = 1e-7;
    for (int iteration = 0; iteration < 100; ++iteration) {
        free_indices_.clear();
        for (Eigen::Index row = 0; row < size; ++row) {
            double value = gradient_[row];
            for (Eigen::Index column = 0; column < size; ++column) {
                value += hessian_(row, column) * solution_[column];
            }
            derivative_[row] = value;
            const bool at_lower = std::isfinite(lower_[row])
                && solution_[row] <= lower_[row] + tolerance;
            const bool at_upper = std::isfinite(upper_[row])
                && solution_[row] >= upper_[row] - tolerance;
            if ((!at_lower || value < -tolerance)
                && (!at_upper || value > tolerance)) {
                free_indices_.push_back(row);
            }
        }
        if (free_indices_.empty()) {
            output_velocity = solution_;
            return Result<void, SolverError>::success();
        }

        const auto count = static_cast<Eigen::Index>(free_indices_.size());
        for (Eigen::Index row = 0; row < count; ++row) {
            right_hand_side_[row] = -derivative_[free_indices_[row]];
            for (Eigen::Index column = 0; column <= row; ++column) {
                double sum = hessian_(free_indices_[row], free_indices_[column]);
                for (Eigen::Index previous = 0; previous < column; ++previous) {
                    sum -= factor_(row, previous) * factor_(column, previous);
                }
                if (row == column) {
                    if (!std::isfinite(sum) || sum <= 0) {
                        return Result<void, SolverError>::failure(SolverError::NumericalFailure);
                    }
                    factor_(row, column) = std::sqrt(sum);
                } else {
                    factor_(row, column) = sum / factor_(column, column);
                }
            }
        }
        for (Eigen::Index row = 0; row < count; ++row) {
            double value = right_hand_side_[row];
            for (Eigen::Index previous = 0; previous < row; ++previous) {
                value -= factor_(row, previous) * right_hand_side_[previous];
            }
            right_hand_side_[row] = value / factor_(row, row);
        }
        direction_.setZero();
        for (Eigen::Index row = count; row-- > 0;) {
            double value = right_hand_side_[row];
            for (Eigen::Index next = row + 1; next < count; ++next) {
                value -= factor_(next, row) * direction_[free_indices_[next]];
            }
            direction_[free_indices_[row]] = value / factor_(row, row);
        }
        double maximum_step = 1;
        double largest_direction = 0;
        for (const auto index : free_indices_) {
            const double direction = direction_[index];
            largest_direction = std::max(largest_direction, std::abs(direction));
            if (direction > 0 && std::isfinite(upper_[index])) {
                maximum_step = std::min(maximum_step,
                    (upper_[index] - solution_[index]) / direction);
            } else if (direction < 0 && std::isfinite(lower_[index])) {
                maximum_step = std::min(maximum_step,
                    (lower_[index] - solution_[index]) / direction);
            }
        }
        if (largest_direction < tolerance) {
            output_velocity = solution_;
            return Result<void, SolverError>::success();
        }
        if (!std::isfinite(maximum_step) || maximum_step < 0) {
            return Result<void, SolverError>::failure(SolverError::NumericalFailure);
        }
        for (const auto index : free_indices_) {
            solution_[index] = std::clamp(
                solution_[index] + maximum_step * direction_[index],
                lower_[index], upper_[index]);
        }
    }
    return Result<void, SolverError>::failure(SolverError::NumericalFailure);
}

}  // namespace rocontrol
