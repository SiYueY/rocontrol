#pragma once

#include <rocontrol/solver/solver.hpp>

#include <chrono>

namespace rocontrol {

/** Preallocated dense QP for joint and Cartesian velocity objectives with box limits. */
class QPSolver final : public Solver {
public:
    /** Last QP build and solve duration, for benchmark diagnostics. */
    std::chrono::nanoseconds last_solve_duration() const noexcept { return last_solve_duration_; }
    /** Allocate dense Hessian and active-set workspace for this model. */
    Result<void, SolverError> prepare(
        const RobotModel& model,
        const std::vector<std::unique_ptr<Task>>& tasks,
        const std::vector<std::unique_ptr<Constraint>>& constraints,
        double regularization) override;
    /** Assemble weighted objectives and hard bounds, then write qdot. */
    Result<void, SolverError> solve(
        const RobotModel& model,
        const std::vector<std::unique_ptr<Task>>& tasks,
        const std::vector<std::unique_ptr<Constraint>>& constraints,
        const ControlCycle& cycle,
        Eigen::Ref<Eigen::VectorXd> output_velocity) override;

private:
    Eigen::MatrixXd hessian_;
    Eigen::VectorXd gradient_;
    Eigen::VectorXd lower_;
    Eigen::VectorXd upper_;
    Eigen::VectorXd solution_;
    Eigen::VectorXd derivative_;
    Eigen::VectorXd direction_;
    Eigen::VectorXd right_hand_side_;
    Eigen::MatrixXd factor_;
    std::vector<Eigen::Index> free_indices_;
    double regularization_{0};
    bool prepared_{false};
    std::chrono::nanoseconds last_solve_duration_{0};
};

}  // namespace rocontrol
