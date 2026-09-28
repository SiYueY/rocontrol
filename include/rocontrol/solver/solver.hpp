#pragma once

#include <rocontrol/constraint/constraint.hpp>
#include <rocontrol/cycle.hpp>
#include <rocontrol/model/robot_model.hpp>
#include <rocontrol/result.hpp>
#include <rocontrol/task/task.hpp>

#include <Eigen/Core>
#include <memory>
#include <vector>

namespace rocontrol {

/** QP infeasibility, numerical, or topology/configuration failure. */
enum class SolverError { Infeasible, NumericalFailure, InvalidProblem };

/** Prepared velocity-QP solver; solve() writes a caller-owned canonical DoF vector. */
class Solver {
public:
    /** Destroy a polymorphic velocity-QP solver. */
    virtual ~Solver() = default;
    /** Allocate fixed-dimension workspace before controller startup. */
    virtual Result<void, SolverError> prepare(
        const RobotModel& model,
        const std::vector<std::unique_ptr<Task>>& tasks,
        const std::vector<std::unique_ptr<Constraint>>& constraints,
        double regularization) = 0;
    /** Build and solve one velocity QP using the measured cycle period. */
    virtual Result<void, SolverError> solve(
        const RobotModel& model,
        const std::vector<std::unique_ptr<Task>>& tasks,
        const std::vector<std::unique_ptr<Constraint>>& constraints,
        const ControlCycle& cycle,
        Eigen::Ref<Eigen::VectorXd> output_velocity) = 0;
};

}  // namespace rocontrol
