#include <rocontrol/controller.hpp>
#include <rocontrol/solver/qp_solver.hpp>

int main() {
    rocontrol::QPSolver solver;
    return solver.last_solve_duration().count() == 0 ? 0 : 1;
}
