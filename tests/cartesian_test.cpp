#include <rocontrol/controller.hpp>
#include <rocontrol/solver/qp_solver.hpp>

#undef NDEBUG
#include <cassert>
#include <chrono>
#include <limits>

int main() {
    rocontrol::RobotModelConfig config;
    config.urdf = TEST_URDF_PATH;
    auto created = rocontrol::RobotModel::create(config);
    assert(created);
    auto model = std::move(created).value();
    const auto tool = model->frame("tool");
    assert(tool);

    rocontrol::RobotState state;
    state.stamp = std::chrono::steady_clock::now();
    state.position = Eigen::Vector2d(0.3, 0.1);
    state.velocity = Eigen::Vector2d::Zero();
    assert(model->update(state));
    const auto desired = model->frame_pose(tool.value());
    state.position.setZero();
    state.stamp += std::chrono::milliseconds(2);
    assert(model->update(state));

    rocontrol::QPSolver solver;
    rocontrol::Controller controller(*model, solver);
    auto added = controller.add_cartesian_pose_task(tool.value());
    assert(added);
    auto& task = added.value().get();
    assert(task.set_target(desired));
    rocontrol::Pose invalid = desired;
    invalid.orientation.coeffs().setZero();
    assert(!task.set_target(invalid));
    assert((task.target().position - desired.position).norm() == 0);
    assert(task.set_position_gain(2));
    assert(task.set_orientation_gain(2));
    assert(task.set_position_weight(0.5));
    assert(task.set_orientation_weight(0.5));
    assert(!task.set_position_gain(-1) && task.position_gain() == 2);
    assert(!task.set_orientation_gain(-1) && task.orientation_gain() == 2);
    assert(!task.set_position_weight(0) && task.position_weight() == 0.5);
    assert(!task.set_orientation_weight(0) && task.orientation_weight() == 0.5);
    Eigen::Vector3d invalid_velocity = Eigen::Vector3d::Zero();
    invalid_velocity.x() = std::numeric_limits<double>::quiet_NaN();
    assert(!task.set_reference_velocity(invalid_velocity, Eigen::Vector3d::Zero()));
    assert(task.linear_reference_velocity().isZero());
    task.set_enabled(true);
    assert(controller.start(state));

    rocontrol::JointVelocityCommand command;
    command.velocity.setZero(model->dof());
    rocontrol::ControlCycle cycle;
    cycle.period = std::chrono::duration<double>(0.002);
    cycle.sequence = 1;
    for (int step = 0; step < 2500; ++step) {
        cycle.time = state.stamp;
        assert(controller.update(state, cycle, command));
        state.position += command.velocity * cycle.period.count();
        state.velocity = command.velocity;
        state.stamp += std::chrono::milliseconds(2);
        cycle.sequence++;
        assert(model->update(state));
    }
    assert((model->frame_pose(tool.value()).position - desired.position).norm() < 0.01);
    const Eigen::Quaterniond error =
        desired.orientation * model->frame_pose(tool.value()).orientation.conjugate();
    assert(Eigen::AngleAxisd(error).angle() < 0.01);

    Eigen::internal::set_is_malloc_allowed(false);
    for (int step = 0; step < 100; ++step) {
        cycle.time = state.stamp;
        assert(controller.update(state, cycle, command));
    }
    Eigen::internal::set_is_malloc_allowed(true);
}
