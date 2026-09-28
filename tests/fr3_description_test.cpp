#include <rocontrol/controller.hpp>
#include <rocontrol/solver/qp_solver.hpp>

#undef NDEBUG
#include <cassert>
#include <chrono>
#include <cmath>
#include <string>

int main() {
    rocontrol::RobotModelConfig config;
    config.urdf = TEST_URDF_PATH;
    for (int index = 1; index <= 7; ++index) {
        config.joint_groups["arm"].push_back("fr3_joint" + std::to_string(index));
    }
    auto created = rocontrol::RobotModel::create(config);
    assert(created);
    auto model = std::move(created).value();
    assert(model->dof() == 7);
    const auto group = model->joint_group("arm");
    const auto tool = model->frame("fr3_link8");
    assert(group && tool);

    rocontrol::RobotState state;
    state.position = Eigen::VectorXd::Zero(model->dof());
    state.velocity = Eigen::VectorXd::Zero(model->dof());
    state.stamp = std::chrono::steady_clock::now();
    state.position[model->joint("fr3_joint4").value()] = -1.5;
    state.position[model->joint("fr3_joint6").value()] = 2.0;
    const Eigen::VectorXd initial = state.position;
    Eigen::VectorXd target = initial;
    target[model->joint("fr3_joint1").value()] += 0.1;
    target[model->joint("fr3_joint3").value()] += 0.05;
    state.position = target;
    assert(model->update(state));
    const auto target_pose = model->frame_pose(tool.value());
    state.position = initial;
    state.stamp += std::chrono::milliseconds(2);
    assert(model->update(state));

    rocontrol::QPSolver joint_solver;
    rocontrol::Controller joint_controller(*model, joint_solver);
    auto joint = joint_controller.add_joint_position_task(group.value());
    assert(joint && joint.value().get().set_target(target));
    joint.value().get().set_enabled(true);
    assert(joint_controller.start(state));
    rocontrol::JointVelocityCommand command;
    command.velocity.setZero(model->dof());
    rocontrol::ControlCycle cycle{state.stamp, std::chrono::duration<double>(0.002), 1};
    for (int step = 0; step < 2500; ++step) {
        assert(joint_controller.update(state, cycle, command));
        state.position += command.velocity * cycle.period.count();
        state.velocity = command.velocity;
        state.stamp += std::chrono::milliseconds(2);
        cycle.time = state.stamp;
        ++cycle.sequence;
        assert(model->update(state));
    }
    assert((state.position - target).norm() < 0.01);

    state.position = initial;
    state.velocity.setZero();
    state.stamp += std::chrono::milliseconds(2);
    cycle.time = state.stamp;
    assert(model->update(state));
    rocontrol::QPSolver pose_solver;
    rocontrol::Controller pose_controller(*model, pose_solver);
    auto pose = pose_controller.add_cartesian_pose_task(tool.value());
    assert(pose && pose.value().get().set_target(target_pose));
    pose.value().get().set_enabled(true);
    assert(pose_controller.start(state));
    for (int step = 0; step < 2500; ++step) {
        assert(pose_controller.update(state, cycle, command));
        state.position += command.velocity * cycle.period.count();
        state.velocity = command.velocity;
        state.stamp += std::chrono::milliseconds(2);
        cycle.time = state.stamp;
        ++cycle.sequence;
        assert(model->update(state));
    }
    const auto& actual = model->frame_pose(tool.value());
    assert((actual.position - target_pose.position).norm() < 0.02);
    const Eigen::Quaterniond error = target_pose.orientation * actual.orientation.conjugate();
    assert(Eigen::AngleAxisd(error).angle() < 0.02);
}
