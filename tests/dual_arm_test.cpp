#include <rocontrol/controller.hpp>
#include <rocontrol/solver/qp_solver.hpp>

#undef NDEBUG
#include <cassert>
#include <chrono>
#include <cmath>
#include <string>
#include <iostream>

int main() {
    rocontrol::RobotModelConfig config;
    config.urdf = TEST_URDF_PATH;
    config.joint_groups["spine"] = {"spine"};
    for (const std::string side : {"left", "right"}) {
        auto& group = config.joint_groups[side];
        for (int joint = 1; joint <= 7; ++joint) {
            group.push_back(side + "_joint" + std::to_string(joint));
        }
    }
    auto created = rocontrol::RobotModel::create(config);
    assert(created);
    auto model = std::move(created).value();
    assert(model->dof() == 15);
    const auto left_frame = model->frame("left_ee");
    const auto right_frame = model->frame("right_ee");
    assert(left_frame && right_frame);

    rocontrol::RobotState state;
    state.stamp = std::chrono::steady_clock::now();
    state.position = Eigen::VectorXd::Zero(model->dof());
    state.velocity = Eigen::VectorXd::Zero(model->dof());
    state.position[model->joint("spine").value()] = 0.2;
    state.position[model->joint("left_joint1").value()] = 0.2;
    state.position[model->joint("right_joint1").value()] = -0.2;
    assert(model->update(state));
    const auto desired_left = model->frame_pose(left_frame.value());
    const auto desired_right = model->frame_pose(right_frame.value());
    state.position.setZero();
    state.position[model->joint("spine").value()] = 0.1;
    state.stamp += std::chrono::milliseconds(2);
    assert(model->update(state));

    rocontrol::QPSolver solver;
    rocontrol::Controller controller(*model, solver);
    auto left = controller.add_cartesian_pose_task(left_frame.value());
    auto right = controller.add_cartesian_pose_task(right_frame.value());
    auto spine = controller.add_joint_position_task(model->joint_group("spine").value());
    assert(left && right && spine);
    assert(left.value().get().set_target(desired_left));
    assert(right.value().get().set_target(desired_right));
    Eigen::VectorXd spine_target(1);
    spine_target[0] = 0.2;
    assert(spine.value().get().set_target(spine_target));
    left.value().get().set_enabled(true);
    right.value().get().set_enabled(true);
    spine.value().get().set_enabled(true);
    assert(controller.start(state));

    rocontrol::JointVelocityCommand command;
    command.velocity.setZero(model->dof());
    rocontrol::ControlCycle cycle;
    cycle.period = std::chrono::duration<double>(0.002);
    for (int index = 0; index < 2000; ++index) {
        cycle.time = state.stamp;
        cycle.sequence = index + 1;
        auto update = controller.update(state, cycle, command);
        if (!update) {
            std::cerr << "update failed at " << index << " error="
                      << static_cast<int>(update.error()) << '\n';
        }
        assert(update);
        state.position += command.velocity * cycle.period.count();
        state.velocity = command.velocity;
        state.stamp += std::chrono::milliseconds(2);
        assert(model->update(state));
    }
    assert((model->frame_pose(left_frame.value()).position - desired_left.position).norm() < 0.02);
    assert((model->frame_pose(right_frame.value()).position - desired_right.position).norm() < 0.02);
    assert(std::abs(state.position[model->joint("spine").value()] - 0.2) < 0.02);
}
