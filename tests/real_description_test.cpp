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
#ifndef TEST_DUO_ONLY
    config.joint_groups["spine"] = {"franka_spine_vertical_joint"};
#endif
    for (const std::string side : {"left", "right"}) {
        for (int joint = 1; joint <= 7; ++joint) {
#ifdef TEST_DUO_ONLY
            config.joint_groups[side].push_back(
                side + "_fr3v2_joint" + std::to_string(joint));
#else
            config.joint_groups[side].push_back(
                side + "_fr3v2_1_joint" + std::to_string(joint));
#endif
        }
    }
    auto created = rocontrol::RobotModel::create(config);
    assert(created);
    auto model = std::move(created).value();
#ifdef TEST_DUO_ONLY
    assert(model->dof() == 14);
    const auto left_frame = model->frame("left_fr3v2_link8");
    const auto right_frame = model->frame("right_fr3v2_link8");
#else
    const auto left_frame = model->frame("left_fr3v2_1_hand_tcp_joint");
    const auto right_frame = model->frame("right_fr3v2_1_hand_tcp_joint");
#endif
    assert(left_frame && right_frame);

    rocontrol::RobotState state;
    state.position.setZero(model->dof());
    state.velocity.setZero(model->dof());
    state.stamp = std::chrono::steady_clock::now();
    for (Eigen::Index index = 0; index < state.position.size(); ++index) {
        const double lower = model->lower_position_limits()[index];
        const double upper = model->upper_position_limits()[index];
        if (std::isfinite(lower) && std::isfinite(upper)) {
            state.position[index] = (lower + upper) / 2;
        }
    }
    assert(model->update(state));
    auto left_target = model->frame_pose(left_frame.value());
    auto right_target = model->frame_pose(right_frame.value());
    left_target.position.y() += 0.01;
    right_target.position.y() -= 0.01;

    rocontrol::QPSolver solver;
    rocontrol::Controller controller(*model, solver);
    auto left = controller.add_cartesian_pose_task(left_frame.value());
    auto right = controller.add_cartesian_pose_task(right_frame.value());
#ifndef TEST_DUO_ONLY
    auto spine = controller.add_joint_position_task(model->joint_group("spine").value());
    assert(left && right && spine);
#else
    assert(left && right);
#endif
    assert(left.value().get().set_target(left_target));
    assert(right.value().get().set_target(right_target));
#ifndef TEST_DUO_ONLY
    Eigen::VectorXd spine_target(1);
    spine_target[0] = state.position[model->joint("franka_spine_vertical_joint").value()] + 0.01;
    assert(spine.value().get().set_target(spine_target));
#endif
    left.value().get().set_enabled(true);
    right.value().get().set_enabled(true);
#ifndef TEST_DUO_ONLY
    spine.value().get().set_enabled(true);
#endif
    assert(controller.start(state));
    rocontrol::JointVelocityCommand command;
    command.velocity.setZero(model->dof());
    rocontrol::ControlCycle cycle{state.stamp, std::chrono::duration<double>(0.002), 1};
    for (int count = 0; count < 200; ++count) {
        assert(controller.update(state, cycle, command));
        assert(command.velocity.allFinite());
        state.position += command.velocity * cycle.period.count();
        state.velocity = command.velocity;
        state.stamp += std::chrono::milliseconds(2);
        cycle.time = state.stamp;
        ++cycle.sequence;
        assert(model->update(state));
    }
    assert((model->frame_pose(left_frame.value()).position - left_target.position).norm() < 0.01);
    assert((model->frame_pose(right_frame.value()).position - right_target.position).norm() < 0.01);
#ifndef TEST_DUO_ONLY
    assert(std::abs(state.position[model->joint("franka_spine_vertical_joint").value()]
        - spine_target[0]) < 0.01);
#endif
}
