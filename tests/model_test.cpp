#include <rocontrol/model/robot_model.hpp>

#undef NDEBUG
#include <cassert>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>

int main() {
    rocontrol::RobotModelConfig invalid;
    assert(!rocontrol::RobotModel::create(invalid));

    rocontrol::RobotModelConfig config;
    config.urdf = TEST_URDF_PATH;
    config.joint_groups["arm"] = {"joint1", "spin"};
    auto created = rocontrol::RobotModel::create(config);
    assert(created);
    auto model = std::move(created).value();
    assert(!model->has_state());
    assert(model->dof() == 2);
    assert(model->dof_names()[0] == "joint1");
    assert(model->dof_names()[1] == "spin");
    assert(std::abs(model->lower_position_limits()[0] + 1.0) < 1e-9);
    assert(std::abs(model->upper_position_limits()[0] - 1.0) < 1e-9);
    assert(std::isinf(model->lower_position_limits()[1]));
    assert(std::abs(model->velocity_limits()[0] - 2.0) < 1e-9);
    const auto group = model->joint_group("arm");
    assert(group && group.value().model == model->id());
    assert(group.value().joints[0] == 0 && group.value().joints[1] == 1);
    assert(!model->joint("mimic_joint"));
    assert(!model->joint("tool_fixed"));
    assert(!model->frame("missing_frame"));
    const auto tool = model->frame("tool");
    assert(tool);

    rocontrol::RobotState state;
    state.stamp = std::chrono::steady_clock::now();
    state.position = Eigen::Vector2d(0.2, 0.3);
    state.velocity = Eigen::Vector2d::Zero();
    assert(model->update(state));
    assert(model->has_state());
    assert(model->last_state_stamp() == state.stamp);
    assert(model->frame_pose(tool.value()).position.allFinite());
    assert(model->frame_jacobian(tool.value()).rows() == 6);
    assert(model->frame_jacobian(tool.value()).cols() == 2);

    const auto pose = model->frame_pose(tool.value());
    const auto jacobian = model->frame_jacobian(tool.value());
    constexpr double epsilon = 1e-6;
    for (Eigen::Index column = 0; column < 2; ++column) {
        auto displaced = state;
        displaced.position[column] += epsilon;
        displaced.stamp += std::chrono::nanoseconds(column + 1);
        assert(model->update(displaced));
        const auto& displaced_pose = model->frame_pose(tool.value());
        const Eigen::Vector3d numerical =
            (displaced_pose.position - pose.position) / epsilon;
        assert((numerical - jacobian.block<3, 1>(0, column)).norm() < 1e-4);
        Eigen::Quaterniond delta = displaced_pose.orientation * pose.orientation.conjugate();
        if (delta.w() < 0) delta.coeffs() *= -1;
        const Eigen::AngleAxisd angle_axis(delta);
        const Eigen::Vector3d angular = angle_axis.axis() * angle_axis.angle() / epsilon;
        assert((angular - jacobian.block<3, 1>(3, column)).norm() < 1e-4);
    }

    assert(model->update(state));
    Eigen::internal::set_is_malloc_allowed(false);
    for (int cycle = 0; cycle < 100; ++cycle) {
        state.stamp += std::chrono::milliseconds(2);
        assert(model->update(state));
    }
    Eigen::internal::set_is_malloc_allowed(true);

    state.position.resize(1);
    assert(!model->update(state));
    state.position = Eigen::Vector2d::Zero();
    state.position[0] = std::numeric_limits<double>::quiet_NaN();
    const auto previous_stamp = model->last_state_stamp();
    assert(!model->update(state));
    assert(model->last_state_stamp() == previous_stamp);

    auto duplicate_group = config;
    duplicate_group.joint_groups["arm"] = {"joint1", "joint1"};
    assert(!rocontrol::RobotModel::create(duplicate_group));

    auto invalid_override = config;
    invalid_override.limit_overrides["joint1"].velocity = -1;
    assert(!rocontrol::RobotModel::create(invalid_override));

    auto second = rocontrol::RobotModel::create(config);
    assert(second);
    assert(second.value()->id() != model->id());
    std::cout << "model tests passed\n";
}
