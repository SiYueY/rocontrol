#include <rocontrol/fsm/state_machine.hpp>
#include <rocontrol/trajectory/cartesian_pose_trajectory.hpp>
#include <rocontrol/trajectory/joint_position_trajectory.hpp>

#undef NDEBUG
#include <cassert>
#include <chrono>
#include <cmath>
#include <memory>

namespace {

class CountingState final : public rocontrol::State {
public:
    CountingState(int& enters, int& updates, int& exits, rocontrol::StateStatus outcome)
        : enters_(enters), updates_(updates), exits_(exits), outcome_(outcome) {}
    rocontrol::Result<void, rocontrol::FsmError> enter(const rocontrol::StateContext&) override {
        ++enters_;
        return rocontrol::Result<void, rocontrol::FsmError>::success();
    }
    rocontrol::Result<rocontrol::StateStatus, rocontrol::FsmError>
    update(const rocontrol::StateContext&) override {
        ++updates_;
        return rocontrol::Result<rocontrol::StateStatus, rocontrol::FsmError>::success(outcome_);
    }
    rocontrol::Result<void, rocontrol::FsmError> exit(const rocontrol::StateContext&) override {
        ++exits_;
        return rocontrol::Result<void, rocontrol::FsmError>::success();
    }
private:
    int& enters_;
    int& updates_;
    int& exits_;
    rocontrol::StateStatus outcome_;
};

}  // namespace

int main() {
    const auto start_time = std::chrono::steady_clock::now();
    rocontrol::JointPositionTrajectory joint_trajectory(2);
    rocontrol::JointPositionReference joint_reference;
    joint_reference.position.setZero(2);
    joint_reference.velocity.setZero(2);
    Eigen::Vector2d start = Eigen::Vector2d::Zero();
    Eigen::Vector2d end(1, 2);
    assert(joint_trajectory.reset(start, end, std::chrono::seconds(2), start_time));
    assert(!joint_trajectory.reset(start, end, std::chrono::seconds(0), start_time));
    assert(joint_trajectory.sample(start_time, joint_reference));
    assert(joint_reference.position.isZero() && joint_reference.velocity.isZero());
    assert(joint_trajectory.sample(start_time + std::chrono::seconds(1), joint_reference));
    assert((joint_reference.position - end / 2).norm() < 1e-12);
    assert((joint_reference.velocity - end * 0.9375).norm() < 1e-12);
    assert(!joint_trajectory.sample(start_time, joint_reference));
    assert(joint_trajectory.sample(start_time + std::chrono::seconds(2), joint_reference));
    assert((joint_reference.position - end).norm() < 1e-12);
    assert(joint_reference.velocity.isZero());
    assert(joint_trajectory.finished(start_time + std::chrono::seconds(2)));

    rocontrol::CartesianPoseTrajectory cartesian_trajectory;
    rocontrol::Pose pose_start;
    rocontrol::Pose pose_end;
    pose_end.position.x() = 1;
    pose_end.orientation = Eigen::Quaterniond(
        Eigen::AngleAxisd(std::acos(-1.0) / 2, Eigen::Vector3d::UnitZ()));
    rocontrol::CartesianPoseReference pose_reference;
    assert(cartesian_trajectory.reset(pose_start, pose_end, std::chrono::seconds(2), start_time));
    assert(!cartesian_trajectory.reset(
        pose_start, pose_end, std::chrono::seconds(-1), start_time));
    assert(cartesian_trajectory.sample(start_time + std::chrono::seconds(1), pose_reference));
    assert(std::abs(pose_reference.pose.position.x() - 0.5) < 1e-12);
    assert(std::abs(pose_reference.linear_velocity.x() - 0.9375) < 1e-12);
    assert(std::abs(pose_reference.angular_velocity.z() - 0.9375 * std::acos(-1.0) / 2) < 1e-12);
    assert(!cartesian_trajectory.sample(start_time, pose_reference));
    assert(cartesian_trajectory.sample(start_time + std::chrono::seconds(2), pose_reference));
    assert((pose_reference.pose.position - pose_end.position).norm() < 1e-12);
    assert(pose_reference.linear_velocity.isZero() && pose_reference.angular_velocity.isZero());

    rocontrol::RobotModelConfig config;
    config.urdf = TEST_URDF_PATH;
    auto created = rocontrol::RobotModel::create(config);
    assert(created);
    auto model = std::move(created).value();
    rocontrol::RobotState state;
    state.stamp = start_time;
    state.position = Eigen::Vector2d::Zero();
    state.velocity = Eigen::Vector2d::Zero();
    assert(model->update(state));
    rocontrol::ControlCycle cycle{start_time, std::chrono::duration<double>(0), 0};
    rocontrol::StateContext context{state, *model, cycle};

    int a_enter = 0, a_update = 0, a_exit = 0;
    int b_enter = 0, b_update = 0, b_exit = 0;
    int c_enter = 0, c_update = 0, c_exit = 0;
    rocontrol::StateMachine fsm;
    auto a = fsm.add_state(std::make_unique<CountingState>(
        a_enter, a_update, a_exit, rocontrol::StateStatus::Succeeded));
    auto b = fsm.add_state(std::make_unique<CountingState>(
        b_enter, b_update, b_exit, rocontrol::StateStatus::Running));
    auto c = fsm.add_state(std::make_unique<CountingState>(
        c_enter, c_update, c_exit, rocontrol::StateStatus::Succeeded));
    assert(a && b && c);
    assert(fsm.add_transition(a.value(), rocontrol::StateStatus::Succeeded, b.value()));
    assert(fsm.start(a.value(), context));
    assert(a_enter == 1 && a_update == 0);
    assert(fsm.update(context));
    assert(a_update == 1 && a_exit == 0 && b_enter == 0);
    assert(fsm.request_transition(c.value()));
    assert(fsm.update(context));
    assert(a_exit == 1 && c_enter == 1 && c_update == 1 && b_enter == 0);
    assert(fsm.update(context));
    assert(c_exit == 1 && fsm.status() == rocontrol::FsmStatus::Completed);
}
