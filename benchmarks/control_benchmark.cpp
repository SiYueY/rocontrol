#include <rocontrol/controller.hpp>
#include <rocontrol/fsm/state_machine.hpp>
#include <rocontrol/solver/qp_solver.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <new>
#include <string>
#include <thread>
#include <vector>

namespace {

std::atomic<std::size_t> allocations{0};

class IdleState final : public rocontrol::State {
public:
    rocontrol::Result<void, rocontrol::FsmError>
    enter(const rocontrol::StateContext&) override {
        return rocontrol::Result<void, rocontrol::FsmError>::success();
    }
    rocontrol::Result<rocontrol::StateStatus, rocontrol::FsmError>
    update(const rocontrol::StateContext&) override {
        return rocontrol::Result<rocontrol::StateStatus, rocontrol::FsmError>::success(
            rocontrol::StateStatus::Running);
    }
    rocontrol::Result<void, rocontrol::FsmError>
    exit(const rocontrol::StateContext&) override {
        return rocontrol::Result<void, rocontrol::FsmError>::success();
    }
};

struct Statistics {
    double p50;
    double p95;
    double p99;
    double maximum;
};

Statistics summarize(std::vector<double>& values) {
    std::sort(values.begin(), values.end());
    const auto percentile = [&](double fraction) {
        return values[static_cast<std::size_t>((values.size() - 1) * fraction)];
    };
    return {percentile(0.5), percentile(0.95), percentile(0.99), values.back()};
}

void print(const char* label, const Statistics& result) {
    std::cout << label << " ms: p50=" << result.p50 << " p95=" << result.p95
              << " p99=" << result.p99 << " max=" << result.maximum << '\n';
}

double milliseconds(std::chrono::steady_clock::duration duration) {
    return std::chrono::duration<double, std::milli>(duration).count();
}

}  // namespace

void* operator new(std::size_t size) {
    allocations.fetch_add(1, std::memory_order_relaxed);
    if (void* memory = std::malloc(size)) return memory;
    throw std::bad_alloc();
}
void* operator new[](std::size_t size) {
    allocations.fetch_add(1, std::memory_order_relaxed);
    if (void* memory = std::malloc(size)) return memory;
    throw std::bad_alloc();
}
void operator delete(void* memory) noexcept { std::free(memory); }
void operator delete[](void* memory) noexcept { std::free(memory); }
void operator delete(void* memory, std::size_t) noexcept { std::free(memory); }
void operator delete[](void* memory, std::size_t) noexcept { std::free(memory); }

int main(int argc, char** argv) {
    const std::string mode = argc > 1 ? argv[1] : "--compute";
    if (mode != "--compute" && mode != "--realtime" && mode != "--spin") {
        std::cerr << "usage: rocontrol_benchmark [--compute|--realtime|--spin]\n";
        return 1;
    }
    const bool realtime = mode != "--compute";
    const bool spin = mode == "--spin";
    constexpr std::size_t cycles = 300000;
    rocontrol::RobotModelConfig config;
    config.urdf = TEST_URDF_PATH;
    config.joint_groups["spine"] = {"spine"};
    auto created = rocontrol::RobotModel::create(config);
    if (!created) return 1;
    auto model = std::move(created).value();
    const auto spine_index = model->joint("spine");
    const auto left_frame = model->frame("left_ee");
    const auto right_frame = model->frame("right_ee");
    if (!spine_index || !left_frame || !right_frame) return 1;

    rocontrol::RobotState state;
    state.position = Eigen::VectorXd::Zero(model->dof());
    state.velocity = Eigen::VectorXd::Zero(model->dof());
    state.position[spine_index.value()] = 0.1;
    const auto initial_time = std::chrono::steady_clock::now();
    state.stamp = initial_time;
    if (!model->update(state)) return 1;
    auto left_target = model->frame_pose(left_frame.value());
    auto right_target = model->frame_pose(right_frame.value());
    left_target.position.y() += 0.03;
    right_target.position.y() -= 0.03;

    rocontrol::QPSolver solver;
    rocontrol::Controller controller(*model, solver);
    auto left = controller.add_cartesian_pose_task(left_frame.value());
    auto right = controller.add_cartesian_pose_task(right_frame.value());
    auto spine = controller.add_joint_position_task(model->joint_group("spine").value());
    if (!left || !right || !spine) return 1;
    Eigen::VectorXd spine_target(1);
    spine_target[0] = 0.2;
    if (!left.value().get().set_target(left_target)
        || !right.value().get().set_target(right_target)
        || !spine.value().get().set_target(spine_target)) return 1;
    left.value().get().set_enabled(true);
    right.value().get().set_enabled(true);
    spine.value().get().set_enabled(true);
    if (!controller.start(state)) return 1;

    rocontrol::StateMachine fsm;
    auto idle = fsm.add_state(std::make_unique<IdleState>());
    rocontrol::ControlCycle cycle{initial_time, std::chrono::duration<double>(0), 0};
    rocontrol::StateContext context{state, *model, cycle};
    if (!idle || !fsm.start(idle.value(), context)) return 1;
    rocontrol::JointVelocityCommand command;
    command.velocity.setZero(model->dof());
    std::vector<double> model_times(cycles), fsm_times(cycles), solver_times(cycles);
    std::vector<double> controller_times(cycles), core_times(cycles);
    const auto allocation_baseline = allocations.load(std::memory_order_relaxed);
    const auto schedule_start = std::chrono::steady_clock::now();
    auto previous_time = schedule_start;
    for (std::size_t index = 0; index < cycles; ++index) {
        if (realtime) {
            const auto deadline = schedule_start + std::chrono::milliseconds(2 * (index + 1));
            if (spin) {
                while (std::chrono::steady_clock::now() < deadline) {}
            } else {
                std::this_thread::sleep_until(deadline);
            }
        }
        const auto begin = std::chrono::steady_clock::now();
        cycle.time = realtime ? begin : schedule_start + std::chrono::milliseconds(2 * (index + 1));
        cycle.period = realtime
            ? std::chrono::duration<double>(begin - previous_time)
            : std::chrono::duration<double>(0.002);
        previous_time = begin;
        cycle.sequence = index + 1;
        state.stamp = cycle.time;
        if (index % 1000 == 0) {
            state.position.setZero();
            state.position[spine_index.value()] = 0.1;
        }
        if (!model->update(state)) return 2;
        const auto after_model = std::chrono::steady_clock::now();
        auto fsm_result = fsm.update(context);
        if (!fsm_result || fsm_result.value() != rocontrol::FsmStatus::Running) return 3;
        const auto after_fsm = std::chrono::steady_clock::now();
        const auto controlled = controller.update(state, cycle, command);
        if (!controlled) {
            std::cerr << "controller failure: cycle=" << cycle.sequence
                      << " actual_period_ms=" << cycle.period.count() * 1000
                      << " error=" << static_cast<int>(controlled.error()) << '\n';
            return 4;
        }
        const auto after_controller = std::chrono::steady_clock::now();
        state.position += command.velocity * cycle.period.count();
        state.velocity = command.velocity;
        model_times[index] = milliseconds(after_model - begin);
        fsm_times[index] = milliseconds(after_fsm - after_model);
        solver_times[index] = std::chrono::duration<double, std::milli>(
            solver.last_solve_duration()).count();
        controller_times[index] = milliseconds(after_controller - after_fsm);
        core_times[index] = milliseconds(after_controller - begin);
    }
    const auto new_allocations = allocations.load(std::memory_order_relaxed) - allocation_baseline;
    print("model", summarize(model_times));
    print("FSM", summarize(fsm_times));
    print("QP", summarize(solver_times));
    print("Controller", summarize(controller_times));
    const auto core = summarize(core_times);
    print("Core", core);
    std::cout << "cycles=" << cycles << " allocations=" << new_allocations
              << " mode=" << mode << '\n';
    return new_allocations == 0 && core.p99 <= 1.0 && core.maximum <= 1.5 ? 0 : 5;
}
