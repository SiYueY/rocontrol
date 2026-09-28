#pragma once

#include <chrono>
#include <cstdint>

namespace rocontrol {

/** Measured control cycle. period is invalid during startup synchronization. */
struct ControlCycle {
    /** Current control-cycle time in the steady clock domain. */
    std::chrono::steady_clock::time_point time;
    /** Measured interval since the last normal cycle, in seconds. */
    std::chrono::duration<double> period{0};
    /** Monotonic cycle number; zero is reserved for startup/invalid output. */
    std::uint64_t sequence{0};
};

}  // namespace rocontrol
