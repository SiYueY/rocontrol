#pragma once

#include <rocontrol/result.hpp>

namespace rocontrol {

/** Task setter validation failures; setters leave the old value unchanged. */
enum class TaskError { InvalidTarget, DimensionMismatch, InvalidGain, InvalidWeight };

/** Weighted objective; disabled tasks do not contribute to QP cost. */
class Task {
public:
    /** Destroy a polymorphic QP objective. */
    virtual ~Task() = default;
    /** Whether this objective contributes to the next QP. */
    bool enabled() const noexcept { return enabled_; }
    /** Control-thread-only toggle; target must already be initialized. */
    void set_enabled(bool enabled) noexcept { enabled_ = enabled; }
    /** Positive scalar objective weight; not a strict priority. */
    double weight() const noexcept { return weight_; }
    /** Set a finite positive weight without changing task topology. */
    Result<void, TaskError> set_weight(double weight);

private:
    bool enabled_{false};
    double weight_{1.0};
};

}  // namespace rocontrol
