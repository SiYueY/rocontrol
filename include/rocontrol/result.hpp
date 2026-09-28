#pragma once

#include <utility>
#include <variant>

namespace rocontrol {

/** A value or a typed error. Neither state requires allocation. */
template <typename T, typename E>
class Result {
public:
    /** Construct a successful result by moving its value. */
    static Result success(T value) { return Result(std::in_place_index<0>, std::move(value)); }
    /** Construct a failed result with a typed error. */
    static Result failure(E error) { return Result(std::in_place_index<1>, error); }

    /** True exactly when a value is available. */
    explicit operator bool() const noexcept { return value_.index() == 0; }
    /** Access the successful value; caller must check success first. */
    T& value() & { return std::get<0>(value_); }
    /** Access the successful value; caller must check success first. */
    const T& value() const& { return std::get<0>(value_); }
    /** Move the successful value; caller must check success first. */
    T&& value() && { return std::get<0>(std::move(value_)); }
    /** Access the typed error; caller must check failure first. */
    E error() const { return std::get<1>(value_); }

private:
    template <typename... Args>
    Result(std::in_place_index_t<0> index, Args&&... args)
        : value_(index, std::forward<Args>(args)...) {}
    Result(std::in_place_index_t<1> index, E error) : value_(index, error) {}
    std::variant<T, E> value_;
};

/** Success without a value or a typed error. */
template <typename E>
class Result<void, E> {
public:
    /** Construct a successful result without a value. */
    static Result success() noexcept { return Result(true, E{}); }
    /** Construct a failed result with a typed error. */
    static Result failure(E error) noexcept { return Result(false, error); }
    /** True exactly when the operation succeeded. */
    explicit operator bool() const noexcept { return success_; }
    /** Access the typed error; caller must check failure first. */
    E error() const noexcept { return error_; }

private:
    Result(bool success, E error) noexcept : success_(success), error_(error) {}
    bool success_;
    E error_;
};

}  // namespace rocontrol
