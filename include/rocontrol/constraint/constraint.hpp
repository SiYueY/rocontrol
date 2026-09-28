#pragma once

namespace rocontrol {

/** A hard feasible-region constraint. */
class Constraint {
public:
    /** Destroy a polymorphic hard constraint. */
    virtual ~Constraint() = default;
};

/** Canonical DoF velocity bounds from RobotModel. */
class JointVelocityLimitConstraint final : public Constraint {};

/** One-step position bounds from RobotModel and actual period. */
class JointPositionLimitConstraint final : public Constraint {};

}  // namespace rocontrol
