#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <chrono>
#include <cstdint>
#include <vector>

namespace rocontrol {

/** Index into RobotModel::dof_names() and every canonical joint vector. */
using JointIndex = std::uint32_t;
/** Index of a frame within one RobotModel. */
using FrameIndex = std::uint32_t;
/** Unique identity assigned when constructing a RobotModel. */
using ModelId = std::uint64_t;

/** Position in metres and orientation relative to the model root frame. */
struct Pose {
    /** Translation from root to frame origin, expressed in root, metres. */
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    /** Orientation from frame to root, represented by a unit quaternion. */
    Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
};

/** Joint vectors follow RobotModel::dof_names() order. Stamp uses steady_clock. */
struct RobotState {
    /** Sample time in the same monotonic clock domain as ControlCycle. */
    std::chrono::steady_clock::time_point stamp;
    /** Canonical DoF positions, rad or m. */
    Eigen::VectorXd position;
    /** Canonical DoF speeds, rad/s or m/s. */
    Eigen::VectorXd velocity;
};

/** Ordered independent controllable joint indices from one RobotModel. */
struct JointGroup {
    /** Identity of the model whose indices appear below. */
    ModelId model{0};
    /** Ordered canonical indices. */
    std::vector<JointIndex> joints;
};

/** QP output in canonical DoF order, rad/s or m/s by joint type. */
struct JointVelocityCommand {
    /** ControlCycle sequence; zero indicates an invalidated output. */
    std::uint64_t sequence{0};
    /** ControlCycle time in the monotonic clock domain. */
    std::chrono::steady_clock::time_point stamp;
    /** Canonical DoF speeds, rad/s or m/s; caller preallocates. */
    Eigen::VectorXd velocity;
};

}  // namespace rocontrol
