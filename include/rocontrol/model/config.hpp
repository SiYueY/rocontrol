#pragma once

#include <filesystem>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace rocontrol {

/** Optional independent-joint limits in native coordinates and units. */
struct JointLimitOverride {
    /** Lower joint position, in rad or m by joint type. */
    std::optional<double> lower_position;
    /** Upper joint position, in rad or m by joint type. */
    std::optional<double> upper_position;
    /** Symmetric speed magnitude, in rad/s or m/s. */
    std::optional<double> velocity;
};

/** Inputs parsed once when constructing a fixed-base RobotModel. */
struct RobotModelConfig {
    /** Path to a fixed-base URDF. */
    std::filesystem::path urdf;
    /** Named groups; each member name defines the group's order. */
    std::map<std::string, std::vector<std::string>> joint_groups;
    /** Independent joint name to explicit limit override. */
    std::map<std::string, JointLimitOverride> limit_overrides;
};

}  // namespace rocontrol
