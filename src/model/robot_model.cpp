#include <rocontrol/model/robot_model.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <urdf_parser/urdf_parser.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <limits>
#include <map>
#include <stdexcept>
#include <utility>

namespace rocontrol {
namespace {

std::atomic<ModelId> next_model_id{1};

bool finite_or_infinite(double value) { return !std::isnan(value); }

struct MimicRelation {
    std::string independent_name;
    double multiplier{1};
    double offset{0};
};

Result<MimicRelation, ModelError> resolve_mimic(
    const urdf::ModelInterface& urdf_model, const std::string& name,
    std::size_t depth = 0) {
    if (depth > urdf_model.joints_.size()) {
        return Result<MimicRelation, ModelError>::failure(ModelError::InvalidUrdf);
    }
    const auto joint = urdf_model.getJoint(name);
    if (!joint) {
        return Result<MimicRelation, ModelError>::failure(ModelError::InvalidUrdf);
    }
    if (!joint->mimic) {
        return Result<MimicRelation, ModelError>::success({name, 1, 0});
    }
    const auto parent = resolve_mimic(urdf_model, joint->mimic->joint_name, depth + 1);
    if (!parent) {
        return parent;
    }
    const double multiplier = joint->mimic->multiplier * parent.value().multiplier;
    const double offset = joint->mimic->multiplier * parent.value().offset
        + joint->mimic->offset;
    if (!std::isfinite(multiplier) || !std::isfinite(offset)) {
        return Result<MimicRelation, ModelError>::failure(ModelError::InvalidLimits);
    }
    return Result<MimicRelation, ModelError>::success(
        {parent.value().independent_name, multiplier, offset});
}

}  // namespace

struct RobotModel::Impl {
    explicit Impl(pinocchio::Model robot_model)
        : model(std::move(robot_model)), data(model), id(next_model_id.fetch_add(1)) {
        configuration.setZero(model.nq);
        lower.setConstant(model.nv, -std::numeric_limits<double>::infinity());
        upper.setConstant(model.nv, std::numeric_limits<double>::infinity());
        velocity.setConstant(model.nv, std::numeric_limits<double>::infinity());
        last_position.setZero(model.nv);
        names.resize(model.nv);
        continuous.resize(model.nv, false);
        poses.resize(model.nframes);
        jacobians.resize(model.nframes);
        for (auto& jacobian : jacobians) {
            jacobian.setZero(6, model.nv);
        }
    }

    pinocchio::Model model;
    pinocchio::Data data;
    ModelId id;
    Eigen::VectorXd configuration;
    Eigen::VectorXd lower;
    Eigen::VectorXd upper;
    Eigen::VectorXd velocity;
    Eigen::VectorXd last_position;
    std::vector<std::string> names;
    std::vector<bool> continuous;
    std::map<std::string, JointIndex> joint_indices;
    std::map<std::string, JointGroup> groups;
    std::vector<Pose> poses;
    std::vector<Eigen::MatrixXd> jacobians;
    std::chrono::steady_clock::time_point last_stamp{};
    bool has_state{false};
};

RobotModel::RobotModel(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}
RobotModel::~RobotModel() = default;

Result<std::unique_ptr<RobotModel>, ModelError> RobotModel::create(
    const RobotModelConfig& config) {
    if (config.urdf.empty()) {
        return Result<std::unique_ptr<RobotModel>, ModelError>::failure(ModelError::InvalidConfig);
    }
    try {
        const auto urdf_model = urdf::parseURDFFile(config.urdf.string());
        if (!urdf_model) {
            return Result<std::unique_ptr<RobotModel>, ModelError>::failure(ModelError::InvalidUrdf);
        }

        pinocchio::Model pinocchio_model;
        pinocchio::urdf::buildModel(config.urdf.string(), pinocchio_model, false, true);
        auto impl = std::make_unique<Impl>(std::move(pinocchio_model));

        for (std::size_t model_joint = 1; model_joint < impl->model.joints.size(); ++model_joint) {
            const auto& name = impl->model.names[model_joint];
            const auto urdf_joint = urdf_model->getJoint(name);
            if (!urdf_joint) {
                return Result<std::unique_ptr<RobotModel>, ModelError>::failure(ModelError::InvalidUrdf);
            }
            if (urdf_joint->mimic) {
                continue;
            }
            if (urdf_joint->type != urdf::Joint::REVOLUTE
                && urdf_joint->type != urdf::Joint::PRISMATIC
                && urdf_joint->type != urdf::Joint::CONTINUOUS) {
                return Result<std::unique_ptr<RobotModel>, ModelError>::failure(ModelError::UnsupportedJoint);
            }
            if (impl->model.nvs[model_joint] != 1) {
                return Result<std::unique_ptr<RobotModel>, ModelError>::failure(ModelError::UnsupportedJoint);
            }
            const auto index = static_cast<JointIndex>(impl->model.idx_vs[model_joint]);
            if (index >= impl->names.size() || !impl->names[index].empty()) {
                return Result<std::unique_ptr<RobotModel>, ModelError>::failure(ModelError::InvalidUrdf);
            }
            impl->names[index] = name;
            impl->continuous[index] = urdf_joint->type == urdf::Joint::CONTINUOUS;
            impl->joint_indices.emplace(name, index);
            if (urdf_joint->limits) {
                impl->velocity[index] = urdf_joint->limits->velocity;
                if (urdf_joint->type != urdf::Joint::CONTINUOUS) {
                    impl->lower[index] = urdf_joint->limits->lower;
                    impl->upper[index] = urdf_joint->limits->upper;
                }
            }
        }
        if (std::any_of(impl->names.begin(), impl->names.end(),
                [](const std::string& name) { return name.empty(); })) {
            return Result<std::unique_ptr<RobotModel>, ModelError>::failure(ModelError::InvalidUrdf);
        }

        for (const auto& [name, override_limits] : config.limit_overrides) {
            const auto joint = impl->joint_indices.find(name);
            if (joint == impl->joint_indices.end()) {
                return Result<std::unique_ptr<RobotModel>, ModelError>::failure(ModelError::InvalidJoint);
            }
            const auto index = joint->second;
            if (override_limits.lower_position) impl->lower[index] = *override_limits.lower_position;
            if (override_limits.upper_position) impl->upper[index] = *override_limits.upper_position;
            if (override_limits.velocity) impl->velocity[index] = *override_limits.velocity;
        }

        for (const auto& [name, urdf_joint] : urdf_model->joints_) {
            if (!urdf_joint->mimic) {
                continue;
            }
            const auto relation = resolve_mimic(*urdf_model, name);
            if (!relation) {
                return Result<std::unique_ptr<RobotModel>, ModelError>::failure(relation.error());
            }
            const auto independent = impl->joint_indices.find(relation.value().independent_name);
            if (independent == impl->joint_indices.end()) {
                return Result<std::unique_ptr<RobotModel>, ModelError>::failure(ModelError::InvalidUrdf);
            }
            const auto index = independent->second;
            const double scale = relation.value().multiplier;
            const double offset = relation.value().offset;
            if (urdf_joint->limits) {
                if (std::abs(scale) > 0) {
                    const double first = (urdf_joint->limits->lower - offset) / scale;
                    const double second = (urdf_joint->limits->upper - offset) / scale;
                    impl->lower[index] = std::max(impl->lower[index], std::min(first, second));
                    impl->upper[index] = std::min(impl->upper[index], std::max(first, second));
                    impl->velocity[index] = std::min(
                        impl->velocity[index], urdf_joint->limits->velocity / std::abs(scale));
                } else if (offset < urdf_joint->limits->lower
                    || offset > urdf_joint->limits->upper) {
                    return Result<std::unique_ptr<RobotModel>, ModelError>::failure(ModelError::InvalidLimits);
                }
            }
        }

        for (Eigen::Index index = 0; index < impl->lower.size(); ++index) {
            if (!finite_or_infinite(impl->lower[index])
                || !finite_or_infinite(impl->upper[index])
                || !std::isfinite(impl->velocity[index])
                || impl->lower[index] > impl->upper[index]
                || impl->velocity[index] <= 0) {
                return Result<std::unique_ptr<RobotModel>, ModelError>::failure(ModelError::InvalidLimits);
            }
        }

        for (const auto& [name, members] : config.joint_groups) {
            JointGroup group;
            group.model = impl->id;
            group.joints.reserve(members.size());
            for (const auto& member : members) {
                const auto joint = impl->joint_indices.find(member);
                if (joint == impl->joint_indices.end()
                    || std::find(group.joints.begin(), group.joints.end(), joint->second)
                        != group.joints.end()) {
                    return Result<std::unique_ptr<RobotModel>, ModelError>::failure(ModelError::InvalidGroup);
                }
                group.joints.push_back(joint->second);
            }
            if (group.joints.empty()) {
                return Result<std::unique_ptr<RobotModel>, ModelError>::failure(ModelError::InvalidGroup);
            }
            impl->groups.emplace(name, std::move(group));
        }
        return Result<std::unique_ptr<RobotModel>, ModelError>::success(
            std::unique_ptr<RobotModel>(new RobotModel(std::move(impl))));
    } catch (const std::exception&) {
        return Result<std::unique_ptr<RobotModel>, ModelError>::failure(ModelError::InvalidUrdf);
    }
}

Result<void, ModelError> RobotModel::update(const RobotState& state) {
    if (state.position.size() != impl_->model.nv || state.velocity.size() != impl_->model.nv) {
        return Result<void, ModelError>::failure(ModelError::DimensionMismatch);
    }
    if (!state.position.allFinite() || !state.velocity.allFinite()) {
        return Result<void, ModelError>::failure(ModelError::InvalidState);
    }
    try {
        for (std::size_t joint = 1; joint < impl_->model.joints.size(); ++joint) {
            if (impl_->model.nvs[joint] != 1) continue;
            const auto velocity_index = impl_->model.idx_vs[joint];
            const auto config_index = impl_->model.idx_qs[joint];
            if (impl_->model.nqs[joint] == 1) {
                impl_->configuration[config_index] = state.position[velocity_index];
            } else if (impl_->model.nqs[joint] == 2) {
                impl_->configuration[config_index] = std::cos(state.position[velocity_index]);
                impl_->configuration[config_index + 1] = std::sin(state.position[velocity_index]);
            }
        }
        pinocchio::forwardKinematics(impl_->model, impl_->data, impl_->configuration);
        pinocchio::computeJointJacobians(impl_->model, impl_->data, impl_->configuration);
        pinocchio::updateFramePlacements(impl_->model, impl_->data);
        for (std::size_t frame_index = 0; frame_index < impl_->poses.size(); ++frame_index) {
            const auto& placement = impl_->data.oMf[frame_index];
            impl_->poses[frame_index].position = placement.translation();
            impl_->poses[frame_index].orientation = Eigen::Quaterniond(placement.rotation());
            pinocchio::getFrameJacobian(impl_->model, impl_->data, frame_index,
                pinocchio::LOCAL_WORLD_ALIGNED, impl_->jacobians[frame_index]);
        }
        impl_->last_position = state.position;
        impl_->last_stamp = state.stamp;
        impl_->has_state = true;
        return Result<void, ModelError>::success();
    } catch (const std::exception&) {
        return Result<void, ModelError>::failure(ModelError::NumericalFailure);
    }
}

ModelId RobotModel::id() const noexcept { return impl_->id; }
std::size_t RobotModel::dof() const noexcept { return impl_->names.size(); }
const std::vector<std::string>& RobotModel::dof_names() const noexcept { return impl_->names; }
Result<JointIndex, ModelError> RobotModel::joint(std::string_view name) const {
    const auto found = impl_->joint_indices.find(std::string(name));
    return found == impl_->joint_indices.end()
        ? Result<JointIndex, ModelError>::failure(ModelError::InvalidJoint)
        : Result<JointIndex, ModelError>::success(found->second);
}
Result<JointGroup, ModelError> RobotModel::joint_group(std::string_view name) const {
    const auto found = impl_->groups.find(std::string(name));
    return found == impl_->groups.end()
        ? Result<JointGroup, ModelError>::failure(ModelError::InvalidGroup)
        : Result<JointGroup, ModelError>::success(found->second);
}
Result<FrameIndex, ModelError> RobotModel::frame(std::string_view name) const {
    const auto index = impl_->model.getFrameId(std::string(name));
    return index >= static_cast<std::size_t>(impl_->model.nframes)
        ? Result<FrameIndex, ModelError>::failure(ModelError::InvalidFrame)
        : Result<FrameIndex, ModelError>::success(static_cast<FrameIndex>(index));
}
bool RobotModel::valid_frame(FrameIndex frame) const noexcept {
    return frame < impl_->poses.size();
}
const Pose& RobotModel::frame_pose(FrameIndex frame) const { return impl_->poses.at(frame); }
const Eigen::MatrixXd& RobotModel::frame_jacobian(FrameIndex frame) const {
    return impl_->jacobians.at(frame);
}
const Eigen::VectorXd& RobotModel::lower_position_limits() const noexcept { return impl_->lower; }
const Eigen::VectorXd& RobotModel::upper_position_limits() const noexcept { return impl_->upper; }
const Eigen::VectorXd& RobotModel::velocity_limits() const noexcept { return impl_->velocity; }
const Eigen::VectorXd& RobotModel::joint_positions() const noexcept { return impl_->last_position; }
bool RobotModel::is_continuous(JointIndex index) const noexcept {
    return index < impl_->continuous.size() && impl_->continuous[index];
}
std::chrono::steady_clock::time_point RobotModel::last_state_stamp() const noexcept {
    return impl_->last_stamp;
}
bool RobotModel::has_state() const noexcept { return impl_->has_state; }

}  // namespace rocontrol
