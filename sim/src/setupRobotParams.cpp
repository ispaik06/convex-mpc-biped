#include "setupRobotParams.h"

#include <array>
#include <cmath>
#include <stdexcept>
#include <string>

#include <mujoco/mujoco.h>

namespace {
template <typename T>
int requireId(const mjModel* model, int objType, const std::string& name, const char* what) {
    const int id = mj_name2id(model, objType, name.c_str());
    if (id < 0) {
        throw std::runtime_error(std::string("Failed to find ") + what + ": " + name);
    }
    return id;
}

template <typename T>
int optionalId(const mjModel* model, int objType, const std::string& name) {
    if (name.empty()) {
        return -1;
    }
    return mj_name2id(model, objType, name.c_str());
}

template <typename T>
void fillDefaultQpos(const mjModel* model, RobotParams<T>& params) {
    params.default_qpos.resize(model->nq);
    for (int i = 0; i < model->nq; ++i) {
        params.default_qpos[i] = static_cast<T>(model->qpos0[i]);
    }
}

template <typename T>
void fillBodyMassProperties(const mjModel* model,
                            const std::string& baseBodyName,
                            RobotParams<T>& params) {
    params.bodyMass = T(0);
    for (int i = 1; i < model->nbody; ++i) {
        params.bodyMass += static_cast<T>(model->body_mass[i]);
    }

    const int baseBodyId = requireId<T>(model, mjOBJ_BODY, baseBodyName, "base body");
    params.bodyInertia.setZero();
    params.bodyInertia(0, 0) = static_cast<T>(model->body_inertia[3 * baseBodyId + 0]);
    params.bodyInertia(1, 1) = static_cast<T>(model->body_inertia[3 * baseBodyId + 1]);
    params.bodyInertia(2, 2) = static_cast<T>(model->body_inertia[3 * baseBodyId + 2]);
    params.bodyComLocation << static_cast<T>(model->body_ipos[3 * baseBodyId + 0]),
        static_cast<T>(model->body_ipos[3 * baseBodyId + 1]),
        static_cast<T>(model->body_ipos[3 * baseBodyId + 2]);
}

template <typename T, std::size_t NJ, std::size_t NA>
void fillJointGroup(const mjModel* model,
                    const std::array<const char*, NJ>& jointNames,
                    const std::array<const char*, NA>& actuatorNames,
                    JointGroupParams<T>& group) {
    static_assert(NJ == NA, "joint and actuator name counts must match");

    group.q_idx.clear();
    group.qd_idx.clear();
    group.actuator_idx.clear();
    group.motorTauMax.resize(static_cast<Eigen::Index>(NA));

    for (std::size_t i = 0; i < jointNames.size(); ++i) {
        const int jointId = requireId<T>(model, mjOBJ_JOINT, jointNames[i], "joint");
        const int actuatorId = requireId<T>(model, mjOBJ_ACTUATOR, actuatorNames[i], "actuator");

        group.q_idx.push_back(model->jnt_qposadr[jointId]);
        group.qd_idx.push_back(model->jnt_dofadr[jointId]);
        group.actuator_idx.push_back(actuatorId);

        const mjtNum lo = model->actuator_ctrlrange[2 * actuatorId + 0];
        const mjtNum hi = model->actuator_ctrlrange[2 * actuatorId + 1];
        group.motorTauMax[static_cast<Eigen::Index>(i)] =
            static_cast<T>(std::max(std::abs(lo), std::abs(hi)));
    }
}

template <typename T>
Vec3<T> firstJointLocationFromBase(const mjModel* model,
                                   const std::string& firstJointName) {
    const int jointId = requireId<T>(model, mjOBJ_JOINT, firstJointName, "joint");
    const int bodyId = model->jnt_bodyid[jointId];

    Vec3<T> hip = Vec3<T>::Zero();
    hip << static_cast<T>(model->body_pos[3 * bodyId + 0]),
        static_cast<T>(model->body_pos[3 * bodyId + 1]),
        static_cast<T>(model->body_pos[3 * bodyId + 2]);
    return hip;
}

template <typename T, std::size_t NJ, std::size_t NA>
LegParams<T> makeLeg(const mjModel* model,
                     Side side,
                     const std::array<const char*, NJ>& jointNames,
                     const std::array<const char*, NA>& actuatorNames,
                     const char* footBodyName,
                     const char* footSiteName) {
    LegParams<T> leg;
    leg.side = side;
    fillJointGroup(model, jointNames, actuatorNames, leg.joints);

    leg.foot.body_name = footBodyName;
    leg.foot.site_name = footSiteName ? footSiteName : "";
    leg.foot.body_id = requireId<T>(model, mjOBJ_BODY, leg.foot.body_name, "foot body");
    leg.foot.site_id = optionalId<T>(model, mjOBJ_SITE, leg.foot.site_name);
    leg.hipLocation_from_body = firstJointLocationFromBase<T>(model, jointNames.front());
    return leg;
}

template <typename T, std::size_t NJ, std::size_t NA>
ArmParams<T> makeArm(const mjModel* model,
                     Side side,
                     const std::array<const char*, NJ>& jointNames,
                     const std::array<const char*, NA>& actuatorNames,
                     const char* handBodyName,
                     const char* handSiteName) {
    ArmParams<T> arm;
    arm.side = side;
    fillJointGroup(model, jointNames, actuatorNames, arm.joints);

    arm.hand.body_name = handBodyName;
    arm.hand.site_name = handSiteName ? handSiteName : "";
    arm.hand.body_id = requireId<T>(model, mjOBJ_BODY, arm.hand.body_name, "hand body");
    arm.hand.site_id = optionalId<T>(model, mjOBJ_SITE, arm.hand.site_name);
    return arm;
}

template <typename T>
RobotParams<T> setupMitHumanoidParams(const mjModel* model) {
    RobotParams<T> params;
    params.roboType = RobotType::MIT_HUMANOID;
    params.nq = model->nq;
    params.nv = model->nv;
    params.nu = model->nu;

    fillDefaultQpos(model, params);
    fillBodyMassProperties(model, "torso", params);

    const std::array<const char*, 5> leftLegJoints = {
        "left_hip_yaw_joint",
        "left_hip_abad_joint",
        "left_hip_pitch_joint",
        "left_knee_joint",
        "left_ankle_joint",
    };
    const std::array<const char*, 5> leftLegActuators = {
        "left_hip_yaw",
        "left_hip_abad",
        "left_hip_pitch",
        "left_knee",
        "left_ankle",
    };
    const std::array<const char*, 5> rightLegJoints = {
        "right_hip_yaw_joint",
        "right_hip_abad_joint",
        "right_hip_pitch_joint",
        "right_knee_joint",
        "right_ankle_joint",
    };
    const std::array<const char*, 5> rightLegActuators = {
        "right_hip_yaw",
        "right_hip_abad",
        "right_hip_pitch",
        "right_knee",
        "right_ankle",
    };
    const std::array<const char*, 4> leftArmJoints = {
        "left_shoulder_pitch_joint",
        "left_shoulder_abad_joint",
        "left_shoulder_yaw_joint",
        "left_elbow_joint",
    };
    const std::array<const char*, 4> leftArmActuators = {
        "left_shoulder_pitch",
        "left_shoulder_abad",
        "left_shoulder_yaw",
        "left_elbow",
    };
    const std::array<const char*, 4> rightArmJoints = {
        "right_shoulder_pitch_joint",
        "right_shoulder_abad_joint",
        "right_shoulder_yaw_joint",
        "right_elbow_joint",
    };
    const std::array<const char*, 4> rightArmActuators = {
        "right_shoulder_pitch",
        "right_shoulder_abad",
        "right_shoulder_yaw",
        "right_elbow",
    };

    params.legs.reserve(2);
    params.legs.push_back(
        makeLeg<T>(model, Side::Left, leftLegJoints, leftLegActuators, "left_foot_link", ""));
    params.legs.push_back(
        makeLeg<T>(model, Side::Right, rightLegJoints, rightLegActuators, "right_foot_link", ""));

    params.arms.reserve(2);
    params.arms.push_back(
        makeArm<T>(model, Side::Left, leftArmJoints, leftArmActuators, "left_forearm_link", ""));
    params.arms.push_back(makeArm<T>(
        model, Side::Right, rightArmJoints, rightArmActuators, "right_forearm_link", ""));

    return params;
}
}  // namespace

template <typename T>
RobotParams<T> setupRobotParams(const RobotType robotType, const mjModel_* model) {
    if (model == nullptr) {
        throw std::runtime_error("setupRobotParams received null mjModel");
    }

    switch (robotType) {
        case RobotType::MIT_HUMANOID:
            return setupMitHumanoidParams<T>(reinterpret_cast<const mjModel*>(model));
        default:
            throw std::runtime_error("setupRobotParams currently supports only MIT_HUMANOID");
    }
}

template RobotParams<float> setupRobotParams<float>(RobotType, const mjModel_*);
template RobotParams<double> setupRobotParams<double>(RobotType, const mjModel_*);
