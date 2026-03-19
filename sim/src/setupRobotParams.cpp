#include <cmath>
#include <stdexcept>
#include <string>
#include <string_view>

#include <mujoco/mujoco.h>

#include "setupRobotParams.h"
#include "models/RobotMujocoSpec.h"

namespace {
std::string asString(std::string_view value) {
    return std::string(value);
}

template <typename T>
int requireId(const mjModel* model, int objType, std::string_view name, const char* what) {
    const std::string key = asString(name);
    const int id = mj_name2id(model, objType, key.c_str());
    if (id < 0) {
        throw std::runtime_error(std::string("Failed to find ") + what + ": " + key);
    }
    return id;
}

template <typename T>
int optionalId(const mjModel* model, int objType, std::string_view name) {
    if (name.empty()) {
        return -1;
    }
    const std::string key = asString(name);
    return mj_name2id(model, objType, key.c_str());
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
                            std::string_view baseBodyName,
                            RobotParams<T>& params) {
    params.bodyMass = T(0);
    // for (int i = 1; i < model->nbody; ++i) {
    //     params.bodyMass += static_cast<T>(model->body_mass[i]);
    // }
    int baseBodyidx = requireId(model, mjOBJ_BODY, baseBodyName, "base body");
    params.bodyMass += static_cast<T>(model->body_mass[baseBodyidx];

    const int baseBodyId = requireId<T>(model, mjOBJ_BODY, baseBodyName, "base body");
    params.bodyInertia.setZero();
    params.bodyInertia(0, 0) = static_cast<T>(model->body_inertia[3 * baseBodyId + 0]);
    params.bodyInertia(1, 1) = static_cast<T>(model->body_inertia[3 * baseBodyId + 1]);
    params.bodyInertia(2, 2) = static_cast<T>(model->body_inertia[3 * baseBodyId + 2]);
    params.bodyComLocation << static_cast<T>(model->body_ipos[3 * baseBodyId + 0]),
        static_cast<T>(model->body_ipos[3 * baseBodyId + 1]),
        static_cast<T>(model->body_ipos[3 * baseBodyId + 2]);
}

template <typename T>
void fillJointGroup(const mjModel* model,
                    const std::vector<JointActuatorSpec>& jointSpecs,
                    JointGroupParams<T>& group) {
    group.q_idx.clear();
    group.qd_idx.clear();
    group.actuator_idx.clear();
    group.motorTauMax.resize(static_cast<Eigen::Index>(jointSpecs.size()));

    for (std::size_t i = 0; i < jointSpecs.size(); ++i) {
        const auto& spec = jointSpecs[i];
        const int jointId = requireId<T>(model, mjOBJ_JOINT, spec.joint, "joint");
        const int actuatorId = requireId<T>(model, mjOBJ_ACTUATOR, spec.actuator, "actuator");

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
                                   const std::vector<JointActuatorSpec>& jointSpecs) {
    if (jointSpecs.empty()) {
        throw std::runtime_error("Limb spec has no joints");
    }

    const int jointId = requireId<T>(model, mjOBJ_JOINT, jointSpecs.front().joint, "joint");
    const int bodyId = model->jnt_bodyid[jointId];

    Vec3<T> hip = Vec3<T>::Zero();
    hip << static_cast<T>(model->body_pos[3 * bodyId + 0]),
        static_cast<T>(model->body_pos[3 * bodyId + 1]),
        static_cast<T>(model->body_pos[3 * bodyId + 2]);
    return hip;
}

template <typename T>
LegParams<T> makeLeg(const mjModel* model, const LimbMujocoSpec& spec) {
    LegParams<T> leg;
    leg.side = spec.side;
    fillJointGroup(model, spec.joints, leg.joints);

    leg.foot.body_name = std::string(spec.endBody);
    leg.foot.site_name = std::string(spec.endSite);
    leg.foot.body_id = requireId<T>(model, mjOBJ_BODY, leg.foot.body_name, "foot body");
    leg.foot.site_id = optionalId<T>(model, mjOBJ_SITE, leg.foot.site_name);
    leg.hipLocation_from_body = firstJointLocationFromBase<T>(model, spec.joints);
    return leg;
}

template <typename T>
ArmParams<T> makeArm(const mjModel* model, const LimbMujocoSpec& spec) {
    ArmParams<T> arm;
    arm.side = spec.side;
    fillJointGroup(model, spec.joints, arm.joints);

    arm.hand.body_name = std::string(spec.endBody);
    arm.hand.site_name = std::string(spec.endSite);
    arm.hand.body_id = requireId<T>(model, mjOBJ_BODY, arm.hand.body_name, "hand body");
    arm.hand.site_id = optionalId<T>(model, mjOBJ_SITE, arm.hand.site_name);
    return arm;
}

template <typename T>
RobotParams<T> buildRobotParamsFromSpec(const mjModel* model, const RobotMujocoSpec& spec) {
    RobotParams<T> params;
    params.roboType = spec.type;
    params.nq = model->nq;
    params.nv = model->nv;
    params.nu = model->nu;

    fillDefaultQpos(model, params);
    fillBodyMassProperties(model, spec.baseBody, params);

    params.legs.reserve(spec.legs.size());
    for (const auto& legSpec : spec.legs) {
        params.legs.push_back(makeLeg<T>(model, legSpec));
    }

    params.arms.reserve(spec.arms.size());
    for (const auto& armSpec : spec.arms) {
        params.arms.push_back(makeArm<T>(model, armSpec));
    }

    return params;
}
}  // namespace

template <typename T>
RobotParams<T> setupRobotParams(const RobotType robotType, const mjModel_* model) {
    if (model == nullptr) {
        throw std::runtime_error("setupRobotParams received null mjModel");
    }

    const auto& spec = getRobotMujocoSpec(robotType);
    return buildRobotParamsFromSpec<T>(reinterpret_cast<const mjModel*>(model), spec);
}

template RobotParams<float> setupRobotParams<float>(RobotType, const mjModel_*);
template RobotParams<double> setupRobotParams<double>(RobotType, const mjModel_*);
