#include <cmath>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include <mujoco/mujoco.h>

#include "setupRobotParams.h"
#include "models/RobotMujocoSpec.h"
#include "Utilities/MatrixUtils.h"

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

bool isCollisionGeomCandidate(const mjModel* model, const int geomId) {
    return model->geom_contype[geomId] != 0 ||
           model->geom_conaffinity[geomId] != 0 ||
           model->geom_group[geomId] == 3;
}

std::vector<int> collisionGeomIdsForBody(const mjModel* model, const int bodyId) {
    std::vector<int> geomIds;
    for (int geomId = 0; geomId < model->ngeom; ++geomId) {
        if (model->geom_bodyid[geomId] == bodyId && isCollisionGeomCandidate(model, geomId)) {
            geomIds.push_back(geomId);
        }
    }
    return geomIds;
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
                            std::string_view torsoBodyName,
                            RobotParams<T>& params,
                            MujocoRobotBindings& bindings) {
    params.bodyMass = T(0);
    for (int i = 1; i < model->nbody; ++i) {
        params.bodyMass += static_cast<T>(model->body_mass[i]);
    }
    const int torsoBodyId = requireId<T>(model, mjOBJ_BODY, torsoBodyName, "base body");
    bindings.torsoBodyId = torsoBodyId;

    params.bodyInertia.setZero();
    params.bodyInertia(0, 0) = static_cast<T>(model->body_inertia[3 * torsoBodyId + 0]);
    params.bodyInertia(1, 1) = static_cast<T>(model->body_inertia[3 * torsoBodyId + 1]);
    params.bodyInertia(2, 2) = static_cast<T>(model->body_inertia[3 * torsoBodyId + 2]);
    params.bodyComLocation << static_cast<T>(model->body_ipos[3 * torsoBodyId + 0]),
        static_cast<T>(model->body_ipos[3 * torsoBodyId + 1]),
        static_cast<T>(model->body_ipos[3 * torsoBodyId + 2]);
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
void fillFixedJoints(const mjModel* model,
                     const std::vector<FixedJointMujocoSpec>& jointSpecs,
                     RobotParams<T>& params) {
    params.fixedJoints.clear();
    params.fixedJoints.reserve(jointSpecs.size());

    for (const auto& spec : jointSpecs) {
        const int jointId = requireId<T>(model, mjOBJ_JOINT, spec.joint, "fixed joint");
        const int actuatorId = requireId<T>(model, mjOBJ_ACTUATOR, spec.actuator, "fixed joint actuator");
        const int qIdx = model->jnt_qposadr[jointId];
        const int qdIdx = model->jnt_dofadr[jointId];

        FixedJointParams<T> joint;
        joint.name = asString(spec.joint);
        joint.q_idx = qIdx;
        joint.qd_idx = qdIdx;
        joint.actuator_idx = actuatorId;
        joint.qDefault = static_cast<T>(model->qpos0[qIdx]);
        joint.kp = static_cast<T>(spec.kp);
        joint.kd = static_cast<T>(spec.kd);
        params.fixedJoints.push_back(joint);
    }
}

template <typename T>
int firstJointBodyId(const mjModel* model, const std::vector<JointActuatorSpec>& jointSpecs) {
    if (jointSpecs.empty()) {
        throw std::runtime_error("Limb spec has no joints");
    }

    const int jointId = requireId<T>(model, mjOBJ_JOINT, jointSpecs.front().joint, "joint");
    return model->jnt_bodyid[jointId];
}

template <typename T>
Vec3<T> firstJointLocationFromBase(const mjModel* model,
                                   const std::vector<JointActuatorSpec>& jointSpecs) {
    const int bodyId = firstJointBodyId<T>(model, jointSpecs);

    Vec3<T> hip = Vec3<T>::Zero();
    hip << static_cast<T>(model->body_pos[3 * bodyId + 0]),
        static_cast<T>(model->body_pos[3 * bodyId + 1]),
        static_cast<T>(model->body_pos[3 * bodyId + 2]);
    return hip;
}

template <typename T>
bool isDescendantOf(const mjModel* model, int bodyId, int ancestorId) {
    while (bodyId > 0) {
        if (bodyId == ancestorId) {
            return true;
        }
        bodyId = model->body_parentid[bodyId];
    }
    return bodyId == ancestorId;
}

template <typename T>
bool isInLegSubtree(const mjModel* model, const MujocoRobotBindings& bindings, const int bodyId) {
    for (const auto& foot : bindings.feet) {
        if (foot.rootBodyId >= 0 && isDescendantOf<T>(model, bodyId, foot.rootBodyId)) {
            return true;
        }
    }
    return false;
}

template <typename T>
void fillLeg(const mjModel* model,
             const LimbMujocoSpec& spec,
             FootEndEffectorSource footEndEffectorSource,
             LegParams<T>& leg,
             MujocoEndEffectorBinding& foot_binding) {
    leg.side = spec.side;
    fillJointGroup(model, spec.joints, leg.joints);
    foot_binding.rootBodyId = firstJointBodyId<T>(model, spec.joints);
    foot_binding.bodyId = requireId<T>(model, mjOBJ_BODY, spec.endBody, "foot body");
    foot_binding.siteId = spec.endSite.empty()
                              ? -1
                              : requireId<T>(model, mjOBJ_SITE, spec.endSite, "foot site");
    foot_binding.collisionGeomIds = collisionGeomIdsForBody(model, foot_binding.bodyId);
    if (footEndEffectorSource == FootEndEffectorSource::Site && spec.endSite.empty()) {
        throw std::runtime_error("Foot end-effector source is site, but the leg spec has no endSite");
    }
    if (footEndEffectorSource == FootEndEffectorSource::CollisionGeomCenter &&
        foot_binding.collisionGeomIds.empty()) {
        throw std::runtime_error(
            "Foot end-effector source is collision_geom_center, but the foot body has no collision geoms");
    }
    leg.hipLocationFromBody = firstJointLocationFromBase<T>(model, spec.joints);
}

template <typename T>
void fillArm(const mjModel* model,
             const LimbMujocoSpec& spec,
             ArmParams<T>& arm,
             MujocoEndEffectorBinding& hand_binding) {
    arm.side = spec.side;
    fillJointGroup(model, spec.joints, arm.joints);
    hand_binding.rootBodyId = firstJointBodyId<T>(model, spec.joints);
    hand_binding.bodyId = requireId<T>(model, mjOBJ_BODY, spec.endBody, "hand body");
    hand_binding.siteId = optionalId<T>(model, mjOBJ_SITE, spec.endSite);
}

template <typename T>
void fillReducedBodyMassProperties(const mjModel* model,
                                   std::string_view torsoBodyName,
                                   const MujocoRobotBindings& bindings,
                                   RobotParams<T>& params) {
    const int torsoBodyId = requireId<T>(model, mjOBJ_BODY, torsoBodyName, "base body");

    T upperBodyMass = T(0);
    Vec3<T> upperBodyCom_B = Vec3<T>::Zero();

    // Sum torso + arms in the torso-root frame at q = 0.
    for (int bodyId = 1; bodyId < model->nbody; ++bodyId) {
        if (isInLegSubtree<T>(model, bindings, bodyId)) {
            continue;
        }

        const T mass = static_cast<T>(model->body_mass[bodyId]);
        if (mass <= T(0)) {
            continue;
        }

        // Walk up the kinematic tree and compute the transform relative to the base at q = 0.
        Vec3<T> offsetBody = Vec3<T>::Zero();
        Mat3<T> rotBody = Mat3<T>::Identity();
        int curr = bodyId;
        while (curr > 0 && curr != torsoBodyId) {
            Vec3<T> p(static_cast<T>(model->body_pos[3 * curr + 0]),
                      static_cast<T>(model->body_pos[3 * curr + 1]),
                      static_cast<T>(model->body_pos[3 * curr + 2]));
            Quat<T> q(static_cast<T>(model->body_quat[4 * curr + 0]),
                      static_cast<T>(model->body_quat[4 * curr + 1]),
                      static_cast<T>(model->body_quat[4 * curr + 2]),
                      static_cast<T>(model->body_quat[4 * curr + 3]));
            Mat3<T> R = q.toRotationMatrix();

            offsetBody = p + R * offsetBody;
            rotBody = R * rotBody;

            curr = model->body_parentid[curr];
        }

        Vec3<T> ipos(static_cast<T>(model->body_ipos[3 * bodyId + 0]),
                     static_cast<T>(model->body_ipos[3 * bodyId + 1]),
                     static_cast<T>(model->body_ipos[3 * bodyId + 2]));

        const Vec3<T> comBody_B = offsetBody + rotBody * ipos;

        upperBodyMass += mass;
        upperBodyCom_B += mass * comBody_B;
    }

    if (upperBodyMass > T(0)) {
        upperBodyCom_B /= upperBodyMass;
    }

    Mat3<T> upperBodyInertia_B = Mat3<T>::Zero();

    for (int bodyId = 1; bodyId < model->nbody; ++bodyId) {
        if (isInLegSubtree<T>(model, bindings, bodyId)) {
            continue;
        }

        const T mass = static_cast<T>(model->body_mass[bodyId]);
        if (mass <= T(0)) {
            continue;
        }

        Vec3<T> offsetBody = Vec3<T>::Zero();
        Mat3<T> rotBody = Mat3<T>::Identity();
        int curr = bodyId;
        while (curr > 0 && curr != torsoBodyId) {
            Vec3<T> p(static_cast<T>(model->body_pos[3 * curr + 0]),
                      static_cast<T>(model->body_pos[3 * curr + 1]),
                      static_cast<T>(model->body_pos[3 * curr + 2]));
            Quat<T> q(static_cast<T>(model->body_quat[4 * curr + 0]),
                      static_cast<T>(model->body_quat[4 * curr + 1]),
                      static_cast<T>(model->body_quat[4 * curr + 2]),
                      static_cast<T>(model->body_quat[4 * curr + 3]));
            Mat3<T> R = q.toRotationMatrix();

            offsetBody = p + R * offsetBody;
            rotBody = R * rotBody;

            curr = model->body_parentid[curr];
        }

        Vec3<T> ipos(static_cast<T>(model->body_ipos[3 * bodyId + 0]),
                     static_cast<T>(model->body_ipos[3 * bodyId + 1]),
                     static_cast<T>(model->body_ipos[3 * bodyId + 2]));
        Quat<T> iquat(static_cast<T>(model->body_iquat[4 * bodyId + 0]),
                      static_cast<T>(model->body_iquat[4 * bodyId + 1]),
                      static_cast<T>(model->body_iquat[4 * bodyId + 2]),
                      static_cast<T>(model->body_iquat[4 * bodyId + 3]));

        const Vec3<T> comBody_B = offsetBody + rotBody * ipos;
        const Mat3<T> R_i = rotBody * iquat.toRotationMatrix();

        Mat3<T> I_diag = Mat3<T>::Zero();
        I_diag(0, 0) = static_cast<T>(model->body_inertia[3 * bodyId + 0]);
        I_diag(1, 1) = static_cast<T>(model->body_inertia[3 * bodyId + 1]);
        I_diag(2, 2) = static_cast<T>(model->body_inertia[3 * bodyId + 2]);

        const Mat3<T> I_body_B = R_i * I_diag * R_i.transpose();
        const Vec3<T> offset_B = comBody_B - upperBodyCom_B;

        // Parallel axis theorem about the reduced-body COM (B origin) in the torso-root frame.
        upperBodyInertia_B += I_body_B +
            mass * ((offset_B.squaredNorm() * Mat3<T>::Identity()) - (offset_B * offset_B.transpose()));
    }

    params.bodyMass = upperBodyMass;
    params.bodyComLocation = upperBodyCom_B;
    params.bodyInertia = upperBodyInertia_B;
}

template <typename T>
T yawFromRotationMatrix(const Mat3<T>& R_WT) {
    return std::atan2(R_WT(1, 0), R_WT(0, 0));
}

template <typename T>
MujocoRobotSetup<T> buildRobotParamsFromSpec(const mjModel* model,
                                             const RobotMujocoSpec& spec,
                                             FootEndEffectorSource footEndEffectorSource) {
    MujocoRobotSetup<T> setup;
    RobotParams<T>& params = setup.params;
    MujocoRobotBindings& bindings = setup.bindings;
    params.roboType = spec.type;
    params.nq = model->nq;
    params.nv = model->nv;
    params.nu = model->nu;

    fillDefaultQpos(model, params);
    const int torsoBodyId = requireId<T>(model, mjOBJ_BODY, spec.baseBody, "base body");
    bindings.torsoBodyId = torsoBodyId;
    bindings.footSource = footEndEffectorSource;

    params.legs.reserve(spec.legs.size());
    bindings.feet.reserve(spec.legs.size());
    for (const auto& legSpec : spec.legs) {
        params.legs.emplace_back();
        bindings.feet.emplace_back();
        fillLeg<T>(model,
                   legSpec,
                   footEndEffectorSource,
                   params.legs.back(),
                   bindings.feet.back());
    }

    params.arms.reserve(spec.arms.size());
    bindings.hands.reserve(spec.arms.size());
    for (const auto& armSpec : spec.arms) {
        params.arms.emplace_back();
        bindings.hands.emplace_back();
        fillArm<T>(model, armSpec, params.arms.back(), bindings.hands.back());
    }
    fillFixedJoints<T>(model, spec.fixedJoints, params);

    fillReducedBodyMassProperties(model, spec.baseBody, bindings, params);

    return setup;
}
}  // namespace

template <typename T>
MujocoRobotSetup<T> setupRobotParams(const RobotType robotType,
                                     const mjModel_* model,
                                     FootEndEffectorSource footEndEffectorSource) {
    if (model == nullptr) {
        throw std::runtime_error("setupRobotParams received null mjModel");
    }

    const auto& spec = getRobotMujocoSpec(robotType);
    return buildRobotParamsFromSpec<T>(reinterpret_cast<const mjModel*>(model),
                                      spec,
                                      footEndEffectorSource);
}

template MujocoRobotSetup<float> setupRobotParams<float>(RobotType,
                                                         const mjModel_*,
                                                         FootEndEffectorSource);
template MujocoRobotSetup<double> setupRobotParams<double>(RobotType,
                                                          const mjModel_*,
                                                          FootEndEffectorSource);

template <typename T>
void updateReducedBodyMassPropertiesFromData(const mjModel_* modelPtr,
                                             const mjData_* dataPtr,
                                             const MujocoRobotBindings& bindings,
                                             RobotParams<T>& params) {
    const auto* model = reinterpret_cast<const mjModel*>(modelPtr);
    const auto* data = reinterpret_cast<const mjData*>(dataPtr);
    if (model == nullptr || data == nullptr) {
        throw std::runtime_error("updateReducedBodyMassPropertiesFromData received null MuJoCo pointers");
    }
    if (bindings.torsoBodyId < 0) {
        throw std::runtime_error("updateReducedBodyMassPropertiesFromData requires a valid base body binding");
    }

    T upperBodyMass = T(0);
    Vec3<T> upperBodyCom_W = Vec3<T>::Zero();

    for (int bodyId = 1; bodyId < model->nbody; ++bodyId) {
        if (isInLegSubtree<T>(model, bindings, bodyId)) {
            continue;
        }

        const T bodyMass = static_cast<T>(model->body_mass[bodyId]);
        if (bodyMass <= T(0)) {
            continue;
        }

        Vec3<T> bodyComWorld = Vec3<T>::Zero();
        for (int i = 0; i < 3; ++i) {
            bodyComWorld[i] = static_cast<T>(data->xipos[3 * bodyId + i]);
        }

        upperBodyMass += bodyMass;
        upperBodyCom_W += bodyMass * bodyComWorld;
    }

    if (upperBodyMass <= T(0)) {
        throw std::runtime_error("Reduced-body mass must be positive");
    }
    upperBodyCom_W /= upperBodyMass;

    Mat3<T> upperBodyInertia_W = Mat3<T>::Zero();
    for (int bodyId = 1; bodyId < model->nbody; ++bodyId) {
        if (isInLegSubtree<T>(model, bindings, bodyId)) {
            continue;
        }

        const T bodyMass = static_cast<T>(model->body_mass[bodyId]);
        if (bodyMass <= T(0)) {
            continue;
        }

        Mat3<T> inertiaFrameRotation = Mat3<T>::Zero();
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                inertiaFrameRotation(row, col) =
                    static_cast<T>(data->ximat[9 * bodyId + 3 * row + col]);
            }
        }

        Mat3<T> bodyInertiaDiag = Mat3<T>::Zero();
        bodyInertiaDiag(0, 0) = static_cast<T>(model->body_inertia[3 * bodyId + 0]);
        bodyInertiaDiag(1, 1) = static_cast<T>(model->body_inertia[3 * bodyId + 1]);
        bodyInertiaDiag(2, 2) = static_cast<T>(model->body_inertia[3 * bodyId + 2]);

        const Mat3<T> bodyInertiaWorld =
            inertiaFrameRotation * bodyInertiaDiag * inertiaFrameRotation.transpose();

        Vec3<T> bodyComWorld = Vec3<T>::Zero();
        for (int i = 0; i < 3; ++i) {
            bodyComWorld[i] = static_cast<T>(data->xipos[3 * bodyId + i]);
        }
        const Vec3<T> offset_W = bodyComWorld - upperBodyCom_W;

        upperBodyInertia_W += bodyInertiaWorld +
            bodyMass * ((offset_W.squaredNorm() * Mat3<T>::Identity()) - (offset_W * offset_W.transpose()));
    }

    Mat3<T> R_WT = Mat3<T>::Zero();
    Vec3<T> torsoPos_W = Vec3<T>::Zero();
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
        R_WT(row, col) =
                static_cast<T>(data->xmat[9 * bindings.torsoBodyId + 3 * row + col]);
        }
        torsoPos_W[row] = static_cast<T>(data->xpos[3 * bindings.torsoBodyId + row]);
    }
    const T psi = yawFromRotationMatrix(R_WT);
    const Mat3<T> R_WB = Rz(psi);
    const Mat3<T> R_BW = R_WB.transpose();

    params.bodyMass = upperBodyMass;
    params.bodyInertia = R_BW * upperBodyInertia_W * R_WB;
    params.bodyComLocation = R_BW * (upperBodyCom_W - torsoPos_W);
}

template void updateReducedBodyMassPropertiesFromData<float>(
    const mjModel_*, const mjData_*, const MujocoRobotBindings&, RobotParams<float>&);
template void updateReducedBodyMassPropertiesFromData<double>(
    const mjModel_*, const mjData_*, const MujocoRobotBindings&, RobotParams<double>&);
