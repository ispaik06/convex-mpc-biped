#include "MujocoCheaterStateReader.h"

#include <stdexcept>
#include <vector>

#include <mujoco/mujoco.h>

namespace {
template <typename Scalar>
using RowMajorMatrix =
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

Vec3<double> readBodyPosition(const mjData* data, int body_id) {
    if (body_id < 0) {
        throw std::runtime_error("Invalid body id for world position query");
    }

    Vec3<double> pos = Vec3<double>::Zero();
    for (int i = 0; i < 3; ++i) {
        pos[i] = static_cast<double>(data->xpos[3 * body_id + i]);
    }
    return pos;
}

Vec3<double> readBodyComPosition(const mjData* data, int body_id) {
    if (body_id < 0) {
        throw std::runtime_error("Invalid body id for body COM query");
    }

    Vec3<double> pos = Vec3<double>::Zero();
    for (int i = 0; i < 3; ++i) {
        pos[i] = static_cast<double>(data->xipos[3 * body_id + i]);
    }
    return pos;
}

Quat<double> readBodyQuaternion(const mjData* data, int body_id) {
    if (body_id < 0) {
        throw std::runtime_error("Invalid body id for quaternion query");
    }

    const mjtNum* q = &data->xquat[4 * body_id];
    return Quat<double>(static_cast<double>(q[0]),
                        static_cast<double>(q[1]),
                        static_cast<double>(q[2]),
                        static_cast<double>(q[3]));
}

Vec3<double> readSitePosition(const mjData* data, int site_id) {
    if (site_id < 0) {
        throw std::runtime_error("Invalid site id for world position query");
    }

    Vec3<double> pos = Vec3<double>::Zero();
    for (int i = 0; i < 3; ++i) {
        pos[i] = static_cast<double>(data->site_xpos[3 * site_id + i]);
    }
    return pos;
}

Vec3<double> readObjectLinearVelocity(const mjModel* model,
                                      const mjData* data,
                                      int object_type,
                                      int object_id) {
    if (object_id < 0) {
        throw std::runtime_error("Invalid object id for velocity query");
    }

    mjtNum velocity[6] = {};
    mj_objectVelocity(model, data, object_type, object_id, velocity, 0);

    Vec3<double> linear = Vec3<double>::Zero();
    for (int i = 0; i < 3; ++i) {
        linear[i] = static_cast<double>(velocity[3 + i]);
    }
    return linear;
}

Vec3<double> readBodyAngularVelocity(const mjModel* model, const mjData* data, int body_id) {
    if (body_id < 0) {
        throw std::runtime_error("Invalid body id for angular velocity query");
    }

    mjtNum velocity[6] = {};
    mj_objectVelocity(model, data, mjOBJ_BODY, body_id, velocity, 0);

    Vec3<double> angular = Vec3<double>::Zero();
    for (int i = 0; i < 3; ++i) {
        angular[i] = static_cast<double>(velocity[i]);
    }
    return angular;
}

Vec3<double> readBodyLinearVelocity(const mjModel* model, const mjData* data, int body_id) {
    return readObjectLinearVelocity(model, data, mjOBJ_BODY, body_id);
}

Vec3<double> readBodyAngularAcceleration(const mjModel* model, const mjData* data, int body_id) {
    if (body_id < 0) {
        throw std::runtime_error("Invalid body id for angular acceleration query");
    }

    mjtNum acceleration[6] = {};
    mj_objectAcceleration(model, data, mjOBJ_BODY, body_id, acceleration, 0);

    Vec3<double> angular = Vec3<double>::Zero();
    for (int i = 0; i < 3; ++i) {
        angular[i] = static_cast<double>(acceleration[i]);
    }
    return angular;
}

Vec3<double> readBodyLinearAcceleration(const mjModel* model, const mjData* data, int body_id) {
    if (body_id < 0) {
        throw std::runtime_error("Invalid body id for linear acceleration query");
    }

    mjtNum acceleration[6] = {};
    mj_objectAcceleration(model, data, mjOBJ_BODY, body_id, acceleration, 0);

    Vec3<double> linear = Vec3<double>::Zero();
    for (int i = 0; i < 3; ++i) {
        linear[i] = static_cast<double>(acceleration[3 + i]);
    }
    return linear;
}

Vec3<double> readFootEndEffectorPosition(const mjData* data,
                                         const MujocoEndEffectorBinding& foot,
                                         const FootEndEffectorSource source) {
    switch (source) {
        case FootEndEffectorSource::Site:
            if (foot.siteId < 0) {
                throw std::runtime_error("Foot end-effector source is site, but no siteId is bound");
            }
            return readSitePosition(data, foot.siteId);
        case FootEndEffectorSource::BodyCom:
            return readBodyComPosition(data, foot.bodyId);
    }

    throw std::runtime_error("Unsupported foot end-effector source");
}

Vec3<double> readFootEndEffectorVelocity(const mjModel* model,
                                         const mjData* data,
                                         const MujocoEndEffectorBinding& foot,
                                         const FootEndEffectorSource source) {
    switch (source) {
        case FootEndEffectorSource::Site:
            if (foot.siteId < 0) {
                throw std::runtime_error("Foot end-effector source is site, but no siteId is bound");
            }
            return readObjectLinearVelocity(model, data, mjOBJ_SITE, foot.siteId);
        case FootEndEffectorSource::BodyCom:
            return readBodyLinearVelocity(model, data, foot.bodyId);
    }

    throw std::runtime_error("Unsupported foot end-effector source");
}

Vec3<double> readEndEffectorPosition(const mjData* data, int site_id, int body_id) {
    if (site_id >= 0) {
        return readSitePosition(data, site_id);
    }
    return readBodyPosition(data, body_id);
}

Vec3<double> readEndEffectorVelocity(const mjModel* model,
                                     const mjData* data,
                                     int site_id,
                                     int body_id) {
    if (site_id >= 0) {
        return readObjectLinearVelocity(model, data, mjOBJ_SITE, site_id);
    }
    return readBodyLinearVelocity(model, data, body_id);
}

void readDenseMassMatrix(const mjModel* model, const mjData* data, DMat<double>& massMatrix) {
    thread_local std::vector<mjtNum> rawMassMatrix;
    rawMassMatrix.resize(static_cast<std::size_t>(model->nv * model->nv));
    mj_fullM(model, rawMassMatrix.data(), data->qM);

    Eigen::Map<const RowMajorMatrix<mjtNum>> massMap(rawMassMatrix.data(), model->nv, model->nv);
    massMatrix = massMap.template cast<double>();
}

void copyContiguous(const mjtNum* src, Eigen::Index size, DVec<double>& dst, const char* name) {
    if (dst.size() != size) {
        throw std::runtime_error(std::string(name) + " size does not match destination");
    }

    for (Eigen::Index i = 0; i < size; ++i) {
        dst[i] = static_cast<double>(src[i]);
    }
}

void copyIndexed(const mjtNum* src, const std::vector<int>& indices, DVec<double>& dst) {
    if (dst.size() != static_cast<Eigen::Index>(indices.size())) {
        throw std::runtime_error("CheaterState segment size does not match index count");
    }
    for (Eigen::Index i = 0; i < dst.size(); ++i) {
        dst[i] = static_cast<double>(src[indices[static_cast<std::size_t>(i)]]);
    }
}
}  // namespace

void fillCheaterState(const mjModel* model,
                      const mjData* data,
                      const RobotParams<double>& params,
                      const MujocoRobotBindings& bindings,
                      CheaterState<double>& cheater_state) {
    if (model == nullptr || data == nullptr) {
        throw std::runtime_error("fillCheaterState received null MuJoCo pointers");
    }
    if (bindings.feet.size() != params.legs.size() || bindings.hands.size() != params.arms.size()) {
        throw std::runtime_error("Mujoco bindings do not match RobotParams limb counts");
    }

    cheater_state.time = data->time;
    cheater_state.torsoPos_W = readBodyPosition(data, bindings.torsoBodyId);
    cheater_state.torsoQuat_W = readBodyQuaternion(data, bindings.torsoBodyId);
    cheater_state.torsoLinVel_W = readBodyLinearVelocity(model, data, bindings.torsoBodyId);
    cheater_state.torsoAngVel_W = readBodyAngularVelocity(model, data, bindings.torsoBodyId);
    cheater_state.torsoLinAcc_W = readBodyLinearAcceleration(model, data, bindings.torsoBodyId);
    cheater_state.torsoAngAcc_W = readBodyAngularAcceleration(model, data, bindings.torsoBodyId);
    copyContiguous(data->qvel, model->nv, cheater_state.dynamics.qd, "qvel");
    copyContiguous(data->qfrc_bias, model->nv, cheater_state.dynamics.bias, "qfrc_bias");
    readDenseMassMatrix(model, data, cheater_state.dynamics.massMatrix);

    for (std::size_t leg = 0; leg < params.legs.size(); ++leg) {
        const auto& joints = params.legs[leg].joints;
        const auto& foot = bindings.feet[leg];
        auto& leg_state = cheater_state.legs[leg];
        copyIndexed(data->qpos, joints.q_idx, leg_state.q);
        copyIndexed(data->qvel, joints.qd_idx, leg_state.qd);

        const auto& tau_idx =
            joints.actuator_idx.empty() ? joints.qd_idx : joints.actuator_idx;
        copyIndexed(data->actuator_force, tau_idx, leg_state.tauEstimate);
        leg_state.footPos_W = readFootEndEffectorPosition(data, foot, bindings.footSource);
        leg_state.footEndPos_W = leg_state.footPos_W;
        leg_state.footVel_W = readFootEndEffectorVelocity(model, data, foot, bindings.footSource);
        leg_state.footEndVel_W = leg_state.footVel_W;
        leg_state.hasFootKinematics = true;
        leg_state.hasLegDynamics = false;
    }

    for (std::size_t arm = 0; arm < params.arms.size(); ++arm) {
        const auto& joints = params.arms[arm].joints;
        const auto& hand = bindings.hands[arm];
        auto& arm_state = cheater_state.arms[arm];
        copyIndexed(data->qpos, joints.q_idx, arm_state.q);
        copyIndexed(data->qvel, joints.qd_idx, arm_state.qd);

        const auto& tau_idx =
            joints.actuator_idx.empty() ? joints.qd_idx : joints.actuator_idx;
        copyIndexed(data->actuator_force, tau_idx, arm_state.tauEstimate);
        arm_state.handPos_W = readEndEffectorPosition(data, hand.siteId, hand.bodyId);
        arm_state.handVel_W =
            readEndEffectorVelocity(model, data, hand.siteId, hand.bodyId);
    }
}
