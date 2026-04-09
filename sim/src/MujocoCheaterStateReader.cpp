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

void readBodyComJacobians(const mjModel* model,
                          const mjData* data,
                          int body_id,
                          DMat<double>& JvWorld,
                          DMat<double>& JwWorld) {
    if (body_id < 0) {
        throw std::runtime_error("Invalid body id for Jacobian query");
    }

    std::vector<mjtNum> jacp(3 * model->nv, mjtNum(0));
    std::vector<mjtNum> jacr(3 * model->nv, mjtNum(0));
    mj_jacBodyCom(model, data, jacp.data(), jacr.data(), body_id);

    Eigen::Map<const RowMajorMatrix<mjtNum>> jacpMap(jacp.data(), 3, model->nv);
    Eigen::Map<const RowMajorMatrix<mjtNum>> jacrMap(jacr.data(), 3, model->nv);
    JvWorld = jacpMap.template cast<double>();
    JwWorld = jacrMap.template cast<double>();
}

void readBodyComJacobianDot(const mjModel* model,
                            const mjData* data,
                            int body_id,
                            DMat<double>& JvDotWorld) {
    if (body_id < 0) {
        throw std::runtime_error("Invalid body id for Jacobian-dot query");
    }

    const mjtNum point[3] = {
        data->xipos[3 * body_id + 0],
        data->xipos[3 * body_id + 1],
        data->xipos[3 * body_id + 2],
    };

    std::vector<mjtNum> jacDotp(3 * model->nv, mjtNum(0));
    mj_jacDot(model, data, jacDotp.data(), nullptr, point, body_id);

    Eigen::Map<const RowMajorMatrix<mjtNum>> jacDotMap(jacDotp.data(), 3, model->nv);
    JvDotWorld = jacDotMap.template cast<double>();
}

void readDenseMassMatrix(const mjModel* model, const mjData* data, DMat<double>& massMatrix) {
    std::vector<mjtNum> rawMassMatrix(model->nv * model->nv, mjtNum(0));
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
    cheater_state.resize(params);
    cheater_state.basePos = readBodyPosition(data, bindings.baseBodyId);
    cheater_state.baseQuat = readBodyQuaternion(data, bindings.baseBodyId);
    cheater_state.baseLinVel = readBodyLinearVelocity(model, data, bindings.baseBodyId);
    cheater_state.baseAngVel = readBodyAngularVelocity(model, data, bindings.baseBodyId);
    cheater_state.baseLinAcc = readBodyLinearAcceleration(model, data, bindings.baseBodyId);
    cheater_state.baseAngAcc = readBodyAngularAcceleration(model, data, bindings.baseBodyId);
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
        leg_state.footPosWorld = readBodyComPosition(data, foot.bodyId);
        readBodyComJacobians(model, data, foot.bodyId, leg_state.JvWorld, leg_state.JwWorld);
        readBodyComJacobianDot(model, data, foot.bodyId, leg_state.JvDotWorld);
        leg_state.footVelWorld = leg_state.JvWorld * cheater_state.dynamics.qd;
        leg_state.hasFootKinematics = true;
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
        arm_state.handPosWorld = readEndEffectorPosition(data, hand.siteId, hand.bodyId);
        arm_state.handVelWorld =
            readEndEffectorVelocity(model, data, hand.siteId, hand.bodyId);
    }
}
