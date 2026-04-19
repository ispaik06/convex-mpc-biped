#include "LegSwingDynamicsProvider.h"

#include <array>
#include <stdexcept>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>

#include "models/RobotMujocoSpec.h"

namespace {
template <typename Scalar>
using RowMajorMatrix =
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

std::string requireObjectName(const mjModel* model, const int objType, const int objId, const char* what) {
    if (objId < 0) {
        throw std::runtime_error(std::string("Invalid id for ") + what);
    }

    const char* name = mj_id2name(model, objType, objId);
    if (name == nullptr) {
        throw std::runtime_error(std::string("Failed to resolve name for ") + what);
    }
    return std::string(name);
}

std::vector<std::string> collectElementNames(mjSpec* spec, const mjtObj type) {
    std::vector<std::string> names;
    for (mjsElement* element = mjs_firstElement(spec, type);
         element != nullptr;
         element = mjs_nextElement(spec, element)) {
        if (auto* name = mjs_getName(element); name != nullptr && !name->empty()) {
            names.push_back(*name);
        }
    }
    return names;
}

void deleteElementsByType(mjSpec* spec, const mjtObj type) {
    for (const auto& name : collectElementNames(spec, type)) {
        if (mjsElement* element = mjs_findElement(spec, type, name.c_str()); element != nullptr) {
            mjs_delete(spec, element);
        }
    }
}

void deleteBodyIfExists(mjSpec* spec, const std::string& bodyName) {
    if (mjsBody* body = mjs_findBody(spec, bodyName.c_str()); body != nullptr) {
        mjs_delete(spec, body->element);
    }
}

std::string freeJointName(const mjModel* fullModel) {
    for (int jointId = 0; jointId < fullModel->njnt; ++jointId) {
        if (fullModel->jnt_type[jointId] == mjJNT_FREE) {
            return requireObjectName(fullModel, mjOBJ_JOINT, jointId, "free joint");
        }
    }
    throw std::runtime_error("Expected a floating-base joint in the full MuJoCo model");
}

void assignBasePose(mjModel* model,
                    const int baseBodyId,
                    const Vec3<double>& basePos,
                    const Quat<double>& baseQuat) {
    for (int i = 0; i < 3; ++i) {
        model->body_pos[3 * baseBodyId + i] = static_cast<mjtNum>(basePos[i]);
    }

    model->body_quat[4 * baseBodyId + 0] = static_cast<mjtNum>(baseQuat.w());
    model->body_quat[4 * baseBodyId + 1] = static_cast<mjtNum>(baseQuat.x());
    model->body_quat[4 * baseBodyId + 2] = static_cast<mjtNum>(baseQuat.y());
    model->body_quat[4 * baseBodyId + 3] = static_cast<mjtNum>(baseQuat.z());
}

Vec3<double> readBodyComPosition(const mjData* data, const int bodyId) {
    Vec3<double> pos = Vec3<double>::Zero();
    for (int i = 0; i < 3; ++i) {
        pos[i] = static_cast<double>(data->xipos[3 * bodyId + i]);
    }
    return pos;
}

void readBodyComJacobians(const mjModel* model,
                          const mjData* data,
                          const int bodyId,
                          std::vector<mjtNum>& jacpScratch,
                          std::vector<mjtNum>& jacrScratch,
                          DMat<double>& JvWorld,
                          DMat<double>& JwWorld) {
    mj_jacBodyCom(model, data, jacpScratch.data(), jacrScratch.data(), bodyId);

    Eigen::Map<const RowMajorMatrix<mjtNum>> jacpMap(jacpScratch.data(), 3, model->nv);
    Eigen::Map<const RowMajorMatrix<mjtNum>> jacrMap(jacrScratch.data(), 3, model->nv);
    JvWorld = jacpMap.template cast<double>();
    JwWorld = jacrMap.template cast<double>();
}

void readBodyComJacobianDot(const mjModel* model,
                            const mjData* data,
                            const int bodyId,
                            std::vector<mjtNum>& jacDotpScratch,
                            DMat<double>& JvDotWorld) {
    const mjtNum point[3] = {
        data->xipos[3 * bodyId + 0],
        data->xipos[3 * bodyId + 1],
        data->xipos[3 * bodyId + 2],
    };

    mj_jacDot(model, data, jacDotpScratch.data(), nullptr, point, bodyId);

    Eigen::Map<const RowMajorMatrix<mjtNum>> jacDotMap(jacDotpScratch.data(), 3, model->nv);
    JvDotWorld = jacDotMap.template cast<double>();
}

void readDenseMassMatrix(const mjModel* model,
                         const mjData* data,
                         std::vector<mjtNum>& denseMassScratch,
                         DMat<double>& massMatrix) {
    mj_fullM(model, denseMassScratch.data(), data->qM);

    Eigen::Map<const RowMajorMatrix<mjtNum>> massMap(denseMassScratch.data(), model->nv, model->nv);
    massMatrix = massMap.template cast<double>();
}

void copyBias(const mjData* data, DVec<double>& bias) {
    for (Eigen::Index i = 0; i < bias.size(); ++i) {
        bias[i] = static_cast<double>(data->qfrc_bias[i]);
    }
}
}  // namespace

LegSwingDynamicsProvider::LegSwingDynamicsProvider(const RobotType robotType,
                                                   const mjModel* fullModel,
                                                   const RobotParams<double>& params,
                                                   const MujocoRobotBindings& bindings) {
    if (fullModel == nullptr) {
        throw std::runtime_error("LegSwingDynamicsProvider requires a valid full MuJoCo model");
    }
    if (bindings.feet.size() != params.legs.size() || bindings.hands.size() != params.arms.size()) {
        throw std::runtime_error("LegSwingDynamicsProvider bindings do not match robot params");
    }

    const std::string xmlPath = robotXmlPath(robotType);
    const RobotMujocoSpec& robotSpec = getRobotMujocoSpec(robotType);
    const std::string floatingJointName = freeJointName(fullModel);

    _auxiliaryLegModels.resize(params.legs.size());
    for (std::size_t leg = 0; leg < params.legs.size(); ++leg) {
        std::array<char, 1024> error{};
        mjSpec* spec = mj_parseXML(xmlPath.c_str(), nullptr, error.data(), static_cast<int>(error.size()));
        if (spec == nullptr) {
            throw std::runtime_error("Failed to parse auxiliary leg XML: " + std::string(error.data()));
        }

        deleteElementsByType(spec, mjOBJ_ACTUATOR);
        deleteElementsByType(spec, mjOBJ_SENSOR);
        deleteElementsByType(spec, mjOBJ_KEY);

        if (mjsElement* floatingJoint = mjs_findElement(spec, mjOBJ_JOINT, floatingJointName.c_str());
            floatingJoint != nullptr) {
            mjs_delete(spec, floatingJoint);
        }

        for (std::size_t otherLeg = 0; otherLeg < bindings.feet.size(); ++otherLeg) {
            if (otherLeg == leg) {
                continue;
            }
            deleteBodyIfExists(
                spec, requireObjectName(fullModel, mjOBJ_BODY, bindings.feet[otherLeg].rootBodyId, "leg root body"));
        }

        for (const auto& hand : bindings.hands) {
            deleteBodyIfExists(spec, requireObjectName(fullModel, mjOBJ_BODY, hand.rootBodyId, "arm root body"));
        }

        AuxiliaryLegModel& auxModel = _auxiliaryLegModels[leg];
        auxModel.model = mj_compile(spec, nullptr);
        if (auxModel.model == nullptr) {
            const char* compileError = mjs_getError(spec);
            const std::string message =
                (compileError != nullptr && compileError[0] != '\0')
                    ? std::string(compileError)
                    : std::string("unknown MuJoCo compile failure");
            mj_deleteSpec(spec);
            throw std::runtime_error("Failed to compile auxiliary leg model: " + message);
        }
        mj_deleteSpec(spec);

        auxModel.data = mj_makeData(auxModel.model);
        if (auxModel.data == nullptr) {
            destroy(auxModel);
            throw std::runtime_error("Failed to allocate auxiliary leg mjData");
        }

        auxModel.baseBodyId = mj_name2id(auxModel.model, mjOBJ_BODY, std::string(robotSpec.baseBody).c_str());
        auxModel.footBodyId =
            mj_name2id(auxModel.model, mjOBJ_BODY, std::string(robotSpec.legs[leg].endBody).c_str());
        if (auxModel.baseBodyId < 0 || auxModel.footBodyId < 0) {
            destroy(auxModel);
            throw std::runtime_error("Auxiliary leg model is missing required base or foot body");
        }

        auxModel.qposAdr.reserve(robotSpec.legs[leg].joints.size());
        auxModel.qvelAdr.reserve(robotSpec.legs[leg].joints.size());
        for (const auto& jointSpec : robotSpec.legs[leg].joints) {
            const int jointId = mj_name2id(auxModel.model, mjOBJ_JOINT, std::string(jointSpec.joint).c_str());
            if (jointId < 0) {
                destroy(auxModel);
                throw std::runtime_error("Auxiliary leg model is missing required leg joint");
            }

            auxModel.qposAdr.push_back(auxModel.model->jnt_qposadr[jointId]);
            auxModel.qvelAdr.push_back(auxModel.model->jnt_dofadr[jointId]);
        }

        auxModel.jacpScratch.resize(static_cast<std::size_t>(3 * auxModel.model->nv));
        auxModel.jacrScratch.resize(static_cast<std::size_t>(3 * auxModel.model->nv));
        auxModel.jacDotpScratch.resize(static_cast<std::size_t>(3 * auxModel.model->nv));
        auxModel.denseMassScratch.resize(
            static_cast<std::size_t>(auxModel.model->nv * auxModel.model->nv));
    }
}

LegSwingDynamicsProvider::~LegSwingDynamicsProvider() {
    for (auto& auxModel : _auxiliaryLegModels) {
        destroy(auxModel);
    }
}

void LegSwingDynamicsProvider::update(StateEstimate<double>& stateEstimate) {
    if (stateEstimate.legs.size() != _auxiliaryLegModels.size()) {
        throw std::runtime_error("LegSwingDynamicsProvider state leg count does not match auxiliary models");
    }

    for (std::size_t leg = 0; leg < _auxiliaryLegModels.size(); ++leg) {
        auto& auxModel = _auxiliaryLegModels[leg];
        auto& legState = stateEstimate.legs[leg];

        assignBasePose(auxModel.model, auxModel.baseBodyId, stateEstimate.basePos, stateEstimate.baseQuat);

        if (legState.q.size() != static_cast<Eigen::Index>(auxModel.qposAdr.size()) ||
            legState.qd.size() != static_cast<Eigen::Index>(auxModel.qvelAdr.size())) {
            throw std::runtime_error("LegSwingDynamicsProvider leg state dimension mismatch");
        }

        for (Eigen::Index i = 0; i < legState.q.size(); ++i) {
            auxModel.data->qpos[auxModel.qposAdr[static_cast<std::size_t>(i)]] =
                static_cast<mjtNum>(legState.q[i]);
            auxModel.data->qvel[auxModel.qvelAdr[static_cast<std::size_t>(i)]] =
                static_cast<mjtNum>(legState.qd[i]);
        }

        mj_forward(auxModel.model, auxModel.data);

        legState.footPosWorld = readBodyComPosition(auxModel.data, auxModel.footBodyId);
        readBodyComJacobians(auxModel.model,
                             auxModel.data,
                             auxModel.footBodyId,
                             auxModel.jacpScratch,
                             auxModel.jacrScratch,
                             legState.JvWorld,
                             legState.JwWorld);
        readBodyComJacobianDot(auxModel.model,
                               auxModel.data,
                               auxModel.footBodyId,
                               auxModel.jacDotpScratch,
                               legState.JvDotWorld);
        legState.footVelWorld = legState.JvWorld * legState.qd;
        readDenseMassMatrix(auxModel.model,
                            auxModel.data,
                            auxModel.denseMassScratch,
                            legState.massMatrix);
        copyBias(auxModel.data, legState.bias);
        legState.hasFootKinematics = true;
        legState.hasLegDynamics = true;
    }
}

std::string LegSwingDynamicsProvider::robotXmlPath(const RobotType robotType) {
    (void)robotType;
    const std::string root(PROJECT_ROOT_DIR);
    return root + "/models/mit_humanoid/mit_humanoid.xml";
}

void LegSwingDynamicsProvider::destroy(AuxiliaryLegModel& auxModel) {
    if (auxModel.data != nullptr) {
        mj_deleteData(auxModel.data);
        auxModel.data = nullptr;
    }
    if (auxModel.model != nullptr) {
        mj_deleteModel(auxModel.model);
        auxModel.model = nullptr;
    }
    auxModel.baseBodyId = -1;
    auxModel.footBodyId = -1;
    auxModel.qposAdr.clear();
    auxModel.qvelAdr.clear();
    auxModel.jacpScratch.clear();
    auxModel.jacrScratch.clear();
    auxModel.jacDotpScratch.clear();
    auxModel.denseMassScratch.clear();
}
