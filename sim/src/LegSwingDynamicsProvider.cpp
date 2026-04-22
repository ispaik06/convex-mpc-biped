#include <array>
#include <stdexcept>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>

#include "LegSwingDynamicsProvider.h"
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

int findLegIndexBySide(const RobotParams<double>& params, const Side side) {
    for (std::size_t leg = 0; leg < params.legs.size(); ++leg) {
        if (params.legs[leg].side == side) {
            return static_cast<int>(leg);
        }
    }
    return -1;
}

void assignBasePose(mjModel* model,
                    const int torsoBodyId,
                    const Vec3<double>& torsoPos_W,
                    const Quat<double>& torsoQuat_W) {
    for (int i = 0; i < 3; ++i) {
        model->body_pos[3 * torsoBodyId + i] = static_cast<mjtNum>(torsoPos_W[i]);
    }

    model->body_quat[4 * torsoBodyId + 0] = static_cast<mjtNum>(torsoQuat_W.w());
    model->body_quat[4 * torsoBodyId + 1] = static_cast<mjtNum>(torsoQuat_W.x());
    model->body_quat[4 * torsoBodyId + 2] = static_cast<mjtNum>(torsoQuat_W.y());
    model->body_quat[4 * torsoBodyId + 3] = static_cast<mjtNum>(torsoQuat_W.z());
}

Vec3<double> readBodyComPosition(const mjData* data, const int bodyId) {
    Vec3<double> pos = Vec3<double>::Zero();
    for (int i = 0; i < 3; ++i) {
        pos[i] = static_cast<double>(data->xipos[3 * bodyId + i]);
    }
    return pos;
}

Vec3<double> readSitePosition(const mjData* data, const int siteId) {
    if (siteId < 0) {
        throw std::runtime_error("Invalid site id for world position query");
    }

    Vec3<double> pos = Vec3<double>::Zero();
    for (int i = 0; i < 3; ++i) {
        pos[i] = static_cast<double>(data->site_xpos[3 * siteId + i]);
    }
    return pos;
}

Mat3<double> readRotationMatrix(const mjtNum* raw) {
    Mat3<double> rotation = Mat3<double>::Identity();
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            rotation(row, col) = static_cast<double>(raw[3 * row + col]);
        }
    }
    return rotation;
}

Mat3<double> readBodyRotation(const mjData* data, const int bodyId) {
    if (bodyId < 0) {
        throw std::runtime_error("Invalid body id for world rotation query");
    }
    return readRotationMatrix(data->xmat + 9 * bodyId);
}

Mat3<double> readSiteRotation(const mjData* data, const int siteId) {
    if (siteId < 0) {
        throw std::runtime_error("Invalid site id for world rotation query");
    }
    return readRotationMatrix(data->site_xmat + 9 * siteId);
}

void readBodyComJacobians(const mjModel* model,
                          const mjData* data,
                          const int bodyId,
                          std::vector<mjtNum>& jacpScratch,
                          std::vector<mjtNum>& jacrScratch,
                          DMat<double>& Jv_W,
                          DMat<double>& Jw_W) {
    mj_jacBodyCom(model, data, jacpScratch.data(), jacrScratch.data(), bodyId);

    Eigen::Map<const RowMajorMatrix<mjtNum>> jacpMap(jacpScratch.data(), 3, model->nv);
    Eigen::Map<const RowMajorMatrix<mjtNum>> jacrMap(jacrScratch.data(), 3, model->nv);
    Jv_W = jacpMap.template cast<double>();
    Jw_W = jacrMap.template cast<double>();
}

void readBodyComJacobianDot(const mjModel* model,
                            const mjData* data,
                            const int bodyId,
                            std::vector<mjtNum>& jacDotpScratch,
                            DMat<double>& JvDot_W) {
    const mjtNum point[3] = {
        data->xipos[3 * bodyId + 0],
        data->xipos[3 * bodyId + 1],
        data->xipos[3 * bodyId + 2],
    };

    mj_jacDot(model, data, jacDotpScratch.data(), nullptr, point, bodyId);

    Eigen::Map<const RowMajorMatrix<mjtNum>> jacDotMap(jacDotpScratch.data(), 3, model->nv);
    JvDot_W = jacDotMap.template cast<double>();
}

void readSiteJacobians(const mjModel* model,
                       const mjData* data,
                       const int siteId,
                       std::vector<mjtNum>& jacpScratch,
                       std::vector<mjtNum>& jacrScratch,
                       DMat<double>& Jv_W,
                       DMat<double>& Jw_W) {
    mj_jacSite(model, data, jacpScratch.data(), jacrScratch.data(), siteId);

    Eigen::Map<const RowMajorMatrix<mjtNum>> jacpMap(jacpScratch.data(), 3, model->nv);
    Eigen::Map<const RowMajorMatrix<mjtNum>> jacrMap(jacrScratch.data(), 3, model->nv);
    Jv_W = jacpMap.template cast<double>();
    Jw_W = jacrMap.template cast<double>();
}

void readSiteJacobianDot(const mjModel* model,
                         const mjData* data,
                         const int siteId,
                         std::vector<mjtNum>& jacDotpScratch,
                         DMat<double>& JvDot_W) {
    if (siteId < 0) {
        throw std::runtime_error("Invalid site id for Jacobian time-derivative query");
    }

    const mjtNum point[3] = {
        data->site_xpos[3 * siteId + 0],
        data->site_xpos[3 * siteId + 1],
        data->site_xpos[3 * siteId + 2],
    };

    mj_jacDot(model, data, jacDotpScratch.data(), nullptr, point, model->site_bodyid[siteId]);

    Eigen::Map<const RowMajorMatrix<mjtNum>> jacDotMap(jacDotpScratch.data(), 3, model->nv);
    JvDot_W = jacDotMap.template cast<double>();
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

void copySelectedJacobianColumns(const mjModel* model,
                                 const std::vector<mjtNum>& source,
                                 const std::vector<int>& columnIndices,
                                 DMat<double>& target,
                                 const Eigen::Index rowOffset) {
    Eigen::Map<const RowMajorMatrix<mjtNum>> sourceMap(source.data(), 3, model->nv);
    for (Eigen::Index row = 0; row < 3; ++row) {
        for (Eigen::Index col = 0; col < static_cast<Eigen::Index>(columnIndices.size()); ++col) {
            const int sourceCol = columnIndices[static_cast<std::size_t>(col)];
            if (sourceCol < 0 || sourceCol >= model->nv) {
                throw std::runtime_error("Auxiliary standing Jacobian column index is out of range");
            }
            target(rowOffset + row, col) = static_cast<double>(sourceMap(row, sourceCol));
        }
    }
}

void readStandingFootJacobians(const mjModel* model,
                               const mjData* data,
                               const FootEndEffectorSource footSource,
                               const int footBodyId,
                               const int footSiteId,
                               const std::vector<int>& combinedQvelIndex,
                               std::vector<mjtNum>& jacpScratch,
                               std::vector<mjtNum>& jacrScratch,
                               DMat<double>& Jv_W,
                               DMat<double>& Jw_W,
                               const Eigen::Index rowOffset) {
    switch (footSource) {
        case FootEndEffectorSource::Site:
            if (footSiteId < 0) {
                throw std::runtime_error("Standing foot source is site, but no foot site was bound");
            }
            mj_jacSite(model, data, jacpScratch.data(), jacrScratch.data(), footSiteId);
            break;
        case FootEndEffectorSource::BodyCom:
            if (footBodyId < 0) {
                throw std::runtime_error("Standing foot source is body COM, but no foot body was bound");
            }
            mj_jacBodyCom(model, data, jacpScratch.data(), jacrScratch.data(), footBodyId);
            break;
    }

    copySelectedJacobianColumns(model, jacpScratch, combinedQvelIndex, Jv_W, rowOffset);
    copySelectedJacobianColumns(model, jacrScratch, combinedQvelIndex, Jw_W, rowOffset);
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

        auxModel.torsoBodyId = mj_name2id(auxModel.model, mjOBJ_BODY, std::string(robotSpec.baseBody).c_str());
        auxModel.footBodyId =
            mj_name2id(auxModel.model, mjOBJ_BODY, std::string(robotSpec.legs[leg].endBody).c_str());
        auxModel.footSource = bindings.footSource;
        if (!robotSpec.legs[leg].endSite.empty()) {
            auxModel.footSiteId =
                mj_name2id(auxModel.model,
                           mjOBJ_SITE,
                           std::string(robotSpec.legs[leg].endSite).c_str());
        }

        if (auxModel.torsoBodyId < 0 || auxModel.footBodyId < 0 ||
            (!robotSpec.legs[leg].endSite.empty() && auxModel.footSiteId < 0)) {
            destroy(auxModel);
            throw std::runtime_error(
                "Auxiliary leg model is missing required base body, foot body, or foot site");
        }

        auxModel.qposIndex.reserve(robotSpec.legs[leg].joints.size());
        auxModel.qvelIndex.reserve(robotSpec.legs[leg].joints.size());
        for (const auto& jointSpec : robotSpec.legs[leg].joints) {
            const int jointId = mj_name2id(auxModel.model, mjOBJ_JOINT, std::string(jointSpec.joint).c_str());
            if (jointId < 0) {
                destroy(auxModel);
                throw std::runtime_error("Auxiliary leg model is missing required leg joint");
            }

            auxModel.qposIndex.push_back(auxModel.model->jnt_qposadr[jointId]);
            auxModel.qvelIndex.push_back(auxModel.model->jnt_dofadr[jointId]);
        }

        auxModel.jacpScratch.resize(static_cast<std::size_t>(3 * auxModel.model->nv));
        auxModel.jacrScratch.resize(static_cast<std::size_t>(3 * auxModel.model->nv));
        auxModel.jacDotpScratch.resize(static_cast<std::size_t>(3 * auxModel.model->nv));
        auxModel.denseMassScratch.resize(
            static_cast<std::size_t>(auxModel.model->nv * auxModel.model->nv));
    }

    const int leftLegIndex = findLegIndexBySide(params, Side::Left);
    const int rightLegIndex = findLegIndexBySide(params, Side::Right);
    if (leftLegIndex >= 0 && rightLegIndex >= 0) {
        std::array<char, 1024> error{};
        mjSpec* spec = mj_parseXML(xmlPath.c_str(), nullptr, error.data(), static_cast<int>(error.size()));
        if (spec == nullptr) {
            throw std::runtime_error("Failed to parse auxiliary standing XML: " + std::string(error.data()));
        }

        deleteElementsByType(spec, mjOBJ_ACTUATOR);
        deleteElementsByType(spec, mjOBJ_SENSOR);
        deleteElementsByType(spec, mjOBJ_KEY);

        if (mjsElement* floatingJoint = mjs_findElement(spec, mjOBJ_JOINT, floatingJointName.c_str());
            floatingJoint != nullptr) {
            mjs_delete(spec, floatingJoint);
        }

        for (const auto& hand : bindings.hands) {
            deleteBodyIfExists(spec, requireObjectName(fullModel, mjOBJ_BODY, hand.rootBodyId, "arm root body"));
        }

        _standingAuxiliaryModel.model = mj_compile(spec, nullptr);
        if (_standingAuxiliaryModel.model == nullptr) {
            const char* compileError = mjs_getError(spec);
            const std::string message =
                (compileError != nullptr && compileError[0] != '\0')
                    ? std::string(compileError)
                    : std::string("unknown MuJoCo compile failure");
            mj_deleteSpec(spec);
            throw std::runtime_error("Failed to compile auxiliary standing model: " + message);
        }
        mj_deleteSpec(spec);

        _standingAuxiliaryModel.data = mj_makeData(_standingAuxiliaryModel.model);
        if (_standingAuxiliaryModel.data == nullptr) {
            destroy(_standingAuxiliaryModel);
            throw std::runtime_error("Failed to allocate auxiliary standing mjData");
        }

        _standingAuxiliaryModel.torsoBodyId =
            mj_name2id(_standingAuxiliaryModel.model,
                       mjOBJ_BODY,
                       std::string(robotSpec.baseBody).c_str());
        _standingAuxiliaryModel.footSource = bindings.footSource;
        _standingAuxiliaryModel.footBodyIds.resize(params.legs.size(), -1);
        _standingAuxiliaryModel.footSiteIds.resize(params.legs.size(), -1);
        _standingAuxiliaryModel.qposIndexByLeg.resize(params.legs.size());
        _standingAuxiliaryModel.qvelIndexByLeg.resize(params.legs.size());
        _standingAuxiliaryModel.footRowLegIndices = {
            static_cast<std::size_t>(leftLegIndex),
            static_cast<std::size_t>(rightLegIndex),
        };

        if (_standingAuxiliaryModel.torsoBodyId < 0) {
            destroy(_standingAuxiliaryModel);
            throw std::runtime_error("Auxiliary standing model is missing required base body");
        }

        for (std::size_t leg = 0; leg < params.legs.size(); ++leg) {
            _standingAuxiliaryModel.footBodyIds[leg] =
                mj_name2id(_standingAuxiliaryModel.model,
                           mjOBJ_BODY,
                           std::string(robotSpec.legs[leg].endBody).c_str());
            if (!robotSpec.legs[leg].endSite.empty()) {
                _standingAuxiliaryModel.footSiteIds[leg] =
                    mj_name2id(_standingAuxiliaryModel.model,
                               mjOBJ_SITE,
                               std::string(robotSpec.legs[leg].endSite).c_str());
            }

            if (_standingAuxiliaryModel.footBodyIds[leg] < 0 ||
                (!robotSpec.legs[leg].endSite.empty() && _standingAuxiliaryModel.footSiteIds[leg] < 0)) {
                destroy(_standingAuxiliaryModel);
                throw std::runtime_error(
                    "Auxiliary standing model is missing required foot body or foot site");
            }

            auto& qposIndex = _standingAuxiliaryModel.qposIndexByLeg[leg];
            auto& qvelIndex = _standingAuxiliaryModel.qvelIndexByLeg[leg];
            qposIndex.reserve(robotSpec.legs[leg].joints.size());
            qvelIndex.reserve(robotSpec.legs[leg].joints.size());
            for (const auto& jointSpec : robotSpec.legs[leg].joints) {
                const int jointId = mj_name2id(_standingAuxiliaryModel.model,
                                               mjOBJ_JOINT,
                                               std::string(jointSpec.joint).c_str());
                if (jointId < 0) {
                    destroy(_standingAuxiliaryModel);
                    throw std::runtime_error("Auxiliary standing model is missing required leg joint");
                }

                qposIndex.push_back(_standingAuxiliaryModel.model->jnt_qposadr[jointId]);
                qvelIndex.push_back(_standingAuxiliaryModel.model->jnt_dofadr[jointId]);
                _standingAuxiliaryModel.combinedQvelIndex.push_back(
                    _standingAuxiliaryModel.model->jnt_dofadr[jointId]);
            }
        }

        _standingAuxiliaryModel.jacpScratch.resize(
            static_cast<std::size_t>(3 * _standingAuxiliaryModel.model->nv));
        _standingAuxiliaryModel.jacrScratch.resize(
            static_cast<std::size_t>(3 * _standingAuxiliaryModel.model->nv));
    }
}

LegSwingDynamicsProvider::~LegSwingDynamicsProvider() {
    for (auto& auxModel : _auxiliaryLegModels) {
        destroy(auxModel);
    }
    destroy(_standingAuxiliaryModel);
}

void LegSwingDynamicsProvider::update(StateEstimate<double>& stateEstimate) {
    if (stateEstimate.legs.size() != _auxiliaryLegModels.size()) {
        throw std::runtime_error("LegSwingDynamicsProvider state leg count does not match auxiliary models");
    }

    for (std::size_t leg = 0; leg < _auxiliaryLegModels.size(); ++leg) {
        auto& auxModel = _auxiliaryLegModels[leg];
        auto& legState = stateEstimate.legs[leg];

        assignBasePose(auxModel.model,
                       auxModel.torsoBodyId,
                       stateEstimate.torsoPos_W,
                       stateEstimate.torsoQuat_W);

        if (legState.q.size() != static_cast<Eigen::Index>(auxModel.qposIndex.size()) ||
            legState.qd.size() != static_cast<Eigen::Index>(auxModel.qvelIndex.size())) {
            throw std::runtime_error("LegSwingDynamicsProvider leg state dimension mismatch");
        }

        for (Eigen::Index i = 0; i < legState.q.size(); ++i) {
            auxModel.data->qpos[auxModel.qposIndex[static_cast<std::size_t>(i)]] =
                static_cast<mjtNum>(legState.q[i]);
            auxModel.data->qvel[auxModel.qvelIndex[static_cast<std::size_t>(i)]] =
                static_cast<mjtNum>(legState.qd[i]);
        }

        mj_forward(auxModel.model, auxModel.data);

        switch (auxModel.footSource) {
            case FootEndEffectorSource::Site:
                if (auxModel.footSiteId < 0) {
                    throw std::runtime_error("Foot end-effector source is site, but no foot site was bound");
                }
                legState.footPos_W = readSitePosition(auxModel.data, auxModel.footSiteId);
                legState.R_WF = readSiteRotation(auxModel.data, auxModel.footSiteId);
                readSiteJacobians(auxModel.model,
                                  auxModel.data,
                                  auxModel.footSiteId,
                                  auxModel.jacpScratch,
                                  auxModel.jacrScratch,
                                  legState.Jv_W,
                                  legState.Jw_W);
                readSiteJacobianDot(auxModel.model,
                                    auxModel.data,
                                    auxModel.footSiteId,
                                    auxModel.jacDotpScratch,
                                    legState.JvDot_W);
                break;
            case FootEndEffectorSource::BodyCom:
                legState.footPos_W = readBodyComPosition(auxModel.data, auxModel.footBodyId);
                legState.R_WF = readBodyRotation(auxModel.data, auxModel.footBodyId);
                readBodyComJacobians(auxModel.model,
                                     auxModel.data,
                                     auxModel.footBodyId,
                                     auxModel.jacpScratch,
                                     auxModel.jacrScratch,
                                     legState.Jv_W,
                                     legState.Jw_W);
                readBodyComJacobianDot(auxModel.model,
                                       auxModel.data,
                                       auxModel.footBodyId,
                                       auxModel.jacDotpScratch,
                                       legState.JvDot_W);
                break;
        }
        legState.footVel_W = legState.Jv_W * legState.qd;
        legState.footEndPos_W = legState.footPos_W;
        legState.footEndVel_W = legState.footVel_W;
        readDenseMassMatrix(auxModel.model,
                            auxModel.data,
                            auxModel.denseMassScratch,
                            legState.massMatrix);
        copyBias(auxModel.data, legState.bias);
        legState.hasFootKinematics = true;
        legState.hasLegDynamics = true;
    }

    stateEstimate.standingFeet.zero();
    if (_standingAuxiliaryModel.model == nullptr || _standingAuxiliaryModel.data == nullptr) {
        return;
    }

    auto& standingModel = _standingAuxiliaryModel;
    assignBasePose(standingModel.model,
                   standingModel.torsoBodyId,
                   stateEstimate.torsoPos_W,
                   stateEstimate.torsoQuat_W);

    for (std::size_t leg = 0; leg < stateEstimate.legs.size(); ++leg) {
        auto& legState = stateEstimate.legs[leg];
        const auto& qposIndex = standingModel.qposIndexByLeg[leg];
        const auto& qvelIndex = standingModel.qvelIndexByLeg[leg];
        if (legState.q.size() != static_cast<Eigen::Index>(qposIndex.size()) ||
            legState.qd.size() != static_cast<Eigen::Index>(qvelIndex.size())) {
            throw std::runtime_error("Auxiliary standing model leg state dimension mismatch");
        }

        for (Eigen::Index i = 0; i < legState.q.size(); ++i) {
            standingModel.data->qpos[qposIndex[static_cast<std::size_t>(i)]] =
                static_cast<mjtNum>(legState.q[i]);
            standingModel.data->qvel[qvelIndex[static_cast<std::size_t>(i)]] =
                static_cast<mjtNum>(legState.qd[i]);
        }
    }

    mj_forward(standingModel.model, standingModel.data);

    const Eigen::Index combinedDof =
        static_cast<Eigen::Index>(standingModel.combinedQvelIndex.size());
    stateEstimate.standingFeet.resize(combinedDof);
    stateEstimate.standingFeet.Jv_W.setZero(6, combinedDof);
    stateEstimate.standingFeet.Jw_W.setZero(6, combinedDof);

    for (std::size_t rowBlock = 0; rowBlock < standingModel.footRowLegIndices.size(); ++rowBlock) {
        const std::size_t leg = standingModel.footRowLegIndices[rowBlock];
        readStandingFootJacobians(standingModel.model,
                                  standingModel.data,
                                  standingModel.footSource,
                                  standingModel.footBodyIds[leg],
                                  standingModel.footSiteIds[leg],
                                  standingModel.combinedQvelIndex,
                                  standingModel.jacpScratch,
                                  standingModel.jacrScratch,
                                  stateEstimate.standingFeet.Jv_W,
                                  stateEstimate.standingFeet.Jw_W,
                                  static_cast<Eigen::Index>(3 * rowBlock));
    }
    stateEstimate.standingFeet.hasFootJacobians = true;
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
    auxModel.torsoBodyId = -1;
    auxModel.footBodyId = -1;
    auxModel.footSiteId = -1;
    auxModel.qposIndex.clear();
    auxModel.qvelIndex.clear();
    auxModel.jacpScratch.clear();
    auxModel.jacrScratch.clear();
    auxModel.jacDotpScratch.clear();
    auxModel.denseMassScratch.clear();
}

void LegSwingDynamicsProvider::destroy(StandingAuxiliaryModel& auxModel) {
    if (auxModel.data != nullptr) {
        mj_deleteData(auxModel.data);
        auxModel.data = nullptr;
    }
    if (auxModel.model != nullptr) {
        mj_deleteModel(auxModel.model);
        auxModel.model = nullptr;
    }
    auxModel.torsoBodyId = -1;
    auxModel.footSource = FootEndEffectorSource::Site;
    auxModel.footBodyIds.clear();
    auxModel.footSiteIds.clear();
    auxModel.footRowLegIndices.clear();
    auxModel.qposIndexByLeg.clear();
    auxModel.qvelIndexByLeg.clear();
    auxModel.combinedQvelIndex.clear();
    auxModel.jacpScratch.clear();
    auxModel.jacrScratch.clear();
}
