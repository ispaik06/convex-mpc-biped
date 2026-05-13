#include "KeyboardGaitSwingRunner.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <thread>

#include <mujoco/mujoco.h>

#include "ControllerConfig.h"
#include "MujocoCheaterStateReader.h"
#include "RobotConfig.h"
#include "SimulationConfig.h"
#include "Utilities/MatrixUtils.h"
#include "ViewerSyncThrottle.h"
#include "setupRobotParams.h"

namespace {
double yawFromRotation(const Mat3<double>& rotation) {
    return std::atan2(rotation(1, 0), rotation(0, 0));
}

void applyMarkerColor(mjModel* model, const int bodyId, const DebugVizMarker<double>& marker) {
    if (model == nullptr || bodyId < 0 || !marker.hasRgba) {
        return;
    }
    if (bodyId >= model->nbody) {
        return;
    }

    const int geomStart = model->body_geomadr[bodyId];
    const int geomCount = model->body_geomnum[bodyId];
    if (geomStart < 0 || geomCount <= 0) {
        return;
    }

    for (int geomOffset = 0; geomOffset < geomCount; ++geomOffset) {
        const int geomId = geomStart + geomOffset;
        if (geomId < 0 || geomId >= model->ngeom) {
            continue;
        }

        float* geomRgba = model->geom_rgba + 4 * geomId;
        for (int channel = 0; channel < 4; ++channel) {
            geomRgba[channel] = static_cast<float>(marker.rgba[channel]);
        }
    }
}
}  // namespace

KeyboardGaitSwingRunner::KeyboardGaitSwingRunner(const RobotType robotType,
                                                   GaitSwingHoldController* controller,
                                                   const bool headless,
                                                   const double torsoZOffset)
    : _robotType(robotType),
      _controller(controller),
      _headless(headless),
    _torsoZOffset(torsoZOffset) {
    if (_controller == nullptr) {
        throw std::invalid_argument("KeyboardGaitSwingRunner requires a controller");
    }
    _controller->setTouchdownTargetZOffset(_torsoZOffset);
}

KeyboardGaitSwingRunner::~KeyboardGaitSwingRunner() {
    if (_data != nullptr) {
        mj_deleteData(_data);
        _data = nullptr;
    }
    if (_model != nullptr) {
        mj_deleteModel(_model);
        _model = nullptr;
    }
}

void KeyboardGaitSwingRunner::init() {
    setActiveRobotType(_robotType);
    const auto& controllerConfig = getControllerConfig(_robotType);
    _keyboardCommand.setWalkingLimits(controllerConfig.userCommandFilter.xDotMax,
                                      controllerConfig.userCommandFilter.yDotMax,
                                      controllerConfig.userCommandFilter.psiDotMax);
    const auto& runtimeConfig = getRobotRuntimeConfig(_robotType);
    const std::string xmlPath = controllerConfig.gaitSwingHoldTest.xmlPath.empty()
                                    ? runtimeConfig.modelXmlPath
                                    : controllerConfig.gaitSwingHoldTest.xmlPath;
    const std::string keyframeName = controllerConfig.gaitSwingHoldTest.keyframeName;
    _modelPath = resolveProjectPath(xmlPath);

    if (mjVERSION_HEADER != mj_version()) {
        throw std::runtime_error("MuJoCo header/library version mismatch");
    }

    std::array<char, 1024> error{};
    _model = mj_loadXML(_modelPath.c_str(), nullptr, error.data(), error.size());
    if (_model == nullptr) {
        throw std::runtime_error("mj_loadXML failed for " + _modelPath + ": " + error.data());
    }

    _data = mj_makeData(_model);
    if (_data == nullptr) {
        mj_deleteModel(_model);
        _model = nullptr;
        throw std::runtime_error("mj_makeData failed");
    }

    configureSimulationModel(_model);

    locateFloatingBase();
    applyCopiedStateKeyframe(keyframeName);
    cachePlanarBasePose();
    cacheFrozenQpos();
    _planarMotionEnabled = true;

    std::cout << "Loaded MuJoCo model: " << _modelPath << '\n';
    std::cout << "nq=" << _model->nq
              << ", nv=" << _model->nv
              << ", nu=" << _model->nu << '\n';
    std::cout << "[KeyboardGaitSwingHoldTest] initial keyframe: " << keyframeName
              << ", torso z offset=" << _torsoZOffset << " m" << std::endl;
}

void KeyboardGaitSwingRunner::run() {
    _stopRequested = false;

    if (_headless) {
        runPhysicsLoop(false, false);
    } else {
        _stopRequested = false;
        _mainThread.init();

        std::thread physicsThread(&KeyboardGaitSwingRunner::runPhysicsLoop,
                                  this,
                                  true,
                                  true);
        _mainThread.run();

        _stopRequested = true;
        physicsThread.join();
    }

    _keyboardCommand.stop();

    mj_deleteData(_data);
    mj_deleteModel(_model);
    _data = nullptr;
    _model = nullptr;
}

void KeyboardGaitSwingRunner::runPhysicsLoop(const bool throttleRealtime, const bool syncViewer) {
    const auto wallStart = std::chrono::steady_clock::now();
    const double simStart = _data->time;
    const auto& simulationConfig = getSimulationConfig();
    const auto viewerSyncPeriod =
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(1.0 / simulationConfig.viewerSyncHz));
    std::chrono::steady_clock::time_point nextViewerSync = std::chrono::steady_clock::now();

    if (syncViewer) {
        _mainThread.load(_model, _data, _modelPath);
        _mainThread.sync();
        nextViewerSync = std::chrono::steady_clock::now() + viewerSyncPeriod;
    }

    while (!_stopRequested && (!syncViewer || !_mainThread.exitRequested())) {
        _keyboardCommand.start();
        _userCommand = _keyboardCommand.getUserCommand();

        clampFrozenQpos();
        runRobotControl();
        clampFrozenQpos();
        mj_step(_model, _data);
        clampFrozenQpos();

        if (_planarMotionEnabled) {
            advancePlanarBasePose(_model->opt.timestep);
        }

        ++_iterations;

        if (syncViewer && sim::shouldSyncViewer(nextViewerSync, viewerSyncPeriod)) {
            _mainThread.sync();
        }

        if (!throttleRealtime) {
            continue;
        }

        const double simElapsed = _data->time - simStart;
        const double wallElapsed =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - wallStart).count();
        if (simElapsed > wallElapsed) {
            std::this_thread::sleep_for(std::chrono::duration<double>(simElapsed - wallElapsed));
        }
    }

    std::cout << '\n' << "Simulated " << _iterations
              << " steps, sim time=" << _data->time << " sec" << "\n\n";
}

void KeyboardGaitSwingRunner::initializeControllerRuntime() {
    const auto robotSetup = setupRobotParams<double>(
        _robotType,
        _model,
        _controller->footEndEffectorSource());
    _params = robotSetup.params;
    _bindings = robotSetup.bindings;
    updateReducedBodyMassPropertiesFromData(_model, _data, _bindings, _params);
    _cheaterState.resize(_params);
    _stateEstimate.resize(_params);
    _robotCommand.resize(_model->nu);
    _robotCommand.tau.setZero();

    _robotModel = std::make_unique<RobotModel<double>>(&_params);
    if (!_robotModel->validate()) {
        throw std::runtime_error("Invalid RobotParams");
    }
    _legController = std::make_unique<LegController<double>>(*_robotModel);
    _armController = std::make_unique<ArmController<double>>(*_robotModel);
    _legController->setEnabled(true);
    _armController->setEnabled(false);

    _legQposIndices.clear();
    _legQvelIndices.clear();
    for (const auto& leg : _params.legs) {
        _legQposIndices.insert(_legQposIndices.end(),
                               leg.joints.q_idx.begin(),
                               leg.joints.q_idx.end());
        _legQvelIndices.insert(_legQvelIndices.end(),
                               leg.joints.qd_idx.begin(),
                               leg.joints.qd_idx.end());
    }
    if (_legQposIndices.empty() || _legQvelIndices.empty()) {
        throw std::runtime_error("KeyboardGaitSwingRunner could not find leg qpos indices");
    }

    _legSwingDynamicsProvider =
        std::make_unique<LegSwingDynamicsProvider>(_robotType, _model, _params, _bindings);
    _controller->bindRuntime(&_stateEstimate,
                             &_params,
                             _legController.get(),
                             _armController.get(),
                             &_userCommand);
}

void KeyboardGaitSwingRunner::runRobotControl() {
    if (_firstControllerRun) {
        initializeControllerRuntime();
        _firstControllerRun = false;
    }

    fillCheaterState(_model, _data, _params, _bindings, _cheaterState);
    _stateEstimator.update(_cheaterState, _stateEstimate);
    if (_legSwingDynamicsProvider != nullptr) {
        _legSwingDynamicsProvider->update(_stateEstimate);
    }

    _legController->zeroCommand();
    _legController->zeroData();
    _armController->zeroCommand();
    _armController->zeroData();

    for (std::size_t leg = 0; leg < _stateEstimate.legs.size(); ++leg) {
        const auto& legState = _stateEstimate.legs[leg];
        _legController->setLegJointData(static_cast<int>(leg), legState.q, legState.qd);
        if (legState.tauEstimate.size() == _legController->datas[leg].tauEstimate.size()) {
            _legController->setLegTauEstimate(static_cast<int>(leg), legState.tauEstimate);
        }
        if (legState.hasFootJacobians) {
            _legController->setLegCartesianData(static_cast<int>(leg),
                                                legState.footPos_W,
                                                legState.footVel_W,
                                                legState.Jv_W,
                                                legState.JvDot_W,
                                                legState.Jw_W);
        }
        if (legState.hasLegDynamics) {
            _legController->setLegDynamicsData(static_cast<int>(leg),
                                               legState.massMatrix,
                                               legState.bias);
        }
    }

    _legController->setEnabled(true);
    _controller->runController();
    updateDebugVisualization();

    _robotCommand.tau.setZero(_model->nu);
    _legController->updateCommand(_robotCommand.tau);
    applyRobotCommand();
}

void KeyboardGaitSwingRunner::applyRobotCommand() {
    if (_robotCommand.tau.size() != _model->nu) {
        throw std::runtime_error("RobotCommand torque dimension does not match model->nu");
    }

    for (int i = 0; i < _model->nu; ++i) {
        const double tau = _robotCommand.tau[i];
        if (_model->actuator_ctrllimited[i]) {
            const double lo = static_cast<double>(_model->actuator_ctrlrange[2 * i + 0]);
            const double hi = static_cast<double>(_model->actuator_ctrlrange[2 * i + 1]);
            _data->ctrl[i] = std::clamp(tau, lo, hi);
        } else {
            _data->ctrl[i] = tau;
        }
    }
}

void KeyboardGaitSwingRunner::updateDebugVisualization() {
    if (_model == nullptr || _data == nullptr || _controller == nullptr) {
        return;
    }

    DebugVizState<double> debugViz;
    _controller->collectDebugVisualization(debugViz);
    if (debugViz.markers.empty()) {
        return;
    }

    for (const auto& marker : debugViz.markers) {
        if (!marker.active) {
            continue;
        }

        auto it = std::find_if(_debugMocapBindings.begin(),
                               _debugMocapBindings.end(),
                               [&](const DebugMocapBinding& binding) {
                                   return binding.name == marker.name;
                               });

        if (it == _debugMocapBindings.end()) {
            DebugMocapBinding binding;
            binding.name = marker.name;
            binding.bodyId = mj_name2id(_model, mjOBJ_BODY, marker.name.c_str());
            if (binding.bodyId < 0) {
                continue;
            }

            binding.mocapId = _model->body_mocapid[binding.bodyId];
            if (binding.mocapId < 0) {
                continue;
            }

            _debugMocapBindings.push_back(binding);
            it = std::prev(_debugMocapBindings.end());
        }

        applyMarkerColor(_model, it->bodyId, marker);

        const Eigen::Index posOffset = static_cast<Eigen::Index>(3 * it->mocapId);
        _data->mocap_pos[posOffset + 0] = marker.position_W[0];
        _data->mocap_pos[posOffset + 1] = marker.position_W[1];
        _data->mocap_pos[posOffset + 2] = marker.position_W[2];

        const Eigen::Index quatOffset = static_cast<Eigen::Index>(4 * it->mocapId);
        _data->mocap_quat[quatOffset + 0] = marker.orientation_W.w();
        _data->mocap_quat[quatOffset + 1] = marker.orientation_W.x();
        _data->mocap_quat[quatOffset + 2] = marker.orientation_W.y();
        _data->mocap_quat[quatOffset + 3] = marker.orientation_W.z();
    }
}

void KeyboardGaitSwingRunner::applyCopiedStateKeyframe(const std::string& keyframeName) {
    const int keyId = mj_name2id(_model, mjOBJ_KEY, keyframeName.c_str());
    if (keyId < 0) {
        throw std::runtime_error(std::string("Failed to find MuJoCo keyframe: ") +
                                 keyframeName);
    }

    mj_resetData(_model, _data);
    const mjtNum* keyQpos = _model->key_qpos + keyId * _model->nq;
    for (int i = 0; i < _model->nq; ++i) {
        _data->qpos[i] = keyQpos[i];
    }
    _data->qpos[_freeJointQposIndex + 2] += static_cast<mjtNum>(_torsoZOffset);
    mj_forward(_model, _data);
}

void KeyboardGaitSwingRunner::locateFloatingBase() {
    _freeJointQposIndex = -1;
    _freeJointQvelIndex = -1;

    for (int jointId = 0; jointId < _model->njnt; ++jointId) {
        if (_model->jnt_type[jointId] == mjJNT_FREE) {
            _freeJointQposIndex = _model->jnt_qposadr[jointId];
            _freeJointQvelIndex = _model->jnt_dofadr[jointId];
            break;
        }
    }

    if (_freeJointQposIndex < 0 || _freeJointQvelIndex < 0) {
        throw std::runtime_error("KeyboardGaitSwingRunner could not find a floating base joint");
    }
}

void KeyboardGaitSwingRunner::cacheFrozenQpos() {
    _frozenQpos.assign(_model->nq, 0.0);
    for (int i = 0; i < _model->nq; ++i) {
        _frozenQpos[static_cast<std::size_t>(i)] = static_cast<double>(_data->qpos[i]);
    }
}

bool KeyboardGaitSwingRunner::isLegQposIndex(const int qposIndex) const {
    return std::find(_legQposIndices.begin(), _legQposIndices.end(), qposIndex) !=
           _legQposIndices.end();
}

bool KeyboardGaitSwingRunner::isLegQvelIndex(const int qvelIndex) const {
    return std::find(_legQvelIndices.begin(), _legQvelIndices.end(), qvelIndex) !=
           _legQvelIndices.end();
}

bool KeyboardGaitSwingRunner::isFloatingBaseQposIndex(const int qposIndex) const {
    return qposIndex >= _freeJointQposIndex && qposIndex < (_freeJointQposIndex + 7);
}

bool KeyboardGaitSwingRunner::isFloatingBaseQvelIndex(const int qvelIndex) const {
    return qvelIndex >= _freeJointQvelIndex && qvelIndex < (_freeJointQvelIndex + 6);
}

void KeyboardGaitSwingRunner::clampFrozenQpos() {
    if (_frozenQpos.empty()) {
        return;
    }

    for (int i = 0; i < _model->nq; ++i) {
        if (isFloatingBaseQposIndex(i) || isLegQposIndex(i)) {
            continue;
        }
        _data->qpos[i] = static_cast<mjtNum>(_frozenQpos[static_cast<std::size_t>(i)]);
    }
    for (int i = 0; i < _model->nv; ++i) {
        if (isFloatingBaseQvelIndex(i) || isLegQvelIndex(i)) {
            continue;
        }
        _data->qvel[i] = mjtNum(0);
    }

    mj_forward(_model, _data);
}

void KeyboardGaitSwingRunner::cachePlanarBasePose() {
    if (_data == nullptr) {
        throw std::runtime_error("KeyboardGaitSwingRunner requires MuJoCo data to cache base pose");
    }

    const mjtNum* qpos = _data->qpos + _freeJointQposIndex;
    _planarBasePosition_W = Vec3<double>(
        static_cast<double>(qpos[0]),
        static_cast<double>(qpos[1]),
        static_cast<double>(qpos[2]));
    _planarBaseZ = _planarBasePosition_W.z();

    const Quat<double> baseQuat_W(static_cast<double>(qpos[3]),
                                 static_cast<double>(qpos[4]),
                                 static_cast<double>(qpos[5]),
                                 static_cast<double>(qpos[6]));
    const Mat3<double> baseRotation_W = baseQuat_W.toRotationMatrix();
    _planarBaseYaw = yawFromRotation(baseRotation_W);
    _planarRotationNoYaw = Rz(-_planarBaseYaw) * baseRotation_W;

    applyPlanarBasePose(Vec3<double>::Zero(), Vec3<double>::Zero());
}

void KeyboardGaitSwingRunner::advancePlanarBasePose(const double dt) {
    if (!_planarMotionEnabled) {
        return;
    }

    const double step = std::max(0.0, dt);
    const Mat3<double> baseRotation_W = Rz(_planarBaseYaw) * _planarRotationNoYaw;
    const UserCommand clampedCommand = clampUserCommand(_userCommand);
    const Vec3<double> bodyLinearCommand(
        clampedCommand.x_dot,
        clampedCommand.y_dot,
        0.0);
    Vec3<double> worldLinearVelocity = baseRotation_W * bodyLinearCommand;
    worldLinearVelocity.z() = 0.0;
    const Vec3<double> worldAngularVelocity(0.0, 0.0, clampedCommand.psi_dot);
    const Vec3<double> bodyAngularVelocity = baseRotation_W.transpose() * worldAngularVelocity;

    _planarBasePosition_W += worldLinearVelocity * step;
    _planarBasePosition_W.z() = _planarBaseZ;
    _planarBaseYaw += clampedCommand.psi_dot * step;

    applyPlanarBasePose(worldLinearVelocity, bodyAngularVelocity);
}

void KeyboardGaitSwingRunner::applyPlanarBasePose(const Vec3<double>& worldLinearVelocity,
                                                   const Vec3<double>& bodyAngularVelocity) {
    if (_data == nullptr) {
        throw std::runtime_error("KeyboardGaitSwingRunner requires MuJoCo data to move the base");
    }

    const Mat3<double> baseRotation_W = Rz(_planarBaseYaw) * _planarRotationNoYaw;
    const Quat<double> baseQuat_W(baseRotation_W);

    _data->qpos[_freeJointQposIndex + 0] = static_cast<mjtNum>(_planarBasePosition_W.x());
    _data->qpos[_freeJointQposIndex + 1] = static_cast<mjtNum>(_planarBasePosition_W.y());
    _data->qpos[_freeJointQposIndex + 2] = static_cast<mjtNum>(_planarBaseZ);
    _data->qpos[_freeJointQposIndex + 3] = static_cast<mjtNum>(baseQuat_W.w());
    _data->qpos[_freeJointQposIndex + 4] = static_cast<mjtNum>(baseQuat_W.x());
    _data->qpos[_freeJointQposIndex + 5] = static_cast<mjtNum>(baseQuat_W.y());
    _data->qpos[_freeJointQposIndex + 6] = static_cast<mjtNum>(baseQuat_W.z());

    _data->qvel[_freeJointQvelIndex + 0] = static_cast<mjtNum>(worldLinearVelocity.x());
    _data->qvel[_freeJointQvelIndex + 1] = static_cast<mjtNum>(worldLinearVelocity.y());
    _data->qvel[_freeJointQvelIndex + 2] = static_cast<mjtNum>(worldLinearVelocity.z());
    _data->qvel[_freeJointQvelIndex + 3] = static_cast<mjtNum>(bodyAngularVelocity.x());
    _data->qvel[_freeJointQvelIndex + 4] = static_cast<mjtNum>(bodyAngularVelocity.y());
    _data->qvel[_freeJointQvelIndex + 5] = static_cast<mjtNum>(bodyAngularVelocity.z());

    mj_forward(_model, _data);
}
