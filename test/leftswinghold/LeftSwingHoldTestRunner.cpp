#include "LeftSwingHoldTestRunner.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <thread>

#include <mujoco/mujoco.h>

#include "ControllerConfig.h"
#include "MujocoCheaterStateReader.h"
#include "RobotConfig.h"
#include "SimulationConfig.h"
#include "ViewerSyncThrottle.h"
#include "setupRobotParams.h"

namespace {
constexpr const char* kInitialKeyframeName = "copied_state";
}  // namespace

LeftSwingHoldTestRunner::LeftSwingHoldTestRunner(const RobotType robotType,
                                                 LeftSwingHoldController* controller,
                                                 const bool headless,
                                                 const double torsoZOffset)
    : _robotType(robotType),
      _controller(controller),
      _headless(headless),
      _torsoZOffset(torsoZOffset) {
    if (_controller == nullptr) {
        throw std::invalid_argument("LeftSwingHoldTestRunner requires a controller");
    }
}

LeftSwingHoldTestRunner::~LeftSwingHoldTestRunner() {
    if (_data != nullptr) {
        mj_deleteData(_data);
        _data = nullptr;
    }
    if (_model != nullptr) {
        mj_deleteModel(_model);
        _model = nullptr;
    }
}

void LeftSwingHoldTestRunner::init() {
    setActiveRobotType(_robotType);
    const auto& controllerConfig = getControllerConfig(_robotType);
    const auto& runtimeConfig = getRobotRuntimeConfig(_robotType);
    const std::string xmlPath = controllerConfig.leftSwingHoldTest.xmlPath.empty()
                                    ? runtimeConfig.modelXmlPath
                                    : controllerConfig.leftSwingHoldTest.xmlPath;
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
    applyCopiedStateKeyframe();
    cacheFrozenQpos();
    clampFrozenQpos();

    std::cout << "Loaded MuJoCo model: " << _modelPath << '\n';
    std::cout << "nq=" << _model->nq
              << ", nv=" << _model->nv
              << ", nu=" << _model->nu << '\n';
    std::cout << "[LeftSwingHoldTest] initial keyframe: " << kInitialKeyframeName
              << ", torso z offset=" << _torsoZOffset << " m" << std::endl;
}

void LeftSwingHoldTestRunner::run() {
    if (_headless) {
        runPhysicsLoop(false, false);
        return;
    }

    _stopRequested = false;
    _mainThread.init();

    std::thread physicsThread(&LeftSwingHoldTestRunner::runPhysicsLoop, this, true, true);
    _mainThread.run();

    _stopRequested = true;
    physicsThread.join();
}

void LeftSwingHoldTestRunner::runPhysicsLoop(const bool throttleRealtime, const bool syncViewer) {
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
        clampFrozenQpos();
        runRobotControl();
        clampFrozenQpos();
        mj_step(_model, _data);
        clampFrozenQpos();

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

void LeftSwingHoldTestRunner::initializeControllerRuntime() {
    const auto robotSetup = setupRobotParams<double>(
        _robotType,
        _model,
        _controller->footEndEffectorSource());
    _params = robotSetup.params;
    _bindings = robotSetup.bindings;

    if (_bindings.feet.size() != _params.legs.size()) {
        throw std::runtime_error("Mujoco bindings do not match leg count");
    }

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

    _leftLegQposIndices.clear();
    for (const auto& leg : _params.legs) {
        if (leg.side != Side::Left) {
            continue;
        }
        _leftLegQposIndices = leg.joints.q_idx;
        _leftLegQvelIndices = leg.joints.qd_idx;
        break;
    }
    if (_leftLegQposIndices.empty() || _leftLegQvelIndices.empty()) {
        throw std::runtime_error("LeftSwingHoldTestRunner could not find left leg qpos indices");
    }

    _legSwingDynamicsProvider =
        std::make_unique<LegSwingDynamicsProvider>(_robotType, _model, _params, _bindings);
    _controller->bindRuntime(&_stateEstimate,
                             &_params,
                             _legController.get(),
                             _armController.get(),
                             &_userCommand);
}

void LeftSwingHoldTestRunner::runRobotControl() {
    if (_firstControllerRun) {
        initializeControllerRuntime();
        _firstControllerRun = false;
    }

    fillCheaterState(_model, _data, _params, _bindings, _cheaterState);
    _stateEstimator.update(_cheaterState, _stateEstimate);
    if (_legSwingDynamicsProvider) {
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

void LeftSwingHoldTestRunner::applyRobotCommand() {
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

void LeftSwingHoldTestRunner::updateDebugVisualization() {
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

void LeftSwingHoldTestRunner::applyCopiedStateKeyframe() {
    const int keyId = mj_name2id(_model, mjOBJ_KEY, kInitialKeyframeName);
    if (keyId < 0) {
        throw std::runtime_error(std::string("Failed to find MuJoCo keyframe: ") +
                                 kInitialKeyframeName);
    }

    mj_resetData(_model, _data);
    const mjtNum* keyQpos = _model->key_qpos + keyId * _model->nq;
    for (int i = 0; i < _model->nq; ++i) {
        _data->qpos[i] = keyQpos[i];
    }
    _data->qpos[_freeJointQposIndex + 2] += static_cast<mjtNum>(_torsoZOffset);
    mj_forward(_model, _data);
}

void LeftSwingHoldTestRunner::locateFloatingBase() {
    _freeJointQposIndex = -1;

    for (int jointId = 0; jointId < _model->njnt; ++jointId) {
        if (_model->jnt_type[jointId] == mjJNT_FREE) {
            _freeJointQposIndex = _model->jnt_qposadr[jointId];
            _freeJointQvelIndex = _model->jnt_dofadr[jointId];
            break;
        }
    }

    if (_freeJointQposIndex < 0 || _freeJointQvelIndex < 0) {
        throw std::runtime_error("LeftSwingHoldTestRunner could not find a floating base joint");
    }
}

void LeftSwingHoldTestRunner::cacheFrozenQpos() {
    _frozenQpos.assign(_model->nq, 0.0);
    for (int i = 0; i < _model->nq; ++i) {
        _frozenQpos[static_cast<std::size_t>(i)] = static_cast<double>(_data->qpos[i]);
    }
}

bool LeftSwingHoldTestRunner::isLeftLegQposIndex(const int qposIndex) const {
    return std::find(_leftLegQposIndices.begin(), _leftLegQposIndices.end(), qposIndex) !=
           _leftLegQposIndices.end();
}

bool LeftSwingHoldTestRunner::isLeftLegQvelIndex(const int qvelIndex) const {
    return std::find(_leftLegQvelIndices.begin(), _leftLegQvelIndices.end(), qvelIndex) !=
           _leftLegQvelIndices.end();
}

void LeftSwingHoldTestRunner::clampFrozenQpos() {
    if (_frozenQpos.empty()) {
        return;
    }

    for (int i = 0; i < _model->nq; ++i) {
        if (isLeftLegQposIndex(i)) {
            continue;
        }
        _data->qpos[i] = static_cast<mjtNum>(_frozenQpos[static_cast<std::size_t>(i)]);
    }
    for (int i = 0; i < _model->nv; ++i) {
        if (isLeftLegQvelIndex(i)) {
            continue;
        }
        _data->qvel[i] = mjtNum(0);
    }

    mj_forward(_model, _data);
}
