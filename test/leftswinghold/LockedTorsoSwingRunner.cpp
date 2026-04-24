#include "LockedTorsoSwingRunner.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <thread>

#include <mujoco/mujoco.h>

#include "MujocoCheaterStateReader.h"
#include "SimulationConfig.h"
#include "ViewerSyncThrottle.h"
#include "setupRobotParams.h"

LockedTorsoSwingRunner::LockedTorsoSwingRunner(const RobotType robotType,
                                               RobotController* controller,
                                               const bool headless)
    : _robotType(robotType),
      _robotRunner(std::make_unique<RobotRunner>(controller)),
      _headless(headless) {}

LockedTorsoSwingRunner::~LockedTorsoSwingRunner() {
    if (_data != nullptr) {
        mj_deleteData(_data);
        _data = nullptr;
    }
    if (_model != nullptr) {
        mj_deleteModel(_model);
        _model = nullptr;
    }
}

void LockedTorsoSwingRunner::init() {
    _modelPath = std::string(PROJECT_ROOT_DIR) + "/models/mit_humanoid/scene.xml";

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
    mj_forward(_model, _data);

    locateFloatingBase();

    std::cout << "Loaded MuJoCo model: " << _modelPath << '\n';
    std::cout << "nq=" << _model->nq
              << ", nv=" << _model->nv
              << ", nu=" << _model->nu << '\n';
}

void LockedTorsoSwingRunner::run() {
    if (_headless) {
        runPhysicsLoop(false, false);
        return;
    }

    _stopRequested = false;
    _mainThread.init();

    std::thread physicsThread(&LockedTorsoSwingRunner::runPhysicsLoop, this, true, true);
    _mainThread.run();

    _stopRequested = true;
    physicsThread.join();
}

void LockedTorsoSwingRunner::runPhysicsLoop(const bool throttleRealtime, const bool syncViewer) {
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
        if (_torsoLocked) {
            clampFloatingBase();
        }

        runRobotControl();

        if (_torsoLocked) {
            clampFloatingBase();
        }

        mj_step(_model, _data);

        if (_torsoLocked) {
            clampFloatingBase();
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

void LockedTorsoSwingRunner::runRobotControl() {
    if (_firstControllerRun) {
        const auto robotSetup = setupRobotParams<double>(
            _robotType,
            _model,
            _robotRunner->_robot_ctrl->footEndEffectorSource());
        _params = robotSetup.params;
        _bindings = robotSetup.bindings;
        updateReducedBodyMassPropertiesFromData(_model, _data, _bindings, _params);
        _cheaterState.resize(_params);
        _stateEstimate.resize(_params);
        _legSwingDynamicsProvider =
            std::make_unique<LegSwingDynamicsProvider>(_robotType, _model, _params, _bindings);
        _robotRunner->init(&_params, _model->opt.timestep, &_userCommand);
        _firstControllerRun = false;
    }

    fillCheaterState(_model, _data, _params, _bindings, _cheaterState);
    _stateEstimator.update(_cheaterState, _stateEstimate);
    if (_legSwingDynamicsProvider) {
        _legSwingDynamicsProvider->update(_stateEstimate);
    }

    _robotRunner->run(_stateEstimate, _robotCommand);
    applyRobotCommand();

    if (!_torsoLocked && _robotRunner->initializationComplete()) {
        cacheLockedBasePose();
        _torsoLocked = true;
        clampFloatingBase();
        std::cout << "[LeftSwingHoldTest] torso lock engaged after initial pose convergence" << std::endl;
    }
}

void LockedTorsoSwingRunner::applyRobotCommand() {
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

void LockedTorsoSwingRunner::locateFloatingBase() {
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
        throw std::runtime_error("LockedTorsoSwingRunner could not find a floating base joint");
    }
}

void LockedTorsoSwingRunner::cacheLockedBasePose() {
    for (int i = 0; i < 7; ++i) {
        _lockedBaseQpos[static_cast<std::size_t>(i)] =
            static_cast<double>(_data->qpos[_freeJointQposIndex + i]);
    }
}

void LockedTorsoSwingRunner::clampFloatingBase() {
    for (int i = 0; i < 7; ++i) {
        _data->qpos[_freeJointQposIndex + i] =
            static_cast<mjtNum>(_lockedBaseQpos[static_cast<std::size_t>(i)]);
    }
    for (int i = 0; i < 6; ++i) {
        _data->qvel[_freeJointQvelIndex + i] = mjtNum(0);
    }
    _data->qpos[_freeJointQposIndex + 2] += 0.2;

    mj_forward(_model, _data);
}
