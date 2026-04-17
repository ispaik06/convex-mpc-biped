#include "FixedBaseSwingTestRunner.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <thread>

#include <mujoco/mujoco.h>

#include "MujocoCheaterStateReader.h"
#include "setupRobotParams.h"

FixedBaseSwingTestRunner::FixedBaseSwingTestRunner(const RobotType robotType,
                                                   RobotController* controller,
                                                   const bool headless)
    : _robotType(robotType),
      _robotRunner(std::make_unique<RobotRunner>(controller)),
      _headless(headless) {}

FixedBaseSwingTestRunner::~FixedBaseSwingTestRunner() {
    if (_data != nullptr) {
        mj_deleteData(_data);
        _data = nullptr;
    }
    if (_model != nullptr) {
        mj_deleteModel(_model);
        _model = nullptr;
    }
}

void FixedBaseSwingTestRunner::init() {
    if (_robotType == RobotType::MIT_HUMANOID) {
        _modelPath = std::string(PROJECT_ROOT_DIR) + "/models/mit_humanoid/scene.xml";
    } else if (_robotType == RobotType::UNITREE_G1) {
        _modelPath = std::string(PROJECT_ROOT_DIR) + "/models/unitree_robots/g1/scene_23dof.xml";
    } else if (_robotType == RobotType::UNITREE_H1) {
        _modelPath = std::string(PROJECT_ROOT_DIR) + "/models/unitree_robots/h1/scene.xml";
    } else {
        throw std::runtime_error("Unsupported robot type for FixedBaseSwingTestRunner");
    }

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

    mj_forward(_model, _data);
    _model->opt.timestep = 0.002;
    _model->opt.integrator = mjINT_IMPLICITFAST;

    cacheFloatingBaseState();
    clampFloatingBase();

    std::cout << "Loaded MuJoCo model: " << _modelPath << '\n';
    std::cout << "nq=" << _model->nq
              << ", nv=" << _model->nv
              << ", nu=" << _model->nu << '\n';
    std::cout << "[SwingLegTest] fixed-base mode enabled" << std::endl;
}

void FixedBaseSwingTestRunner::run() {
    _keyboardCommand.start();

    if (_headless) {
        // Intentionally unbounded in headless mode for manual inspection runs.
        runPhysicsLoop(false, false);
        _keyboardCommand.stop();
        return;
    }

    _stopRequested = false;
    _mainThread.init();

    std::thread physicsThread(&FixedBaseSwingTestRunner::runPhysicsLoop, this, true, true);
    _mainThread.run();

    _stopRequested = true;
    physicsThread.join();
    _keyboardCommand.stop();
}

void FixedBaseSwingTestRunner::runPhysicsLoop(const bool throttleRealtime, const bool syncViewer) {
    const auto wallStart = std::chrono::steady_clock::now();
    const double simStart = _data->time;

    if (syncViewer) {
        _mainThread.load(_model, _data, _modelPath);
        _mainThread.sync();
    }

    while (!_stopRequested && (!syncViewer || !_mainThread.exitRequested())) {
        runRobotControl();
        mj_step(_model, _data);
        clampFloatingBase();
        ++_iterations;

        if (syncViewer) {
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

void FixedBaseSwingTestRunner::runRobotControl() {
    if (_firstControllerRun) {
        const auto robotSetup = setupRobotParams<double>(_robotType, _model);
        _params = robotSetup.params;
        _bindings = robotSetup.bindings;
        updateReducedBodyMassPropertiesFromData(_model, _data, _bindings, _params);
        _cheaterState.resize(_params);
        _stateEstimate.resize(_params);
        _legSwingDynamicsProvider =
            std::make_unique<LegSwingDynamicsProvider>(_robotType, _model, _params, _bindings);
        _robotRunner->init(&_params, _model->opt.timestep, &_userCommand);
        _firstControllerRun = false;
        std::cout << _model->opt.timestep << std::endl;
    }

    fillCheaterState(_model, _data, _params, _bindings, _cheaterState);
    _stateEstimator.update(_cheaterState, _stateEstimate);
    if (_legSwingDynamicsProvider) {
        _legSwingDynamicsProvider->update(_stateEstimate);
    }

    _userCommand = _keyboardCommand.getUserCommand();

    _robotRunner->run(_stateEstimate, _robotCommand);
    applyRobotCommand();
}

void FixedBaseSwingTestRunner::applyRobotCommand() {
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

void FixedBaseSwingTestRunner::cacheFloatingBaseState() {
    _freeJointQposAdr = -1;
    _freeJointQvelAdr = -1;
    for (int jointId = 0; jointId < _model->njnt; ++jointId) {
        if (_model->jnt_type[jointId] == mjJNT_FREE) {
            _freeJointQposAdr = _model->jnt_qposadr[jointId];
            _freeJointQvelAdr = _model->jnt_dofadr[jointId];
            break;
        }
    }

    if (_freeJointQposAdr < 0 || _freeJointQvelAdr < 0) {
        return;
    }

    for (int i = 0; i < 7; ++i) {
        _fixedBaseQpos[static_cast<std::size_t>(i)] =
            static_cast<double>(_data->qpos[_freeJointQposAdr + i]);
    }
}

void FixedBaseSwingTestRunner::clampFloatingBase() {
    if (_freeJointQposAdr < 0 || _freeJointQvelAdr < 0) {
        return;
    }

    for (int i = 0; i < 7; ++i) {
        _data->qpos[_freeJointQposAdr + i] =
            static_cast<mjtNum>(_fixedBaseQpos[static_cast<std::size_t>(i)]);
    }
    for (int i = 0; i < 6; ++i) {
        _data->qvel[_freeJointQvelAdr + i] = mjtNum(0);
    }

    mj_forward(_model, _data);
}
