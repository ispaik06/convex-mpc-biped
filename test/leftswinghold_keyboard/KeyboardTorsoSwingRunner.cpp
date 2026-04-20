#include "KeyboardTorsoSwingRunner.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <thread>

#include <mujoco/mujoco.h>

#include "MujocoCheaterStateReader.h"
#include "Utilities/MatrixUtils.h"
#include "setupRobotParams.h"

namespace {
double wrapAngle(const double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

double yawFromRotation(const Mat3<double>& rotation) {
    return std::atan2(rotation(1, 0), rotation(0, 0));
}
}  // namespace

KeyboardTorsoSwingRunner::KeyboardTorsoSwingRunner(const RobotType robotType,
                                                   RobotController* controller,
                                                   const bool headless)
    : _robotType(robotType),
      _robotRunner(std::make_unique<RobotRunner>(controller)),
      _headless(headless) {}

KeyboardTorsoSwingRunner::~KeyboardTorsoSwingRunner() {
    if (_data != nullptr) {
        mj_deleteData(_data);
        _data = nullptr;
    }
    if (_model != nullptr) {
        mj_deleteModel(_model);
        _model = nullptr;
    }
}

void KeyboardTorsoSwingRunner::init() {
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

    mj_forward(_model, _data);
    _model->opt.timestep = 0.002;
    _model->opt.integrator = mjINT_IMPLICITFAST;

    locateFloatingBase();

    std::cout << "Loaded MuJoCo model: " << _modelPath << '\n';
    std::cout << "nq=" << _model->nq
              << ", nv=" << _model->nv
              << ", nu=" << _model->nu << '\n';
}

void KeyboardTorsoSwingRunner::run() {
    _stopRequested = false;
    _keyboardCommand.start();

    if (_headless) {
        runPhysicsLoop(false, false);
    } else {
        _stopRequested = false;
        _mainThread.init();

        std::thread physicsThread(&KeyboardTorsoSwingRunner::runPhysicsLoop,
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

void KeyboardTorsoSwingRunner::runPhysicsLoop(const bool throttleRealtime, const bool syncViewer) {
    const auto wallStart = std::chrono::steady_clock::now();
    const double simStart = _data->time;

    if (syncViewer) {
        _mainThread.load(_model, _data, _modelPath);
        _mainThread.sync();
    }

    while (!_stopRequested && (!syncViewer || !_mainThread.exitRequested())) {
        _userCommand = _keyboardCommand.getUserCommand();

        runRobotControl();
        mj_step(_model, _data);

        if (_planarMotionEnabled) {
            advancePlanarBasePose(_model->opt.timestep);
        }

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

void KeyboardTorsoSwingRunner::runRobotControl() {
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
    if (_legSwingDynamicsProvider != nullptr) {
        _legSwingDynamicsProvider->update(_stateEstimate);
    }

    _robotRunner->run(_stateEstimate, _robotCommand);
    applyRobotCommand();

    if (!_planarMotionEnabled && _robotRunner->initializationComplete()) {
        cachePlanarBasePose();
        _planarMotionEnabled = true;
        std::cout << "[KeyboardLeftSwingHoldTest] planar torso motion enabled after initial pose convergence"
                  << std::endl;
    }
}

void KeyboardTorsoSwingRunner::applyRobotCommand() {
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

void KeyboardTorsoSwingRunner::locateFloatingBase() {
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
        throw std::runtime_error("KeyboardTorsoSwingRunner could not find a floating base joint");
    }
}

void KeyboardTorsoSwingRunner::cachePlanarBasePose() {
    if (_data == nullptr) {
        throw std::runtime_error("KeyboardTorsoSwingRunner requires MuJoCo data to cache base pose");
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
    _planarBaseYaw = wrapAngle(yawFromRotation(baseRotation_W));
    _planarRotationNoYaw = Rz(-_planarBaseYaw) * baseRotation_W;

    applyPlanarBasePose(Vec3<double>::Zero(), Vec3<double>::Zero());
}

void KeyboardTorsoSwingRunner::advancePlanarBasePose(const double dt) {
    if (!_planarMotionEnabled) {
        return;
    }

    const double step = std::max(0.0, dt);
    const Mat3<double> baseRotation_W = Rz(_planarBaseYaw) * _planarRotationNoYaw;
    const Vec3<double> bodyLinearCommand(
        _userCommand.x_dot,
        _userCommand.y_dot,
        0.0);
    Vec3<double> worldLinearVelocity = baseRotation_W * bodyLinearCommand;
    worldLinearVelocity.z() = 0.0;
    const Vec3<double> worldAngularVelocity(0.0, 0.0, _userCommand.psi_dot);
    const Vec3<double> bodyAngularVelocity = baseRotation_W.transpose() * worldAngularVelocity;

    _planarBasePosition_W += worldLinearVelocity * step;
    _planarBasePosition_W.z() = _planarBaseZ;
    _planarBaseYaw = wrapAngle(_planarBaseYaw + _userCommand.psi_dot * step);

    applyPlanarBasePose(worldLinearVelocity, bodyAngularVelocity);
}

void KeyboardTorsoSwingRunner::applyPlanarBasePose(const Vec3<double>& worldLinearVelocity,
                                                   const Vec3<double>& bodyAngularVelocity) {
    if (_data == nullptr) {
        throw std::runtime_error("KeyboardTorsoSwingRunner requires MuJoCo data to move the base");
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
