#include "SwingLegTestController.h"

#include <algorithm>
#include <iostream>
#include <stdexcept>

#include "Controllers/LegController.h"
#include "Dynamics/OperationalSpaceDynamics.h"
#include "Utilities/MatrixUtils.h"

Mat3<double> SwingLegTestController::makeDiagonal(const double x,
                                                  const double y,
                                                  const double z) {
    Mat3<double> diagonal = Mat3<double>::Zero();
    diagonal.diagonal() << x, y, z;
    return diagonal;
}

void SwingLegTestController::initializeController() {
    if (_initialized) {
        return;
    }

    initializeRuntime();
}

void SwingLegTestController::initializeRuntime() {
    if (_legController == nullptr || _stateEstimate == nullptr || _robotParams == nullptr) {
        throw std::runtime_error(
            "SwingLegTestController initialization requires leg controller, state estimate, and robot params");
    }
    if (_robotParams->legs.empty()) {
        throw std::runtime_error("SwingLegTestController requires at least one leg");
    }

    const auto& config = getControllerConfig();
    _swingDuration = swingTime();
    _swingHeight = std::max(config.swing.height, 0.08);
    _swingNaturalFrequency = config.swing.naturalFrequency;
    _swingKd = makeDiagonal(config.swing.kdDiag[0], config.swing.kdDiag[1], config.swing.kdDiag[2]);

    const Vec3<double> comWorld = reducedBodyComWorld();
    _legs.resize(_robotParams->legs.size());
    for (std::size_t leg = 0; leg < _robotParams->legs.size(); ++leg) {
        auto& runtime = _legs[leg];
        runtime.centerFootPosition = _stateEstimate->legs[leg].footPos_W;
        runtime.nominalFootFromCom = runtime.centerFootPosition - comWorld;
    }
    resetTrajectories();

    _lastTime = _stateEstimate->time;
    _iteration = 0;
    _initialized = true;

    std::cout << "[SwingLegTest] body fixed, both legs use swing controller" << std::endl;
}

Vec3<double> SwingLegTestController::reducedBodyComWorld() const {
    if (_stateEstimate == nullptr || _robotParams == nullptr) {
        throw std::runtime_error("SwingLegTestController requires state and params for reduced COM");
    }

    return _stateEstimate->torsoPos_W + Rz(_stateEstimate->psi) * _robotParams->bodyComLocation;
}

Vec3<double> SwingLegTestController::touchdownTargetWorld(const std::size_t legIndex) const {
    if (_stateEstimate == nullptr || _robotParams == nullptr) {
        throw std::runtime_error("SwingLegTestController touchdownTargetWorld requires state");
    }

    const Vec3<double> comWorld = reducedBodyComWorld();
    const Vec3<double> commandedVelocityBody(
        _userCommand != nullptr ? _userCommand->x_dot : 0.0,
        _userCommand != nullptr ? _userCommand->y_dot : 0.0,
        0.0);
    const double psiDot = (_userCommand != nullptr) ? _userCommand->psi_dot : 0.0;

    const double psi = _stateEstimate->psi;
    const Mat3<double> R_WB = Rz(psi);
    const Mat3<double> R_yaw = Rz(0.5 * psiDot * _swingDuration);

    const Vec3<double> nominalWorld = _legs[legIndex].nominalFootFromCom;
    const Vec3<double> nominalBody = R_WB.transpose() * nominalWorld;
    const Vec3<double> translatedBody = R_yaw * nominalBody + commandedVelocityBody * _swingDuration;

    Vec3<double> target = comWorld + R_WB * translatedBody;
    target.z() = _legs[legIndex].centerFootPosition.z();
    return target;
}

void SwingLegTestController::resetTrajectories() {
    for (std::size_t leg = 0; leg < _legs.size(); ++leg) {
        const Vec3<double>& currentFootPosition = _stateEstimate->legs[leg].footPos_W;
        _legs[leg].trajectory.reset(
            currentFootPosition, touchdownTargetWorld(leg), _swingHeight, _swingDuration);
    }
}

void SwingLegTestController::configureSwingLeg(const std::size_t legIndex) {
    auto& command = _legController->commands[legIndex];
    const auto& runtime = _legs[legIndex];
    const auto& legData = _legController->datas[legIndex];

    command.mode = LegControlMode::SwingFoot;
    command.tauFeedForward.setZero(command.dof());
    command.forceFeedForward_W.setZero();
    command.momentFeedForward_W.setZero();
    command.kpCartesian = computeSwingCartesianKp(legData.Jv_W, legData.massMatrix, _swingNaturalFrequency);
    command.pDes_W = runtime.trajectory.position();
    command.vDes_W = runtime.trajectory.velocity();
    command.aDes_W = runtime.trajectory.acceleration();
    command.kdCartesian = _swingKd;
}

void SwingLegTestController::printStatus() const {
    if ((_iteration % 200) != 0) {
        return;
    }

    std::cout << "[SwingLegTest] cmd=("
              << (_userCommand != nullptr ? _userCommand->x_dot : 0.0) << ", "
              << (_userCommand != nullptr ? _userCommand->y_dot : 0.0) << ", "
              << (_userCommand != nullptr ? _userCommand->psi_dot : 0.0) << ")" << std::endl;

    for (std::size_t leg = 0; leg < _legs.size(); ++leg) {
        const auto& desired = _legs[leg].trajectory.position();
        const auto& actual = _stateEstimate->legs[leg].footPos_W;
        std::cout << "  leg " << leg
                  << " p_des: " << desired.transpose()
                  << " | p_act: " << actual.transpose() << std::endl;
    }
}

void SwingLegTestController::runController() {
    if (!_initialized) {
        initializeRuntime();
    }

    if (_stateEstimate == nullptr || _legController == nullptr || _robotParams == nullptr) {
        throw std::runtime_error("SwingLegTestController requires initialized runtime pointers");
    }

    const double time = _stateEstimate->time;
    const double dt = std::max(0.0, time - _lastTime);

    for (std::size_t leg = 0; leg < _legs.size(); ++leg) {
        auto& runtime = _legs[leg];
        if (!runtime.trajectory.active()) {
            runtime.trajectory.reset(
                _stateEstimate->legs[leg].footPos_W,
                touchdownTargetWorld(leg),
                _swingHeight,
                _swingDuration);
        } else {
            runtime.trajectory.setFinalPosition(touchdownTargetWorld(leg));
            runtime.trajectory.advance(dt);
        }

        configureSwingLeg(leg);
    }

    printStatus();

    _lastTime = time;
    ++_iteration;
}
