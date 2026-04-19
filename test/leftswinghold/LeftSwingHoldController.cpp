#include "LeftSwingHoldController.h"

#include <algorithm>
#include <iostream>
#include <stdexcept>

#include "Controllers/LegController.h"
#include "Dynamics/OperationalSpaceDynamics.h"

Mat3<double> LeftSwingHoldController::makeDiagonal(const double x,
                                                   const double y,
                                                   const double z) {
    Mat3<double> diagonal = Mat3<double>::Zero();
    diagonal.diagonal() << x, y, z;
    return diagonal;
}

JointPdGains<double> LeftSwingHoldController::makeInitialJointGains(const RobotType robotType,
                                                                    const Eigen::Index dof) {
    (void)robotType;
    JointPdGains<double> gains;

    if (dof != 5) {
        throw std::runtime_error("MIT humanoid leg dof no longer matches initial joint gains");
    }

    gains.set({60.0, 50.0, 55.0, 60.0, 40.0},
              {15.0, 10.0, 7.0, 9.0, 10.0});
    return gains;
}

void LeftSwingHoldController::initializeController() {
    if (_initialized) {
        return;
    }

    initializeRuntime();
}

void LeftSwingHoldController::initializeRuntime() {
    if (_legController == nullptr || _stateEstimate == nullptr || _robotParams == nullptr) {
        throw std::runtime_error(
            "LeftSwingHoldController initialization requires leg controller, state estimate, and robot params");
    }

    _leftLegIndex = findLegIndex(Side::Left);
    _rightLegIndex = findLegIndex(Side::Right);

    _leftHoldQ = _stateEstimate->legs[static_cast<std::size_t>(_leftLegIndex)].q;
    _rightHoldQ = _stateEstimate->legs[static_cast<std::size_t>(_rightLegIndex)].q;
    _leftHoldFootWorld = _stateEstimate->legs[static_cast<std::size_t>(_leftLegIndex)].footPos_W;

    _jointHoldGains = makeInitialJointGains(_robotParams->roboType, _leftHoldQ.size());

    const auto& config = getControllerConfig();
    _swingDuration = swingTime();
    _holdDuration = stanceTime();
    _swingHeight = std::max(config.swing.height, 0.06);
    _swingNaturalFrequency = config.swing.naturalFrequency;
    _swingKd = makeDiagonal(config.swing.kdDiag[0], config.swing.kdDiag[1], config.swing.kdDiag[2]);

    _phase = Phase::Swing;
    _phaseElapsed = 0.0;
    _lastTime = _stateEstimate->time;
    startSwingPhase();
    _initialized = true;

    std::cout << "[LeftSwingHoldTest] torso locked after init pose, left leg alternates swing/hold" << std::endl;
}

int LeftSwingHoldController::findLegIndex(const Side side) const {
    if (_robotParams == nullptr) {
        throw std::runtime_error("LeftSwingHoldController requires robot params");
    }

    for (std::size_t leg = 0; leg < _robotParams->legs.size(); ++leg) {
        if (_robotParams->legs[leg].side == side) {
            return static_cast<int>(leg);
        }
    }

    throw std::runtime_error("Requested leg side is missing from RobotParams");
}

void LeftSwingHoldController::startSwingPhase() {
    if (_stateEstimate == nullptr) {
        throw std::runtime_error("LeftSwingHoldController requires state estimate to start swing");
    }

    _phase = Phase::Swing;
    _phaseElapsed = 0.0;
    _leftSwingTrajectory.reset(
        _stateEstimate->legs[static_cast<std::size_t>(_leftLegIndex)].footPos_W,
        _leftHoldFootWorld,
        _swingHeight,
        _swingDuration);
}

void LeftSwingHoldController::startHoldPhase() {
    _phase = Phase::Hold;
    _phaseElapsed = 0.0;
    _leftSwingTrajectory.deactivate();
}

void LeftSwingHoldController::configureJointHold(const int legIndex, const DVec<double>& qHold) {
    auto& command = _legController->commands[static_cast<std::size_t>(legIndex)];

    command.mode = LegControlMode::JointPd;
    command.tauFeedForward.setZero(command.dof());
    command.forceFeedForward_W.setZero();
    command.momentFeedForward_W.setZero();
    command.qDes = qHold;
    command.qdDes.setZero(command.dof());
    command.pDes_W.setZero();
    command.vDes_W.setZero();
    command.aDes_W.setZero();
    command.kpCartesian.setZero();
    command.kdCartesian.setZero();
    _jointHoldGains.applyTo(command);
}

void LeftSwingHoldController::configureSwingLeg(const int legIndex) {
    auto& command = _legController->commands[static_cast<std::size_t>(legIndex)];
    const auto& legData = _legController->datas[static_cast<std::size_t>(legIndex)];

    command.mode = LegControlMode::SwingFoot;
    command.tauFeedForward.setZero(command.dof());
    command.forceFeedForward_W.setZero();
    command.momentFeedForward_W.setZero();
    command.qDes.setZero(command.dof());
    command.qdDes.setZero(command.dof());
    command.kpJoint.setZero(command.dof(), command.dof());
    command.kdJoint.setZero(command.dof(), command.dof());
    command.kpCartesian =
        computeSwingCartesianKp(legData.Jv_W, legData.massMatrix, _swingNaturalFrequency);
    command.pDes_W = _leftSwingTrajectory.position();
    command.vDes_W = _leftSwingTrajectory.velocity();
    command.aDes_W = _leftSwingTrajectory.acceleration();
    command.kdCartesian = _swingKd;
}

void LeftSwingHoldController::maybePrintStatus() const {
    if ((_iteration % 5000) != 0 || _stateEstimate == nullptr) {
        return;
    }

        const auto& leftFoot = _stateEstimate->legs[static_cast<std::size_t>(_leftLegIndex)].footPos_W;
    std::cout << "[LeftSwingHoldTest] phase="
              << (_phase == Phase::Swing ? "swing" : "hold")
              << " left_foot_des=" << _leftSwingTrajectory.position().transpose()
              << " left_foot_act=" << leftFoot.transpose() << std::endl;
}

void LeftSwingHoldController::runController() {
    if (!_initialized) {
        initializeRuntime();
    }

    if (_legController == nullptr || _stateEstimate == nullptr) {
        throw std::runtime_error("LeftSwingHoldController requires initialized runtime");
    }

    const double time = _stateEstimate->time;
    const double dt = std::max(0.0, time - _lastTime);
    _phaseElapsed += dt;

    if (_phase == Phase::Swing) {
        _leftSwingTrajectory.setFinalPosition(_leftHoldFootWorld);
        _leftSwingTrajectory.advance(dt);
        if (_phaseElapsed >= _swingDuration || !_leftSwingTrajectory.active()) {
            startHoldPhase();
        }
    } else if (_phaseElapsed >= _holdDuration) {
        startSwingPhase();
    }

    configureJointHold(_rightLegIndex, _rightHoldQ);
    if (_phase == Phase::Swing) {
        configureSwingLeg(_leftLegIndex);
    } else {
        configureJointHold(_leftLegIndex, _leftHoldQ);
    }

    maybePrintStatus();

    _lastTime = time;
    ++_iteration;
}
