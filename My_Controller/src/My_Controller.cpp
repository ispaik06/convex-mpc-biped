#include "My_Controller.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

#include "Controllers/LegController.h"
#include "Utilities/MatrixUtils.h"

namespace {
double clampUnit(const double value) {
    return std::clamp(value, -1.0, 1.0);
}

Vec3<double> desiredFootPositionForSide(const DesiredFootPositions& desiredFootPositions,
                                        const Side side) {
    switch (side) {
        case Side::Left:
            return desiredFootPositions.left_des_W;
        case Side::Right:
            return desiredFootPositions.right_des_W;
        default:
            throw std::runtime_error("MyController only supports left/right leg swing targets");
    }
}

double remainingSwingTime(const GaitScheduler& gaitScheduler, const Side side, const double time) {
    return std::clamp(cycleTime() * (1.0 - gaitScheduler.p(side, time)), 0.0, swingTime());
}

Vec2<double> quaternionToRollPitch(Quat<double> quat) {
    quat.normalize();

    const double w = quat.w();
    const double x = quat.x();
    const double y = quat.y();
    const double z = quat.z();

    Vec2<double> rollPitch = Vec2<double>::Zero();
    rollPitch[0] = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
    rollPitch[1] = std::asin(clampUnit(2.0 * (w * y - z * x)));
    return rollPitch;
}

Vec3<double> reducedBodyOffsetWorld(const StateEstimate<double>& stateEstimate,
                                    const RobotParams<double>& robotParams) {
    return Rz(stateEstimate.psi) * robotParams.bodyComLocation;
}

Vec3<double> reducedBodyComWorld(const StateEstimate<double>& stateEstimate,
                                 const RobotParams<double>& robotParams) {
    return stateEstimate.torsoPos_W + reducedBodyOffsetWorld(stateEstimate, robotParams);
}

Vec3<double> reducedBodyComVelocityWorld(const StateEstimate<double>& stateEstimate,
                                         const RobotParams<double>& robotParams) {
    const Vec3<double> offsetWorld = reducedBodyOffsetWorld(stateEstimate, robotParams);
    const Vec3<double> yawAngularVelocityWorld(0.0, 0.0, stateEstimate.torsoAngVel_W.z());
    return stateEstimate.torsoLinVel_W + yawAngularVelocityWorld.cross(offsetWorld);
}
}  // namespace

Mat3<double> MyController::makeDiagonal(const double x, const double y, const double z) {
    Mat3<double> diagonal = Mat3<double>::Zero();
    diagonal.diagonal() << x, y, z;
    return diagonal;
}

void MyController::initializeController() {
    if (_initialized) {
        return;
    }

    initializeRuntimeObjects();
}

void MyController::initializeRuntimeObjects() {
    if (_stateEstimate == nullptr || _robotParams == nullptr || _legController == nullptr) {
        throw std::runtime_error(
            "MyController initialization requires state estimate, robot params, and leg controller");
    }

    if (_robotParams->legs.empty()) {
        throw std::runtime_error("MyController requires at least one configured leg");
    }

    _horizonClock = std::make_unique<HorizonClock>(_stateEstimate->time);
    _gaitScheduler = std::make_unique<GaitScheduler>(_horizonClock.get());
    _controlFSM = std::make_unique<ControlFSM>(
        _gaitScheduler.get(), _horizonClock.get(), _stateEstimate, _robotParams, _userCommand);
    _mpcFormulation = std::make_unique<MPCFormulation>(_robotParams);
    _convexMPC = std::make_unique<ConvexMPC>();

    const auto& config = getControllerConfig();
    _swingKp = makeDiagonal(config.swing.kpDiag[0], config.swing.kpDiag[1], config.swing.kpDiag[2]);
    _swingKd = makeDiagonal(config.swing.kdDiag[0], config.swing.kdDiag[1], config.swing.kdDiag[2]);
    _swingHeight = config.swing.height;
    _iterationsBetweenMpc = static_cast<u64>(std::max(config.mpc.iterationsBetweenSolve, 1));

    _legRuntime.assign(_robotParams->legs.size(), LegRuntimeState{});
    for (std::size_t leg = 0; leg < _robotParams->legs.size(); ++leg) {
        _legRuntime[leg].wasInStance =
            _gaitScheduler->c(_robotParams->legs[leg].side, _stateEstimate->time);
    }

    _stanceWrenchWorld.setZero();
    _iteration = 0;
    _lastMpcIteration = 0;
    _lastControlTime = _stateEstimate->time;
    _initialized = true;
}

int MyController::findLegIndex(const Side side) const {
    if (_robotParams == nullptr) {
        throw std::runtime_error("MyController::findLegIndex requires robot params");
    }

    for (std::size_t leg = 0; leg < _robotParams->legs.size(); ++leg) {
        if (_robotParams->legs[leg].side == side) {
            return static_cast<int>(leg);
        }
    }

    throw std::runtime_error("MyController could not find requested leg side");
}

Vec13<double> MyController::buildCurrentMpcState() const {
    if (_stateEstimate == nullptr) {
        throw std::runtime_error("MyController::buildCurrentMpcState requires state estimate");
    }

    const Vec2<double> rollPitch = quaternionToRollPitch(_stateEstimate->torsoQuat_W);
    const Vec3<double> comWorld = reducedBodyComWorld(*_stateEstimate, *_robotParams);
    const Vec3<double> comVelocityWorld =
        reducedBodyComVelocityWorld(*_stateEstimate, *_robotParams);

    Vec13<double> x0 = Vec13<double>::Zero();
    x0[0] = rollPitch[0];
    x0[1] = rollPitch[1];
    x0[2] = _stateEstimate->psi;
    x0.template segment<3>(3) = comWorld;
    x0.template segment<3>(6) = _stateEstimate->torsoAngVel_W;
    x0.template segment<3>(9) = comVelocityWorld;
    x0[12] = getControllerConfig().model.gravity;
    return x0;
}

void MyController::updateSwingTrajectories(
    const DesiredFootPositions& desiredFootPositions) {
    if (_stateEstimate == nullptr || _gaitScheduler == nullptr || _robotParams == nullptr) {
        throw std::runtime_error(
            "MyController::updateSwingTrajectories requires initialized controller state");
    }

    const double time = _stateEstimate->time;
    const double dt = std::max(0.0, time - _lastControlTime);
    const double minRemainingTime = getControllerConfig().swing.minRemainingTime;

    for (std::size_t leg = 0; leg < _robotParams->legs.size(); ++leg) {
        const Side side = _robotParams->legs[leg].side;
        const bool isStance = _gaitScheduler->c(side, time);
        auto& runtime = _legRuntime[leg];

        if (isStance) {
            runtime.swingTrajectory.deactivate();
            runtime.wasInStance = true;
            continue;
        }

        const Vec3<double>& currentFootPosition = _stateEstimate->legs[leg].footPos_W;
        const Vec3<double> touchdownTarget = desiredFootPositionForSide(desiredFootPositions, side);
        const double timeRemaining =
            std::max(remainingSwingTime(*_gaitScheduler, side, time), minRemainingTime);

        if (runtime.wasInStance || !runtime.swingTrajectory.active()) {
            runtime.swingTrajectory.reset(
                currentFootPosition, touchdownTarget, _swingHeight, timeRemaining);
        } else {
            runtime.swingTrajectory.setFinalPosition(touchdownTarget);
            runtime.swingTrajectory.advance(dt);
        }

        runtime.wasInStance = false;
    }

    _lastControlTime = time;
}

void MyController::maybeUpdateMpc(const Vec13<double>& x0,
                                  const DesiredFootPositions& desiredFootPositions) {
    if (_gaitScheduler == nullptr || _horizonClock == nullptr || _mpcFormulation == nullptr ||
        _convexMPC == nullptr) {
        throw std::runtime_error("MyController::maybeUpdateMpc requires initialized MPC objects");
    }

    const bool shouldUpdate =
        (_iteration == 0) || ((_iteration - _lastMpcIteration) >= _iterationsBetweenMpc);
    if (!shouldUpdate) {
        return;
    }

    try {
        _gaitScheduler->buildConstraintMatrices();

        ReferenceTrajectory(_userCommand, x0, desiredFootPositions, _horizonClock.get())
            .build(_referenceTrajectoryOutput);
        _mpcFormulation->build(_referenceTrajectoryOutput, _mpcFormulationOutput);

        _convexMPC->updateInput(
            *_gaitScheduler, _mpcFormulationOutput, _referenceTrajectoryOutput, x0);
        _convexMPC->solve();
        _stanceWrenchWorld = _convexMPC->optimalWrench();
    } catch (const std::exception&) {
        _stanceWrenchWorld.setZero();

        const double time = _stateEstimate->time;
        int stanceLegCount = 0;
        for (const auto& leg : _robotParams->legs) {
            if (_gaitScheduler->c(leg.side, time)) {
                ++stanceLegCount;
            }
        }

        if (stanceLegCount > 0) {
            const double verticalForce =
                (_robotParams->bodyMass * getControllerConfig().model.gravity) /
                static_cast<double>(stanceLegCount);
            for (const auto& leg : _robotParams->legs) {
                if (!_gaitScheduler->c(leg.side, time)) {
                    continue;
                }

                if (leg.side == Side::Left) {
                    _stanceWrenchWorld[2] = verticalForce;
                } else if (leg.side == Side::Right) {
                    _stanceWrenchWorld[5] = verticalForce;
                }
            }
        }
    }

    _lastMpcIteration = _iteration;
}

void MyController::maybePrintGaitScheduler() const {
    if (_gaitScheduler == nullptr || _stateEstimate == nullptr) {
        return;
    }

    const int interval = getControllerConfig().logging.gaitStatusInterval;
    if (interval <= 0 || (_iteration % static_cast<u64>(interval)) != 0) {
        return;
    }

    const double time = _stateEstimate->time;
    const bool leftStance = _gaitScheduler->c(Side::Left, time);
    const bool rightStance = _gaitScheduler->c(Side::Right, time);

    std::cout << "[GaitScheduler] L: " << (leftStance ? "stance" : "swing")
              << "  R: " << (rightStance ? "stance" : "swing") << std::endl;
}

void MyController::writeLegCommands() {
    if (_legController == nullptr || _gaitScheduler == nullptr || _stateEstimate == nullptr ||
        _robotParams == nullptr) {
        throw std::runtime_error("MyController::writeLegCommands requires initialized pointers");
    }

    const double time = _stateEstimate->time;

    for (std::size_t leg = 0; leg < _robotParams->legs.size(); ++leg) {
        auto& command = _legController->commands[leg];
        const Side side = _robotParams->legs[leg].side;
        const bool isStance = _gaitScheduler->c(side, time);

        if (isStance) {
            command.mode = LegControlMode::StanceWrench;

            switch (side) {
                case Side::Left:
                    command.forceFeedForward_W = _stanceWrenchWorld.template segment<3>(0);
                    command.momentFeedForward_W = _stanceWrenchWorld.template segment<3>(6);
                    break;
                case Side::Right:
                    command.forceFeedForward_W = _stanceWrenchWorld.template segment<3>(3);
                    command.momentFeedForward_W = _stanceWrenchWorld.template segment<3>(9);
                    break;
                default:
                    throw std::runtime_error(
                        "MyController only supports left/right stance wrench mapping");
            }
            continue;
        }

        command.mode = LegControlMode::SwingFoot;
        command.pDes_W = _legRuntime[leg].swingTrajectory.position();
        command.vDes_W = _legRuntime[leg].swingTrajectory.velocity();
        command.aDes_W = _legRuntime[leg].swingTrajectory.acceleration();
        command.kpCartesian = _swingKp;
        command.kdCartesian = _swingKd;
    }
}

void MyController::runController() {
    if (!_initialized) {
        initializeController();
    }

    if (_stateEstimate == nullptr || _horizonClock == nullptr || _controlFSM == nullptr) {
        throw std::runtime_error("MyController::runController requires initialized runtime");
    }

    _horizonClock->sync(_stateEstimate->time);
    maybePrintGaitScheduler();

    const Vec13<double> x0 = buildCurrentMpcState();
    const DesiredFootPositions desiredFootPositions = _controlFSM->SwingFootDesPos();

    updateSwingTrajectories(desiredFootPositions);
    maybeUpdateMpc(x0, desiredFootPositions);
    writeLegCommands();

    ++_iteration;
}
