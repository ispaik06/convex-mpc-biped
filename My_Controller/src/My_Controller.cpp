#include "My_Controller.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

#include <Eigen/Geometry>

#include "Controllers/LegController.h"
#include "Dynamics/OperationalSpaceDynamics.h"
#include "StandingMpcDebugLogger.h"
#include "Utilities/MatrixUtils.h"

namespace {
double clampUnit(const double value) {
    return std::clamp(value, -1.0, 1.0);
}

double wrapAngle(const double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
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

Quat<double> rollPitchYawToQuaternion(const Vec3<double>& euler_W) {
    const Quat<double> q_roll(Eigen::AngleAxis<double>(euler_W[0], Eigen::Vector3d::UnitX()));
    const Quat<double> q_pitch(Eigen::AngleAxis<double>(euler_W[1], Eigen::Vector3d::UnitY()));
    const Quat<double> q_yaw(Eigen::AngleAxis<double>(euler_W[2], Eigen::Vector3d::UnitZ()));
    Quat<double> quat = q_yaw * q_pitch * q_roll;
    quat.normalize();
    return quat;
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
    const Vec3<double> bodyBOffset_W = reducedBodyOffsetWorld(stateEstimate, robotParams);
    // Use the full torso angular velocity here so the reduced-body COM velocity stays
    // consistent when the torso has roll/pitch motion. If we want a yaw-only approximation
    // later, that should be an explicit model choice rather than an accidental truncation.
    return stateEstimate.torsoLinVel_W + stateEstimate.torsoAngVel_W.cross(bodyBOffset_W);
}

Quat<double> reducedBodyOrientationWorld(const StateEstimate<double>& stateEstimate) {
    return stateEstimate.torsoQuat_W;
}

Vec2<double> averageFootSiteXY(const StateEstimate<double>& stateEstimate,
                              const RobotParams<double>& robotParams) {
    if (robotParams.legs.empty()) {
        throw std::runtime_error("Standing target seed requires at least one leg");
    }

    Vec2<double> sumXY = Vec2<double>::Zero();
    for (std::size_t leg = 0; leg < robotParams.legs.size(); ++leg) {
        if (leg >= stateEstimate.legs.size()) {
            throw std::runtime_error("Standing target seed requires matching leg state");
        }
        sumXY[0] += stateEstimate.legs[leg].footPos_W.x();
        sumXY[1] += stateEstimate.legs[leg].footPos_W.y();
    }

    return sumXY / static_cast<double>(robotParams.legs.size());
}

Vec3<double> reducedBodyComWorldFromBasePose(const Vec3<double>& basePosition_W,
                                             const Vec3<double>& baseEuler_W,
                                             const RobotParams<double>& robotParams) {
    // Keep the same yaw-aligned COM-offset convention as the online reduced-body model.
    return basePosition_W + Rz(baseEuler_W[2]) * robotParams.bodyComLocation;
}

Vec3<double> footLocalXAxisWorld(const StateEstimate<double>& stateEstimate,
                                 const RobotParams<double>& robotParams,
                                 const Side side) {
    for (std::size_t leg = 0; leg < robotParams.legs.size(); ++leg) {
        if (robotParams.legs[leg].side != side) {
            continue;
        }
        if (leg >= stateEstimate.legs.size() || !stateEstimate.legs[leg].hasFootFrame) {
            throw std::runtime_error("Foot-local contact wrench model requires foot frame data");
        }
        return stateEstimate.legs[leg].R_WF.col(0);
    }

    throw std::runtime_error("Foot-local contact wrench model requires left/right foot axes");
}
}  // namespace

MyController::MyController() = default;

bool MyController::usesStandingOnlyLegDynamics() const {
    return getControllerConfig().locomotionMode == LocomotionMode::Standing;
}

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
    setFootEndEffectorSource(config.swing.footEndEffectorSource);
    _swingNaturalFrequency = config.swing.naturalFrequency;
    _swingKd = makeDiagonal(config.swing.kdDiag[0], config.swing.kdDiag[1], config.swing.kdDiag[2]);
    _swingHeight = config.swing.height;
    _iterationsBetweenMpc = static_cast<u64>(std::max(config.mpc.iterationsBetweenSolve, 1));
    _locomotionMode = config.locomotionMode;
    _gaitScheduler->setLocomotionMode(_locomotionMode);

    _legRuntime.assign(_robotParams->legs.size(), LegRuntimeState{});
    for (std::size_t leg = 0; leg < _robotParams->legs.size(); ++leg) {
        _legRuntime[leg].wasInStance =
            _gaitScheduler->c(_robotParams->legs[leg].side, _stateEstimate->time);
    }

    _stanceWrenchWorld.setZero();
    _iteration = 0;
    _lastMpcIteration = 0;
    _standingMpcDebugLogPending = false;
    _standingMpcDebugLogReady = false;
    _lastStandingMpcDebugLogRequest = 0;
    _nextStandingMpcDebugTriggerIndex = 0;
    _standingMpcDebugRequestSource.clear();
    _standingMpcDebugRequestTime = std::numeric_limits<double>::quiet_NaN();
    _standingMpcDebugTriggerTime = std::numeric_limits<double>::quiet_NaN();
    _lastControlTime = _stateEstimate->time;
    _bodyTarget = BodyTargetState{};
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
    x0.template segment<3>(6) = _stateEstimate->torsoAngVel_W; //? approximate reduced-body as rigid body
    x0.template segment<3>(9) = comVelocityWorld;
    x0[12] = getControllerConfig().model.gravity;
    return x0;
}

void MyController::updateBodyTarget(const Vec13<double>& x0, const double dt) {
    if (_stateEstimate == nullptr) {
        throw std::runtime_error("MyController::updateBodyTarget requires state estimate");
    }

    if (!_bodyTarget.initialized) {
        const auto& initialPose = getControllerConfig().initialPose;
        if (initialPose.hasBasePose) {
            _bodyTarget.position_W = reducedBodyComWorldFromBasePose(initialPose.basePosition_W,
                                                                     initialPose.baseEuler_W,
                                                                     *_robotParams);
            _bodyTarget.euler_W = initialPose.baseEuler_W;
        } else {
            _bodyTarget.position_W = x0.template segment<3>(3);
            _bodyTarget.euler_W.template segment<3>(0) << 0, 0, x0[2];
        }
        _bodyTarget.eulerSeed_W = _bodyTarget.euler_W;
        _bodyTarget.initialized = true;
    }

    if (_locomotionMode == LocomotionMode::Standing) {
        const Vec2<double> avgFootSiteXY = averageFootSiteXY(*_stateEstimate, *_robotParams);
        // Keep the stance footprint centered under the feet and move only in height.
        _bodyTarget.position_W[0] = avgFootSiteXY[0];
        _bodyTarget.position_W[1] = avgFootSiteXY[1];
        const double z_dot = (_userCommand != nullptr) ? _userCommand->z_dot : 0.0;
        const double standingRollOffset =
            (_userCommand != nullptr) ? _userCommand->standing_roll_offset_rad : 0.0;
        const double standingPitchOffset =
            (_userCommand != nullptr) ? _userCommand->standing_pitch_offset_rad : 0.0;
        _bodyTarget.euler_W[0] = _bodyTarget.eulerSeed_W[0] + standingRollOffset;
        _bodyTarget.euler_W[1] = _bodyTarget.eulerSeed_W[1] + standingPitchOffset;
        if (dt > 0.0) {
            _bodyTarget.position_W[2] += z_dot * dt;
        }
        return;
    }

    if (dt <= 0.0) {
        return;
    }

    const double x_dot = (_userCommand != nullptr) ? _userCommand->x_dot : 0.0;
    const double y_dot = (_userCommand != nullptr) ? _userCommand->y_dot : 0.0;
    const double psi_dot = (_userCommand != nullptr) ? _userCommand->psi_dot : 0.0;
    const Vec3<double> v_cmd_B(x_dot, y_dot, 0.0);

    _bodyTarget.position_W += Rz(_bodyTarget.euler_W[2]) * v_cmd_B * dt;
    _bodyTarget.euler_W[2] = wrapAngle(_bodyTarget.euler_W[2] + psi_dot * dt);
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
                currentFootPosition,
                touchdownTarget,
                _swingHeight,
                timeRemaining);
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
        if (getControllerConfig().mpc.contactWrenchModel == ContactWrenchModel::NoRollMoment) {
            _gaitScheduler->setFootLocalXAxesWorld(
                footLocalXAxisWorld(*_stateEstimate, *_robotParams, Side::Left),
                footLocalXAxisWorld(*_stateEstimate, *_robotParams, Side::Right));
        }
        _gaitScheduler->buildConstraintMatrices();

        Vec13<double> referenceSeed = x0;
        referenceSeed.template segment<3>(0) = _bodyTarget.euler_W;
        referenceSeed.template segment<3>(3) = _bodyTarget.position_W;

        UserCommand referenceCommand{};
        if (_userCommand != nullptr) {
            if (_locomotionMode == LocomotionMode::Standing) {
                referenceCommand.z_dot = _userCommand->z_dot;
            } else {
                referenceCommand = *_userCommand;
                referenceCommand.z_dot = 0.0;
            }
        }

        ReferenceTrajectory(&referenceCommand, referenceSeed, desiredFootPositions, _horizonClock.get())
            .build(_referenceTrajectoryOutput);
        _mpcFormulation->build(_referenceTrajectoryOutput, _mpcFormulationOutput);

        _convexMPC->updateInput(
            *_gaitScheduler, _mpcFormulationOutput, _referenceTrajectoryOutput, x0);
        _convexMPC->solve();
        _stanceWrenchWorld = _convexMPC->optimalWrench();
        if (_locomotionMode == LocomotionMode::Standing && _standingMpcDebugLogPending) {
            _standingMpcDebugLogReady = true;
        }

    } catch (const std::exception&) {
        _stanceWrenchWorld.setZero();
        _standingMpcDebugLogReady = false;

        const double time = _stateEstimate->time;
        int stanceLegCount = 0;
        for (const auto& leg : _robotParams->legs) {
            if (_gaitScheduler->c(leg.side, time)) {
                ++stanceLegCount;
            }
        }

        if (stanceLegCount > 0) {
            const double verticalForce =
                (_robotParams->bodyMass * std::abs(getControllerConfig().model.gravity)) /
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

void MyController::queueStandingMpcDebugLog(const std::string& source,
                                            const double requestTime,
                                            const double triggerTime) {
    _standingMpcDebugLogPending = true;
    _standingMpcDebugLogReady = false;
    _standingMpcDebugRequestSource = source;
    _standingMpcDebugRequestTime = requestTime;
    _standingMpcDebugTriggerTime = triggerTime;
}

void MyController::updateStandingMpcDebugRequest() {
    if (_locomotionMode != LocomotionMode::Standing || _stateEstimate == nullptr) {
        return;
    }

    const double time = _stateEstimate->time;
    if (_userCommand != nullptr) {
        const unsigned long long request = _userCommand->standing_mpc_debug_log_request;
        if (request > _lastStandingMpcDebugLogRequest) {
            _lastStandingMpcDebugLogRequest = request;
            queueStandingMpcDebugLog(
                "keyboard",
                time,
                std::numeric_limits<double>::quiet_NaN());
            std::cout << "[StandingMPCDebug] keyboard request #" << request
                      << " at t=" << time
                      << " queued for the next scheduled MPC solve" << std::endl;
        }
    }

    if (_standingMpcDebugLogPending) {
        return;
    }

    const auto& triggerTimes = getControllerConfig().logging.standingMpcDebugTriggerTimes;
    if (_nextStandingMpcDebugTriggerIndex >= triggerTimes.size()) {
        return;
    }

    const double triggerTime = triggerTimes[_nextStandingMpcDebugTriggerIndex];
    if (time < triggerTime) {
        return;
    }

    ++_nextStandingMpcDebugTriggerIndex;
    queueStandingMpcDebugLog("time", time, triggerTime);
    std::cout << "[StandingMPCDebug] time trigger t=" << triggerTime
              << " reached at t=" << time
              << "; queued for the next scheduled MPC solve" << std::endl;
}

void MyController::maybeWriteStandingMpcDebugLog(
    const Vec13<double>& x0,
    const DesiredFootPositions& desiredFootPositions) {
    if (_locomotionMode != LocomotionMode::Standing ||
        !_standingMpcDebugLogPending ||
        !_standingMpcDebugLogReady ||
        _convexMPC == nullptr ||
        !_convexMPC->hasSolution() ||
        _stateEstimate == nullptr ||
        _robotParams == nullptr ||
        _legController == nullptr) {
        return;
    }

    try {
        const StandingMpcDebugSnapshot snapshot{
            *_stateEstimate,
            *_robotParams,
            *_legController,
            _armController,
            desiredFootPositions,
            x0,
            _referenceTrajectoryOutput,
            _mpcFormulationOutput,
            _convexMPC->optimalWrenchHorizon(),
            _iteration,
            _standingMpcDebugRequestSource,
            _standingMpcDebugRequestTime,
            _standingMpcDebugTriggerTime,
        };

        const std::string logPath = writeStandingMpcDebugLog(snapshot);
        std::cout << "[StandingMPCDebug] wrote " << logPath << std::endl;
        _standingMpcDebugLogPending = false;
        _standingMpcDebugLogReady = false;
        _standingMpcDebugRequestSource.clear();
        _standingMpcDebugRequestTime = std::numeric_limits<double>::quiet_NaN();
        _standingMpcDebugTriggerTime = std::numeric_limits<double>::quiet_NaN();
    } catch (const std::exception& exception) {
        std::cerr << "[StandingMPCDebug] failed to write log: "
                  << exception.what() << std::endl;
        _standingMpcDebugLogPending = false;
        _standingMpcDebugLogReady = false;
        _standingMpcDebugRequestSource.clear();
        _standingMpcDebugRequestTime = std::numeric_limits<double>::quiet_NaN();
        _standingMpcDebugTriggerTime = std::numeric_limits<double>::quiet_NaN();
    }
}

void MyController::collectDebugVisualization(DebugVizState<double>& debugViz) const {
    if (_stateEstimate != nullptr && _robotParams != nullptr) {
        DebugVizMarker<double> marker;
        marker.name = "debug_reduced_body_com";
        marker.position_W = reducedBodyComWorld(*_stateEstimate, *_robotParams);
        marker.orientation_W = reducedBodyOrientationWorld(*_stateEstimate);
        marker.active = true;
        debugViz.markers.push_back(marker);
    }

    if (_bodyTarget.initialized) {
        DebugVizMarker<double> marker;
        marker.name = "debug_body_target";
        marker.position_W = _bodyTarget.position_W;
        marker.orientation_W = rollPitchYawToQuaternion(_bodyTarget.euler_W);
        marker.active = true;
        debugViz.markers.push_back(marker);
    }
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

void MyController::writeStandingLegCommands() {
    if (_legController == nullptr || _stateEstimate == nullptr || _robotParams == nullptr) {
        throw std::runtime_error("MyController::writeStandingLegCommands requires initialized pointers");
    }

    const auto& standingFeet = _stateEstimate->standingFeet;
    if (!standingFeet.hasFootJacobians) {
        throw std::runtime_error("Standing mode requires combined two-foot Jacobians");
    }

    Eigen::Index totalLegDof = 0;
    for (std::size_t leg = 0; leg < _robotParams->legs.size(); ++leg) {
        totalLegDof += _legController->datas[leg].dof();
    }

    if (standingFeet.Jv_W.rows() != 6 || standingFeet.Jw_W.rows() != 6 ||
        standingFeet.Jv_W.cols() != totalLegDof || standingFeet.Jw_W.cols() != totalLegDof) {
        throw std::runtime_error("Standing combined Jacobian dimension does not match leg dofs");
    }

    DVec<double> footForces_W(6);
    DVec<double> footMoments_W(6);
    footForces_W.template segment<3>(0) = -_stanceWrenchWorld.template segment<3>(0);
    footForces_W.template segment<3>(3) = -_stanceWrenchWorld.template segment<3>(3);
    footMoments_W.template segment<3>(0) = -_stanceWrenchWorld.template segment<3>(6);
    footMoments_W.template segment<3>(3) = -_stanceWrenchWorld.template segment<3>(9);

    const DVec<double> combinedLegTorque =
        standingFeet.Jv_W.transpose() * footForces_W +
        standingFeet.Jw_W.transpose() * footMoments_W;

    Eigen::Index offset = 0;
    for (std::size_t leg = 0; leg < _robotParams->legs.size(); ++leg) {
        auto& command = _legController->commands[leg];
        const Eigen::Index dof = _legController->datas[leg].dof();
        command.mode = LegControlMode::JointTorque;
        command.tauFeedForward = combinedLegTorque.segment(offset, dof);
        command.forceFeedForward_W.setZero();
        command.momentFeedForward_W.setZero();
        offset += dof;
    }
}

void MyController::writeLegCommands() {
    if (_legController == nullptr || _gaitScheduler == nullptr || _stateEstimate == nullptr ||
        _robotParams == nullptr) {
        throw std::runtime_error("MyController::writeLegCommands requires initialized pointers");
    }

    if (_locomotionMode == LocomotionMode::Standing) {
        writeStandingLegCommands();
        return;
    }

    const double time = _stateEstimate->time;

    for (std::size_t leg = 0; leg < _robotParams->legs.size(); ++leg) {
        auto& command = _legController->commands[leg];
        const Side side = _robotParams->legs[leg].side;
        const bool isStance = _gaitScheduler->c(side, time);

        if (isStance) {
            command.mode = LegControlMode::StanceWrench;

            //! MPC solves for the ground reaction on the body; the foot command pushes the ground.
            switch (side) {
                case Side::Left:
                    command.forceFeedForward_W = -_stanceWrenchWorld.template segment<3>(0);
                    command.momentFeedForward_W = -_stanceWrenchWorld.template segment<3>(6);
                    break;
                case Side::Right:
                    command.forceFeedForward_W = -_stanceWrenchWorld.template segment<3>(3);
                    command.momentFeedForward_W = -_stanceWrenchWorld.template segment<3>(9);
                    break;
                default:
                    throw std::runtime_error(
                        "MyController only supports left/right stance wrench mapping");
            }
            continue;
        }

        command.mode = LegControlMode::SwingFoot;
        command.kpCartesian = computeSwingCartesianKp(
            _legController->datas[leg].Jv_W,
            _legController->datas[leg].massMatrix,
            _swingNaturalFrequency);
        command.pDes_W = _legRuntime[leg].swingTrajectory.position();
        command.vDes_W = _legRuntime[leg].swingTrajectory.velocity();
        command.aDes_W = _legRuntime[leg].swingTrajectory.acceleration();
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
    // maybePrintGaitScheduler();

    const Vec13<double> x0 = buildCurrentMpcState();
    const double dt = std::max(0.0, _stateEstimate->time - _lastControlTime);
    updateBodyTarget(x0, dt);
    const auto standingFootTarget = [&](const Side side) {
        Vec3<double> target = _stateEstimate->legs[findLegIndex(side)].footPos_W;
        target.z() = -0.005;
        return target;
    };
    const DesiredFootPositions desiredFootPositions =
        (_locomotionMode == LocomotionMode::Standing)
            ? DesiredFootPositions{standingFootTarget(Side::Left),
                                   standingFootTarget(Side::Right)}
            : _controlFSM->SwingFootDesPos();

    updateSwingTrajectories(desiredFootPositions);
    updateStandingMpcDebugRequest();
    maybeUpdateMpc(x0, desiredFootPositions);
    writeLegCommands();
    maybeWriteStandingMpcDebugLog(x0, desiredFootPositions);

    ++_iteration;
}
