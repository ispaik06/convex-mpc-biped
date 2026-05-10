#include "My_Controller.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>

#include <Eigen/Geometry>

#include "Controllers/LegController.h"
#include "Dynamics/OperationalSpaceDynamics.h"
#include "BodyMotionReference.h"
#include "StandingMpcDebugLogger.h"
#include "Utilities/Timing.h"
#include "Utilities/MatrixUtils.h"

namespace {
double clampUnit(const double value) {
    return std::clamp(value, -1.0, 1.0);
}

double wrapAngle(const double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

double swingFootYawPsiOffset(const Side side, const double psi_dot) {
    constexpr double kPsiYawGainDegPerRad = 100.0;
    constexpr double kPsiYawMaxOffsetDeg = 20.0;
    constexpr double kDegToRad = 3.141592653589793238462643383279502884 / 180.0;

    const double offsetMagDeg =
        std::clamp(kPsiYawGainDegPerRad * std::abs(psi_dot), 0.0, kPsiYawMaxOffsetDeg);
    // Keep the sign rule explicit: left-foot bias for +psi_dot, right-foot bias for -psi_dot.
    // Backward motion no longer flips the bias sign.
    if (psi_dot > 0.0 && side == Side::Left) {
        return offsetMagDeg * kDegToRad;
    }
    if (psi_dot < 0.0 && side == Side::Right) {
        return -offsetMagDeg * kDegToRad;
    }
    return 0.0;
}

double swingFootYawFromDiagonalStepHeading(const Vec3<double>& currentFootPosition_W,
                                           const Vec3<double>& touchdownTarget_W,
                                           const Vec2<double>& filteredPlanarCommand_B,
                                           const double psi_dot,
                                           const Side side,
                                           const double fallbackYaw_W) {
    constexpr double kDiagonalCommandDeadband = 1e-3;
    constexpr double kFilteredLateralSpeedThreshold = 0.1;
    constexpr double kHalfPi = 1.570796326794896619231321691639751442;
    const double psiBias_W = swingFootYawPsiOffset(side, psi_dot);
    if (std::abs(filteredPlanarCommand_B.y()) < kFilteredLateralSpeedThreshold) {
        return wrapAngle(fallbackYaw_W + psiBias_W);
    }

    if (std::abs(filteredPlanarCommand_B.x()) <= kDiagonalCommandDeadband) {
        return wrapAngle(fallbackYaw_W + psiBias_W);
    }

    const bool sideMatchesLateralDirection =
        (filteredPlanarCommand_B.y() > 0.0 && side == Side::Left) ||
        (filteredPlanarCommand_B.y() < 0.0 && side == Side::Right);
    if (!sideMatchesLateralDirection) {
        return wrapAngle(fallbackYaw_W + psiBias_W);
    }

    const Vec2<double> stepXY_W =
        (touchdownTarget_W - currentFootPosition_W).template head<2>();
    if (stepXY_W.squaredNorm() <= 1e-8) {
        return wrapAngle(fallbackYaw_W + psiBias_W);
    }

    double yaw_W = std::atan2(stepXY_W.y(), stepXY_W.x());
    if (filteredPlanarCommand_B.x() < 0.0) {
        yaw_W += (side == Side::Right) ? kHalfPi : -kHalfPi;
    }
    return wrapAngle(yaw_W + psiBias_W);
}

double lowPassBlendAlpha(const double tau, const double dt) {
    if (!(tau > 0.0) || !(dt > 0.0)) {
        return 1.0;
    }
    return std::clamp(-std::expm1(-dt / tau), 0.0, 1.0);
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

DVec<double> computeSwingAttitudeLevelTorque(const RobotLegState<double>& legState,
                                             const LegControllerData<double>& legData,
                                             const double desiredYaw_W,
                                             const double pitchKp,
                                             const double pitchKd,
                                             const double yawKp,
                                             const double yawKd) {
    const Eigen::Index dof = legData.dof();
    DVec<double> torque = DVec<double>::Zero(dof);
    const bool pitchEnabled = pitchKp > 0.0 || pitchKd > 0.0;
    const bool yawEnabled = yawKp > 0.0 || yawKd > 0.0;
    if (!pitchEnabled && !yawEnabled) {
        return torque;
    }
    if (!legState.hasFootFrame) {
        throw std::runtime_error("Swing attitude control requires foot frame data");
    }
    if (!legData.hasFootData) {
        throw std::runtime_error("Swing attitude control requires foot angular Jacobian data");
    }
    if (legData.Jw_W.rows() != 3 || legData.Jw_W.cols() != dof || legData.qd.size() != dof) {
        throw std::runtime_error("Swing attitude control received inconsistent leg angular data");
    }

    Vec3<double> footX_W = legState.R_WF.col(0);
    Vec3<double> footY_W = legState.R_WF.col(1);
    Vec3<double> footZ_W = legState.R_WF.col(2);
    if (!footX_W.allFinite() || !footY_W.allFinite() || !footZ_W.allFinite() ||
        footX_W.norm() <= 1e-9 || footY_W.norm() <= 1e-9 || footZ_W.norm() <= 1e-9) {
        throw std::runtime_error("Swing attitude control received invalid foot frame axes");
    }
    footX_W.normalize();
    footY_W.normalize();
    footZ_W.normalize();

    const Vec3<double> worldUp = Vec3<double>::UnitZ();
    const Vec3<double> omegaFoot_W = legData.Jw_W * legData.qd;
    Vec3<double> moment_W = Vec3<double>::Zero();

    if (pitchEnabled) {
        const double sinPitchError = footY_W.dot(footZ_W.cross(worldUp));
        const double cosPitchError = footZ_W.dot(worldUp);
        const double pitchError = std::atan2(sinPitchError, cosPitchError);
        const double pitchRate = footY_W.dot(omegaFoot_W);
        moment_W += (pitchKp * pitchError - pitchKd * pitchRate) * footY_W;
    }

    if (yawEnabled) {
        Vec3<double> footXProj_W = footX_W - footX_W.dot(worldUp) * worldUp;
        if (footXProj_W.norm() <= 1e-9) {
            throw std::runtime_error("Swing attitude control received degenerate yaw axis");
        }
        footXProj_W.normalize();
        const Vec3<double> desiredX_W(std::cos(desiredYaw_W), std::sin(desiredYaw_W), 0.0);
        const double sinYawError = worldUp.dot(footXProj_W.cross(desiredX_W));
        const double cosYawError = footXProj_W.dot(desiredX_W);
        const double yawError = std::atan2(sinYawError, cosYawError);
        const double yawRate = worldUp.dot(omegaFoot_W);
        moment_W += (yawKp * yawError - yawKd * yawRate) * worldUp;
    }

    torque = legData.Jw_W.transpose() * moment_W;
    return torque;
}

const char* locomotionModeName(const LocomotionMode mode) {
    switch (mode) {
        case LocomotionMode::Walking:
            return "walking";
        case LocomotionMode::Standing:
            return "standing";
    }

    return "unknown";
}

const char* sideCompactName(const Side side) {
    switch (side) {
        case Side::Left:
            return "L";
        case Side::Right:
            return "R";
        case Side::FL:
            return "FL";
        case Side::FR:
            return "FR";
        case Side::BL:
            return "BL";
        case Side::BR:
            return "BR";
    }
    return "?";
}

double blockNormOrZero(const DMat<double>& matrix,
                       const Eigen::Index row,
                       const Eigen::Index col,
                       const Eigen::Index rows,
                       const Eigen::Index cols) {
    if (matrix.rows() < row + rows || matrix.cols() < col + cols) {
        return 0.0;
    }
    return matrix.block(row, col, rows, cols).norm();
}

double vectorValueOrNan(const DVec<double>& vector, const Eigen::Index index) {
    if (index < 0 || index >= vector.size()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return vector(index);
}

void writeMpcFailureConstraintSummary(const GaitScheduler* gaitScheduler) {
    if (gaitScheduler == nullptr || gaitScheduler->D.size() == 0 ||
        gaitScheduler->C_bound.size() == 0) {
        std::cerr << "[MPC] constraint summary unavailable" << std::endl;
        return;
    }

    const Eigen::Index stepsFromD = gaitScheduler->D.rows() / 12;
    const Eigen::Index stepsFromBounds = gaitScheduler->C_bound.size() / 24;
    const std::size_t stepsToPrint = static_cast<std::size_t>(
        std::min<Eigen::Index>({stepsFromD, stepsFromBounds, horizonSteps(), 4}));
    if (stepsToPrint == 0) {
        std::cerr << "[MPC] constraint summary unavailable; empty constraint matrices"
                  << std::endl;
        return;
    }

    std::ostringstream constraintLine;
    constraintLine << std::fixed << std::setprecision(3)
                   << "[MPC] constraints";
    for (std::size_t k = 0; k < stepsToPrint; ++k) {
        const Eigen::Index dOffset = static_cast<Eigen::Index>(12 * k);
        const Eigen::Index cOffset = static_cast<Eigen::Index>(24 * k);
        const bool leftForceZeroEq =
            blockNormOrZero(gaitScheduler->D, dOffset + 0, dOffset + 0, 3, 3) > 1e-9;
        const bool rightForceZeroEq =
            blockNormOrZero(gaitScheduler->D, dOffset + 3, dOffset + 3, 3, 3) > 1e-9;
        const bool leftMomentEq =
            blockNormOrZero(gaitScheduler->D, dOffset + 6, dOffset + 6, 3, 3) > 1e-9;
        const bool rightMomentEq =
            blockNormOrZero(gaitScheduler->D, dOffset + 9, dOffset + 9, 3, 3) > 1e-9;
        const double leftFzMax = vectorValueOrNan(gaitScheduler->C_bound, cOffset + 4);
        const double leftFzMin = -vectorValueOrNan(gaitScheduler->C_bound, cOffset + 5);
        const double rightFzMax = vectorValueOrNan(gaitScheduler->C_bound, cOffset + 16);
        const double rightFzMin = -vectorValueOrNan(gaitScheduler->C_bound, cOffset + 17);

        constraintLine << " k" << k
                       << "(L_DF=" << static_cast<int>(leftForceZeroEq)
                       << ",L_DM=" << static_cast<int>(leftMomentEq)
                       << ",L_fz=[" << leftFzMin << "," << leftFzMax << "]"
                       << ",R_DF=" << static_cast<int>(rightForceZeroEq)
                       << ",R_DM=" << static_cast<int>(rightMomentEq)
                       << ",R_fz=[" << rightFzMin << "," << rightFzMax << "])";
    }
    std::cerr << constraintLine.str() << std::endl;
}

void writeMpcFailureContactSummary(const ContactManager* contactManager,
                                   const GaitScheduler* gaitScheduler,
                                   const HorizonClock* horizonClock,
                                   const double time,
                                   const ContactScheduleOverride* contactOverride) {
    if (contactManager == nullptr) {
        std::cerr << "[MPC] contact manager unavailable for failure summary" << std::endl;
    } else {
        const vectorAligned<ContactManagerLegState> legStates = contactManager->legStates();
        int activeContactCount = 0;
        std::ostringstream contactLine;
        contactLine << std::fixed << std::setprecision(3)
                    << "[MPC] contact active_count=";
        for (const auto& state : legStates) {
            if (state.activeContact) {
                ++activeContactCount;
            }
        }
        contactLine << activeContactCount;
        for (const auto& state : legStates) {
            contactLine << " | " << sideCompactName(state.side)
                        << " sched=" << static_cast<int>(state.scheduledContact)
                        << " est=" << static_cast<int>(state.estimatedContact)
                        << " active=" << static_cast<int>(state.activeContact)
                        << " early=" << static_cast<int>(state.earlyContact)
                        << " late=" << static_cast<int>(state.lateContact)
                        << " hold=" << static_cast<int>(state.liftoffHold)
                        << " search=" << static_cast<int>(state.searchModeActive)
                        << " fail=" << static_cast<int>(state.recoveryFailure)
                        << " Fn=" << state.contactNormalForce
                        << " alpha=" << state.contactRampAlpha;
        }
        std::cerr << contactLine.str() << std::endl;
    }

    if (gaitScheduler != nullptr && horizonClock != nullptr) {
        std::ostringstream gaitLine;
        gaitLine << std::fixed << std::setprecision(3)
                 << "[MPC] gait elapsed=" << (time - horizonClock->t0())
                 << " t0=" << horizonClock->t0()
                 << " cycle=" << cycleTime()
                 << " stance_fraction=" << (stanceTime() / cycleTime());
        try {
            gaitLine << " pL=" << gaitScheduler->p(Side::Left, time)
                     << " pR=" << gaitScheduler->p(Side::Right, time)
                     << " schedL=" << static_cast<int>(gaitScheduler->c(Side::Left, time))
                     << " schedR=" << static_cast<int>(gaitScheduler->c(Side::Right, time));
        } catch (const std::exception& exception) {
            gaitLine << " phase_unavailable=\"" << exception.what() << "\"";
        }
        std::cerr << gaitLine.str() << std::endl;
    } else {
        std::cerr << "[MPC] gait phase unavailable for failure summary" << std::endl;
    }

    writeMpcFailureConstraintSummary(gaitScheduler);

    if (contactOverride == nullptr || contactOverride->steps.empty()) {
        std::cerr << "[MPC] near horizon contact override unavailable; using nominal gait schedule"
                  << std::endl;
        return;
    }

    std::ostringstream horizonLine;
    horizonLine << std::fixed << std::setprecision(3)
                << "[MPC] near horizon lock_steps=" << contactOverride->steps.size();
    const std::size_t maxStepsToPrint = std::min<std::size_t>(contactOverride->steps.size(), 4);
    for (std::size_t k = 0; k < maxStepsToPrint; ++k) {
        const auto& step = contactOverride->steps[k];
        horizonLine << " k" << k
                    << "(L=" << static_cast<int>(step.leftContact)
                    << ",R=" << static_cast<int>(step.rightContact)
                    << ",minScaleL=" << step.leftNormalForceMinScale
                    << ",minScaleR=" << step.rightNormalForceMinScale << ")";
    }
    std::cerr << horizonLine.str() << std::endl;
}

std::string formatTimeSeconds(const double time) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(2) << time;
    return out.str();
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

Vec2<double> averageFootEndEffectorXY(const StateEstimate<double>& stateEstimate,
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
    return false;
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
    _contactManager = std::make_unique<ContactManager>(getControllerConfig().contactManager);
    _swingFootPlanner = std::make_unique<SwingFootPlanner>(
        _gaitScheduler.get(),
        _horizonClock.get(),
        _stateEstimate,
        _robotParams,
        &_filteredUserCommand);
    _mpcFormulation = std::make_unique<MPCFormulation>(_robotParams);
    _convexMPC = std::make_unique<ConvexMPC>();

    const auto& config = getControllerConfig();
    _locomotionFSM = std::make_unique<LocomotionFSM>(
        config.requestedLocomotionMode,
        config.startup.postInitStandingSettleTime,
        _stateEstimate->time);
    setFootEndEffectorSource(config.model.footEndEffectorSource);
    _swingNaturalFrequency = config.swing.naturalFrequency;
    _swingKd = makeDiagonal(config.swing.kdDiag[0], config.swing.kdDiag[1], config.swing.kdDiag[2]);
    _swingHeight = config.swing.height;
    _iterationsBetweenMpc = static_cast<u64>(std::max(config.mpc.iterationsBetweenSolve, 1));
    const LocomotionFSMOutput locomotionOutput = _locomotionFSM->output();
    _locomotionMode = locomotionOutput.mode;
    _legDynamicsRequest = locomotionOutput.dynamicsRequest;
    _gaitScheduler->setLocomotionMode(_locomotionMode);
    _contactManager->reset(*_stateEstimate, *_robotParams, *_gaitScheduler, _locomotionMode);
    std::cout << "[MyController] locomotion mode initialized to "
              << locomotionModeName(_locomotionMode) << " at t="
              << formatTimeSeconds(_stateEstimate->time) << std::endl;

    updateFilteredUserCommand(0.0);

    _legRuntime.assign(_robotParams->legs.size(), LegRuntimeState{});
    for (std::size_t leg = 0; leg < _robotParams->legs.size(); ++leg) {
        _legRuntime[leg].wasInStance =
            _gaitScheduler->c(_robotParams->legs[leg].side, _stateEstimate->time);
        _legRuntime[leg].wasSearchMode = false;
        _legRuntime[leg].touchdownYaw_W =
            swingFootYawTargetWorld(_robotParams->legs[leg].side);
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

void MyController::prepareController() {
    if (!_initialized) {
        return;
    }

    if (_stateEstimate == nullptr || _locomotionFSM == nullptr) {
        throw std::runtime_error("MyController::prepareController requires initialized runtime");
    }

    syncLocomotionFSM();
}

LegDynamicsRequest MyController::legDynamicsRequest() const {
    if (_initialized) {
        return _legDynamicsRequest;
    }

    const auto& config = getControllerConfig();
    LegDynamicsRequest request;
    if (config.requestedLocomotionMode == LocomotionMode::Walking &&
        config.startup.postInitStandingSettleTime <= 0.0) {
        request.swingLegDynamics = true;
        request.standingFootJacobians = false;
    } else {
        request.swingLegDynamics = false;
        request.standingFootJacobians = true;
    }
    return request;
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

void MyController::seedBodyTargetFromCurrentState() {
    if (_stateEstimate == nullptr || _robotParams == nullptr) {
        throw std::runtime_error("MyController::seedBodyTargetFromCurrentState requires state and params");
    }

    const Vec2<double> rollPitch = quaternionToRollPitch(_stateEstimate->torsoQuat_W);
    _bodyTarget.nominalPosition_W = reducedBodyComWorld(*_stateEstimate, *_robotParams);
    _bodyTarget.position_W = _bodyTarget.nominalPosition_W;
    _bodyTarget.euler_W << rollPitch[0], rollPitch[1], _stateEstimate->psi;
    _bodyTarget.eulerSeed_W = _bodyTarget.euler_W;
    _bodyTarget.initialized = true;
}

void MyController::resetSwingState() {
    if (_stateEstimate == nullptr || _gaitScheduler == nullptr || _robotParams == nullptr) {
        throw std::runtime_error("MyController::resetSwingState requires initialized runtime");
    }

    for (std::size_t leg = 0; leg < _legRuntime.size(); ++leg) {
        _legRuntime[leg].swingTrajectory.deactivate();
        _legRuntime[leg].wasInStance =
            _gaitScheduler->c(_robotParams->legs[leg].side, _stateEstimate->time);
        _legRuntime[leg].wasSearchMode = false;
    }

    if (_swingFootPlanner != nullptr) {
        _swingFootPlanner->reset();
    }
    if (_contactManager != nullptr) {
        _contactManager->reset(*_stateEstimate, *_robotParams, *_gaitScheduler, _locomotionMode);
    }
    _lastControlTime = _stateEstimate->time;
}

void MyController::applyLocomotionOutput(const LocomotionFSMOutput& output) {
    if (_gaitScheduler == nullptr || _horizonClock == nullptr || _stateEstimate == nullptr) {
        throw std::runtime_error("MyController::applyLocomotionOutput requires initialized runtime");
    }

    _locomotionMode = output.mode;
    _legDynamicsRequest = output.dynamicsRequest;
    _gaitScheduler->setLocomotionMode(_locomotionMode);

    if (output.resetGaitClock) {
        _horizonClock->reset(_stateEstimate->time);
    }
    if (output.resetSwingState) {
        resetSwingState();
    }
    if (output.justTransitioned) {
        // seedBodyTargetFromCurrentState();
        std::cout << "[LocomotionFSM] switched to " << locomotionModeName(output.mode)
                  << " at t=" << formatTimeSeconds(_stateEstimate->time) << std::endl;
    }
}

LocomotionFSMOutput MyController::syncLocomotionFSM() {
    if (_locomotionFSM == nullptr || _stateEstimate == nullptr) {
        throw std::runtime_error("MyController::syncLocomotionFSM requires initialized runtime");
    }

    const LocomotionFSMOutput output = _locomotionFSM->update(_stateEstimate->time);
    applyLocomotionOutput(output);
    return output;
}

void MyController::updateFilteredUserCommand(const double dt) {
    const UserCommand rawCommand =
        clampUserCommand((_userCommand != nullptr) ? *_userCommand : UserCommand{});
    if (!_filteredUserCommandInitialized) {
        _filteredUserCommand = rawCommand;
        _filteredUserCommandInitialized = true;
        return;
    }

    const UserCommand previousCommand = _filteredUserCommand;
    _filteredUserCommand = rawCommand;

    const auto& filter = getControllerConfig().userCommandFilter;
    _filteredUserCommand.x_dot = previousCommand.x_dot +
                                 lowPassBlendAlpha(filter.xDotTau, dt) *
                                     (rawCommand.x_dot - previousCommand.x_dot);
    _filteredUserCommand.y_dot = previousCommand.y_dot +
                                 lowPassBlendAlpha(filter.yDotTau, dt) *
                                     (rawCommand.y_dot - previousCommand.y_dot);
    _filteredUserCommand.psi_dot = previousCommand.psi_dot +
                                   lowPassBlendAlpha(filter.psiDotTau, dt) *
                                       (rawCommand.psi_dot - previousCommand.psi_dot);
    _filteredUserCommand.z_dot = previousCommand.z_dot +
                                 lowPassBlendAlpha(filter.zDotTau, dt) *
                                     (rawCommand.z_dot - previousCommand.z_dot);
    _filteredUserCommand.standing_roll_offset_rad =
        previousCommand.standing_roll_offset_rad +
        lowPassBlendAlpha(filter.standingRollOffsetTau, dt) *
            (rawCommand.standing_roll_offset_rad - previousCommand.standing_roll_offset_rad);
    _filteredUserCommand.standing_pitch_offset_rad =
        previousCommand.standing_pitch_offset_rad +
        lowPassBlendAlpha(filter.standingPitchOffsetTau, dt) *
            (rawCommand.standing_pitch_offset_rad - previousCommand.standing_pitch_offset_rad);

    _filteredUserCommand = clampUserCommand(_filteredUserCommand);
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
            _bodyTarget.nominalPosition_W =
                reducedBodyComWorldFromBasePose(initialPose.basePosition_W,
                                                initialPose.baseEuler_W,
                                                *_robotParams);
            _bodyTarget.euler_W = initialPose.baseEuler_W;
        } else {
            _bodyTarget.nominalPosition_W = x0.template segment<3>(3);
            _bodyTarget.euler_W.template segment<3>(0) << 0, 0, x0[2];
        }
        _bodyTarget.eulerSeed_W = _bodyTarget.euler_W;
        _bodyTarget.initialized = true;
    }

    const double z_dot = _filteredUserCommand.z_dot;
    const double standingRollOffset = _filteredUserCommand.standing_roll_offset_rad;
    const double standingPitchOffset = _filteredUserCommand.standing_pitch_offset_rad;
    const auto applyPoseOffsets = [&]() {
        _bodyTarget.euler_W[0] = _bodyTarget.eulerSeed_W[0] + standingRollOffset;
        _bodyTarget.euler_W[1] = _bodyTarget.eulerSeed_W[1] + standingPitchOffset;
        _bodyTarget.nominalPosition_W[2] += z_dot * dt;
        _bodyTarget.position_W = _bodyTarget.nominalPosition_W;
    };

    if (_locomotionMode == LocomotionMode::Standing) {
        const Vec2<double> avgFootXY =
            averageFootEndEffectorXY(*_stateEstimate, *_robotParams);
        _bodyTarget.nominalPosition_W[0] = avgFootXY[0];
        _bodyTarget.nominalPosition_W[1] = avgFootXY[1];
        _bodyTarget.euler_W[2] = BodyMotionReference::advanceYaw(_gaitScheduler.get(),
                                                                 _bodyTarget.euler_W[2],
                                                                 _filteredUserCommand.psi_dot,
                                                                 dt,
                                                                 _stateEstimate->time);
        applyPoseOffsets();
        return;
    }

    const double x_dot = _filteredUserCommand.x_dot;
    const double y_dot = _filteredUserCommand.y_dot;
    const double psi_dot = _filteredUserCommand.psi_dot;

    _bodyTarget.nominalPosition_W =
        BodyMotionReference::advancePlanarPosition(_bodyTarget.nominalPosition_W,
                                                   _bodyTarget.euler_W[2],
                                                   Vec2<double>(x_dot, y_dot),
                                                   dt);
    _bodyTarget.euler_W[2] = BodyMotionReference::advanceYaw(_gaitScheduler.get(),
                                                             _bodyTarget.euler_W[2],
                                                             psi_dot,
                                                             dt,
                                                             _stateEstimate->time);
    applyPoseOffsets();
}

bool MyController::activeContactForSide(const Side side, const double time) const {
    if (_locomotionMode == LocomotionMode::Walking && _contactManager != nullptr) {
        return _contactManager->activeContact(side);
    }
    if (_gaitScheduler == nullptr) {
        throw std::runtime_error("MyController::activeContactForSide requires gait scheduler");
    }
    return _gaitScheduler->c(side, time);
}

double MyController::contactRampAlphaForSide(const Side side) const {
    if (_locomotionMode == LocomotionMode::Walking && _contactManager != nullptr) {
        return _contactManager->contactRampAlpha(side);
    }
    return 1.0;
}

std::vector<ControllerContactDebugLegState> MyController::contactDebugLegStates() const {
    std::vector<ControllerContactDebugLegState> out;
    if (_contactManager == nullptr) {
        return out;
    }

    const vectorAligned<ContactManagerLegState> states = _contactManager->legStates();
    out.reserve(states.size());
    for (const auto& state : states) {
        ControllerContactDebugLegState debugState;
        debugState.side = state.side;
        debugState.scheduledContact = state.scheduledContact;
        debugState.estimatedContact = state.estimatedContact;
        debugState.activeContact = state.activeContact;
        debugState.earlyContact = state.earlyContact;
        debugState.lateContact = state.lateContact;
        debugState.liftoffHold = state.liftoffHold;
        debugState.searchModeActive = state.searchModeActive;
        debugState.recoveryFailure = state.recoveryFailure;
        debugState.contactRampAlpha = state.contactRampAlpha;
        debugState.lateContactTime = state.lateContactTime;
        debugState.liftoffHoldTime = state.liftoffHoldTime;
        out.push_back(debugState);
    }
    return out;
}

ControllerBodyTargetDebugState MyController::bodyTargetDebugState() const {
    ControllerBodyTargetDebugState out;
    out.position_W = _bodyTarget.position_W;
    out.euler_W = _bodyTarget.euler_W;
    out.initialized = _bodyTarget.initialized;
    out.gaitRecoveryHoldActive = false;
    out.gaitRecoveryHoldTime = 0.0;
    return out;
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
        const bool isStance = activeContactForSide(side, time);
        auto& runtime = _legRuntime[leg];

        if (isStance) {
            runtime.swingTrajectory.deactivate();
            runtime.wasInStance = true;
            runtime.wasSearchMode = false;
            continue;
        }

        const Vec3<double>& currentFootPosition = _stateEstimate->legs[leg].footPos_W;
        const Vec3<double> touchdownTarget = desiredFootPositionForSide(desiredFootPositions, side);
        const Vec2<double> filteredPlanarCommand_B(_filteredUserCommand.x_dot,
                                                   _filteredUserCommand.y_dot);
        const double fallbackYaw_W = swingFootYawTargetWorld();
        const double psi_dot = _filteredUserCommand.psi_dot;
        const bool searchMode =
            _contactManager != nullptr && _contactManager->searchModeActive(side);
        if (searchMode) {
            const double searchTrackingTime =
                std::max(getControllerConfig().contactManager.groundSearchTrackingTime,
                         minRemainingTime);
            if (!runtime.wasSearchMode || !runtime.swingTrajectory.active()) {
                runtime.touchdownYaw_W = swingFootYawFromDiagonalStepHeading(
                    currentFootPosition,
                    touchdownTarget,
                    filteredPlanarCommand_B,
                    psi_dot,
                    side,
                    fallbackYaw_W);
                runtime.swingTrajectory.reset(
                    currentFootPosition,
                    touchdownTarget,
                    0.0,
                    searchTrackingTime);
            } else {
                runtime.swingTrajectory.setFinalPosition(touchdownTarget);
                runtime.swingTrajectory.advance(dt);
            }

            runtime.wasInStance = false;
            runtime.wasSearchMode = true;
            continue;
        }

        runtime.wasSearchMode = false;
        const double timeRemaining =
            std::max(remainingSwingTime(*_gaitScheduler, side, time), minRemainingTime);

        if (runtime.wasInStance || !runtime.swingTrajectory.active()) {
            runtime.touchdownYaw_W = swingFootYawFromDiagonalStepHeading(
                currentFootPosition,
                touchdownTarget,
                filteredPlanarCommand_B,
                psi_dot,
                side,
                fallbackYaw_W);
            runtime.swingTrajectory.reset(currentFootPosition,
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

void MyController::updateTouchdownDebugTarget(const DesiredFootPositions& desiredFootPositions) {
    _leftTouchdownTarget_W = desiredFootPositions.left_des_W;
    _rightTouchdownTarget_W = desiredFootPositions.right_des_W;

    const std::size_t leftLegIndex = static_cast<std::size_t>(findLegIndex(Side::Left));
    if (leftLegIndex < _legRuntime.size()) {
        _leftTouchdownTargetYaw_W = _legRuntime[leftLegIndex].touchdownYaw_W;
    } else {
        _leftTouchdownTargetYaw_W = swingFootYawTargetWorld(Side::Left);
    }
    _leftTouchdownTargetInitialized = true;

    const std::size_t rightLegIndex = static_cast<std::size_t>(findLegIndex(Side::Right));
    if (rightLegIndex < _legRuntime.size()) {
        _rightTouchdownTargetYaw_W = _legRuntime[rightLegIndex].touchdownYaw_W;
    } else {
        _rightTouchdownTargetYaw_W = swingFootYawTargetWorld(Side::Right);
    }
    _rightTouchdownTargetInitialized = true;
}

double MyController::swingFootYawTargetWorld() const {
    if (_stateEstimate == nullptr) {
        throw std::runtime_error("MyController::swingFootYawTargetWorld requires state estimate");
    }

    const double psi_dot = _filteredUserCommand.psi_dot;
    const double baseYaw_W = _stateEstimate->psi;
    const double leadScale = getControllerConfig().swing.swingFootYawLeadScale;
    const double previewTime =
        std::max(0.0, (0.5 + getControllerConfig().swing.bodyVelocityHalfStanceOffset) *
                          stanceTime());
    return wrapAngle(baseYaw_W + leadScale * psi_dot * previewTime);
}

double MyController::swingFootYawTargetWorld(const Side side) const {
    return wrapAngle(
        swingFootYawTargetWorld() +
        swingFootYawPsiOffset(side, _filteredUserCommand.psi_dot));
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

    ContactScheduleOverride contactOverride;
    const ContactScheduleOverride* contactOverridePtr = nullptr;
    try {
        {
            profiling::ScopedTimer setupTimer(_mpcSetupTime);

            if (contactWrenchModel(_locomotionMode) == ContactWrenchModel::NoRollMoment) {
                _gaitScheduler->setFootLocalXAxesWorld(
                    footLocalXAxisWorld(*_stateEstimate, *_robotParams, Side::Left),
                    footLocalXAxisWorld(*_stateEstimate, *_robotParams, Side::Right));
            }
            if (_locomotionMode == LocomotionMode::Walking && _contactManager != nullptr) {
                contactOverride = _contactManager->buildHorizonOverride();
                contactOverridePtr = &contactOverride;
            }

            {
                profiling::ScopedTimer timer(_gaitConstraintTime);
                _gaitScheduler->buildConstraintMatrices(contactOverridePtr);
            }

            Vec13<double> referenceSeed = x0;
            referenceSeed.template segment<3>(0) = _bodyTarget.euler_W;
            referenceSeed.template segment<3>(3) = _bodyTarget.position_W;

            UserCommand referenceCommand = _filteredUserCommand;
            {
                profiling::ScopedTimer timer(_referenceTrajectoryTime);
                ReferenceTrajectory(&referenceCommand,
                                    referenceSeed,
                                    desiredFootPositions,
                                    _horizonClock.get(),
                                    _gaitScheduler.get())
                    .build(_referenceTrajectoryOutput);
            }

            {
                profiling::ScopedTimer timer(_mpcFormulationTime);
                _mpcFormulation->build(_referenceTrajectoryOutput, _mpcFormulationOutput);
            }

            {
                profiling::ScopedTimer timer(_mpcUpdateInputTime);
                _convexMPC->updateInput(
                    *_gaitScheduler,
                    _mpcFormulationOutput,
                    _referenceTrajectoryOutput,
                    x0,
                    _locomotionMode);
            }
        }

        {
            profiling::ScopedTimer timer(_mpcSolveTime);
            _convexMPC->solve();
            _stanceWrenchWorld = _convexMPC->optimalWrench();
            if (_standingMpcDebugLogPending) {
                _standingMpcDebugLogReady = true;
            }
        }

    } catch (const std::exception& exception) {
        std::cerr << "[MPC] maybeUpdateMpc failed at iteration " << _iteration
                  << ", t=" << (_stateEstimate != nullptr ? _stateEstimate->time : 0.0)
                  << ": " << exception.what() << std::endl;
        writeMpcFailureContactSummary(_contactManager.get(),
                                      _gaitScheduler.get(),
                                      _horizonClock.get(),
                                      _stateEstimate != nullptr ? _stateEstimate->time : 0.0,
                                      contactOverridePtr);
        _stanceWrenchWorld.setZero();
        _standingMpcDebugLogReady = false;

        const double time = _stateEstimate->time;
        int stanceLegCount = 0;
        for (const auto& leg : _robotParams->legs) {
            if (activeContactForSide(leg.side, time)) {
                ++stanceLegCount;
            }
        }

        if (stanceLegCount > 0) {
            const double verticalForce =
                (_robotParams->bodyMass * std::abs(getControllerConfig().model.gravity)) /
                static_cast<double>(stanceLegCount);
            for (const auto& leg : _robotParams->legs) {
                if (!activeContactForSide(leg.side, time)) {
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

void MyController::printProfilingSummary(std::ostream& out) const {
    if (!profiling::enabled()) {
        return;
    }

    out << profiling::formatTimingStats("runController", _runControllerTime) << '\n';
    out << profiling::formatTimingStats("mpc_setup", _mpcSetupTime) << '\n';
    out << profiling::formatTimingStats("gait_constraints", _gaitConstraintTime) << '\n';
    out << profiling::formatTimingStats("reference_trajectory", _referenceTrajectoryTime) << '\n';
    out << profiling::formatTimingStats("mpc_formulation", _mpcFormulationTime) << '\n';
    out << profiling::formatTimingStats("mpc_update_input", _mpcUpdateInputTime) << '\n';
    out << profiling::formatTimingStats("mpc_solve", _mpcSolveTime) << '\n';
    out << profiling::formatTimingStats("torque_compute", _torqueComputeTime) << '\n';
    if (_convexMPC != nullptr) {
        _convexMPC->printProfilingSummary(out);
    }
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
    if (_stateEstimate == nullptr) {
        return;
    }

    const double time = _stateEstimate->time;
    const unsigned long long request = _filteredUserCommand.standing_mpc_debug_log_request;
    if (request > _lastStandingMpcDebugLogRequest) {
        _lastStandingMpcDebugLogRequest = request;
        queueStandingMpcDebugLog(
            "keyboard",
            time,
            std::numeric_limits<double>::quiet_NaN());
        std::cout << "[MPCDebug] keyboard request #" << request
                  << " (" << locomotionModeName(_locomotionMode) << ")"
                  << " at t=" << time
                  << " queued for the next scheduled MPC solve" << std::endl;
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
    std::cout << "[MPCDebug] time trigger t=" << triggerTime
              << " (" << locomotionModeName(_locomotionMode) << ")"
              << " reached at t=" << time
              << "; queued for the next scheduled MPC solve" << std::endl;
}

void MyController::maybeWriteStandingMpcDebugLog(
    const Vec13<double>& x0,
    const DesiredFootPositions& desiredFootPositions) {
    if (!_standingMpcDebugLogPending ||
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
            _locomotionMode,
            _locomotionFSM != nullptr ? _locomotionFSM->state() : LocomotionState::StandingSettle,
            _horizonClock != nullptr ? _horizonClock->t0() : 0.0,
            _filteredUserCommand,
            _standingMpcDebugRequestSource,
            _standingMpcDebugRequestTime,
            _standingMpcDebugTriggerTime,
            _contactManager != nullptr ? _contactManager->legStates()
                                       : vectorAligned<ContactManagerLegState>{},
        };

        const std::string logPath = writeStandingMpcDebugLog(snapshot);
        std::cout << "[MPCDebug] wrote " << logPath << std::endl;
        _standingMpcDebugLogPending = false;
        _standingMpcDebugLogReady = false;
        _standingMpcDebugRequestSource.clear();
        _standingMpcDebugRequestTime = std::numeric_limits<double>::quiet_NaN();
        _standingMpcDebugTriggerTime = std::numeric_limits<double>::quiet_NaN();
    } catch (const std::exception& exception) {
        std::cerr << "[MPCDebug] failed to write log: "
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

    const auto addTouchdownMarker =
        [&](const char* name,
            const Side side,
            const Vec3<double>& target_W,
            const double targetYaw_W) {
        DebugVizMarker<double> marker;
        marker.name = name;
        marker.position_W = target_W;
        const Vec3<double> yawEuler_W(0.0, 0.0, targetYaw_W);
        marker.orientation_W = rollPitchYawToQuaternion(yawEuler_W);
        TouchdownMarkerContactState markerState = TouchdownMarkerContactState::Swing;
        if (_contactManager != nullptr) {
            const ContactManagerLegState& contactState = _contactManager->legState(side);
            if (contactState.lateContact) {
                markerState = TouchdownMarkerContactState::LateContact;
            } else if (contactState.earlyContact) {
                markerState = TouchdownMarkerContactState::EarlyContact;
            } else if (contactState.activeContact) {
                markerState = TouchdownMarkerContactState::Stance;
            }
        } else if (_gaitScheduler != nullptr && _stateEstimate != nullptr &&
                   activeContactForSide(side, _stateEstimate->time)) {
            markerState = TouchdownMarkerContactState::Stance;
        }
        marker.hasRgba = true;
        marker.rgba = touchdownMarkerRgba<double>(markerState);
        marker.active = true;
        debugViz.markers.push_back(marker);
    };

    if (_leftTouchdownTargetInitialized) {
        addTouchdownMarker("debug_left_touchdown_target",
                           Side::Left,
                           _leftTouchdownTarget_W,
                           _leftTouchdownTargetYaw_W);
    }

    if (_rightTouchdownTargetInitialized) {
        addTouchdownMarker("debug_right_touchdown_target",
                           Side::Right,
                           _rightTouchdownTarget_W,
                           _rightTouchdownTargetYaw_W);
    }
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
        const bool isStance = activeContactForSide(side, time);

        if (isStance) {
            command.mode = LegControlMode::StanceWrench;
            const double contactRampAlpha = contactRampAlphaForSide(side);

            //! MPC solves for the ground reaction on the body; the foot command pushes the ground.
            switch (side) {
                case Side::Left:
                    command.forceFeedForward_W =
                        -contactRampAlpha * _stanceWrenchWorld.template segment<3>(0);
                    command.momentFeedForward_W =
                        -contactRampAlpha * _stanceWrenchWorld.template segment<3>(6);
                    break;
                case Side::Right:
                    command.forceFeedForward_W =
                        -contactRampAlpha * _stanceWrenchWorld.template segment<3>(3);
                    command.momentFeedForward_W =
                        -contactRampAlpha * _stanceWrenchWorld.template segment<3>(9);
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
        command.tauFeedForward += computeSwingAttitudeLevelTorque(
            _stateEstimate->legs[leg],
            _legController->datas[leg],
            _legRuntime[leg].touchdownYaw_W,
            getControllerConfig().swing.pitchKp,
            getControllerConfig().swing.pitchKd,
            getControllerConfig().swing.yawKp,
            getControllerConfig().swing.yawKd);
    }
}

void MyController::runController() {
    if (!_initialized) {
        initializeController();
    }

    profiling::ScopedTimer totalTimer(_runControllerTime);

    if (_stateEstimate == nullptr || _horizonClock == nullptr || _locomotionFSM == nullptr ||
        _swingFootPlanner == nullptr) {
        throw std::runtime_error("MyController::runController requires initialized runtime");
    }

    syncLocomotionFSM();
    _horizonClock->sync(_stateEstimate->time);

    const Vec13<double> x0 = buildCurrentMpcState();
    const double dt = std::max(0.0, _stateEstimate->time - _lastControlTime);
    updateFilteredUserCommand(dt);
    updateBodyTarget(x0, dt);
    if (_bodyTarget.initialized) {
        _swingFootPlanner->setBodyTargetWorld(_bodyTarget.position_W, _bodyTarget.euler_W[2]);
    }
    const auto standingFootTarget = [&](const Side side) {
        Vec3<double> target = _stateEstimate->legs[findLegIndex(side)].footPos_W;
        target.z() = -0.005;
        return target;
    };
    const DesiredFootPositions nominalDesiredFootPositions =
        (_locomotionMode == LocomotionMode::Standing)
            ? DesiredFootPositions{standingFootTarget(Side::Left),
                                   standingFootTarget(Side::Right)}
            : _swingFootPlanner->desiredFootPositions();
    if (_contactManager != nullptr) {
        _contactManager->update(*_stateEstimate,
                                *_robotParams,
                                *_gaitScheduler,
                                _locomotionMode,
                                nominalDesiredFootPositions);
    }
    const DesiredFootPositions desiredFootPositions =
        (_contactManager != nullptr)
            ? _contactManager->managedFootPositions(nominalDesiredFootPositions)
            : nominalDesiredFootPositions;

    updateSwingTrajectories(desiredFootPositions);
    updateTouchdownDebugTarget(desiredFootPositions);
    updateStandingMpcDebugRequest();
    maybeUpdateMpc(x0, desiredFootPositions);
    {
        profiling::ScopedTimer torqueTimer(_torqueComputeTime);
        writeLegCommands();
    }
    maybeWriteStandingMpcDebugLog(x0, desiredFootPositions);

    ++_iteration;
}
