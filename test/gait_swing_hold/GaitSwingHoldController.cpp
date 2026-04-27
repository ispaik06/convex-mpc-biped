#include "GaitSwingHoldController.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <sstream>

#include <Eigen/Geometry>

#include "Controllers/LegController.h"
#include "Dynamics/OperationalSpaceDynamics.h"
#include "Utilities/MatrixUtils.h"

namespace {
constexpr double kSwingFootTargetZ = -0.005;

std::string formatTimeSeconds(const double time) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(2) << time;
    return out.str();
}

const char* sideName(const Side side) {
    switch (side) {
        case Side::Left:
            return "left";
        case Side::Right:
            return "right";
        default:
            return "unknown";
    }
}

double remainingSwingTime(const GaitScheduler& gaitScheduler, const Side side, const double time) {
    return std::clamp(cycleTime() * (1.0 - gaitScheduler.p(side, time)), 0.0, swingTime());
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

Quat<double> yawQuaternion(const double yaw) {
    return Quat<double>(Eigen::AngleAxis<double>(yaw, Eigen::Vector3d::UnitZ()));
}

double swingFootYawTargetWorld(const StateEstimate<double>* stateEstimate,
                               const UserCommand* userCommand) {
    if (stateEstimate == nullptr) {
        throw std::runtime_error("Swing yaw target requires state estimate");
    }

    const double psi_dot = (userCommand != nullptr) ? userCommand->psi_dot : 0.0;
    return std::atan2(std::sin(stateEstimate->psi + psi_dot * stanceTime() * 0.5),
                      std::cos(stateEstimate->psi + psi_dot * stanceTime() * 0.5));
}
}  // namespace

GaitSwingHoldController::GaitSwingHoldController() {
    setFootEndEffectorSource(getControllerConfig().model.footEndEffectorSource);
}

void GaitSwingHoldController::bindRuntime(const StateEstimate<double>* stateEstimate,
                                          const RobotParams<double>* robotParams,
                                          LegController<double>* legController,
                                          ArmController<double>* armController,
                                          const UserCommand* userCommand) {
    _stateEstimate = stateEstimate;
    _robotParams = robotParams;
    _legController = legController;
    _armController = armController;
    _userCommand = userCommand;
}

void GaitSwingHoldController::setTouchdownTargetZOffset(const double offset) {
    if (!std::isfinite(offset)) {
        throw std::invalid_argument("GaitSwingHoldController touchdown z offset must be finite");
    }
    _touchdownTargetZOffset = offset;
}

Mat3<double> GaitSwingHoldController::makeDiagonal(const double x,
                                                   const double y,
                                                   const double z) {
    Mat3<double> diagonal = Mat3<double>::Zero();
    diagonal.diagonal() << x, y, z;
    return diagonal;
}

JointPdGains<double> GaitSwingHoldController::makeInitialJointGains(const RobotType robotType,
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

void GaitSwingHoldController::initializeController() {
    if (_initialized) {
        return;
    }

    initializeRuntime();
}

void GaitSwingHoldController::initializeRuntime() {
    if (_legController == nullptr || _stateEstimate == nullptr || _robotParams == nullptr) {
        throw std::runtime_error(
            "GaitSwingHoldController initialization requires leg controller, state estimate, and robot params");
    }

    _leftLegIndex = findLegIndex(Side::Left);
    _rightLegIndex = findLegIndex(Side::Right);

    const auto& config = getControllerConfig();
    _swingDuration = swingTime();
    _swingHeight = config.swing.height;
    _swingNaturalFrequency = config.swing.naturalFrequency;
    _swingKd = makeDiagonal(config.swing.kdDiag[0], config.swing.kdDiag[1], config.swing.kdDiag[2]);
    _touchdownTargetMode = config.gaitSwingHoldTest.touchdownTargetMode;
    _horizonClock = std::make_unique<HorizonClock>(_stateEstimate->time);
    _gaitScheduler = std::make_unique<GaitScheduler>(_horizonClock.get());
    _gaitScheduler->setLocomotionMode(LocomotionMode::Walking);

    const Eigen::Index legDof =
        _stateEstimate->legs[static_cast<std::size_t>(_leftLegIndex)].q.size();
    _jointHoldGains = makeInitialJointGains(_robotParams->roboType, legDof);

    _legRuntime.resize(_robotParams->legs.size());
    for (std::size_t leg = 0; leg < _robotParams->legs.size(); ++leg) {
        _legRuntime[leg].holdQ = _stateEstimate->legs[leg].q;
        _legRuntime[leg].swingTrajectory.deactivate();
        _legRuntime[leg].wasInStance = true;
        _legRuntime[leg].touchdownYaw_W = swingFootYawTargetWorld(_stateEstimate, _userCommand);
    }

    _tracePath = std::string(PROJECT_ROOT_DIR) + "/build/gait_swing_hold_trace.csv";
    _traceSegmentId = 0;
    openSwingTrace();

    _lastTime = _stateEstimate->time;
    _initialized = true;
    _touchdownTarget_W = touchdownTargetWorld(static_cast<std::size_t>(_leftLegIndex));

    std::cout << "[GaitSwingHoldTest] gait-scheduler swing hold initialized from copied_state at t="
              << formatTimeSeconds(_stateEstimate->time) << std::endl;
}

void GaitSwingHoldController::openSwingTrace() {
    if (_traceStream.is_open()) {
        _traceStream.close();
    }

    std::filesystem::path tracePath(_tracePath);
    if (tracePath.has_parent_path()) {
        std::error_code ec;
        std::filesystem::create_directories(tracePath.parent_path(), ec);
    }

    _traceStream.open(_tracePath, std::ios::out | std::ios::trunc);
    if (!_traceStream.is_open()) {
        throw std::runtime_error("Failed to open swing trace CSV at " + _tracePath);
    }

    _traceStream << "segment,time,phase,desired_x,desired_y,desired_z,actual_x,actual_y,actual_z\n";
    _traceStream.flush();
}

int GaitSwingHoldController::findLegIndex(const Side side) const {
    if (_robotParams == nullptr) {
        throw std::runtime_error("GaitSwingHoldController requires robot params");
    }

    for (std::size_t leg = 0; leg < _robotParams->legs.size(); ++leg) {
        if (_robotParams->legs[leg].side == side) {
            return static_cast<int>(leg);
        }
    }

    throw std::runtime_error("Requested leg side is missing from RobotParams");
}

const char* GaitSwingHoldController::sideName(const Side side) {
    return ::sideName(side);
}

double GaitSwingHoldController::touchdownTargetZ() const {
    return kSwingFootTargetZ + _touchdownTargetZOffset;
}

Vec3<double> GaitSwingHoldController::touchdownTargetWorld(const std::size_t legIndex) const {
    if (_stateEstimate == nullptr || _robotParams == nullptr ||
        legIndex >= _robotParams->legs.size()) {
        throw std::runtime_error("GaitSwingHoldController requires state to compute touchdown target");
    }

    const auto& legParams = _robotParams->legs[legIndex];
    const auto& footPlacement = getControllerConfig().footPlacement;
    const double psi = _stateEstimate->psi;
    const Mat3<double> R_WB = Rz(psi);
    const Mat3<double> R_BW = R_WB.transpose();
    const Vec3<double> bodyBOffset_W = R_WB * _robotParams->bodyComLocation;
    const Vec3<double> p_com_W = _stateEstimate->torsoPos_W + bodyBOffset_W;
    const Vec3<double> v_com_W =
        _stateEstimate->torsoLinVel_W + _stateEstimate->torsoAngVel_W.cross(bodyBOffset_W);
    const Vec3<double> u_com_B = R_BW * v_com_W;
    const Vec3<double> u_des_B = Vec3<double>{
        _userCommand != nullptr ? _userCommand->x_dot : 0.0,
        _userCommand != nullptr ? _userCommand->y_dot : 0.0,
        0.0};
    const double psi_dot = (_userCommand != nullptr) ? _userCommand->psi_dot : 0.0;
    const double time = _stateEstimate->time;
    const double swingPhase = std::clamp(
        (_gaitScheduler != nullptr) ? _gaitScheduler->p(legParams.side, time) : 0.0,
        0.0,
        1.0);
    const double T_rem = std::max(cycleTime() * (1.0 - swingPhase), 0.0);

    switch (_touchdownTargetMode) {
        case TouchdownTargetMode::BodyVelocityHalfStance: {
            const auto& footNow = _stateEstimate->legs[legIndex].footPos_W;
            const double stanceFraction = 0.5 + getControllerConfig().swing.bodyVelocityHalfStanceOffset;
            Vec3<double> target = footNow + R_WB * u_des_B * (stanceFraction * stanceTime());
            target.z() = touchdownTargetZ();
            return target;
        }
        case TouchdownTargetMode::LegacyComYawCorrected: {
            Vec3<double> p_nom_B(-0.002851214,  0.072812741, -0.752981881);
            p_nom_B[1] += (_robotParams->legs[legIndex].side == Side::Left ? 1.0 : -1.0) *
                           footPlacement.nominalLateralOffset;

            const double yaw_correction = psi_dot * stanceTime() / 2.0;
            const Mat3<double> R_yaw_correction = Rz(yaw_correction);

            double delta_x =
                (0.5 + footPlacement.swingBias) * u_com_B[0] * stanceTime();
                // + footPlacement.velocityFeedbackGain * (u_com_B[0] - u_des_B[0])
                // + (0.5 * z_com / std::abs(model.gravity)) * (u_com_B[1] * psi_dot);

            double delta_y =
                0.5 * u_com_B[1] * stanceTime();
                // + footPlacement.velocityFeedbackGain * (u_com_B[1] - u_des_B[1])
                // + (0.5 * z_com / std::abs(model.gravity)) * (-u_com_B[0] * psi_dot);

            delta_x = std::clamp(delta_x, -footPlacement.placementClamp, footPlacement.placementClamp);
            delta_y = std::clamp(delta_y, -footPlacement.placementClamp, footPlacement.placementClamp);

            const Vec3<double> feedback_B(delta_x, delta_y, 0.0);
            Vec3<double> target =
                p_com_W + R_WB * (R_yaw_correction * p_nom_B + u_des_B * T_rem + feedback_B);
            target.z() = touchdownTargetZ();
            return target;
        }
    }

    throw std::runtime_error("Unsupported touchdown_target_mode in GaitSwingHoldController");
}

void GaitSwingHoldController::configureJointHold(const int legIndex, const DVec<double>& qHold) {
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

void GaitSwingHoldController::configureSwingLeg(const int legIndex) {
    auto& command = _legController->commands[static_cast<std::size_t>(legIndex)];
    const auto& legData = _legController->datas[static_cast<std::size_t>(legIndex)];
    const auto& runtime = _legRuntime[static_cast<std::size_t>(legIndex)];

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
    command.pDes_W = runtime.swingTrajectory.position();
    command.vDes_W = runtime.swingTrajectory.velocity();
    command.aDes_W = runtime.swingTrajectory.acceleration();
    command.kdCartesian = _swingKd;
    command.tauFeedForward += computeSwingAttitudeLevelTorque(
        _stateEstimate->legs[static_cast<std::size_t>(legIndex)],
        legData,
        runtime.touchdownYaw_W,
        getControllerConfig().swing.pitchKp,
        getControllerConfig().swing.pitchKd,
        getControllerConfig().swing.yawKp,
        getControllerConfig().swing.yawKd);
}

void GaitSwingHoldController::maybePrintStatus() const {
    if ((_iteration % 5000) != 0 || _stateEstimate == nullptr || _robotParams == nullptr ||
        _gaitScheduler == nullptr) {
        return;
    }

    std::cout << "[GaitSwingHoldTest] t=" << formatTimeSeconds(_stateEstimate->time);
    for (std::size_t leg = 0; leg < _robotParams->legs.size(); ++leg) {
        const auto& legParams = _robotParams->legs[leg];
        const bool isStance = _gaitScheduler->c(legParams.side, _stateEstimate->time);
        std::cout << "  " << sideName(legParams.side) << '='
                  << (isStance ? "stance" : "swing");
        if (leg == static_cast<std::size_t>(_leftLegIndex)) {
            std::cout << " touchdown=" << _touchdownTarget_W.transpose();
        }
    }
    std::cout << std::endl;
}

void GaitSwingHoldController::logSwingTraceSample(const int legIndex) const {
    if (_traceStream.is_open() == false || _stateEstimate == nullptr ||
        _leftLegIndex < 0 || legIndex != _leftLegIndex) {
        return;
    }

    const auto& runtime = _legRuntime[static_cast<std::size_t>(legIndex)];
    if (!runtime.swingTrajectory.active()) {
        return;
    }

    const auto& actual = _stateEstimate->legs[static_cast<std::size_t>(legIndex)].footEndPos_W;
    const auto desired = runtime.swingTrajectory.position();
    _traceStream << std::fixed << std::setprecision(9)
                 << _traceSegmentId << ","
                 << _stateEstimate->time << ",swing,"
                 << desired.x() << "," << desired.y() << "," << desired.z() << ","
                 << actual.x() << "," << actual.y() << "," << actual.z() << "\n";
    _traceStream.flush();
}

void GaitSwingHoldController::logHoldTraceMarker(const int legIndex) const {
    if (_traceStream.is_open() == false || _stateEstimate == nullptr ||
        _leftLegIndex < 0 || legIndex != _leftLegIndex) {
        return;
    }

    const auto& actual = _stateEstimate->legs[static_cast<std::size_t>(legIndex)].footEndPos_W;
    _traceStream << std::fixed << std::setprecision(9)
                 << _traceSegmentId << ","
                 << _stateEstimate->time << ",hold,"
                 << actual.x() << "," << actual.y() << "," << actual.z() << ","
                 << actual.x() << "," << actual.y() << "," << actual.z() << "\n";
    _traceStream.flush();
}

void GaitSwingHoldController::runController() {
    if (!_initialized) {
        initializeRuntime();
    }

    if (_legController == nullptr || _stateEstimate == nullptr) {
        throw std::runtime_error("GaitSwingHoldController requires initialized runtime");
    }

    const double time = _stateEstimate->time;
    const double dt = std::max(0.0, time - _lastTime);

    if (_horizonClock != nullptr) {
        _horizonClock->sync(time);
    }

    for (std::size_t leg = 0; leg < _robotParams->legs.size(); ++leg) {
        const auto& legParams = _robotParams->legs[leg];
        auto& runtime = _legRuntime[leg];
        const bool isStance = (_gaitScheduler != nullptr) ? _gaitScheduler->c(legParams.side, time)
                                                          : true;

        if (isStance) {
            if (!runtime.wasInStance) {
                runtime.holdQ = _stateEstimate->legs[leg].q;
                logHoldTraceMarker(static_cast<int>(leg));
            }
            runtime.swingTrajectory.deactivate();
            configureJointHold(static_cast<int>(leg), runtime.holdQ);
            runtime.wasInStance = true;
            continue;
        }

        const double timeRemaining =
            std::max((_gaitScheduler != nullptr)
                         ? remainingSwingTime(*_gaitScheduler, legParams.side, time)
                         : _swingDuration,
                     getControllerConfig().swing.minRemainingTime);

        if (runtime.wasInStance || !runtime.swingTrajectory.active()) {
            if (leg == static_cast<std::size_t>(_leftLegIndex)) {
                ++_traceSegmentId;
            }
            runtime.touchdownYaw_W = swingFootYawTargetWorld(_stateEstimate, _userCommand);
            runtime.touchdownTarget_W = touchdownTargetWorld(leg);
            if (leg == static_cast<std::size_t>(_leftLegIndex)) {
                _touchdownTarget_W = runtime.touchdownTarget_W;
            }
            runtime.swingTrajectory.reset(
                _stateEstimate->legs[leg].footPos_W,
                runtime.touchdownTarget_W,
                _swingHeight,
                timeRemaining);
        } else {
            if (_touchdownTargetMode == TouchdownTargetMode::LegacyComYawCorrected) {
                runtime.touchdownTarget_W = touchdownTargetWorld(leg);
                if (leg == static_cast<std::size_t>(_leftLegIndex)) {
                    _touchdownTarget_W = runtime.touchdownTarget_W;
                }
                runtime.swingTrajectory.setFinalPosition(runtime.touchdownTarget_W);
            }
            runtime.swingTrajectory.advance(dt);
        }

        configureSwingLeg(static_cast<int>(leg));
        logSwingTraceSample(static_cast<int>(leg));
        runtime.wasInStance = false;
    }

    maybePrintStatus();

    _lastTime = time;
    ++_iteration;
}

void GaitSwingHoldController::collectDebugVisualization(DebugVizState<double>& debugViz) const {
    if (!_initialized || _leftLegIndex < 0) {
        return;
    }

    DebugVizMarker<double> marker;
    marker.name = "debug_left_touchdown_target";
    marker.position_W = _touchdownTarget_W;
    if (static_cast<std::size_t>(_leftLegIndex) < _legRuntime.size()) {
        marker.orientation_W = yawQuaternion(_legRuntime[static_cast<std::size_t>(_leftLegIndex)].touchdownYaw_W);
    } else {
        marker.orientation_W = Quat<double>::Identity();
    }
    marker.active = true;
    debugViz.markers.push_back(marker);
}
