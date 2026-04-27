#include "LeftSwingHoldController.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <sstream>

#include "Controllers/LegController.h"
#include "Dynamics/OperationalSpaceDynamics.h"
#include "Utilities/MatrixUtils.h"

namespace {
std::string formatTimeSeconds(const double time) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(2) << time;
    return out.str();
}

DVec<double> computeSwingPitchLevelTorque(const RobotLegState<double>& legState,
                                          const LegControllerData<double>& legData,
                                          const double pitchKp,
                                          const double pitchKd) {
    const Eigen::Index dof = legData.dof();
    DVec<double> torque = DVec<double>::Zero(dof);
    if (pitchKp <= 0.0 && pitchKd <= 0.0) {
        return torque;
    }
    if (!legState.hasFootFrame) {
        throw std::runtime_error("Swing pitch leveling requires foot frame data");
    }
    if (!legData.hasFootData) {
        throw std::runtime_error("Swing pitch leveling requires foot angular Jacobian data");
    }
    if (legData.Jw_W.rows() != 3 || legData.Jw_W.cols() != dof || legData.qd.size() != dof) {
        throw std::runtime_error("Swing pitch leveling received inconsistent leg angular data");
    }

    Vec3<double> footY_W = legState.R_WF.col(1);
    Vec3<double> footZ_W = legState.R_WF.col(2);
    if (!footY_W.allFinite() || !footZ_W.allFinite() ||
        footY_W.norm() <= 1e-9 || footZ_W.norm() <= 1e-9) {
        throw std::runtime_error("Swing pitch leveling received invalid foot frame axes");
    }
    footY_W.normalize();
    footZ_W.normalize();

    const Vec3<double> worldUp = Vec3<double>::UnitZ();
    const double sinPitchError = footY_W.dot(footZ_W.cross(worldUp));
    const double cosPitchError = footZ_W.dot(worldUp);
    const double pitchError = std::atan2(sinPitchError, cosPitchError);
    const Vec3<double> omegaFoot_W = legData.Jw_W * legData.qd;
    const double pitchRate = footY_W.dot(omegaFoot_W);
    const Vec3<double> pitchMoment_W = (pitchKp * pitchError - pitchKd * pitchRate) * footY_W;
    torque = legData.Jw_W.transpose() * pitchMoment_W;
    return torque;
}
}  // namespace

LeftSwingHoldController::LeftSwingHoldController() {
    setFootEndEffectorSource(getControllerConfig().model.footEndEffectorSource);
}

void LeftSwingHoldController::bindRuntime(const StateEstimate<double>* stateEstimate,
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

    _jointHoldGains = makeInitialJointGains(_robotParams->roboType, _leftHoldQ.size());

    const auto& config = getControllerConfig();
    _swingDuration = swingTime();
    _holdDuration = stanceTime();
    _swingHeight = config.swing.height;
    _swingNaturalFrequency = config.swing.naturalFrequency;
    _swingKd = makeDiagonal(config.swing.kdDiag[0], config.swing.kdDiag[1], config.swing.kdDiag[2]);
    _touchdownTargetMode = config.leftSwingHoldTest.touchdownTargetMode;
    _tracePath = std::string(PROJECT_ROOT_DIR) + "/build/left_swing_hold_trace.csv";
    _traceSegmentId = 0;
    openSwingTrace();

    _phaseElapsed = 0.0;
    _lastTime = _stateEstimate->time;
    _initialized = true;
    startSwingPhase();

    std::cout << "[LeftSwingHoldTest] left swing started from copied_state at t="
              << formatTimeSeconds(_stateEstimate->time) << std::endl;
}

void LeftSwingHoldController::openSwingTrace() {
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

const char* LeftSwingHoldController::phaseName() const {
    switch (_phase) {
        case Phase::Swing:
            return "swing";
        case Phase::Hold:
            return "hold";
    }

    return "unknown";
}

Vec3<double> LeftSwingHoldController::touchdownTargetWorld() const {
    if (_stateEstimate == nullptr || _leftLegIndex < 0 || _robotParams == nullptr) {
        throw std::runtime_error("LeftSwingHoldController requires state to compute touchdown target");
    }

    const std::size_t legIndex = static_cast<std::size_t>(_leftLegIndex);
    const auto& footPlacement = getControllerConfig().footPlacement;
    const auto& model = getControllerConfig().model;
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
    const double z_com = p_com_W[2];
    const double swingPhase =
        (_swingDuration > 0.0) ? std::clamp(_phaseElapsed / _swingDuration, 0.0, 1.0) : 0.0;
    const double T_rem = std::max(_swingDuration * (1.0 - swingPhase), 0.0);

    switch (_touchdownTargetMode) {
        case TouchdownTargetMode::BodyVelocityHalfStance: {
            const auto& footNow = _stateEstimate->legs[legIndex].footPos_W;
            const double stanceFraction = 0.5 + getControllerConfig().swing.bodyVelocityHalfStanceOffset;
            Vec3<double> target = footNow + R_WB * u_des_B * (stanceFraction * stanceTime());
            target.z() = _stateEstimate->legs[legIndex].footPos_W.z();
            return target;
        }
        case TouchdownTargetMode::LegacyComYawCorrected: {
            const Vec3<double> hipWorld =
                _stateEstimate->torsoPos_W +
                _stateEstimate->torsoQuat_W.toRotationMatrix() *
                    _robotParams->legs[legIndex].hipLocationFromBody;
            Vec3<double> p_nom_B(-0.002851214,  0.072812741, -0.752981881);
            // p_nom_B[1] += (_robotParams->legs[legIndex].side == Side::Left ? 1.0 : -1.0) *
            //               footPlacement.nominalLateralOffset;

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

            const Vec3<double> feedback_B(delta_x, delta_y, footPlacement.touchdownHeight);
            Vec3<double> target =
                p_com_W + R_WB * (R_yaw_correction * p_nom_B + u_des_B * T_rem + feedback_B);
            return target;
        }
    }

    throw std::runtime_error("Unsupported touchdown_target_mode in LeftSwingHoldController");
}

void LeftSwingHoldController::startSwingPhase() {
    if (_stateEstimate == nullptr) {
        throw std::runtime_error("LeftSwingHoldController requires state estimate to start swing");
    }

    _phase = Phase::Swing;
    _phaseElapsed = 0.0;
    ++_traceSegmentId;
    _touchdownTarget_W = touchdownTargetWorld();
    _leftSwingTrajectory.reset(
        _stateEstimate->legs[static_cast<std::size_t>(_leftLegIndex)].footPos_W,
        _touchdownTarget_W,
        _swingHeight,
        _swingDuration);
}

void LeftSwingHoldController::startHoldPhase() {
    _phase = Phase::Hold;
    _phaseElapsed = 0.0;
    _leftSwingTrajectory.deactivate();
    logHoldTraceMarker();
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
    command.tauFeedForward += computeSwingPitchLevelTorque(
        _stateEstimate->legs[static_cast<std::size_t>(legIndex)],
        legData,
        getControllerConfig().swing.pitchKp,
        getControllerConfig().swing.pitchKd);
}

void LeftSwingHoldController::maybePrintStatus() const {
    if ((_iteration % 5000) != 0 || _stateEstimate == nullptr) {
        return;
    }

    const auto& leftFoot = _stateEstimate->legs[static_cast<std::size_t>(_leftLegIndex)].footPos_W;
    std::cout << "[LeftSwingHoldTest] phase="
              << phaseName()
              << ' ';
    std::cout << "left_foot_des=" << _leftSwingTrajectory.position().transpose();
    std::cout
              << " left_foot_act=" << leftFoot.transpose()
              << " left_ee_act="
              << _stateEstimate->legs[static_cast<std::size_t>(_leftLegIndex)].footEndPos_W.transpose()
              << std::endl;
}

void LeftSwingHoldController::logSwingTraceSample() const {
    if (_traceStream.is_open() == false || _stateEstimate == nullptr || _leftLegIndex < 0) {
        return;
    }

    if (_phase != Phase::Swing) {
        return;
    }

    const auto& actual = _stateEstimate->legs[static_cast<std::size_t>(_leftLegIndex)].footEndPos_W;
    const auto desired = _leftSwingTrajectory.position();
    _traceStream << std::fixed << std::setprecision(9)
                 << _traceSegmentId << ","
                 << _stateEstimate->time << ",swing,"
                 << desired.x() << "," << desired.y() << "," << desired.z() << ","
                 << actual.x() << "," << actual.y() << "," << actual.z() << "\n";
    _traceStream.flush();
}

void LeftSwingHoldController::logHoldTraceMarker() const {
    if (_traceStream.is_open() == false || _stateEstimate == nullptr || _leftLegIndex < 0) {
        return;
    }

    const auto& actual = _stateEstimate->legs[static_cast<std::size_t>(_leftLegIndex)].footEndPos_W;
    _traceStream << std::fixed << std::setprecision(9)
                 << _traceSegmentId << ","
                 << _stateEstimate->time << ",hold,"
                 << actual.x() << "," << actual.y() << "," << actual.z() << ","
                 << actual.x() << "," << actual.y() << "," << actual.z() << "\n";
    _traceStream.flush();
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

    if (_phase == Phase::Swing) {
        if (_touchdownTargetMode == TouchdownTargetMode::LegacyComYawCorrected) {
            // Legacy mode replans online; body_velocity_half_stance stays fixed from swing start.
            _touchdownTarget_W = touchdownTargetWorld();
            _leftSwingTrajectory.setFinalPosition(_touchdownTarget_W);
        }
        _leftSwingTrajectory.advance(dt);
        _phaseElapsed += dt;
        if (_phaseElapsed >= _swingDuration || !_leftSwingTrajectory.active()) {
            startHoldPhase();
        }
    } else if (_phase == Phase::Hold) {
        _phaseElapsed += dt;
        if (_phaseElapsed >= _holdDuration) {
            startSwingPhase();
        }
    }

    if (_phase == Phase::Swing) {
        configureSwingLeg(_leftLegIndex);
        logSwingTraceSample();
    } else {
        configureJointHold(_leftLegIndex, _leftHoldQ);
    }

    maybePrintStatus();

    _lastTime = time;
    ++_iteration;
}

void LeftSwingHoldController::collectDebugVisualization(DebugVizState<double>& debugViz) const {
    if (!_initialized || _leftLegIndex < 0) {
        return;
    }

    DebugVizMarker<double> marker;
    marker.name = "debug_left_touchdown_target";
    marker.position_W = _touchdownTarget_W;
    marker.orientation_W = Quat<double>::Identity();
    marker.active = true;
    debugViz.markers.push_back(marker);
}
