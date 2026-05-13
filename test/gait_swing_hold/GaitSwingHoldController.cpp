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
#include "Dynamics/SwingAttitudeControl.h"
#include "Dynamics/SwingYawTarget.h"
#include "JointTrackingConfig.h"
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

Quat<double> yawQuaternion(const double yaw) {
    return Quat<double>(Eigen::AngleAxis<double>(yaw, Eigen::Vector3d::UnitZ()));
}

double swingFootYawTargetWorld(const StateEstimate<double>* stateEstimate,
                               const UserCommand* userCommand) {
    if (stateEstimate == nullptr) {
        throw std::runtime_error("Swing yaw target requires state estimate");
    }

    const UserCommand clampedCommand =
        clampUserCommand((userCommand != nullptr) ? *userCommand : UserCommand{});
    const double psi_dot = clampedCommand.psi_dot;
    const double leadScale = getControllerConfig().swing.swingFootYawLeadScale;
    const double previewTime =
        (0.5 + getControllerConfig().swing.bodyVelocityHalfStanceOffset) * stanceTime();
    const double yaw = stateEstimate->psi + leadScale * psi_dot * previewTime;
    return std::atan2(std::sin(yaw), std::cos(yaw));
}

double swingFootYawTargetWorld(const StateEstimate<double>* stateEstimate,
                               const UserCommand* userCommand,
                               const Side side) {
    const UserCommand clampedCommand =
        clampUserCommand((userCommand != nullptr) ? *userCommand : UserCommand{});
    const double baseYaw_W = swingFootYawTargetWorld(stateEstimate, userCommand);
    return std::atan2(std::sin(baseYaw_W +
                               swingyaw::swingFootYawPsiOffset(side, clampedCommand.psi_dot)),
                      std::cos(baseYaw_W +
                               swingyaw::swingFootYawPsiOffset(side, clampedCommand.psi_dot)));
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
    const auto& jointTracking = getJointTrackingConfig(robotType);
    if (jointTracking.legKp.empty() || jointTracking.legKd.empty()) {
        throw std::runtime_error("joint_tracking.leg gains are missing for gait_swing_hold");
    }

    if (static_cast<Eigen::Index>(jointTracking.legKp.size()) != dof) {
        throw std::runtime_error("joint_tracking.leg gains do not match leg dof in gait_swing_hold");
    }

    JointPdGains<double> gains;
    gains.set(jointTracking.legKp, jointTracking.legKd);
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
        _legRuntime[leg].touchdownYaw_W =
            swingFootYawTargetWorld(_stateEstimate, _userCommand, _robotParams->legs[leg].side);
    }

    _tracePath = std::string(PROJECT_ROOT_DIR) + "/build/gait_swing_hold_trace.csv";
    _traceSegmentId = 0;
    openSwingTrace();

    _lastTime = _stateEstimate->time;
    _initialized = true;
    _touchdownTarget_W = touchdownTargetWorld(static_cast<std::size_t>(_leftLegIndex));

    std::cout << "[GaitSwingHoldTest] gait-scheduler swing hold initialized from "
              << getControllerConfig().gaitSwingHoldTest.keyframeName << " at t="
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

    const double psi = _stateEstimate->psi;
    const Mat3<double> R_WB = Rz(psi);
    const UserCommand clampedCommand =
        clampUserCommand((_userCommand != nullptr) ? *_userCommand : UserCommand{});
    const Vec3<double> u_des_B = Vec3<double>{clampedCommand.x_dot, clampedCommand.y_dot, 0.0};
    const auto& footNow = _stateEstimate->legs[legIndex].footPos_W;
    const double stanceFraction = 0.5 + getControllerConfig().swing.bodyVelocityHalfStanceOffset;
    Vec3<double> target = footNow + R_WB * u_des_B * (stanceFraction * stanceTime());
    target.z() = touchdownTargetZ();
    return target;
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
        getControllerConfig().swing.rollKp,
        getControllerConfig().swing.rollKd,
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

        const UserCommand clampedCommand =
            clampUserCommand((_userCommand != nullptr) ? *_userCommand : UserCommand{});
        const Vec2<double> filteredPlanarCommand_B(clampedCommand.x_dot, clampedCommand.y_dot);
        const double fallbackYaw_W = swingFootYawTargetWorld(_stateEstimate, _userCommand);
        const double psi_dot = clampedCommand.psi_dot;

        if (runtime.wasInStance || !runtime.swingTrajectory.active()) {
            if (leg == static_cast<std::size_t>(_leftLegIndex)) {
                ++_traceSegmentId;
            }
            runtime.touchdownYaw_W = swingyaw::swingFootYawFromDiagonalStepHeading(
                _stateEstimate->legs[leg].footPos_W,
                touchdownTargetWorld(leg),
                filteredPlanarCommand_B,
                psi_dot,
                legParams.side,
                fallbackYaw_W);
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
    if (!_initialized || _leftLegIndex < 0 || _rightLegIndex < 0) {
        return;
    }

    const auto addTouchdownMarker = [&](const char* name, const Side side, const int legIndex) {
        const std::size_t leg = static_cast<std::size_t>(legIndex);
        if (leg >= _legRuntime.size()) {
            return;
        }

        DebugVizMarker<double> marker;
        marker.name = name;
        marker.position_W = _legRuntime[leg].touchdownTarget_W;
        marker.orientation_W = yawQuaternion(_legRuntime[leg].touchdownYaw_W);
        const bool isStance =
            _gaitScheduler != nullptr && _stateEstimate != nullptr &&
            _gaitScheduler->c(side, _stateEstimate->time);
        marker.hasRgba = true;
        marker.rgba = touchdownMarkerRgba<double>(isStance);
        marker.active = true;
        debugViz.markers.push_back(marker);
    };

    addTouchdownMarker("debug_left_touchdown_target", Side::Left, _leftLegIndex);
    addTouchdownMarker("debug_right_touchdown_target", Side::Right, _rightLegIndex);
}
