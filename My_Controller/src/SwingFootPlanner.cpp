#include "SwingFootPlanner.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "Utilities/MatrixUtils.h"

namespace {
constexpr double kSwingFootTargetZ = -0.005;

Vec3<double> reducedBodyOffsetWorld(const StateEstimate<double>& stateEstimate,
                                    const RobotParams<double>& robotParams) {
    return Rz(stateEstimate.yaw_W_unwrapped) * robotParams.bodyComLocation;
}

Vec3<double> reducedBodyComWorld(const StateEstimate<double>& stateEstimate,
                                 const RobotParams<double>& robotParams) {
    return stateEstimate.torsoPos_W + reducedBodyOffsetWorld(stateEstimate, robotParams);
}

Vec3<double> reducedBodyComVelocityWorld(const StateEstimate<double>& stateEstimate,
                                         const RobotParams<double>& robotParams) {
    const Vec3<double> bodyOffset_W = reducedBodyOffsetWorld(stateEstimate, robotParams);
    return stateEstimate.torsoLinVel_W + stateEstimate.torsoAngVel_W.cross(bodyOffset_W);
}
}  // namespace

void SwingFootPlanner::reset() {
    _touchdownTargets.clear();
    _touchdownTargetValid.clear();
    _nominalFootOffsets_B.clear();
    _nominalFootOffsetValid.clear();
    _wasInStance.clear();
    _bodyYawTargetValid = false;
    _stopRecenterClearTicks = 0;
    _stopRecenterWasActive = false;
    _previousPlanarCommand_B.setZero();
    _previousYawRateCommand = 0.0;
    _previousCommandValid = false;
    _turnStopCenter_W.setZero();
    _turnStopYaw_W = 0.0;
    _turnStopFrameValid = false;
}

void SwingFootPlanner::seedTouchdownTargets(const DesiredFootPositions& desiredFootPositions) {
    if (_gaitScheduler == nullptr || _stateEstimate == nullptr || _robotParams == nullptr) {
        throw std::runtime_error(
            "SwingFootPlanner::seedTouchdownTargets requires initialized pointers");
    }

    ensureSwingTouchdownCache();
    for (std::size_t leg = 0; leg < _robotParams->legs.size(); ++leg) {
        switch (_robotParams->legs[leg].side) {
            case Side::Left:
                _touchdownTargets[leg] = desiredFootPositions.left_des_W;
                break;
            case Side::Right:
                _touchdownTargets[leg] = desiredFootPositions.right_des_W;
                break;
            default:
                throw std::runtime_error(
                    "SwingFootPlanner::seedTouchdownTargets only supports left/right legs");
        }
        _touchdownTargetValid[leg] = true;
        _wasInStance[leg] = _gaitScheduler->c(_robotParams->legs[leg].side, _stateEstimate->time);
    }
}

void SwingFootPlanner::setBodyYawTargetWorld(const double yaw_W) {
    if (!std::isfinite(yaw_W)) {
        _bodyYawTargetValid = false;
        return;
    }

    _bodyYawTarget_W = yaw_W;
    _bodyYawTargetValid = true;
}

void SwingFootPlanner::syncHorizonClock() {
    if (_horizonClock == nullptr || _stateEstimate == nullptr) {
        throw std::runtime_error("SwingFootPlanner::syncHorizonClock requires initialized pointers");
    }

    _horizonClock->sync(_stateEstimate->time);
}

void SwingFootPlanner::ensureSwingTouchdownCache() {
    if (_robotParams == nullptr) {
        throw std::runtime_error("SwingFootPlanner::ensureSwingTouchdownCache requires robot params");
    }

    const std::size_t legCount = _robotParams->legs.size();
    if (_touchdownTargets.size() == legCount &&
        _touchdownTargetValid.size() == legCount &&
        _nominalFootOffsets_B.size() == legCount &&
        _nominalFootOffsetValid.size() == legCount &&
        _wasInStance.size() == legCount) {
        return;
    }

    _touchdownTargets.assign(legCount, Vec3<double>::Zero());
    _touchdownTargetValid.assign(legCount, false);
    _nominalFootOffsets_B.assign(legCount, Vec3<double>::Zero());
    _nominalFootOffsetValid.assign(legCount, false);
    _wasInStance.assign(legCount, true);
}

void SwingFootPlanner::ensureNominalFootOffsets() {
    if (_stateEstimate == nullptr || _robotParams == nullptr) {
        throw std::runtime_error("SwingFootPlanner::ensureNominalFootOffsets requires initialized pointers");
    }

    bool allValid = _nominalFootOffsetValid.size() == _robotParams->legs.size();
    for (bool valid : _nominalFootOffsetValid) {
        allValid = allValid && valid;
    }
    if (allValid) {
        return;
    }

    Vec3<double> footCenter_W = Vec3<double>::Zero();
    for (std::size_t leg = 0; leg < _robotParams->legs.size(); ++leg) {
        footCenter_W += _stateEstimate->legs[leg].footPos_W;
    }
    footCenter_W /= static_cast<double>(_robotParams->legs.size());
    const auto& swing = getControllerConfig().swing;
    if (!swing.nominalFootOffsets_B.empty()) {
        if (swing.nominalFootOffsets_B.size() != _robotParams->legs.size()) {
            throw std::runtime_error(
                "swing.nominal_foot_offsets_B must contain one 3-vector per leg");
        }

        _nominalFootOffsets_B = swing.nominalFootOffsets_B;
        std::fill(_nominalFootOffsetValid.begin(), _nominalFootOffsetValid.end(), true);
        return;
    }

    const Mat3<double> R_BW = Rz(bodyYawTargetWorld()).transpose();
    for (std::size_t leg = 0; leg < _robotParams->legs.size(); ++leg) {
        Vec3<double> offset_B = R_BW * (_stateEstimate->legs[leg].footPos_W - footCenter_W);
        offset_B.x() = 0.0;
        offset_B.z() = 0.0;
        _nominalFootOffsets_B[leg] = offset_B;
        _nominalFootOffsetValid[leg] = true;
    }
}

Vec3<double> SwingFootPlanner::currentFootTouchdownTarget(const std::size_t legIndex) const {
    return _stateEstimate->legs[legIndex].footPos_W;
}

Vec2<double> SwingFootPlanner::currentPlanarCommandBodyFrame() const {
    return Vec2<double>(_userCommand != nullptr ? _userCommand->x_dot : 0.0,
                        _userCommand != nullptr ? _userCommand->y_dot : 0.0);
}

double SwingFootPlanner::selectedBodyVelocityHalfStanceOffset(
    const Vec2<double>& currentPlanarCommand) const {
    const auto& swing = getControllerConfig().swing;
    const double speed = currentPlanarCommand.norm();
    if (speed > swing.highSpeedBodyVelocityHalfStanceOffsetSwitchSpeed) {
        return swing.highSpeedBodyVelocityHalfStanceOffset;
    }
    if (speed > swing.bodyVelocityHalfStanceOffsetSwitchSpeed) {
        return swing.midSpeedBodyVelocityHalfStanceOffset;
    }
    return swing.bodyVelocityHalfStanceOffset;
}

double SwingFootPlanner::bodyYawTargetWorld() const {
    return _bodyYawTargetValid ? _bodyYawTarget_W : _stateEstimate->yaw_W_unwrapped;
}

double SwingFootPlanner::touchdownPreviewTime(const Vec2<double>& currentPlanarCommand) const {
    const double stanceFraction = 0.5 + selectedBodyVelocityHalfStanceOffset(currentPlanarCommand);
    return std::max(0.0, stanceFraction * stanceTime());
}

bool SwingFootPlanner::stopRecenterRequested(const Vec2<double>& currentPlanarCommand) const {
    const auto& swing = getControllerConfig().swing;
    const double psi_dot = (_userCommand != nullptr) ? _userCommand->psi_dot : 0.0;
    return currentPlanarCommand.norm() <= swing.stopVelocityDeadband &&
           std::abs(psi_dot) <= swing.stopVelocityDeadband;
}

bool SwingFootPlanner::stopRecenterActive(const Vec2<double>& currentPlanarCommand) {
    const auto& swing = getControllerConfig().swing;
    if (stopRecenterRequested(currentPlanarCommand)) {
        _stopRecenterClearTicks = 0;
        return true;
    }

    if (!_stopRecenterWasActive) {
        _stopRecenterClearTicks = 0;
        return false;
    }

    if (_stopRecenterClearTicks < swing.stopBrakingLatchClearTicks) {
        ++_stopRecenterClearTicks;
        return true;
    }

    return false;
}

Vec3<double> SwingFootPlanner::computeStopStanceCenterWorld() const {
    const auto& swing = getControllerConfig().swing;
    const double yaw_W = _turnStopFrameValid ? _turnStopYaw_W : bodyYawTargetWorld();
    Vec3<double> stoppingOffset_B = Vec3<double>::Zero();
    if (swing.hasStopBrakingOffset) {
        stoppingOffset_B = swing.stopBrakingOffset_B;
    } else {
        const Vec3<double> velocity_B =
            Rz(yaw_W).transpose() * reducedBodyComVelocityWorld(*_stateEstimate, *_robotParams);
        stoppingOffset_B.template head<2>() =
            swing.stopCapturePointGain * velocity_B.template head<2>();

        const double planarNorm = stoppingOffset_B.template head<2>().norm();
        if (swing.stopCapturePointMaxOffset > 0.0 &&
            planarNorm > swing.stopCapturePointMaxOffset) {
            stoppingOffset_B.template head<2>() *=
                swing.stopCapturePointMaxOffset / planarNorm;
        }
    }

    Vec3<double> center_W = _turnStopFrameValid
                                ? _turnStopCenter_W
                                : reducedBodyComWorld(*_stateEstimate, *_robotParams) +
                                      Rz(yaw_W) * stoppingOffset_B;
    center_W.z() = kSwingFootTargetZ;
    return center_W;
}

Vec3<double> SwingFootPlanner::touchdownTargetWorldBodyVelocityHalfStance(
    const std::size_t legIndex,
    const Vec2<double>& currentPlanarCommand,
    const bool stopRecenter) const {
    const Vec3<double> previewVelocity_B(currentPlanarCommand.x(),
                                         currentPlanarCommand.y(),
                                         0.0);
    const double previewTime = touchdownPreviewTime(currentPlanarCommand);
    const double psi_dot = (_userCommand != nullptr) ? _userCommand->psi_dot : 0.0;
    const double yaw0 = _turnStopFrameValid ? _turnStopYaw_W : bodyYawTargetWorld();
    const double translationYaw_W = yaw0 + 0.5 * psi_dot * previewTime;
    // Use the remaining swing time to estimate the yaw at the eventual touchdown instant.
    const double remainingSwingTime =
        std::clamp(cycleTime() * (1.0 - _gaitScheduler->p(_robotParams->legs[legIndex].side,
                                                          _stateEstimate->time)),
                   0.0,
                   swingTime());
    const double touchdownYaw_W = yaw0 + psi_dot * remainingSwingTime;
    const Vec3<double> step_W = Rz(translationYaw_W) * previewVelocity_B * previewTime;
    const Vec3<double> currentCenter_W =
        stopRecenter ? computeStopStanceCenterWorld()
                     : reducedBodyComWorld(*_stateEstimate, *_robotParams);
    const Vec3<double>& touchdownOffset_B = _nominalFootOffsets_B[legIndex];
    Vec3<double> plannedOffset_B =
        Rz(yaw0).transpose() * step_W +
        Rz(touchdownYaw_W - yaw0) * touchdownOffset_B;
    if (stopRecenter) {
        // Both feet use a shared state-derived stop center rather than preserving the
        // preceding stride in the sequential footprint.
        plannedOffset_B = Rz(touchdownYaw_W - yaw0) * touchdownOffset_B;
    }
    const double nominalLateralOffset = touchdownOffset_B.y();
    if (nominalLateralOffset > 0.0) {
        plannedOffset_B.y() = std::max(plannedOffset_B.y(), 0.0);
    } else if (nominalLateralOffset < 0.0) {
        plannedOffset_B.y() = std::min(plannedOffset_B.y(), 0.0);
    }
    Vec3<double> target = currentCenter_W + Rz(yaw0) * plannedOffset_B;
    target.z() = kSwingFootTargetZ;
    return target;
}

DesiredFootPositions SwingFootPlanner::desiredFootPositions() {
    if (_gaitScheduler == nullptr || _horizonClock == nullptr || _stateEstimate == nullptr ||
        _robotParams == nullptr) {
        throw std::runtime_error("SwingFootPlanner::desiredFootPositions requires initialized pointers");
    }

    syncHorizonClock();
    ensureSwingTouchdownCache();
    ensureNominalFootOffsets();

    const Vec2<double> currentPlanarCommand = currentPlanarCommandBodyFrame();
    const double currentYawRateCommand =
        (_userCommand != nullptr) ? _userCommand->psi_dot : 0.0;
    const bool shouldStopRecenter = stopRecenterActive(currentPlanarCommand);
    const bool stopRecenterJustActivated =
        shouldStopRecenter && !_stopRecenterWasActive;
    if (stopRecenterJustActivated && _previousCommandValid) {
        double nominalFootRadius = 0.0;
        for (const Vec3<double>& offset_B : _nominalFootOffsets_B) {
            nominalFootRadius = std::max(nominalFootRadius, offset_B.template head<2>().norm());
        }
        const auto& swing = getControllerConfig().swing;
        const double previousTurnTangentialSpeed =
            std::abs(_previousYawRateCommand) * nominalFootRadius;
        const bool wasTurnDominant =
            std::abs(_previousYawRateCommand) > swing.stopVelocityDeadband &&
            _previousPlanarCommand_B.norm() <=
                previousTurnTangentialSpeed + swing.stopVelocityDeadband;
        if (wasTurnDominant) {
            _turnStopCenter_W = reducedBodyComWorld(*_stateEstimate, *_robotParams);
            _turnStopCenter_W.z() = kSwingFootTargetZ;
            _turnStopYaw_W = _stateEstimate->yaw_W_unwrapped;
            _turnStopFrameValid = true;
        }
    } else if (!shouldStopRecenter) {
        _turnStopCenter_W.setZero();
        _turnStopYaw_W = 0.0;
        _turnStopFrameValid = false;
    }
    if (shouldStopRecenter && _turnStopFrameValid) {
        _turnStopCenter_W = reducedBodyComWorld(*_stateEstimate, *_robotParams);
        _turnStopCenter_W.z() = kSwingFootTargetZ;
        _turnStopYaw_W = _stateEstimate->yaw_W_unwrapped;
    }
    const double time = _stateEstimate->time;
    auto computeDesiredFootPos = [&](Side side) -> Vec3<double> {
        int legIndex = -1;
        for (std::size_t i = 0; i < _robotParams->legs.size(); ++i) {
            if (_robotParams->legs[i].side == side) {
                legIndex = static_cast<int>(i);
                break;
            }
        }
        if (legIndex < 0) {
            throw std::runtime_error("SwingFootPlanner could not find requested leg");
        }

        const bool isStance = _gaitScheduler->c(side, time);
        const std::size_t leg = static_cast<std::size_t>(legIndex);
        if (isStance) {
            if (!_touchdownTargetValid[leg]) {
                _touchdownTargets[leg] = currentFootTouchdownTarget(leg);
                _touchdownTargetValid[leg] = true;
            }
            _wasInStance[leg] = true;
            return _touchdownTargets[leg];
        }

        const bool wasInStance = _wasInStance[leg];
        _wasInStance[leg] = false;
        const bool shouldUpdateTouchdownTarget =
            wasInStance || !_touchdownTargetValid[leg] || stopRecenterJustActivated ||
            _turnStopFrameValid;

        if (shouldUpdateTouchdownTarget) {
            _touchdownTargets[leg] = touchdownTargetWorldBodyVelocityHalfStance(
                leg,
                currentPlanarCommand,
                shouldStopRecenter);
            _touchdownTargetValid[leg] = true;
        }

        return _touchdownTargets[leg];
    };

    DesiredFootPositions desiredFootPositions;
    desiredFootPositions.left_des_W = computeDesiredFootPos(Side::Left);
    desiredFootPositions.right_des_W = computeDesiredFootPos(Side::Right);
    _stopRecenterWasActive = shouldStopRecenter;
    _previousPlanarCommand_B = currentPlanarCommand;
    _previousYawRateCommand = currentYawRateCommand;
    _previousCommandValid = true;
    return desiredFootPositions;
}
