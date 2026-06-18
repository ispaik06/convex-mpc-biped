#include "SwingFootPlanner.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "Utilities/MatrixUtils.h"

namespace {
constexpr double kSwingFootTargetZ = -0.005;
}  // namespace

void SwingFootPlanner::reset() {
    _touchdownTargets.clear();
    _touchdownTargetValid.clear();
    _nominalFootOffsets_B.clear();
    _nominalFootOffsetValid.clear();
    _wasInStance.clear();
    _footprintCenter_W.setZero();
    _footprintCenterValid = false;
    _bodyPositionTargetValid = false;
    _bodyYawTargetValid = false;
    _stopRecenterClearTicks = 0;
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

    _footprintCenter_W =
        0.5 * (desiredFootPositions.left_des_W + desiredFootPositions.right_des_W);
    _footprintCenterValid = true;
}

void SwingFootPlanner::setBodyTargetWorld(const Vec3<double>& position_W, const double yaw_W) {
    if (!position_W.allFinite() || !std::isfinite(yaw_W)) {
        _bodyPositionTargetValid = false;
        _bodyYawTargetValid = false;
        return;
    }

    _bodyPositionTarget_W = position_W;
    _bodyPositionTargetValid = true;
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
    _footprintCenter_W.setZero();
    _footprintCenterValid = false;
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
    if (!_footprintCenterValid) {
        _footprintCenter_W = footCenter_W;
        _footprintCenterValid = true;
    }

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

    if (_stopRecenterClearTicks < swing.stopBrakingLatchClearTicks) {
        ++_stopRecenterClearTicks;
        return true;
    }

    return false;
}

double SwingFootPlanner::computeStopRecenterXBody() const {
    const auto& swing = getControllerConfig().swing;
    if (swing.hasStopBrakingOffset) {
        return swing.stopBrakingOffset_B.x();
    }

    const double yaw0 = bodyYawTargetWorld();
    const Vec3<double> velocity_B =
        Rz(yaw0).transpose() * _stateEstimate->torsoLinVel_W;
    const double captureX =
        swing.stopCapturePointGain * velocity_B.x();
    return std::clamp(captureX,
                      -swing.stopCapturePointMaxOffset,
                      swing.stopCapturePointMaxOffset);
}

Vec3<double> SwingFootPlanner::touchdownTargetWorldBodyVelocityHalfStance(
    const std::size_t legIndex,
    const Vec2<double>& currentPlanarCommand,
    const bool stopRecenter,
    double* touchdownYaw_W_out,
    Vec3<double>* effectiveTouchdownOffset_B_out) const {
    const Vec3<double> previewVelocity_B(currentPlanarCommand.x(),
                                         currentPlanarCommand.y(),
                                         0.0);
    const double previewTime = touchdownPreviewTime(currentPlanarCommand);
    const double psi_dot = (_userCommand != nullptr) ? _userCommand->psi_dot : 0.0;
    const double yaw0 = bodyYawTargetWorld();
    const double translationYaw_W = yaw0 + 0.5 * psi_dot * previewTime;
    // Use the remaining swing time to estimate the yaw at the eventual touchdown instant.
    const double remainingSwingTime =
        std::clamp(cycleTime() * (1.0 - _gaitScheduler->p(_robotParams->legs[legIndex].side,
                                                          _stateEstimate->time)),
                   0.0,
                   swingTime());
    const double touchdownYaw_W = yaw0 + psi_dot * remainingSwingTime;
    if (touchdownYaw_W_out != nullptr) {
        *touchdownYaw_W_out = touchdownYaw_W;
    }
    const Vec3<double> step_W = Rz(translationYaw_W) * previewVelocity_B * previewTime;
    const Vec3<double> currentCenter_W =
        (_bodyPositionTargetValid
             ? _bodyPositionTarget_W
             : (_footprintCenterValid ? _footprintCenter_W
                                      : currentFootTouchdownTarget(legIndex)));
    const Vec3<double>& touchdownOffset_B = _nominalFootOffsets_B[legIndex];
    Vec3<double> plannedOffset_B =
        Rz(yaw0).transpose() * step_W +
        Rz(touchdownYaw_W - yaw0) * touchdownOffset_B;
    Vec3<double> effectiveTouchdownOffset_B = touchdownOffset_B;
    if (stopRecenter) {
        plannedOffset_B.x() = computeStopRecenterXBody();
        effectiveTouchdownOffset_B.x() = plannedOffset_B.x();
    }
    if (effectiveTouchdownOffset_B_out != nullptr) {
        *effectiveTouchdownOffset_B_out = effectiveTouchdownOffset_B;
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

void SwingFootPlanner::recordSequentialTouchdown(const Vec3<double>& target_W,
                                                 const double touchdownYaw_W,
                                                 const Vec3<double>& touchdownOffset_B) {
    _footprintCenter_W = target_W - Rz(touchdownYaw_W) * touchdownOffset_B;
    _footprintCenter_W.z() = target_W.z();
    _footprintCenterValid = true;
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
    const double previewTime = touchdownPreviewTime(currentPlanarCommand);
    const bool shouldStopRecenter = stopRecenterActive(currentPlanarCommand);
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
        const bool shouldUpdateTouchdownTarget = wasInStance || !_touchdownTargetValid[leg];

        if (shouldUpdateTouchdownTarget) {
            double touchdownYaw_W = 0.0;
            Vec3<double> effectiveTouchdownOffset_B = _nominalFootOffsets_B[leg];
            _touchdownTargets[leg] = touchdownTargetWorldBodyVelocityHalfStance(
                leg,
                currentPlanarCommand,
                shouldStopRecenter,
                &touchdownYaw_W,
                &effectiveTouchdownOffset_B);
            recordSequentialTouchdown(_touchdownTargets[leg],
                                      touchdownYaw_W,
                                      effectiveTouchdownOffset_B);
            _touchdownTargetValid[leg] = true;
        }

        return _touchdownTargets[leg];
    };

    DesiredFootPositions desiredFootPositions;
    desiredFootPositions.left_des_W = computeDesiredFootPos(Side::Left);
    desiredFootPositions.right_des_W = computeDesiredFootPos(Side::Right);
    return desiredFootPositions;
}
