#include "SwingFootPlanner.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "Utilities/MatrixUtils.h"

namespace {
constexpr double kSwingFootTargetZ = -0.005;

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
    return stateEstimate.torsoLinVel_W + stateEstimate.torsoAngVel_W.cross(bodyBOffset_W);
}
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
    _latchedBrakingOffset_B.setZero();
    _latchedBrakingOffsetValid = false;
    _previousPlanarCommand_B.setZero();
    _previousPlanarCommandValid = false;
    _previousBrakingOffsetActive = false;
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

double SwingFootPlanner::bodyYawTargetWorld() const {
    return _bodyYawTargetValid ? _bodyYawTarget_W : _stateEstimate->psi;
}

double SwingFootPlanner::touchdownPreviewTime() const {
    const double stanceFraction = 0.5 + getControllerConfig().swing.bodyVelocityHalfStanceOffset;
    return std::max(0.0, stanceFraction * stanceTime());
}

bool SwingFootPlanner::shouldApplyBrakingOffset(const Vec2<double>& currentPlanarCommand) const {
    const auto& swing = getControllerConfig().swing;
    if (!_previousPlanarCommandValid) {
        return false;
    }

    const double currentNorm = currentPlanarCommand.norm();
    const double previousNorm = _previousPlanarCommand_B.norm();
    if (!(previousNorm > swing.stopVelocityDeadband)) {
        return false;
    }

    if (currentNorm <= swing.stopVelocityDeadband) {
        return true;
    }

    return currentNorm + 1e-9 < previousNorm;
}

Vec2<double> SwingFootPlanner::computeStopBrakingOffsetBodyFrame(
    const Vec2<double>& currentPlanarCommand) const {
    if (_stateEstimate == nullptr || _robotParams == nullptr) {
        return Vec2<double>::Zero();
    }
    if (!shouldApplyBrakingOffset(currentPlanarCommand)) {
        return Vec2<double>::Zero();
    }

    const auto& swing = getControllerConfig().swing;
    if (swing.hasStopBrakingOffset) {
        return Vec2<double>(swing.stopBrakingOffset_B.x(), swing.stopBrakingOffset_B.y());
    }

    const Vec3<double> comVelocity_W = reducedBodyComVelocityWorld(*_stateEstimate, *_robotParams);
    const Vec3<double> comVelocity_B = Rz(bodyYawTargetWorld()).transpose() * comVelocity_W;
    Vec2<double> offset_B(comVelocity_B.x(), comVelocity_B.y());
    if (!(offset_B.norm() > swing.stopVelocityDeadband)) {
        return Vec2<double>::Zero();
    }

    const double comHeight = std::max(1e-6, reducedBodyComWorld(*_stateEstimate, *_robotParams).z());
    const double gravity = std::abs(getControllerConfig().model.gravity);
    if (!(gravity > 0.0)) {
        return Vec2<double>::Zero();
    }

    const double omega0 = std::sqrt(gravity / comHeight);
    if (!(omega0 > 0.0) || !std::isfinite(omega0)) {
        return Vec2<double>::Zero();
    }

    offset_B *= swing.stopCapturePointGain / omega0;

    const double maxOffset = swing.stopCapturePointMaxOffset;
    if (maxOffset > 0.0) {
        const double norm = offset_B.norm();
        if (norm > maxOffset) {
            offset_B *= maxOffset / norm;
        }
    }

    return offset_B;
}

Vec2<double> SwingFootPlanner::stopBrakingOffsetBodyFrame(
    const Vec2<double>& currentPlanarCommand) const {
    if (_latchedBrakingOffsetValid) {
        return _latchedBrakingOffset_B;
    }
    return computeStopBrakingOffsetBodyFrame(currentPlanarCommand);
}

Vec3<double> SwingFootPlanner::touchdownTargetWorldBodyVelocityHalfStance(
    const std::size_t legIndex,
    const Vec2<double>& currentPlanarCommand) const {
    const Vec3<double> v_body_cmd(currentPlanarCommand.x(),
                                  currentPlanarCommand.y(),
                                  0.0);
    const double previewTime = touchdownPreviewTime();
    const double psi_dot = (_userCommand != nullptr) ? _userCommand->psi_dot : 0.0;
    const double yaw0 = bodyYawTargetWorld();
    const double translationYaw_W = yaw0 + 0.5 * psi_dot * previewTime;
    const Vec3<double> step_W = Rz(translationYaw_W) * v_body_cmd * previewTime;
    const Vec3<double> currentCenter_W =
        (_bodyPositionTargetValid
             ? _bodyPositionTarget_W
             : (_footprintCenterValid ? _footprintCenter_W
                                      : currentFootTouchdownTarget(legIndex)));
    const Vec2<double> brakingOffset_B = stopBrakingOffsetBodyFrame(currentPlanarCommand);
    Vec3<double> target =
        currentCenter_W + step_W +
        Rz(yaw0) * Vec3<double>(brakingOffset_B.x(), brakingOffset_B.y(), 0.0) +
        Rz(yaw0) * _nominalFootOffsets_B[legIndex];
    target.z() = kSwingFootTargetZ;
    return target;
}

void SwingFootPlanner::recordSequentialTouchdown(const std::size_t legIndex,
                                                 const Vec3<double>& target_W) {
    _footprintCenter_W = target_W - Rz(bodyYawTargetWorld()) * _nominalFootOffsets_B[legIndex];
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
    const bool brakingOffsetActive = shouldApplyBrakingOffset(currentPlanarCommand);
    if (brakingOffsetActive && !_previousBrakingOffsetActive) {
        _latchedBrakingOffset_B = computeStopBrakingOffsetBodyFrame(currentPlanarCommand);
        _latchedBrakingOffsetValid = _latchedBrakingOffset_B.norm() > 0.0;
    } else if (_latchedBrakingOffsetValid && !brakingOffsetActive &&
               currentPlanarCommand.norm() > getControllerConfig().swing.stopVelocityDeadband) {
        _latchedBrakingOffsetValid = false;
        _latchedBrakingOffset_B.setZero();
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
        const bool shouldUpdateTouchdownTarget = wasInStance || !_touchdownTargetValid[leg];

        if (shouldUpdateTouchdownTarget) {
            _touchdownTargets[leg] =
                touchdownTargetWorldBodyVelocityHalfStance(leg, currentPlanarCommand);
            recordSequentialTouchdown(leg, _touchdownTargets[leg]);
            _touchdownTargetValid[leg] = true;
        }

        return _touchdownTargets[leg];
    };

    DesiredFootPositions desiredFootPositions;
    desiredFootPositions.left_des_W = computeDesiredFootPos(Side::Left);
    desiredFootPositions.right_des_W = computeDesiredFootPos(Side::Right);
    _previousPlanarCommand_B = currentPlanarCommand;
    _previousPlanarCommandValid = true;
    _previousBrakingOffsetActive = brakingOffsetActive;
    return desiredFootPositions;
}
