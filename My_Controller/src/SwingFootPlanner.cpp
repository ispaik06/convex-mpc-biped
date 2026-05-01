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

double SwingFootPlanner::bodyYawTargetWorld() const {
    return _bodyYawTargetValid ? _bodyYawTarget_W : _stateEstimate->psi;
}

double SwingFootPlanner::touchdownPreviewTime() const {
    const double stanceFraction = 0.5 + getControllerConfig().swing.bodyVelocityHalfStanceOffset;
    return std::max(0.0, stanceFraction * stanceTime());
}

Vec3<double> SwingFootPlanner::touchdownTargetWorldBodyVelocityHalfStance(
    const std::size_t legIndex) const {
    const Vec3<double> v_body_cmd(
        _userCommand != nullptr ? _userCommand->x_dot : 0.0,
        _userCommand != nullptr ? _userCommand->y_dot : 0.0,
        0.0);
    const double previewTime = touchdownPreviewTime();
    const double psi_dot = (_userCommand != nullptr) ? _userCommand->psi_dot : 0.0;
    const double yaw0 = bodyYawTargetWorld();
    const double translationYaw_W = yaw0 + 0.5 * psi_dot * previewTime;
    const Vec3<double> step_W = Rz(translationYaw_W) * v_body_cmd * previewTime;
    const Vec3<double> currentCenter_W =
        _bodyPositionTargetValid
            ? _bodyPositionTarget_W
            : (_footprintCenterValid ? _footprintCenter_W : currentFootTouchdownTarget(legIndex));
    Vec3<double> target =
        currentCenter_W + step_W + Rz(yaw0) * _nominalFootOffsets_B[legIndex];
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
            _touchdownTargets[leg] = touchdownTargetWorldBodyVelocityHalfStance(leg);
            recordSequentialTouchdown(leg, _touchdownTargets[leg]);
            _touchdownTargetValid[leg] = true;
        }

        return _touchdownTargets[leg];
    };

    DesiredFootPositions desiredFootPositions;
    desiredFootPositions.left_des_W = computeDesiredFootPos(Side::Left);
    desiredFootPositions.right_des_W = computeDesiredFootPos(Side::Right);
    return desiredFootPositions;
}
