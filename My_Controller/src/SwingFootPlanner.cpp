#include "SwingFootPlanner.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "Utilities/MatrixUtils.h"

namespace {
constexpr double kSwingFootTargetZ = -0.005;
constexpr double kCommandZeroEpsilon = 1e-12;

bool hasPlanarMotionCommand(const UserCommand* command) {
    if (command == nullptr) {
        return false;
    }
    return std::abs(command->x_dot) > kCommandZeroEpsilon ||
           std::abs(command->y_dot) > kCommandZeroEpsilon ||
           std::abs(command->psi_dot) > kCommandZeroEpsilon;
}
}  // namespace

void SwingFootPlanner::reset() {
    _touchdownTargets.clear();
    _touchdownTargetValid.clear();
    _wasInStance.clear();
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
        _wasInStance.size() == legCount) {
        return;
    }

    _touchdownTargets.assign(legCount, Vec3<double>::Zero());
    _touchdownTargetValid.assign(legCount, false);
    _wasInStance.assign(legCount, true);
}

Vec3<double> SwingFootPlanner::currentFootTouchdownTarget(const std::size_t legIndex) const {
    return _stateEstimate->legs[legIndex].footPos_W;
}

Vec3<double> SwingFootPlanner::touchdownTargetWorldBodyVelocityHalfStance(
    const std::size_t legIndex) const {
    const Vec3<double> p_init_W =
        _touchdownTargetValid[legIndex] ? _touchdownTargets[legIndex]
                                        : currentFootTouchdownTarget(legIndex);
    const Vec3<double> v_body_cmd(
        _userCommand != nullptr ? _userCommand->x_dot : 0.0,
        _userCommand != nullptr ? _userCommand->y_dot : 0.0,
        0.0);
    const double stanceFraction = 0.5 + getControllerConfig().swing.bodyVelocityHalfStanceOffset;
    Vec3<double> target =
        p_init_W + Rz(_stateEstimate->psi) * v_body_cmd * (stanceFraction * stanceTime());
    target.z() = kSwingFootTargetZ;
    return target;
}

Vec3<double> SwingFootPlanner::touchdownTargetWorldLegacy(const std::size_t legIndex) const {
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
    const double time = _stateEstimate->time;
    const double phase = _gaitScheduler->p(_robotParams->legs[legIndex].side, time);
    const double T_rem = std::max(cycleTime() * (1.0 - phase), 0.0);

    Vec3<double> p_nom_B(-0.002851214, 0.072812741, -0.752981881);
    p_nom_B[1] += (_robotParams->legs[legIndex].side == Side::Left ? 1.0 : -1.0) *
                   footPlacement.nominalLateralOffset;

    const double yaw_correction = psi_dot * stanceTime() / 2.0;
    const Mat3<double> R_yaw_correction = Rz(yaw_correction);

    double delta_x =
        (0.5 + footPlacement.swingBias) * u_com_B[0] * stanceTime()
        + footPlacement.velocityFeedbackGain * (u_com_B[0] - u_des_B[0])
        + (0.5 * z_com / std::abs(model.gravity)) * (u_com_B[1] * psi_dot);

    double delta_y =
        0.5 * u_com_B[1] * stanceTime()
        + footPlacement.velocityFeedbackGain * (u_com_B[1] - u_des_B[1])
        + (0.5 * z_com / std::abs(model.gravity)) * (-u_com_B[0] * psi_dot);

    delta_x = std::clamp(delta_x, -footPlacement.placementClamp, footPlacement.placementClamp);
    delta_y = std::clamp(delta_y, -footPlacement.placementClamp, footPlacement.placementClamp);

    Vec3<double> feedback_B(delta_x, delta_y, 0.0);
    Vec3<double> target =
        p_com_W + R_WB * (R_yaw_correction * p_nom_B + u_des_B * T_rem + feedback_B);
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

    const double time = _stateEstimate->time;
    const auto& touchdownTargetMode = getControllerConfig().swing.touchdownTargetMode;

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
            if (!hasPlanarMotionCommand(_userCommand)) {
                if (!_touchdownTargetValid[leg]) {
                    _touchdownTargets[leg] = currentFootTouchdownTarget(leg);
                }
            } else {
                switch (touchdownTargetMode) {
                    case TouchdownTargetMode::BodyVelocityHalfStance:
                        _touchdownTargets[leg] =
                            touchdownTargetWorldBodyVelocityHalfStance(leg);
                        break;
                    case TouchdownTargetMode::LegacyComYawCorrected:
                        _touchdownTargets[leg] = touchdownTargetWorldLegacy(leg);
                        break;
                    default:
                        throw std::runtime_error("Unsupported swing.touchdown_target_mode");
                }
            }
            _touchdownTargetValid[leg] = true;
        }

        return _touchdownTargets[leg];
    };

    DesiredFootPositions desiredFootPositions;
    desiredFootPositions.left_des_W = computeDesiredFootPos(Side::Left);
    desiredFootPositions.right_des_W = computeDesiredFootPos(Side::Right);
    return desiredFootPositions;
}
