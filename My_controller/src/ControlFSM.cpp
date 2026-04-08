#include "ControlFSM.h"

#include <algorithm>
#include <stdexcept>

#include "Utilities/MatrixUtils.h"

void ControlFSM::syncHorizonClock() {
    if (_horizonClock == nullptr || _stateEstimate == nullptr) {
        throw std::runtime_error("ControlFSM::syncHorizonClock requires initialized pointers");
    }

    _horizonClock->sync(_stateEstimate->time);
}

DesiredFootPositions ControlFSM::SwingFootDesPos() {
    if (_gaitScheduler == nullptr || _horizonClock == nullptr || _stateEstimate == nullptr ||
        _robotParams == nullptr) {
        throw std::runtime_error("ControlFSM::SwingFootDesPos requires initialized pointers");
    }

    syncHorizonClock();

    const double kv = 0.03;
    const double footPlacementClamp = 0.3;
    const double z_td = -0.003;
    const double nominalLateralOffset = 0.065;
    const double bonusSwing = 0.0;
    const double gravity = 9.81;

    const Vec3<double> u_des_B = Vec3<double>{
        _userCommand != nullptr ? _userCommand->x_dot : 0.0,
        _userCommand != nullptr ? _userCommand->y_dot : 0.0,
        0.0};
    const double psi = _stateEstimate->psi;
    const Mat3<double> R_WB = Rz(psi);
    const Mat3<double> R_BW = R_WB.transpose();

    const Vec3<double> u_com_B = R_BW * _stateEstimate->baseLinVel;
    const double psi_dot = (_userCommand != nullptr) ? _userCommand->psi_dot : 0.0;
    const double z_com = _stateEstimate->basePos[2];
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
            throw std::runtime_error("ControlFSM::SwingFootDesPos could not find requested leg");
        }

        if (_gaitScheduler->c(side, time)) {
            return _stateEstimate->legs[legIndex].footPosWorld;
        }

        const double phase = _gaitScheduler->p(side, time);
        const double T_rem = std::max(T_cycle * (1.0 - phase), 0.0);

        Vec3<double> p_nom_B = _robotParams->legs[legIndex].hipLocation_from_body;
        p_nom_B[1] += (side == Side::Left ? 1.0 : -1.0) * nominalLateralOffset;

        const double yaw_correction = psi_dot * T_stance / 2.0;
        const Mat3<double> R_yaw_correction = Rz(yaw_correction);

        double delta_x =
            (0.5 + bonusSwing) * u_com_B[0] * T_stance
            + kv * (u_com_B[0] - u_des_B[0])
            + (0.5 * z_com / gravity) * (u_com_B[1] * psi_dot);

        double delta_y =
            0.5 * u_com_B[1] * T_stance
            + kv * (u_com_B[1] - u_des_B[1])
            + (0.5 * z_com / gravity) * (-u_com_B[0] * psi_dot);

        delta_x = std::clamp(delta_x, -footPlacementClamp, footPlacementClamp);
        delta_y = std::clamp(delta_y, -footPlacementClamp, footPlacementClamp);

        Vec3<double> feedback_B(delta_x, delta_y, z_td);
        return _stateEstimate->basePos
             + R_WB * (R_yaw_correction * p_nom_B + u_des_B * T_rem + feedback_B);
    };

    DesiredFootPositions desiredFootPositions;
    desiredFootPositions.left_des_W = computeDesiredFootPos(Side::Left);
    desiredFootPositions.right_des_W = computeDesiredFootPos(Side::Right);
    return desiredFootPositions;
}
