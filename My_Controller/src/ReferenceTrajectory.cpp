#include "ReferenceTrajectory.h"

#include <stdexcept>

#include "Utilities/MatrixUtils.h"

void ReferenceTrajectory::build(ReferenceTrajectoryOutput& out) const {
    if (_horizonClock == nullptr) {
        throw std::runtime_error("ReferenceTrajectory::build requires initialized HorizonClock");
    }

    out.resizeIfNeeded();
    out.setZero();

    const double psi_dot_des = (_userCommand != nullptr) ? _userCommand->psi_dot : 0.0;
    const Vec3<double> u_des_B(
        _userCommand != nullptr ? _userCommand->x_dot : 0.0,
        _userCommand != nullptr ? _userCommand->y_dot : 0.0,
        0.0);

    const double psi0 = _x0[2];
    const double gravity = _x0[12];
    const int steps = horizonSteps();

    Vec3<double> p_ref = _x0.template segment<3>(3);
    Vec3<double> v_ref_W = Vec3<double>::Zero();

    for (int k = 0; k < steps; ++k) {
        const double tk = _horizonClock->tk(k);
        const double tau_k = tk - _horizonClock->t0();
        const double psi_k = psi0 + psi_dot_des * tau_k;
        const Mat3<double> R_WB = Rz(psi_k);

        v_ref_W = R_WB * u_des_B;

        if (k > 0) {
            p_ref += v_ref_W * dtMpc();
        }

        out.tk[k] = tk;
        out.psi[k] = psi_k;
        out.r_left.col(k) = _desiredFootPositions.left_des_W - p_ref;
        out.r_right.col(k) = _desiredFootPositions.right_des_W - p_ref;

        const Eigen::Index stateOffset = static_cast<Eigen::Index>(13 * k);
        auto x_ref_k = out.X_ref.segment(stateOffset, 13);
        x_ref_k.setZero();
        x_ref_k[0] = 0.0;
        x_ref_k[1] = 0.0;
        x_ref_k[2] = psi_k;
        x_ref_k.template segment<3>(3) = p_ref;
        x_ref_k[6] = 0.0;
        x_ref_k[7] = 0.0;
        x_ref_k[8] = psi_dot_des;
        x_ref_k.template segment<3>(9) = v_ref_W;
        x_ref_k[12] = gravity;
    }
}
