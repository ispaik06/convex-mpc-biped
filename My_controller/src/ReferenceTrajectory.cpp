#include "ReferenceTrajectory.h"

#include "Utilities/MatrixUtils.h"

ReferenceTrajectoryOutput ReferenceTrajectory::build() const {
    ReferenceTrajectoryOutput out;

    const double dt = T_horizon / N;
    const double psi_dot_des = (_userCommand != nullptr) ? _userCommand->psi_dot : 0.0;
    const Vec3<double> u_des_B(
        _userCommand != nullptr ? _userCommand->x_dot : 0.0,
        _userCommand != nullptr ? _userCommand->y_dot : 0.0,
        0.0);

    const double psi0 = _x0[2];
    const double gravity = _x0[12];

    Vec3<double> p_ref = _x0.template segment<3>(3);
    Vec3<double> v_ref_W = Vec3<double>::Zero();

    for (int k = 0; k < N; ++k) {
        const double psi_k = psi0 + psi_dot_des * k * dt;
        const Mat3<double> R_WB = Rz(psi_k);

        v_ref_W = R_WB * u_des_B;

        if (k > 0) {
            p_ref += v_ref_W * dt;
        }

        out.tk[k] = _t0 + k * dt;
        out.psi[k] = psi_k;
        out.r_left.col(k) = _desiredFootPositions.left_des_W - p_ref;
        out.r_right.col(k) = _desiredFootPositions.right_des_W - p_ref;

        Vec13<double> x_ref_k = Vec13<double>::Zero();
        x_ref_k[0] = 0.0;
        x_ref_k[1] = 0.0;
        x_ref_k[2] = psi_k;
        x_ref_k.template segment<3>(3) = p_ref;
        x_ref_k[6] = 0.0;
        x_ref_k[7] = 0.0;
        x_ref_k[8] = psi_dot_des;
        x_ref_k.template segment<3>(9) = v_ref_W;
        x_ref_k[12] = gravity;

        out.X_ref.col(k) = x_ref_k;
    }

    return out;
}
