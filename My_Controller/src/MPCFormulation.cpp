#include <stdexcept>

#include "MPCFormulation.h"
#include "Utilities/MatrixUtils.h"

namespace {
constexpr int kStateDim = 13;
constexpr int kInputDim = 12;

void discretizeZOH(const Mat13d& A_c,
                   const Mat13x12d& B_c,
                   const double dt,
                   Mat13d& A_d,
                   Mat13x12d& B_d) {
    const Mat13d A_c_sq = A_c * A_c;
    const Mat13x12d A_c_B_c = A_c * B_c;
    const Mat13x12d A_c_sq_B_c = A_c_sq * B_c;

    A_d = Mat13d::Identity();
    A_d.noalias() += dt * A_c;
    A_d.noalias() += 0.5 * dt * dt * A_c_sq;

    B_d = dt * B_c;
    B_d.noalias() += 0.5 * dt * dt * A_c_B_c;
    B_d.noalias() += (dt * dt * dt / 6.0) * A_c_sq_B_c;
}
}  // namespace

void MPCFormulation::resizeScratchIfNeeded() {
    const int steps = horizonSteps();
    if (static_cast<int>(_A_d.size()) == steps && static_cast<int>(_B_d.size()) == steps) {
        return;
    }

    _A_d.assign(steps, Mat13d::Identity());
    _B_d.assign(steps, Mat13x12d::Zero());
}

void MPCFormulation::build(const ReferenceTrajectoryOutput& referenceTrajectory,
                           MPCFormulationOutput& out) {
    if (_robotParams == nullptr) {
        throw std::runtime_error("MPCFormulation received null RobotParams");
    }

    if (_robotParams->bodyMass <= 0.0) {
        throw std::runtime_error("MPCFormulation requires positive bodyMass");
    }

    const double dt = dtMpc();
    const double mass = _robotParams->bodyMass;
    // Reduced-body inertia is maintained in the yaw-aligned SRB frame.
    const Mat3<double>& I_body = _robotParams->bodyInertia;
    const Mat3<double> I_body_inv = I_body.inverse();
    const Mat3<double> I3 = Mat3<double>::Identity();

    const int steps = horizonSteps();
    resizeScratchIfNeeded();
    out.resizeIfNeeded();
    out.setZero();

    //* Input order follows the current formulation sketch:
    //* u_k = [F_left, F_right, M_left, M_right].
    for (int k = 0; k < steps; ++k) {
        const double psi_k = referenceTrajectory.psi[k];
        const Vec3<double> r_left_k = referenceTrajectory.r_left.col(k);
        const Vec3<double> r_right_k = referenceTrajectory.r_right.col(k);

        const Mat3<double> R_k = Rz(psi_k);
        const Mat3<double> I_k_inv = R_k * I_body_inv * R_k.transpose();

        Mat13d A_c_k = Mat13d::Zero();
        A_c_k.block<3, 3>(0, 6) = R_k.transpose();
        A_c_k.block<3, 3>(3, 9) = I3;
        A_c_k.block<3, 1>(9, 12) << 0.0, 0.0, -1.0;

        Mat13x12d B_c_k = Mat13x12d::Zero();
        B_c_k.block<3, 3>(6, 0) = I_k_inv * skew(r_left_k);
        B_c_k.block<3, 3>(6, 3) = I_k_inv * skew(r_right_k);
        B_c_k.block<3, 3>(6, 6) = I_k_inv;
        B_c_k.block<3, 3>(6, 9) = I_k_inv;
        B_c_k.block<3, 3>(9, 0) = I3 / mass;
        B_c_k.block<3, 3>(9, 3) = I3 / mass;

        discretizeZOH(A_c_k, B_c_k, dt, _A_d[k], _B_d[k]);
    }

    Mat13d prefixTransition = Mat13d::Identity();
    for (int k = 0; k < steps; ++k) {
        prefixTransition = _A_d[k] * prefixTransition;
        out.A_qp.block(k * kStateDim, 0, kStateDim, kStateDim) = prefixTransition;
    }

    for (int j = 0; j < steps; ++j) {
        Mat13x12d liftedInput = _B_d[j];
        for (int k = j; k < steps; ++k) {
            out.B_qp.block(k * kStateDim, j * kInputDim, kStateDim, kInputDim) = liftedInput;
            if ((k + 1) < steps) {
                liftedInput = _A_d[k + 1] * liftedInput;
            }
        }
    }
}
