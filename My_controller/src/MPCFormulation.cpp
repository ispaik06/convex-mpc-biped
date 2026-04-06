#include <stdexcept>
#include <unsupported/Eigen/MatrixFunctions>

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
    DMat<double> M = DMat<double>::Zero(kStateDim + kInputDim, kStateDim + kInputDim);
    M.block(0, 0, kStateDim, kStateDim) = A_c;
    M.block(0, kStateDim, kStateDim, kInputDim) = B_c;

    const DMat<double> M_exp = (M * dt).exp();
    A_d = M_exp.block(0, 0, kStateDim, kStateDim);
    B_d = M_exp.block(0, kStateDim, kStateDim, kInputDim);
}
}  // namespace

MPCFormulationOutput MPCFormulation::build(
    const ReferenceTrajectoryOutput& referenceTrajectory) const {
    if (_robotParams == nullptr) {
        throw std::runtime_error("MPCFormulation received null RobotParams");
    }

    if (_robotParams->bodyMass <= 0.0) {
        throw std::runtime_error("MPCFormulation requires positive bodyMass");
    }

    const double dt = T_horizon / N;
    const double mass = _robotParams->bodyMass;
    //? Note: bodyInertia currently comes from the MuJoCo base body only.
    //? It is not the whole-robot inertia about the whole-robot COM yet.
    const Mat3<double>& I_body = _robotParams->bodyInertia;
    const Mat3<double> I3 = Mat3<double>::Identity();

    MPCFormulationOutput out;
    vectorAligned<Mat13d> A_d(N, Mat13d::Identity());
    vectorAligned<Mat13x12d> B_d(N, Mat13x12d::Zero());

    //* Input order follows the current formulation sketch:
    //* u_k = [F_left, F_right, M_left, M_right].
    for (int k = 0; k < N; ++k) {
        const double psi_k = referenceTrajectory.psi[k];
        const Vec3<double> r_left_k = referenceTrajectory.r_left.col(k);
        const Vec3<double> r_right_k = referenceTrajectory.r_right.col(k);

        const Mat3<double> R_k = Rz(psi_k);
        const Mat3<double> I_k = R_k * I_body * R_k.transpose();
        const Mat3<double> I_k_inv = I_k.inverse();

        Mat13d A_c_k = Mat13d::Zero();
        A_c_k.block<3, 3>(0, 6) = R_k.transpose();
        A_c_k.block<3, 3>(3, 9) = I3;

        Mat13x12d B_c_k = Mat13x12d::Zero();
        B_c_k.block<3, 3>(6, 0) = I_k_inv * skew(r_left_k);
        B_c_k.block<3, 3>(6, 3) = I_k_inv * skew(r_right_k);
        B_c_k.block<3, 3>(6, 6) = I_k_inv;
        B_c_k.block<3, 3>(6, 9) = I_k_inv;
        B_c_k.block<3, 3>(9, 0) = I3 / mass;
        B_c_k.block<3, 3>(9, 3) = I3 / mass;

        discretizeZOH(A_c_k, B_c_k, dt, A_d[k], B_d[k]);
    }

    Mat13d prefixTransition = Mat13d::Identity();
    for (int k = 0; k < N; ++k) {
        prefixTransition = A_d[k] * prefixTransition;
        out.A_qp.block(k * kStateDim, 0, kStateDim, kStateDim) = prefixTransition;

        for (int j = 0; j <= k; ++j) {
            Mat13d inputTransition = Mat13d::Identity();
            for (int ell = j + 1; ell <= k; ++ell) {
                inputTransition = A_d[ell] * inputTransition;
            }
            out.B_qp.block(k * kStateDim, j * kInputDim, kStateDim, kInputDim) =
                inputTransition * B_d[j];
        }
    }

    return out;
}
