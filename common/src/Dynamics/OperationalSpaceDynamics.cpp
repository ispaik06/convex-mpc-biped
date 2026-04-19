#include <stdexcept>

#include "Dynamics/OperationalSpaceDynamics.h"

namespace {
template <typename T>
void validateSwingInputs(const DMat<T>& Jv_W,
                         const DMat<T>& JvDot_W,
                         const DMat<T>& massMatrix,
                         const DVec<T>& qd,
                         const DVec<T>& bias) {
    if (Jv_W.rows() != 3 || JvDot_W.rows() != 3 || Jv_W.cols() != JvDot_W.cols()) {
        throw std::invalid_argument("Swing leg Jacobian must have 3 rows");
    }

    if (massMatrix.rows() != massMatrix.cols() || massMatrix.rows() != Jv_W.cols()) {
        throw std::invalid_argument("Mass matrix size does not match swing leg Jacobian");
    }

    if (qd.size() != Jv_W.cols() || bias.size() != Jv_W.cols()) {
        throw std::invalid_argument("Whole-body dynamics vectors do not match swing leg Jacobian");
    }
}

template <typename T>
void validateStanceInputs(const DMat<T>& Jv_W, const DMat<T>& Jw_W) {
    if (Jv_W.rows() != 3 || Jw_W.rows() != 3 || Jv_W.cols() != Jw_W.cols()) {
        throw std::invalid_argument("Stance leg Jacobians must be 3 x nv");
    }
}
}  // namespace

template <typename T>
DMat<T> computeApparentInertia(const DMat<T>& Jv_W, const DMat<T>& massMatrix) {
    const DMat<T> zeroJvDot = DMat<T>::Zero(3, Jv_W.cols());
    const DVec<T> zeroQd = DVec<T>::Zero(Jv_W.cols());
    const DVec<T> zeroBias = DVec<T>::Zero(Jv_W.cols());
    validateSwingInputs(Jv_W, zeroJvDot, massMatrix, zeroQd, zeroBias);

    const Eigen::LDLT<DMat<T>> massSolver(massMatrix);
    const DMat<T> minvJt = massSolver.solve(Jv_W.transpose());
    DMat<T> lambdaInv = Jv_W * minvJt;

    lambdaInv.diagonal().array() += T(1e-9);
    return lambdaInv.ldlt().solve(DMat<T>::Identity(lambdaInv.rows(), lambdaInv.cols()));
}

template <typename T>
DVec<T> computeSwingLegJointTorque(const DMat<T>& Jv_W,
                                   const DMat<T>& JvDot_W,
                                   const DMat<T>& massMatrix,
                                   const DVec<T>& qd,
                                   const DVec<T>& bias,
                                   const Vec3<T>& pDes_W,
                                   const Vec3<T>& vDes_W,
                                   const Vec3<T>& aDes_W,
                                   const Vec3<T>& p_W,
                                   const Vec3<T>& v_W,
                                   const Mat3<T>& kp,
                                   const Mat3<T>& kd,
                                   const Vec3<T>& forceFeedForward_W) {
    validateSwingInputs(Jv_W, JvDot_W, massMatrix, qd, bias);

    const Vec3<T> feedbackForce =
        forceFeedForward_W + kp * (pDes_W - p_W) + kd * (vDes_W - v_W);
    const Vec3<T> taskAccelerationResidual = aDes_W - JvDot_W * qd;
    const DMat<T> lambda = computeApparentInertia(Jv_W, massMatrix);

    return Jv_W.transpose() * feedbackForce
         + Jv_W.transpose() * (lambda * taskAccelerationResidual)
         + bias;
}

template <typename T>
DVec<T> computeStanceLegJointTorque(const DMat<T>& Jv_W,
                                    const DMat<T>& Jw_W,
                                    const Vec3<T>& force_W,
                                    const Vec3<T>& moment_W) {
    validateStanceInputs(Jv_W, Jw_W);

    return Jv_W.transpose() * force_W + Jw_W.transpose() * moment_W;
}

template DMat<float> computeApparentInertia(const DMat<float>&, const DMat<float>&);
template DMat<double> computeApparentInertia(const DMat<double>&, const DMat<double>&);

template DVec<float> computeSwingLegJointTorque(const DMat<float>&,
                                                const DMat<float>&,
                                                const DMat<float>&,
                                                const DVec<float>&,
                                                const DVec<float>&,
                                                const Vec3<float>&,
                                                const Vec3<float>&,
                                                const Vec3<float>&,
                                                const Vec3<float>&,
                                                const Vec3<float>&,
                                                const Mat3<float>&,
                                                const Mat3<float>&,
                                                const Vec3<float>&);
template DVec<double> computeSwingLegJointTorque(const DMat<double>&,
                                                 const DMat<double>&,
                                                 const DMat<double>&,
                                                 const DVec<double>&,
                                                 const DVec<double>&,
                                                 const Vec3<double>&,
                                                 const Vec3<double>&,
                                                 const Vec3<double>&,
                                                 const Vec3<double>&,
                                                 const Vec3<double>&,
                                                 const Mat3<double>&,
                                                 const Mat3<double>&,
                                                 const Vec3<double>&);

template DVec<float> computeStanceLegJointTorque(const DMat<float>&,
                                                 const DMat<float>&,
                                                 const Vec3<float>&,
                                                 const Vec3<float>&);
template DVec<double> computeStanceLegJointTorque(const DMat<double>&,
                                                  const DMat<double>&,
                                                  const Vec3<double>&,
                                                  const Vec3<double>&);
