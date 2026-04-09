#include <stdexcept>

#include "Dynamics/OperationalSpaceDynamics.h"

namespace {
template <typename T>
void validateSwingInputs(const DMat<T>& JvWorld,
                         const DMat<T>& JvDotWorld,
                         const DMat<T>& massMatrix,
                         const DVec<T>& qd,
                         const DVec<T>& bias) {
    if (JvWorld.rows() != 3 || JvDotWorld.rows() != 3 || JvWorld.cols() != JvDotWorld.cols()) {
        throw std::invalid_argument("Swing leg Jacobian must have 3 rows");
    }

    if (massMatrix.rows() != massMatrix.cols() || massMatrix.rows() != JvWorld.cols()) {
        throw std::invalid_argument("Mass matrix size does not match swing leg Jacobian");
    }

    if (qd.size() != JvWorld.cols() || bias.size() != JvWorld.cols()) {
        throw std::invalid_argument("Whole-body dynamics vectors do not match swing leg Jacobian");
    }
}

template <typename T>
void validateStanceInputs(const DMat<T>& JvWorld, const DMat<T>& JwWorld) {
    if (JvWorld.rows() != 3 || JwWorld.rows() != 3 || JvWorld.cols() != JwWorld.cols()) {
        throw std::invalid_argument("Stance leg Jacobians must be 3 x nv");
    }
}
}  // namespace

template <typename T>
Mat3<T> computeApparentInertia(const DMat<T>& JvWorld, const DMat<T>& massMatrix) {
    const DMat<T> zeroJvDot = DMat<T>::Zero(3, JvWorld.cols());
    const DVec<T> zeroQd = DVec<T>::Zero(JvWorld.cols());
    const DVec<T> zeroBias = DVec<T>::Zero(JvWorld.cols());
    validateSwingInputs(JvWorld, zeroJvDot, massMatrix, zeroQd, zeroBias);

    const Eigen::LDLT<DMat<T>> massSolver(massMatrix);
    const DMat<T> minvJt = massSolver.solve(JvWorld.transpose());
    Mat3<T> lambdaInv = JvWorld * minvJt;

    lambdaInv.diagonal().array() += T(1e-9);
    return lambdaInv.ldlt().solve(Mat3<T>::Identity());
}

template <typename T>
DVec<T> computeSwingLegGeneralizedForce(const DMat<T>& JvWorld,
                                        const DMat<T>& JvDotWorld,
                                        const DMat<T>& massMatrix,
                                        const DVec<T>& qd,
                                        const DVec<T>& bias,
                                        const Vec3<T>& pDes,
                                        const Vec3<T>& vDes,
                                        const Vec3<T>& aDes,
                                        const Vec3<T>& pWorld,
                                        const Vec3<T>& vWorld,
                                        const Mat3<T>& kp,
                                        const Mat3<T>& kd,
                                        const Vec3<T>& forceFeedForward) {
    validateSwingInputs(JvWorld, JvDotWorld, massMatrix, qd, bias);

    const Vec3<T> feedbackForce =
        forceFeedForward + kp * (pDes - pWorld) + kd * (vDes - vWorld);
    const Vec3<T> taskAccelerationResidual = aDes - JvDotWorld * qd;
    const Mat3<T> lambda = computeApparentInertia(JvWorld, massMatrix);

    return JvWorld.transpose() * feedbackForce
         + JvWorld.transpose() * (lambda * taskAccelerationResidual)
         + bias;
}

template <typename T>
DVec<T> computeStanceLegGeneralizedForce(const DMat<T>& JvWorld,
                                         const DMat<T>& JwWorld,
                                         const Vec3<T>& forceWorld,
                                         const Vec3<T>& momentWorld) {
    validateStanceInputs(JvWorld, JwWorld);

    return JvWorld.transpose() * forceWorld + JwWorld.transpose() * momentWorld;
}

template Mat3<float> computeApparentInertia(const DMat<float>&, const DMat<float>&);
template Mat3<double> computeApparentInertia(const DMat<double>&, const DMat<double>&);

template DVec<float> computeSwingLegGeneralizedForce(const DMat<float>&,
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
template DVec<double> computeSwingLegGeneralizedForce(const DMat<double>&,
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

template DVec<float> computeStanceLegGeneralizedForce(const DMat<float>&,
                                                      const DMat<float>&,
                                                      const Vec3<float>&,
                                                      const Vec3<float>&);
template DVec<double> computeStanceLegGeneralizedForce(const DMat<double>&,
                                                       const DMat<double>&,
                                                       const Vec3<double>&,
                                                       const Vec3<double>&);
