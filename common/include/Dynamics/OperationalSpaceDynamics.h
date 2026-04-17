#ifndef OPERATIONAL_SPACE_DYNAMICS_H
#define OPERATIONAL_SPACE_DYNAMICS_H

#include "cppTypes.h"

template <typename T>
DMat<T> computeApparentInertia(const DMat<T>& JvWorld, const DMat<T>& massMatrix);

template <typename T>
DVec<T> computeSwingLegJointTorque(const DMat<T>& JvWorld,
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
                                   const Vec3<T>& forceFeedForward);

template <typename T>
DVec<T> computeStanceLegJointTorque(const DMat<T>& JvWorld,
                                    const DMat<T>& JwWorld,
                                    const Vec3<T>& forceWorld,
                                    const Vec3<T>& momentWorld);

#endif  // OPERATIONAL_SPACE_DYNAMICS_H
