#ifndef OPERATIONAL_SPACE_DYNAMICS_H
#define OPERATIONAL_SPACE_DYNAMICS_H

#include "cppTypes.h"

template <typename T>
DMat<T> computeApparentInertia(const DMat<T>& Jv_W, const DMat<T>& massMatrix);

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
                                   const Vec3<T>& forceFeedForward_W);

template <typename T>
DVec<T> computeStanceLegJointTorque(const DMat<T>& Jv_W,
                                    const DMat<T>& Jw_W,
                                    const Vec3<T>& force_W,
                                    const Vec3<T>& moment_W);

#endif  // OPERATIONAL_SPACE_DYNAMICS_H
