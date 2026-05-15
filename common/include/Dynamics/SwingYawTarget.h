#ifndef SWING_YAW_TARGET_H
#define SWING_YAW_TARGET_H

#include <algorithm>
#include <cmath>

#include "Robot/RobotParams.h"
#include "Utilities/AngleUtils.h"

namespace swingyaw {
inline double swingFootYawPsiOffset(const Side side, const double psi_dot) {
    constexpr double kPsiYawGainDegPerRad = 100.0;
    constexpr double kPsiYawMaxOffsetDeg = 20.0;
    constexpr double kDegToRad = 3.141592653589793238462643383279502884 / 180.0;

    const double offsetMagDeg =
        std::clamp(kPsiYawGainDegPerRad * std::abs(psi_dot), 0.0, kPsiYawMaxOffsetDeg);
    // Keep the sign rule explicit: left-foot bias for +psi_dot, right-foot bias for -psi_dot.
    // Backward motion no longer flips the bias sign.
    if (psi_dot > 0.0 && side == Side::Left) {
        return offsetMagDeg * kDegToRad;
    }
    if (psi_dot < 0.0 && side == Side::Right) {
        return -offsetMagDeg * kDegToRad;
    }
    return 0.0;
}

inline double swingFootYawTargetWorldWithPsiOffset(const double psi_dot,
                                                   const Side side,
                                                   const double fallbackYaw_W) {
    const double psiBias_W = swingFootYawPsiOffset(side, psi_dot);
    return liftAngleNear(fallbackYaw_W + psiBias_W, fallbackYaw_W);
}
}  // namespace swingyaw

#endif  // SWING_YAW_TARGET_H
