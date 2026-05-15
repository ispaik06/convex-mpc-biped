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

inline double swingFootYawFromDiagonalStepHeading(const Vec3<double>& currentFootPosition_W,
                                                  const Vec3<double>& touchdownTarget_W,
                                                  const Vec2<double>& filteredPlanarCommand_B,
                                                  const double psi_dot,
                                                  const Side side,
                                                  const double fallbackYaw_W) {
    constexpr double kDiagonalCommandDeadband = 1e-3;
    constexpr double kFilteredLateralSpeedThreshold = 0.1;
    const double psiBias_W = swingFootYawPsiOffset(side, psi_dot);
    if (std::abs(filteredPlanarCommand_B.y()) < kFilteredLateralSpeedThreshold) {
        return liftAngleNear(fallbackYaw_W + psiBias_W, fallbackYaw_W);
    }

    if (std::abs(filteredPlanarCommand_B.x()) <= kDiagonalCommandDeadband) {
        return liftAngleNear(fallbackYaw_W + psiBias_W, fallbackYaw_W);
    }

    const bool sideMatchesLateralDirection =
        (filteredPlanarCommand_B.y() > 0.0 && side == Side::Left) ||
        (filteredPlanarCommand_B.y() < 0.0 && side == Side::Right);
    if (!sideMatchesLateralDirection) {
        return liftAngleNear(fallbackYaw_W + psiBias_W, fallbackYaw_W);
    }

    const double xDot = filteredPlanarCommand_B.x();
    const double yDot = filteredPlanarCommand_B.y();
    // Only treat the command as diagonal when the forward/back component dominates in the
    // same-sign case; otherwise keep the simpler fallback heading.
    const bool diagonalHeadingDominates =
        (xDot > 0.0 && xDot >= yDot) || (xDot < 0.0 && xDot <= yDot);
    if (!diagonalHeadingDominates) {
        return liftAngleNear(fallbackYaw_W + psiBias_W, fallbackYaw_W);
    }

    const Vec2<double> stepXY_W =
        (touchdownTarget_W - currentFootPosition_W).template head<2>();
    if (stepXY_W.squaredNorm() <= 1e-8) {
        return liftAngleNear(fallbackYaw_W + psiBias_W, fallbackYaw_W);
    }

    const double yaw_W = std::atan2(stepXY_W.y(), stepXY_W.x());
    return liftAngleNear(yaw_W + psiBias_W, fallbackYaw_W);
}
}  // namespace swingyaw

#endif  // SWING_YAW_TARGET_H
