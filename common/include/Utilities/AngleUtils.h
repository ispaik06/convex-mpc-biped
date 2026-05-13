#ifndef ANGLE_UTILS_H
#define ANGLE_UTILS_H

#include <cmath>

inline double wrapToPi(const double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

inline double unwrapAngle(const double wrappedNow,
                          const double wrappedPrev,
                          const double unwrappedPrev) {
    return unwrappedPrev + wrapToPi(wrappedNow - wrappedPrev);
}

inline double liftAngleNear(const double targetWrapped, const double currentUnwrapped) {
    return currentUnwrapped + wrapToPi(targetWrapped - currentUnwrapped);
}

#endif  // ANGLE_UTILS_H
