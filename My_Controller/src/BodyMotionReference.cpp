#include "BodyMotionReference.h"

#include <cmath>

namespace BodyMotionReference {
namespace {
bool isDoubleSupport(const GaitScheduler* gaitScheduler, const double sampleTime) {
    return gaitScheduler != nullptr && gaitScheduler->bothFeetStance(sampleTime);
}
}  // namespace

bool shouldAdvanceYaw(const GaitScheduler* gaitScheduler,
                      const double sampleTime,
                      const YawIntegrationMode mode) {
    switch (mode) {
        case YawIntegrationMode::SingleSupport:
            return !isDoubleSupport(gaitScheduler, sampleTime);
        case YawIntegrationMode::DoubleSupport:
            return isDoubleSupport(gaitScheduler, sampleTime);
        case YawIntegrationMode::Always:
            return true;
    }
    return false;
}

double advanceYaw(const GaitScheduler* gaitScheduler,
                  const double currentYaw,
                  const double psiDot,
                  const double dt,
                  const double sampleTime,
                  const YawIntegrationMode mode) {
    if (!(dt > 0.0) || !shouldAdvanceYaw(gaitScheduler, sampleTime, mode)) {
        return currentYaw;
    }
    return currentYaw + psiDot * dt;
}

double yawRate(const GaitScheduler* gaitScheduler,
               const double psiDot,
               const double sampleTime,
               const YawIntegrationMode mode) {
    return shouldAdvanceYaw(gaitScheduler, sampleTime, mode) ? psiDot : 0.0;
}

Vec3<double> advancePlanarPosition(const Vec3<double>& position_W,
                                   const double bodyYaw_W,
                                   const Vec2<double>& planarCommand_B,
                                   const double dt) {
    if (!(dt > 0.0)) {
        return position_W;
    }

    const Vec3<double> command_W = Rz(bodyYaw_W) *
                                   Vec3<double>(planarCommand_B.x(), planarCommand_B.y(), 0.0);
    return position_W + command_W * dt;
}

Vec3<double> worldVelocity(const Vec3<double>& bodyCommand_B, const double bodyYaw_W) {
    return Rz(bodyYaw_W) * bodyCommand_B;
}

}  // namespace BodyMotionReference
