#ifndef BODY_MOTION_REFERENCE_H
#define BODY_MOTION_REFERENCE_H

#include "GaitScheduler.h"
#include "Utilities/MatrixUtils.h"

namespace BodyMotionReference {

bool shouldAdvanceYaw(const GaitScheduler* gaitScheduler, double sampleTime);
double advanceYaw(const GaitScheduler* gaitScheduler,
                  double currentYaw,
                  double psiDot,
                  double dt,
                  double sampleTime);
double yawRate(const GaitScheduler* gaitScheduler, double psiDot, double sampleTime);
Vec3<double> advancePlanarPosition(const Vec3<double>& position_W,
                                   double bodyYaw_W,
                                   const Vec2<double>& planarCommand_B,
                                   double dt);
Vec3<double> worldVelocity(const Vec3<double>& bodyCommand_B, double bodyYaw_W);

}  // namespace BodyMotionReference

#endif  // BODY_MOTION_REFERENCE_H
