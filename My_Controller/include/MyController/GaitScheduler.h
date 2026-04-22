#ifndef GAIT_SCHEDULER_H
#define GAIT_SCHEDULER_H

#include "ControllerConfig.h"
#include "HorizonClock.h"
#include "Robot/RobotParams.h"

class GaitScheduler {
public:
    explicit GaitScheduler(const HorizonClock* horizonClock)
        : _horizonClock(horizonClock) {
        rebuildContactConstraintTemplate();
        Ck_bound = Vec24<double>::Zero();
    }

    void setLocomotionMode(LocomotionMode locomotionMode);
    void setFootLocalXAxesWorld(const Vec3<double>& leftFootXAxis_W,
                                const Vec3<double>& rightFootXAxis_W);

    double p(Side, double) const;
    bool c(Side, double) const;

    void buildConstraintMatrices();

    DMat<double> D;
    DMat<double> C;
    DVec<double> C_bound;

private:
    void rebuildContactConstraintTemplate();

    const HorizonClock* _horizonClock = nullptr;
    LocomotionMode _locomotionMode{LocomotionMode::Walking};
    Vec3<double> _leftFootXAxis_W = Vec3<double>::UnitX();
    Vec3<double> _rightFootXAxis_W = Vec3<double>::UnitX();
    Eigen::Matrix<double, 12, 6> C_unit;
    Vec24<double> Ck_bound;
};

#endif  // GAIT_SCHEDULER_H
