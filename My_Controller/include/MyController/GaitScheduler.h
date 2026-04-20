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
    Eigen::Matrix<double, 12, 6> C_unit;
    Vec24<double> Ck_bound;
};

#endif  // GAIT_SCHEDULER_H
