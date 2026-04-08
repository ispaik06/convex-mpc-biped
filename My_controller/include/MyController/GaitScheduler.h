#ifndef GAIT_SCHEDULER_H
#define GAIT_SCHEDULER_H

#include "ControlParameters.h"
#include "HorizonClock.h"
#include "Robot/RobotParams.h"

class GaitScheduler {
public:
    explicit GaitScheduler(const HorizonClock* horizonClock)
        : _horizonClock(horizonClock) {
        C_unit <<
             1, 0, -mu, 0, 0, 0,
            -1, 0, -mu, 0, 0, 0,
             0, 1, -mu, 0, 0, 0,
            0, -1, -mu, 0, 0, 0,
             0, 0,   1, 0, 0, 0,
             0, 0,  -1, 0, 0, 0,
             0, 0, -b, 1, 0, 0,
             0, 0, -b, -1, 0, 0,
             0, 0, -a, 0, 1, 0,
             0, 0, -a, 0, -1, 0,
             0, 0, -gamma*mu, 0, 0, 1,
             0, 0, -gamma*mu, 0, 0, -1;

        Ck_bound = Vec24<double>::Zero();


    }

    double p(Side, double);
    bool c(Side, double);

    void buildConstraintMatrices();

    DMat<double> D;
    DMat<double> C;
    DVec<double> C_bound;

private:
    const HorizonClock* _horizonClock = nullptr;
    Mat6<double> C_unit;
    Vec24<double> Ck_bound;
};

#endif  // GAIT_SCHEDULER_H
