#ifndef GAIT_SCHEDULER_H
#define GAIT_SCHEDULER_H

#include "Robot/RobotParams.h"

// dt_sim = model->opt.timestep (= 0.002)
// dt_ctrl = 0.002 (500Hz)
// dt_mpc = 0.02 (50Hz) : MPC period
// horizon N = 10 or 15
// T_horizon = 0.5
// T_swing = 0.6
// T_stance = 0.4
// T_cycle = 1.0

constexpr int T_cycle = 1.0;
constexpr double T_swing = 0.4;
constexpr double T_stance = 0.6;

constexpr double T_horizon = 0.5;
constexpr int N = 10; // or 15

// dry friction coefficient
constexpr double mu = 0.1;

// half-size of a rectangular foot
constexpr double a = 0.065;
constexpr double b = 0.01;

constexpr double gamma = 0.0657;

constexpr double Fmax = 200;
constexpr double Fmin = 10;


class GaitScheduler {
public:
    GaitScheduler() {
        _dt = T_horizon / N;
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

    void init(double);
    void buildConstraintMatrices();

    DMat<double> D;
    DMat<double> C;
    DVec<double> C_bound;

    double _t0;  // start time of a gait cycle

private:
    Mat6<double> C_unit;
    Vec24<double> Ck_bound;

    double _dt;  // timestep of a single step
};

#endif  // GAIT_SCHEDULER_H