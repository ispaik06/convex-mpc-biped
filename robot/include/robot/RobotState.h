#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

#include "Types.h"

template <typename T>
struct RobotState {
    double time{0.0};
    DVec<T> q;
    DVec<T> qd;
    DVec<T> tauEstimate;

    void resize(int nq, int nv, int nu) {
        q.resize(nq);
        qd.resize(nv);
        tauEstimate.resize(nu);
    }
};

template <typename T>
struct RobotCommand {
    DVec<T> tau;

    void resize(int nu) {
        tau.resize(nu);
    }
};



#endif  // ROBOT_STATE_H
