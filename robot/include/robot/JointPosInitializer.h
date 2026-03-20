#ifndef JOINT_POS_INITIALIZER_H
#define JOINT_POS_INITIALIZER_H

#include "Controllers/LegController.h"
#include "RobotModel.h"
#include "Utilities/BSplineBasic.h"

template <typename T>
class JointPosInitializer {
public:
    JointPosInitializer(const RobotParams<T>* params, T end_time, float dt);
    ~JointPosInitializer() = default;

    bool IsInitialized(LegController<T>*);

private:
    void initializeSpline(const LegController<T>& leg_ctrl);

    const RobotParams<T>* _params = nullptr;
    T _end_time{0};
    T _curr_time{0};
    T _dt{0};
    bool _splineInitialized{false};
    
    // TODO: MIThumanoid::num_act_joint is hard-coded for now. Generalize spline
    // dimension for other robots.
    BS_Basic<T, MIThumanoid::num_act_joint, 3, 1, 2, 2> _jpos_trj;
};


#endif  // JOINT_POS_INITIALIZER_H
