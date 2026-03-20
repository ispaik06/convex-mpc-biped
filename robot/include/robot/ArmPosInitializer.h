#ifndef ARM_POS_INITIALIZER_H
#define ARM_POS_INITIALIZER_H

#include "Controllers/ArmController.h"
#include "RobotModel.h"
#include "Utilities/BSplineBasic.h"

template <typename T>
class ArmPosInitializer {
public:
    ArmPosInitializer(const RobotParams<T>* params, T end_time, float dt);
    ~ArmPosInitializer() = default;

    bool IsInitialized(ArmController<T>*);

private:
    void initializeSpline(const ArmController<T>& arm_ctrl);

    const RobotParams<T>* _params = nullptr;
    T _end_time{0};
    T _curr_time{0};
    T _dt{0};
    bool _splineInitialized{false};

    // TODO: MIThumanoid::num_arm_joint is hard-coded for now. Generalize spline
    // dimension for other robots.
    BS_Basic<T, MIThumanoid::num_arm_joint * 2, 3, 1, 2, 2> _jpos_trj;
};

#endif  // ARM_POS_INITIALIZER_H
