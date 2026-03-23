#ifndef SETUP_ROBOT_PARAMS_H
#define SETUP_ROBOT_PARAMS_H

#include <mujoco/mjmodel.h>

#include "Types.h"
#include "MujocoRobotBindings.h"
#include "RobotParams.h"

template <typename T>
struct MujocoRobotSetup {
    RobotParams<T> params;
    MujocoRobotBindings bindings;
};

template <typename T>
MujocoRobotSetup<T> setupRobotParams(const RobotType, const mjModel_*);


#endif  // SETUP_ROBOT_PARAMS_H
