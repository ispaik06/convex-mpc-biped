#ifndef SETUP_ROBOT_PARAMS_H
#define SETUP_ROBOT_PARAMS_H

#include <mujoco/mjmodel.h>

#include "Types.h"
#include "RobotParams.h"

template <typename T>
RobotParams<T> setupRobotParams(const RobotType, const mjModel_*);


#endif  // SETUP_ROBOT_PARAMS_H