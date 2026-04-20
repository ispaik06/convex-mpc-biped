#ifndef SETUP_ROBOT_PARAMS_H
#define SETUP_ROBOT_PARAMS_H

#include <mujoco/mjmodel.h>

#include "Types.h"
#include "MujocoRobotBindings.h"
#include "Robot/RobotParams.h"

template <typename T>
struct MujocoRobotSetup {
    RobotParams<T> params;
    MujocoRobotBindings bindings;
};

template <typename T>
MujocoRobotSetup<T> setupRobotParams(const RobotType, const mjModel_*, FootEndEffectorSource);

template <typename T>
void updateReducedBodyMassPropertiesFromData(const mjModel_* model,
                                             const mjData_* data,
                                             const MujocoRobotBindings& bindings,
                                             RobotParams<T>& params);


#endif  // SETUP_ROBOT_PARAMS_H
