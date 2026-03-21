#ifndef MUJOCO_CHEATER_STATE_READER_H
#define MUJOCO_CHEATER_STATE_READER_H

#include "RobotParams.h"
#include "SimulationIO.h"

struct mjModel_;
struct mjData_;
using mjModel = mjModel_;
using mjData = mjData_;

void fillCheaterState(const mjModel* model,
                      const mjData* data,
                      const RobotParams<double>& params,
                      CheaterState<double>& cheater_state);

#endif  // MUJOCO_CHEATER_STATE_READER_H
