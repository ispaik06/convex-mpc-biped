#ifndef SIMULATION_CONFIG_H
#define SIMULATION_CONFIG_H

#include <mujoco/mjmodel.h>

struct SimulationConfig {
	double physicsTimestepSec;
	mjtIntegrator physicsIntegrator;
	double viewerSyncHz;
};

const SimulationConfig& getSimulationConfig();
void configureSimulationModel(mjModel* model);

#endif  // SIMULATION_CONFIG_H
