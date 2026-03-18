#ifndef SIMULATION_RUNNER_H
#define SIMULATION_RUNNER_H

#include <string>
#include <mujoco/mujoco.h>

#include "RobotController.h"
#include "RobotRunner.h"
#include "Types.h"

class SimulationRunner {
public:
	explicit SimulationRunner(RobotType robot, RobotController* ctrl) :
	_robot(robot) {
		_robotRunner = new RobotRunner(ctrl);
	}

	void init();
	void run();
	void runRobotControl();
	~SimulationRunner() {
		delete _robotRunner;
	}

private:
	RobotType _robot;
	RobotRunner* _robotRunner = nullptr;
	bool _firstControllerRun = true;
	std::string _modelPath;
	u64 _iteration = 0;
	mjModel* model = nullptr;
	mjData* data = nullptr;

};

#endif  // SIMULATION_RUNNER_H
