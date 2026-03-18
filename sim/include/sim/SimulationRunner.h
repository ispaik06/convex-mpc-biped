#ifndef SIMULATION_RUNNER_H
#define SIMULATION_RUNNER_H

#include <string>

#include "RobotController.h"
#include "RobotRunner.h"
#include "SimViewer.h"
#include "Types.h"

struct mjModel_;
struct mjData_;

class SimulationRunner {
public:
	explicit SimulationRunner(RobotType robot, RobotController* ctrl, bool headless) :
	_robot(robot), _headless(headless) {
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
	SimViewer _viewer;
	mjModel_* model = nullptr;
	mjData_* data = nullptr;
	bool _headless = true;
};

#endif  // SIMULATION_RUNNER_H
