#ifndef SIMULATION_RUNNER_H
#define SIMULATION_RUNNER_H

#include <atomic>
#include <string>

#include "main_thread.h"
#include "RobotController.h"
#include "RobotRunner.h"
#include "Types.h"

struct mjModel_;
struct mjData_;
using mjModel = mjModel_;
using mjData = mjData_;

class SimulationRunner {
public:
	explicit SimulationRunner(RobotType robot, RobotController* ctrl, bool headless) :
	_robot(robot), _headless(headless) {
		_robotRunner = new RobotRunner(ctrl);
	}

	void init();
	void run();
	void runRobotControl();
	void runPhysicsLoop(bool throttleRealtime, bool syncViewer);
	~SimulationRunner() {
		delete _robotRunner;
	}

private:
	RobotType _robot;
	RobotRunner* _robotRunner = nullptr;
	bool _firstControllerRun = true;
	std::string _modelPath;
	u64 _iteration = 0;
	MainThread _mainThread;
	mjModel* model = nullptr;
	mjData* data = nullptr;
	bool _headless = true;
	std::atomic<bool> _stopRequested = false;
};

#endif  // SIMULATION_RUNNER_H
