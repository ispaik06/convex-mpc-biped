#ifndef SIMULATION_RUNNER_H
#define SIMULATION_RUNNER_H

#include <atomic>
#include <string>

#include "Estimator/StateEstimator.h"
#include "main_thread.h"
#include "RobotController.h"
#include "RobotRunner.h"
#include "RobotParams.h"
#include "SimulationIO.h"
#include "Types.h"

struct mjModel_;
struct mjData_;
using mjModel = mjModel_;
using mjData = mjData_;

class SimulationRunner {
public:
	explicit SimulationRunner(RobotType robot, RobotController* ctrl, bool headless) :
	_robot(robot), _headless(headless) {
		_robotRunner = std::make_unique<RobotRunner>(ctrl);
	}

	void init();
	void run();
	void runRobotControl();
	void runPhysicsLoop(bool throttleRealtime, bool syncViewer);


private:
	void applyRobotCommand();

	RobotType _robot;
	RobotParams<double> _params;
	CheaterState<double> _cheaterState;
	StateEstimate<double> _stateEstimate;
	StateEstimator<double> _stateEstimator{StateEstimatorMode::Cheater};
	RobotCommand<double> _robotCommand;
	std::unique_ptr<RobotRunner> _robotRunner = nullptr;
	bool _firstControllerRun = true;
	std::string _modelPath;
	u64 _iterations = 0;
	MainThread _mainThread;
	mjModel* model = nullptr;
	mjData* data = nullptr;
	bool _headless = true;
	std::atomic<bool> _stopRequested = false;
};

#endif  // SIMULATION_RUNNER_H
