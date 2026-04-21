#ifndef SIMULATION_RUNNER_H
#define SIMULATION_RUNNER_H

#include <atomic>
#include <string>
#include <vector>

#include "DebugVisualization.h"
#include "StateEstimator/StateEstimator.h"
#include "LegSwingDynamicsProvider.h"
#include "MujocoRobotBindings.h"
#include "main_thread.h"
#include "RobotController.h"
#include "RobotRunner.h"
#include "Robot/RobotParams.h"
#include "SimulationIO.h"
#include "Types.h"
#include "Utilities/KeyboardCommand.h"

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
	void updateDebugVisualization();

	struct DebugMocapBinding {
		std::string name;
		int bodyId{-1};
		int mocapId{-1};
	};

	RobotType _robot;
	RobotParams<double> _params;
	MujocoRobotBindings _bindings;
	CheaterState<double> _cheaterState;
	StateEstimate<double> _stateEstimate;
	UserCommand _userCommand;
	StateEstimator<double> _stateEstimator{StateEstimatorMode::Cheater};
	RobotCommand<double> _robotCommand;
	std::unique_ptr<RobotRunner> _robotRunner = nullptr;
	std::unique_ptr<LegSwingDynamicsProvider> _legSwingDynamicsProvider = nullptr;
	std::vector<DebugMocapBinding> _debugMocapBindings;
	bool _firstControllerRun = true;
	std::string _modelPath;
	u64 _iterations = 0;
	MainThread _mainThread;
	mjModel* model = nullptr;
	mjData* data = nullptr;
	bool _headless = true;
	std::atomic<bool> _stopRequested = false;
	KeyboardCommand _keyboardCommand;
};

#endif  // SIMULATION_RUNNER_H
