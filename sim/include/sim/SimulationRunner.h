#ifndef SIMULATION_RUNNER_H
#define SIMULATION_RUNNER_H

#include <atomic>
#include <fstream>
#include <iosfwd>
#include <memory>
#include <string>
#include <vector>

#include "DebugVisualization.h"
#include "StateEstimator/StateEstimator.h"
#include "SharedMemoryPublisher.h"
#include "SharedMemoryTelemetryPublisher.h"
#include "LegSwingDynamicsProvider.h"
#include "MujocoRobotBindings.h"
#include "main_thread.h"
#include "RobotController.h"
#include "RobotRunner.h"
#include "Robot/RobotParams.h"
#include "SimulationIO.h"
#include "Types.h"
#include "Utilities/KeyboardCommand.h"
#include "Utilities/Timing.h"

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

	~SimulationRunner();

	void init();
	void run();
	void runRobotControl();
	void runPhysicsLoop(bool throttleRealtime, bool syncViewer);


private:
	void applyFixedJointCommands();
	void applyRobotCommand();
	void updateDebugVisualization();
	bool maybeStartKeyboardCommand(double simTime);
	void applyHeadlessUserCommandSchedule(double simTime);
	void writeHeadlessTelemetry();

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
	std::unique_ptr<DashboardSharedMemoryPublisher> _dashboardPublisher = nullptr;
    std::unique_ptr<SharedMemoryTelemetryPublisher> _telemetryPublisher = nullptr;
	std::vector<DebugMocapBinding> _debugMocapBindings;
	bool _firstControllerRun = true;
	bool _keyboardCommandStartAttempted{false};
	double _keyboardInputEnableTime{0.0};
	std::string _modelPath;
	FootEndEffectorSource _footEndEffectorSource{FootEndEffectorSource::Site};
	u64 _iterations = 0;
	MainThread _mainThread;
	mjModel* model = nullptr;
	mjData* data = nullptr;
    bool _headless = true;
    std::atomic<bool> _stopRequested = false;
    KeyboardCommand _keyboardCommand;
    std::unique_ptr<std::ofstream> _headlessTelemetryOut;
    std::string _headlessTelemetryPath;
    double _headlessTelemetryInterval{0.02};
    double _nextHeadlessTelemetryTime{0.0};
    bool _headlessTelemetryInitialized{false};
    double _telemetryInterval{0.05};
    double _nextTelemetryTime{0.0};
    bool _headlessScheduleAnnounced{false};
    profiling::TimingStats _mjStepTime;
};

#endif  // SIMULATION_RUNNER_H
