#ifndef FIXED_BASE_SWING_TEST_RUNNER_H
#define FIXED_BASE_SWING_TEST_RUNNER_H

#include <array>
#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include "LegSwingDynamicsProvider.h"
#include "MujocoRobotBindings.h"
#include "RobotController.h"
#include "RobotRunner.h"
#include "Robot/RobotParams.h"
#include "SimulationIO.h"
#include "StateEstimator/StateEstimator.h"
#include "Types.h"
#include "Utilities/KeyboardCommand.h"
#include "main_thread.h"

struct mjModel_;
struct mjData_;
using mjModel = mjModel_;
using mjData = mjData_;

class FixedBaseSwingTestRunner {
public:
    FixedBaseSwingTestRunner(RobotType robotType, RobotController* controller, bool headless);
    ~FixedBaseSwingTestRunner();

    void init();
    void run();

private:
    void runPhysicsLoop(bool throttleRealtime, bool syncViewer);
    void runRobotControl();
    void applyRobotCommand();
    void clampFloatingBase();
    void cacheFloatingBaseState();

    RobotType _robotType;
    RobotParams<double> _params;
    MujocoRobotBindings _bindings;
    CheaterState<double> _cheaterState;
    StateEstimate<double> _stateEstimate;
    StateEstimator<double> _stateEstimator{StateEstimatorMode::Cheater};
    UserCommand _userCommand;
    RobotCommand<double> _robotCommand;
    std::unique_ptr<RobotRunner> _robotRunner;
    std::unique_ptr<LegSwingDynamicsProvider> _legSwingDynamicsProvider;
    bool _firstControllerRun{true};
    std::string _modelPath;
    u64 _iterations{0};
    MainThread _mainThread;
    mjModel* _model{nullptr};
    mjData* _data{nullptr};
    bool _headless{true};
    std::atomic<bool> _stopRequested{false};
    KeyboardCommand _keyboardCommand;
    int _freeJointQposAdr{-1};
    int _freeJointQvelAdr{-1};
    std::array<double, 7> _fixedBaseQpos{};
};

#endif  // FIXED_BASE_SWING_TEST_RUNNER_H
