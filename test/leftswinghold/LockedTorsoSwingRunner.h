#ifndef LOCKED_TORSO_SWING_RUNNER_H
#define LOCKED_TORSO_SWING_RUNNER_H

#include <array>
#include <atomic>
#include <memory>
#include <string>

#include "LegSwingDynamicsProvider.h"
#include "MujocoRobotBindings.h"
#include "RobotController.h"
#include "RobotRunner.h"
#include "Robot/RobotParams.h"
#include "SimulationIO.h"
#include "StateEstimator/StateEstimator.h"
#include "Types.h"
#include "main_thread.h"

struct mjModel_;
struct mjData_;
using mjModel = mjModel_;
using mjData = mjData_;

class LockedTorsoSwingRunner {
public:
    LockedTorsoSwingRunner(RobotType robotType, RobotController* controller, bool headless);
    ~LockedTorsoSwingRunner();

    void init();
    void run();

private:
    void runPhysicsLoop(bool throttleRealtime, bool syncViewer);
    void runRobotControl();
    void applyRobotCommand();
    void locateFloatingBase();
    void cacheLockedBasePose();
    void clampFloatingBase();

    RobotType _robotType;
    std::unique_ptr<RobotRunner> _robotRunner;
    bool _headless{true};
    bool _firstControllerRun{true};
    bool _torsoLocked{false};
    std::string _modelPath;
    RobotParams<double> _params;
    MujocoRobotBindings _bindings;
    CheaterState<double> _cheaterState;
    StateEstimate<double> _stateEstimate;
    UserCommand _userCommand;
    StateEstimator<double> _stateEstimator{StateEstimatorMode::Cheater};
    RobotCommand<double> _robotCommand;
    std::unique_ptr<LegSwingDynamicsProvider> _legSwingDynamicsProvider;
    u64 _iterations{0};
    MainThread _mainThread;
    mjModel* _model{nullptr};
    mjData* _data{nullptr};
    std::atomic<bool> _stopRequested{false};
    int _freeJointQposIndex{-1};
    int _freeJointQvelIndex{-1};
    std::array<double, 7> _lockedBaseQpos{};
};

#endif  // LOCKED_TORSO_SWING_RUNNER_H
