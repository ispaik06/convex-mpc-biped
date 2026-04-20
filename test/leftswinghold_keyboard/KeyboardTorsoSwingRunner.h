#ifndef KEYBOARD_TORSO_SWING_RUNNER_H
#define KEYBOARD_TORSO_SWING_RUNNER_H

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
#include "Utilities/KeyboardCommand.h"
#include "main_thread.h"

struct mjModel_;
struct mjData_;
using mjModel = mjModel_;
using mjData = mjData_;

class KeyboardTorsoSwingRunner {
public:
    KeyboardTorsoSwingRunner(RobotType robotType, RobotController* controller, bool headless);
    ~KeyboardTorsoSwingRunner();

    void init();
    void run();

private:
    void runPhysicsLoop(bool throttleRealtime, bool syncViewer);
    void runRobotControl();
    void applyRobotCommand();
    void locateFloatingBase();
    void cachePlanarBasePose();
    void advancePlanarBasePose(double dt);
    void applyPlanarBasePose(const Vec3<double>& worldLinearVelocity,
                             const Vec3<double>& bodyAngularVelocity);

    RobotType _robotType;
    std::unique_ptr<RobotRunner> _robotRunner;
    bool _headless{true};
    bool _firstControllerRun{true};
    bool _planarMotionEnabled{false};
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
    Vec3<double> _planarBasePosition_W = Vec3<double>::Zero();
    Mat3<double> _planarRotationNoYaw = Mat3<double>::Identity();
    double _planarBaseYaw{0.0};
    double _planarBaseZ{0.0};
    KeyboardCommand _keyboardCommand;
};

#endif  // KEYBOARD_TORSO_SWING_RUNNER_H
