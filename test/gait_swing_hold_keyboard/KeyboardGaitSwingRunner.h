#ifndef KEYBOARD_GAIT_SWING_RUNNER_H
#define KEYBOARD_GAIT_SWING_RUNNER_H

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include "Controllers/ArmController.h"
#include "Controllers/LegController.h"
#include "DebugVisualization.h"
#include "GaitSwingHoldController.h"
#include "LegSwingDynamicsProvider.h"
#include "MujocoRobotBindings.h"
#include "Robot/RobotModel.h"
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

class KeyboardGaitSwingRunner {
public:
    KeyboardGaitSwingRunner(RobotType robotType,
                             GaitSwingHoldController* controller,
                             bool headless,
                             double torsoZOffset = 0.0);
    ~KeyboardGaitSwingRunner();

    void init();
    void run();

private:
    void runPhysicsLoop(bool throttleRealtime, bool syncViewer);
    void runRobotControl();
    void applyRobotCommand();
    void updateDebugVisualization();
    void initializeControllerRuntime();
    void applyCopiedStateKeyframe(const std::string& keyframeName);
    void locateFloatingBase();
    void cacheFrozenQpos();
    void clampFrozenQpos();
    bool isLegQposIndex(int qposIndex) const;
    bool isLegQvelIndex(int qvelIndex) const;
    bool isFloatingBaseQposIndex(int qposIndex) const;
    bool isFloatingBaseQvelIndex(int qvelIndex) const;
    void cachePlanarBasePose();
    void advancePlanarBasePose(double dt);
    void applyPlanarBasePose(const Vec3<double>& worldLinearVelocity,
                             const Vec3<double>& bodyAngularVelocity);

    RobotType _robotType;
    GaitSwingHoldController* _controller{nullptr};
    std::unique_ptr<RobotModel<double>> _robotModel;
    std::unique_ptr<LegController<double>> _legController;
    std::unique_ptr<ArmController<double>> _armController;
    bool _headless{true};
    bool _firstControllerRun{true};
    bool _planarMotionEnabled{false};
    double _torsoZOffset{0.0};
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
    std::vector<int> _legQposIndices;
    std::vector<int> _legQvelIndices;
    std::vector<double> _frozenQpos;
    Vec3<double> _planarBasePosition_W = Vec3<double>::Zero();
    Mat3<double> _planarRotationNoYaw = Mat3<double>::Identity();
    double _planarBaseYaw{0.0};
    double _planarBaseZ{0.0};
    KeyboardCommand _keyboardCommand;

    struct DebugMocapBinding {
        std::string name;
        int bodyId{-1};
        int mocapId{-1};
    };
    std::vector<DebugMocapBinding> _debugMocapBindings;
};

#endif  // KEYBOARD_GAIT_SWING_RUNNER_H
