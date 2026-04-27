#ifndef LEFT_SWING_HOLD_CONTROLLER_H
#define LEFT_SWING_HOLD_CONTROLLER_H

#include <fstream>
#include <string>

#include "Controllers/ControlGains.h"
#include "ControllerConfig.h"
#include "Robot/RobotParams.h"
#include "RobotController.h"
#include "StateEstimator/StateEstimator.h"
#include "SwingFootTrajectory.h"

class LeftSwingHoldController : public RobotController {
public:
    LeftSwingHoldController();
    ~LeftSwingHoldController() override = default;

    void bindRuntime(const StateEstimate<double>* stateEstimate,
                     const RobotParams<double>* robotParams,
                     LegController<double>* legController,
                     ArmController<double>* armController,
                     const UserCommand* userCommand);
    void initializeController() override;
    void runController() override;
    void collectDebugVisualization(DebugVizState<double>& debugViz) const override;

private:
    enum class Phase {
        Swing,
        Hold,
    };

    static Mat3<double> makeDiagonal(double x, double y, double z);
    static JointPdGains<double> makeInitialJointGains(RobotType robotType, Eigen::Index dof);

    void initializeRuntime();
    int findLegIndex(Side side) const;
    const char* phaseName() const;
    void startSwingPhase();
    void startHoldPhase();
    Vec3<double> touchdownTargetWorld() const;
    void configureJointHold(int legIndex, const DVec<double>& qHold);
    void configureSwingLeg(int legIndex);
    void openSwingTrace();
    void logSwingTraceSample() const;
    void logHoldTraceMarker() const;
    void maybePrintStatus() const;

    bool _initialized{false};
    int _leftLegIndex{-1};
    int _rightLegIndex{-1};
    Phase _phase{Phase::Swing};
    DVec<double> _leftHoldQ;
    SwingFootTrajectory _leftSwingTrajectory;
    Vec3<double> _touchdownTarget_W = Vec3<double>::Zero();
    JointPdGains<double> _jointHoldGains;
    Vec3<double> _swingNaturalFrequency = Vec3<double>::Zero();
    Mat3<double> _swingKd = Mat3<double>::Zero();
    TouchdownTargetMode _touchdownTargetMode{TouchdownTargetMode::LegacyComYawCorrected};
    double _swingDuration{0.8};
    double _holdDuration{0.8};
    double _swingHeight{0.06};
    double _phaseElapsed{0.0};
    double _lastTime{0.0};
    std::string _tracePath;
    mutable std::ofstream _traceStream;
    u64 _traceSegmentId{0};
    u64 _iteration{0};
};

#endif  // LEFT_SWING_HOLD_CONTROLLER_H
