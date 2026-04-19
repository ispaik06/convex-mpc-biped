#ifndef LEFT_SWING_HOLD_CONTROLLER_H
#define LEFT_SWING_HOLD_CONTROLLER_H

#include "Controllers/ControlGains.h"
#include "ControllerConfig.h"
#include "Robot/RobotParams.h"
#include "RobotController.h"
#include "StateEstimator/StateEstimator.h"
#include "SwingFootTrajectory.h"

class LeftSwingHoldController : public RobotController {
public:
    LeftSwingHoldController() = default;
    ~LeftSwingHoldController() override = default;

    void initializeController() override;
    void runController() override;

private:
    enum class Phase {
        Swing,
        Hold,
    };

    static Mat3<double> makeDiagonal(double x, double y, double z);
    static JointPdGains<double> makeInitialJointGains(RobotType robotType, Eigen::Index dof);

    void initializeRuntime();
    int findLegIndex(Side side) const;
    void startSwingPhase();
    void startHoldPhase();
    void configureJointHold(int legIndex, const DVec<double>& qHold);
    void configureSwingLeg(int legIndex);
    void maybePrintStatus() const;

    bool _initialized{false};
    int _leftLegIndex{-1};
    int _rightLegIndex{-1};
    Phase _phase{Phase::Swing};
    DVec<double> _leftHoldQ;
    DVec<double> _rightHoldQ;
    Vec3<double> _leftHoldFootWorld = Vec3<double>::Zero();
    SwingFootTrajectory _leftSwingTrajectory;
    JointPdGains<double> _jointHoldGains;
    Mat3<double> _swingKp = Mat3<double>::Zero();
    Mat3<double> _swingKd = Mat3<double>::Zero();
    double _swingDuration{0.8};
    double _holdDuration{0.8};
    double _swingHeight{0.06};
    double _phaseElapsed{0.0};
    double _lastTime{0.0};
    u64 _iteration{0};
};

#endif  // LEFT_SWING_HOLD_CONTROLLER_H
