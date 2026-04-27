#ifndef GAIT_SWING_HOLD_CONTROLLER_H
#define GAIT_SWING_HOLD_CONTROLLER_H

#include <fstream>
#include <string>

#include "Controllers/ControlGains.h"
#include "ControllerConfig.h"
#include "GaitScheduler.h"
#include "HorizonClock.h"
#include "Robot/RobotParams.h"
#include "RobotController.h"
#include "StateEstimator/StateEstimator.h"
#include "SwingFootTrajectory.h"

class GaitSwingHoldController : public RobotController {
public:
    GaitSwingHoldController();
    ~GaitSwingHoldController() override = default;

    void bindRuntime(const StateEstimate<double>* stateEstimate,
                     const RobotParams<double>* robotParams,
                     LegController<double>* legController,
                     ArmController<double>* armController,
                     const UserCommand* userCommand);
    void setTouchdownTargetZOffset(double offset);
    void initializeController() override;
    void runController() override;
    void collectDebugVisualization(DebugVizState<double>& debugViz) const override;

private:
    struct LegRuntimeState {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        DVec<double> holdQ;
        SwingFootTrajectory swingTrajectory;
        Vec3<double> touchdownTarget_W = Vec3<double>::Zero();
        double touchdownYaw_W{0.0};
        bool wasInStance{true};
    };

    static Mat3<double> makeDiagonal(double x, double y, double z);
    static JointPdGains<double> makeInitialJointGains(RobotType robotType, Eigen::Index dof);

    void initializeRuntime();
    int findLegIndex(Side side) const;
    static const char* sideName(Side side);
    double touchdownTargetZ() const;
    Vec3<double> touchdownTargetWorld(std::size_t legIndex) const;
    void configureJointHold(int legIndex, const DVec<double>& qHold);
    void configureSwingLeg(int legIndex);
    void openSwingTrace();
    void logSwingTraceSample(int legIndex) const;
    void logHoldTraceMarker(int legIndex) const;
    void maybePrintStatus() const;

    bool _initialized{false};
    int _leftLegIndex{-1};
    int _rightLegIndex{-1};
    vectorAligned<LegRuntimeState> _legRuntime;
    std::unique_ptr<HorizonClock> _horizonClock;
    std::unique_ptr<GaitScheduler> _gaitScheduler;
    Vec3<double> _touchdownTarget_W = Vec3<double>::Zero();
    JointPdGains<double> _jointHoldGains;
    Vec3<double> _swingNaturalFrequency = Vec3<double>::Zero();
    Mat3<double> _swingKd = Mat3<double>::Zero();
    TouchdownTargetMode _touchdownTargetMode{TouchdownTargetMode::LegacyComYawCorrected};
    double _touchdownTargetZOffset{0.0};
    double _swingDuration{0.8};
    double _swingHeight{0.06};
    double _lastTime{0.0};
    std::string _tracePath;
    mutable std::ofstream _traceStream;
    u64 _traceSegmentId{0};
    u64 _iteration{0};
};

#endif  // GAIT_SWING_HOLD_CONTROLLER_H
