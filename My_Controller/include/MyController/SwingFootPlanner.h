#ifndef SWING_FOOT_PLANNER_H
#define SWING_FOOT_PLANNER_H

#include <vector>

#include "GaitScheduler.h"
#include "HorizonClock.h"
#include "StateEstimator/StateEstimator.h"
#include "Utilities/UserCommand.h"

struct DesiredFootPositions {
    Vec3<double> left_des_W = Vec3<double>::Zero();
    Vec3<double> right_des_W = Vec3<double>::Zero();
};

class SwingFootPlanner {
public:
    SwingFootPlanner(GaitScheduler* gaitScheduler,
                     HorizonClock* horizonClock,
                     const StateEstimate<double>* stateEstimate,
                     const RobotParams<double>* robotParams,
                     const UserCommand* userCommand)
        : _gaitScheduler(gaitScheduler),
          _horizonClock(horizonClock),
          _stateEstimate(stateEstimate),
          _robotParams(robotParams),
          _userCommand(userCommand) {}

    void reset();
    void setBodyTargetWorld(const Vec3<double>& position_W, double yaw_W);
    DesiredFootPositions desiredFootPositions();

private:
    void syncHorizonClock();
    void ensureSwingTouchdownCache();
    void ensureNominalFootOffsets();
    Vec2<double> currentPlanarCommandBodyFrame() const;
    double bodyYawTargetWorld() const;
    double touchdownPreviewTime() const;
    Vec3<double> currentFootTouchdownTarget(std::size_t legIndex) const;
    bool shouldApplyBrakingOffset(const Vec2<double>& currentPlanarCommand) const;
    Vec2<double> computeStopBrakingOffsetBodyFrame(const Vec2<double>& currentPlanarCommand) const;
    Vec2<double> stopBrakingOffsetBodyFrame(const Vec2<double>& currentPlanarCommand) const;
    Vec3<double> touchdownTargetWorldBodyVelocityHalfStance(std::size_t legIndex,
                                                            const Vec2<double>& currentPlanarCommand,
                                                            double* touchdownYaw_W_out = nullptr) const;
    void recordSequentialTouchdown(std::size_t legIndex,
                                   const Vec3<double>& target_W,
                                   double touchdownYaw_W);

    GaitScheduler* _gaitScheduler = nullptr;
    HorizonClock* _horizonClock = nullptr;
    const StateEstimate<double>* _stateEstimate = nullptr;
    const RobotParams<double>* _robotParams = nullptr;
    const UserCommand* _userCommand = nullptr;
    std::vector<Vec3<double>> _touchdownTargets;
    std::vector<bool> _touchdownTargetValid;
    std::vector<Vec3<double>> _nominalFootOffsets_B;
    std::vector<bool> _nominalFootOffsetValid;
    std::vector<bool> _wasInStance;
    Vec3<double> _footprintCenter_W = Vec3<double>::Zero();
    bool _footprintCenterValid{false};
    Vec3<double> _bodyPositionTarget_W = Vec3<double>::Zero();
    bool _bodyPositionTargetValid{false};
    double _bodyYawTarget_W{0.0};
    bool _bodyYawTargetValid{false};
    Vec2<double> _latchedBrakingOffset_B = Vec2<double>::Zero();
    bool _latchedBrakingOffsetValid{false};
    int _brakingOffsetDeadbandHoldTicks{0};
    Vec2<double> _previousPlanarCommand_B = Vec2<double>::Zero();
    bool _previousPlanarCommandValid{false};
    bool _previousBrakingOffsetActive{false};
};

#endif  // SWING_FOOT_PLANNER_H
