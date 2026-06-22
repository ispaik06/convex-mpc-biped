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
    void seedTouchdownTargets(const DesiredFootPositions& desiredFootPositions);
    void setBodyYawTargetWorld(double yaw_W);
    DesiredFootPositions desiredFootPositions();

private:
    void syncHorizonClock();
    void ensureSwingTouchdownCache();
    void ensureNominalFootOffsets();
    Vec2<double> currentPlanarCommandBodyFrame() const;
    double selectedBodyVelocityHalfStanceOffset(const Vec2<double>& currentPlanarCommand) const;
    double bodyYawTargetWorld() const;
    double touchdownPreviewTime(const Vec2<double>& currentPlanarCommand) const;
    bool stopRecenterRequested(const Vec2<double>& currentPlanarCommand) const;
    bool stopRecenterActive(const Vec2<double>& currentPlanarCommand);
    Vec3<double> computeStopStanceCenterWorld() const;
    Vec3<double> currentFootTouchdownTarget(std::size_t legIndex) const;
    Vec3<double> touchdownTargetWorldBodyVelocityHalfStance(std::size_t legIndex,
                                                            const Vec2<double>& currentPlanarCommand,
                                                            bool stopRecenter) const;

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
    double _bodyYawTarget_W{0.0};
    bool _bodyYawTargetValid{false};
    int _stopRecenterClearTicks{0};
    bool _stopRecenterWasActive{false};
    Vec2<double> _previousPlanarCommand_B = Vec2<double>::Zero();
    double _previousYawRateCommand{0.0};
    bool _previousCommandValid{false};
    Vec3<double> _turnStopCenter_W = Vec3<double>::Zero();
    double _turnStopYaw_W{0.0};
    bool _turnStopFrameValid{false};
};

#endif  // SWING_FOOT_PLANNER_H
