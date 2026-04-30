#ifndef CONTACT_MANAGER_H
#define CONTACT_MANAGER_H

#include <cstddef>
#include "ContactSchedule.h"
#include "ControllerConfig.h"
#include "GaitScheduler.h"
#include "ReferenceTrajectory.h"
#include "Robot/RobotParams.h"
#include "StateEstimator/StateEstimator.h"

struct ContactManagerLegState {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Side side{Side::Left};
    bool scheduledContact{true};
    bool estimatedContact{true};
    bool activeContact{true};
    bool earlyContact{false};
    bool lateContact{false};
    bool searchModeActive{false};
    bool recoveryFailure{false};
    double contactRampAlpha{1.0};
    double lateContactTime{0.0};
    double contactNormalForce{0.0};
    Vec3<double> frozenTouchdownPosition_W = Vec3<double>::Zero();
    Vec3<double> commandedFootTarget_W = Vec3<double>::Zero();
};

class ContactManager {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit ContactManager(const ContactManagerParameters& params);

    void reset(const StateEstimate<double>& stateEstimate,
               const RobotParams<double>& robotParams,
               const GaitScheduler& gaitScheduler,
               LocomotionMode locomotionMode);

    void update(const StateEstimate<double>& stateEstimate,
                const RobotParams<double>& robotParams,
                const GaitScheduler& gaitScheduler,
                LocomotionMode locomotionMode,
                const DesiredFootPositions& nominalDesiredFootPositions);

    bool activeContact(Side side) const;
    bool scheduledContact(Side side) const;
    bool estimatedContact(Side side) const;
    bool searchModeActive(Side side) const;
    double contactRampAlpha(Side side) const;
    const ContactManagerLegState& legState(Side side) const;
    vectorAligned<ContactManagerLegState> legStates() const;

    DesiredFootPositions managedFootPositions(
        const DesiredFootPositions& nominalDesiredFootPositions) const;
    ContactScheduleOverride buildHorizonOverride() const;

private:
    struct RuntimeLegState {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        ContactManagerLegState publicState;
        bool initialized{false};
        bool rawContactEstimate{true};
        bool previousActiveContact{true};
        bool previousLateContact{false};
        bool releasedContactDuringSwing{false};
        int contactOnTicks{0};
        int contactOffTicks{0};
        double contactRampTime{0.0};
        double searchDepth{0.0};
    };

    RuntimeLegState& runtimeForSide(Side side);
    const RuntimeLegState& runtimeForSide(Side side) const;

    void ensureRuntimeLayout(const RobotParams<double>& robotParams);
    bool updateEstimatedContact(RuntimeLegState& runtime,
                                const RobotLegState<double>& legState) const;
    Vec3<double> nominalDesiredFootPosition(const DesiredFootPositions& desiredFootPositions,
                                            Side side) const;

    ContactManagerParameters _params;
    vectorAligned<RuntimeLegState> _legs;
    double _lastUpdateTime{0.0};
    bool _hasLastUpdateTime{false};
};

#endif  // CONTACT_MANAGER_H
