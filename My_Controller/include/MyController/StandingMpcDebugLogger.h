#ifndef STANDING_MPC_DEBUG_LOGGER_H
#define STANDING_MPC_DEBUG_LOGGER_H

#include <string>

#include "Controllers/ArmController.h"
#include "Controllers/LegController.h"
#include "MPCFormulation.h"
#include "ReferenceTrajectory.h"
#include "StateEstimator/StateEstimator.h"

struct StandingMpcDebugSnapshot {
    const StateEstimate<double>& stateEstimate;
    const RobotParams<double>& robotParams;
    const LegController<double>& legController;
    const ArmController<double>* armController{nullptr};
    const DesiredFootPositions& desiredFootPositions;
    const Vec13<double>& x0;
    const ReferenceTrajectoryOutput& referenceTrajectory;
    const MPCFormulationOutput& formulation;
    const DVec<double>& wrenchHorizon;
    u64 iteration{0};
    std::string debugRequestSource;
    double debugRequestTime{0.0};
    double debugTriggerTime{0.0};
};

std::string writeStandingMpcDebugLog(const StandingMpcDebugSnapshot& snapshot);

#endif  // STANDING_MPC_DEBUG_LOGGER_H
