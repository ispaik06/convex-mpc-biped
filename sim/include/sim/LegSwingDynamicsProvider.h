#ifndef LEG_SWING_DYNAMICS_PROVIDER_H
#define LEG_SWING_DYNAMICS_PROVIDER_H

#include <string>
#include <vector>

#include <mujoco/mujoco.h>

#include "MujocoRobotBindings.h"
#include "StateEstimator/StateEstimator.h"
#include "Types.h"

struct mjModel_;
struct mjData_;
using mjModel = mjModel_;
using mjData = mjData_;

class LegSwingDynamicsProvider {
public:
    LegSwingDynamicsProvider(RobotType robotType,
                             const mjModel* fullModel,
                             const RobotParams<double>& params,
                             const MujocoRobotBindings& bindings);
    ~LegSwingDynamicsProvider();

    LegSwingDynamicsProvider(const LegSwingDynamicsProvider&) = delete;
    LegSwingDynamicsProvider& operator=(const LegSwingDynamicsProvider&) = delete;

    void update(StateEstimate<double>& stateEstimate);

private:
    struct AuxiliaryLegModel {
        mjModel* model{nullptr};
        mjData* data{nullptr};
        int torsoBodyId{-1};
        int footBodyId{-1};
        int footSiteId{-1};
        FootEndEffectorSource footSource{FootEndEffectorSource::Site};
        std::vector<int> qposIndex;
        std::vector<int> qvelIndex;
        std::vector<mjtNum> jacpScratch;
        std::vector<mjtNum> jacrScratch;
        std::vector<mjtNum> jacDotpScratch;
        std::vector<mjtNum> denseMassScratch;
    };

    struct StandingAuxiliaryModel {
        mjModel* model{nullptr};
        mjData* data{nullptr};
        int torsoBodyId{-1};
        FootEndEffectorSource footSource{FootEndEffectorSource::Site};
        std::vector<int> footBodyIds;
        std::vector<int> footSiteIds;
        std::vector<std::size_t> footRowLegIndices;
        std::vector<std::vector<int>> qposIndexByLeg;
        std::vector<std::vector<int>> qvelIndexByLeg;
        std::vector<int> combinedQvelIndex;
        std::vector<mjtNum> jacpScratch;
        std::vector<mjtNum> jacrScratch;
    };

    static std::string robotXmlPath(RobotType robotType);
    static void destroy(AuxiliaryLegModel& auxModel);
    static void destroy(StandingAuxiliaryModel& auxModel);

    std::vector<AuxiliaryLegModel> _auxiliaryLegModels;
    StandingAuxiliaryModel _standingAuxiliaryModel;
};

#endif  // LEG_SWING_DYNAMICS_PROVIDER_H
