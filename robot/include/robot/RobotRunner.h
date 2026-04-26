#ifndef ROBOT_RUNNER_H
#define ROBOT_RUNNER_H

#include <memory>

#include "StateEstimator/StateEstimator.h"
#include "RobotController.h"
#include "Robot/RobotModel.h"
#include "SimulationIO.h"
#include "ArmPosInitializer.h"
#include "Controllers/ArmController.h"
#include "Controllers/ControlGains.h"
#include "Controllers/LegController.h"
#include "LegPosInitializer.h"

class RobotRunner {
public:
    explicit RobotRunner(RobotController* robot_ctrl)
        : _robot_ctrl(robot_ctrl), _params(nullptr), _model(nullptr) {}

    void init(RobotParams<double>*, double, const UserCommand*);
    void prepareController(const StateEstimate<double>& state);
    LegDynamicsRequest legDynamicsRequest() const;
    void setupStep(const StateEstimate<double>& state);
    void composeCommand(RobotCommand<double>& command) const;
    void run(const StateEstimate<double>& state, RobotCommand<double>& command);
    void finalizeStep();
    bool legInitializationComplete() const;
    bool armInitializationComplete() const;
    bool initializationComplete() const;

    virtual ~RobotRunner() = default;

    RobotController* _robot_ctrl;
    RobotType _robotType;

private:
    void initializeJointTrackingGains();

    RobotParams<double>* _params = nullptr;
    RobotModel<double> _model;
    std::unique_ptr<LegController<double>> _legController;
    std::unique_ptr<ArmController<double>> _armController;
    std::unique_ptr<LegPosInitializer<double>> _legPosInitializer;
    std::unique_ptr<ArmPosInitializer<double>> _armPosInitializer;
    JointPdGains<double> _legJointTrackingGains;
    JointPdGains<double> _armJointTrackingGains;
    bool _legInitializationComplete = false;
    bool _armInitializationComplete = false;
    double _timestep;
    u64 _iterations = 0;
};


#endif  // ROBOT_RUNNER_H
