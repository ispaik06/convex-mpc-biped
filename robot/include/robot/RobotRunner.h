#ifndef ROBOT_RUNNER_H
#define ROBOT_RUNNER_H

#include <memory>

#include "RobotController.h"
#include "RobotModel.h"
#include "RobotState.h"
#include "Controllers/ArmController.h"
#include "Controllers/LegController.h"
#include "JointPosInitializer.h"

class RobotRunner {
public:
    explicit RobotRunner(RobotController* robot_ctrl)
        : _robotController(robot_ctrl), _params(nullptr), _model(nullptr) {}

    void init(RobotParams<double>*);
    void setupStep(const RobotState<double>& state);
    void run(const RobotState<double>& state, RobotCommand<double>& command);
    void finalizeStep();

    virtual ~RobotRunner() = default;

    RobotController* _robotController;
    RobotType _robotType;

private:
    RobotParams<double>* _params = nullptr;
    RobotModel<double> _model;
    std::unique_ptr<LegController<double>> _legController;
    std::unique_ptr<ArmController<double>> _armController;
    std::unique_ptr<JointPosInitializer<double>> _jointPosInitializer;
    u64 _iterations = 0;
};


#endif  // ROBOT_RUNNER_H
