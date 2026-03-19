#ifndef ROBOT_RUNNER_H
#define ROBOT_RUNNER_H

#include "RobotController.h"
#include "RobotModel.h"
#include "Controllers/LegController.h"

class RobotRunner {
public:
    explicit RobotRunner(RobotController* robot_ctrl)
        : _robotController(robot_ctrl), _model(&_params) {}

    void init();
    void setupStep();
    void run();
    void finalizeStep();

    virtual ~RobotRunner() = default;

    RobotController* _robotController;
    RobotType _robotType;
    RobotParams<double> _params;

private:
    RobotModel<double> _model;
    LegController<double>* _legController;
    u64 _iterations = 0;
};


#endif  // ROBOT_RUNNER_H
