#ifndef ROBOT_RUNNER_H
#define ROBOT_RUNNER_H

#include "RobotController.h"
#include "RobotModel.h"

class RobotRunner {
public:
    RobotRunner(RobotController*);

    void init();
    void run();
    void cleanup();

    virtual ~RobotRunner();

private:
    u64 _iterations = 0;
};


#endif  // ROBOT_RUNNER_H