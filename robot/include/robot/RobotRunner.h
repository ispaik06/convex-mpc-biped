#ifndef ROBOT_RUNNER_H
#define ROBOT_RUNNER_H

#include "RobotController.h"


class RobotRunner {
public:
    RobotRunner(RobotController*);

    void init();
    void run();
    void cleanup();

    virtual ~RobotRunner();

private:

};


#endif  // ROBOT_RUNNER_H