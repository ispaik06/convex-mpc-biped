#ifndef MY_CONTROLLER_H
#define MY_CONTROLLER_H

#include "RobotController.h"

class MyController : public RobotController {
public:
	MyController() = default;
	virtual ~MyController() {}

	virtual void initializeController();

	virtual void runController();

private:
	// ControlFSM<double>* _controlFSM;
	// GaitScheduler<double>* _gaitScheduler;

};

#endif  // MY_CONTROLLER_H
