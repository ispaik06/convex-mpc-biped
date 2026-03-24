#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include "Robot/RobotModel.h"


class RobotController {
public:
	RobotController(){}
	virtual ~RobotController(){}

	virtual void initializeController() = 0;

	virtual void runController() = 0;

private:
	RobotModel<double>* _robotModel = nullptr;

	RobotType _robotType;
};

#endif  // ROBOT_CONTROLLER_H
