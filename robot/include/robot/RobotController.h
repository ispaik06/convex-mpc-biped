#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include "RobotModel.h"


class RobotController {
public:
	RobotController(){}
	virtual ~RobotController(){}

	virtual void initializeController();

	virtual void runController();

private:
	RobotModel<double>* _robotModel = nullptr;

	RobotType _robotType;
};

#endif  // ROBOT_CONTROLLER_H
