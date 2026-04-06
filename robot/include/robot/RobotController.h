#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include "Robot/RobotModel.h"
#include "Utilities/UserCommand.h"

class RobotRunner;

class RobotController {
public:
	RobotController(){}
	virtual ~RobotController(){}

	virtual void initializeController() = 0;

	virtual void runController() = 0;

private:
	friend class RobotRunner;

	RobotModel<double>* _robotModel = nullptr;

	RobotType _robotType;

protected:
	const UserCommand* _userCommand = nullptr;
};

#endif  // ROBOT_CONTROLLER_H
