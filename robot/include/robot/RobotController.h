#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include "Robot/RobotModel.h"
#include "Utilities/UserCommand.h"

class RobotRunner;
template <typename T>
class LegController;
template <typename T>
class ArmController;
template <typename T>
struct StateEstimate;
template <typename T>
struct RobotParams;

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
	const StateEstimate<double>* _stateEstimate = nullptr;
	const RobotParams<double>* _robotParams = nullptr;
	LegController<double>* _legController = nullptr;
	ArmController<double>* _armController = nullptr;
};

#endif  // ROBOT_CONTROLLER_H
