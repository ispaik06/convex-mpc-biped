#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

class RobotController {
public:
	RobotController() = default;
	virtual ~RobotController() = default;

private:
	RobotType _robot;
};

#endif  // ROBOT_CONTROLLER_H
