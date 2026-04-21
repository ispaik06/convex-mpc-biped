#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include "Robot/RobotModel.h"
#include "DebugVisualization.h"
#include "Utilities/UserCommand.h"

enum class FootEndEffectorSource {
	BodyCom,
	Site,
};

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

	FootEndEffectorSource footEndEffectorSource() const { return _footEndEffectorSource; }

	virtual void initializeController() = 0;

	virtual void runController() = 0;

	virtual void collectDebugVisualization(DebugVizState<double>& debugViz) const {}

private:
	friend class RobotRunner;

	RobotModel<double>* _robotModel = nullptr;

	RobotType _robotType;

protected:
	void setFootEndEffectorSource(FootEndEffectorSource source) { _footEndEffectorSource = source; }

	const UserCommand* _userCommand = nullptr;
	const StateEstimate<double>* _stateEstimate = nullptr;
	const RobotParams<double>* _robotParams = nullptr;
	LegController<double>* _legController = nullptr;
	ArmController<double>* _armController = nullptr;
	FootEndEffectorSource _footEndEffectorSource{FootEndEffectorSource::Site};
};

#endif  // ROBOT_CONTROLLER_H
