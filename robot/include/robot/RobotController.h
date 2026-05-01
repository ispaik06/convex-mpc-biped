#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include <vector>

#include "Robot/RobotModel.h"
#include "DebugVisualization.h"
#include "Utilities/UserCommand.h"

enum class FootEndEffectorSource {
	Site,
	CollisionGeomCenter,
};

struct LegDynamicsRequest {
	bool swingLegDynamics{true};
	bool standingFootJacobians{false};
};

struct ControllerContactDebugLegState {
	Side side{Side::Left};
	bool scheduledContact{true};
	bool estimatedContact{true};
	bool activeContact{true};
	bool earlyContact{false};
	bool lateContact{false};
	bool liftoffHold{false};
	bool searchModeActive{false};
	bool recoveryFailure{false};
	double contactRampAlpha{1.0};
    double lateContactTime{0.0};
	double liftoffHoldTime{0.0};
};

struct ControllerBodyTargetDebugState {
	Vec3<double> position_W = Vec3<double>::Zero();
	Vec3<double> euler_W = Vec3<double>::Zero();
	bool initialized{false};
	bool gaitRecoveryHoldActive{false};
	double gaitRecoveryHoldTime{0.0};
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
	virtual bool usesStandingOnlyLegDynamics() const { return false; }
	virtual void prepareController() {}
	virtual LegDynamicsRequest legDynamicsRequest() const { return {}; }

	virtual void initializeController() = 0;

	virtual void runController() = 0;

	virtual void collectDebugVisualization(DebugVizState<double>& debugViz) const {}
	virtual std::vector<ControllerContactDebugLegState> contactDebugLegStates() const { return {}; }
	virtual ControllerBodyTargetDebugState bodyTargetDebugState() const { return {}; }

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
