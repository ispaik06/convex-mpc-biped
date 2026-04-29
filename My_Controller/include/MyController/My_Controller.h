#ifndef MY_CONTROLLER_H
#define MY_CONTROLLER_H

#include <memory>
#include <string>
#include <vector>

#include "Controllers/ControlGains.h"
#include "ConvexMPC.h"
#include "GaitScheduler.h"
#include "HorizonClock.h"
#include "LocomotionFSM.h"
#include "MPCFormulation.h"
#include "RobotController.h"
#include "SwingFootPlanner.h"
#include "SwingFootTrajectory.h"

class MyController : public RobotController {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	MyController();
	virtual ~MyController() {}

	virtual void initializeController() override;
	virtual void prepareController() override;
	virtual LegDynamicsRequest legDynamicsRequest() const override;

	virtual void runController() override;
	virtual void collectDebugVisualization(DebugVizState<double>& debugViz) const override;
	virtual bool usesStandingOnlyLegDynamics() const override;

private:
	struct BodyTargetState {
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		Vec3<double> position_W = Vec3<double>::Zero();
		Vec3<double> euler_W = Vec3<double>::Zero();
		Vec3<double> eulerSeed_W = Vec3<double>::Zero();
		bool initialized{false};
	};

	struct LegRuntimeState {
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		SwingFootTrajectory swingTrajectory;
		double touchdownYaw_W{0.0};
		bool wasInStance{true};
	};

	static Mat3<double> makeDiagonal(double x, double y, double z);

	void initializeRuntimeObjects();
	int findLegIndex(Side side) const;
	Vec13<double> buildCurrentMpcState() const;
	void seedBodyTargetFromCurrentState();
	void resetSwingState();
	void applyLocomotionOutput(const LocomotionFSMOutput& output);
	LocomotionFSMOutput syncLocomotionFSM();
	void updateBodyTarget(const Vec13<double>& x0, double dt);
	void updateSwingTrajectories(const DesiredFootPositions& desiredFootPositions);
	void updateTouchdownDebugTarget(const DesiredFootPositions& desiredFootPositions);
	double swingFootYawTargetWorld() const;
	void maybeUpdateMpc(const Vec13<double>& x0,
		                   const DesiredFootPositions& desiredFootPositions);
	void writeLegCommands();
	void writeStandingLegCommands();
	void queueStandingMpcDebugLog(const std::string& source,
		                          double requestTime,
		                          double triggerTime);
	void updateStandingMpcDebugRequest();
	void maybeWriteStandingMpcDebugLog(const Vec13<double>& x0,
		                               const DesiredFootPositions& desiredFootPositions);
	void maybePrintGaitScheduler() const;

	bool _initialized{false};
	u64 _iteration{0};
	u64 _lastMpcIteration{0};
	bool _standingMpcDebugLogPending{false};
	bool _standingMpcDebugLogReady{false};
	unsigned long long _lastStandingMpcDebugLogRequest{0};
	std::size_t _nextStandingMpcDebugTriggerIndex{0};
	std::string _standingMpcDebugRequestSource;
	double _standingMpcDebugRequestTime{0.0};
	double _standingMpcDebugTriggerTime{0.0};
	double _lastControlTime{0.0};
	Vec12<double> _stanceWrenchWorld = Vec12<double>::Zero();
	vectorAligned<LegRuntimeState> _legRuntime;
	std::unique_ptr<HorizonClock> _horizonClock;
	std::unique_ptr<GaitScheduler> _gaitScheduler;
	std::unique_ptr<LocomotionFSM> _locomotionFSM;
	std::unique_ptr<SwingFootPlanner> _swingFootPlanner;
	std::unique_ptr<MPCFormulation> _mpcFormulation;
	std::unique_ptr<ConvexMPC> _convexMPC;
	ReferenceTrajectoryOutput _referenceTrajectoryOutput;
	MPCFormulationOutput _mpcFormulationOutput;
	Vec3<double> _swingNaturalFrequency = Vec3<double>::Zero();
	Mat3<double> _swingKd = Mat3<double>::Zero();
	double _swingHeight{0.0};
	u64 _iterationsBetweenMpc{10};
	LocomotionMode _locomotionMode{LocomotionMode::Walking};
	LegDynamicsRequest _legDynamicsRequest;
		BodyTargetState _bodyTarget;
		Vec3<double> _leftTouchdownTarget_W = Vec3<double>::Zero();
		Vec3<double> _rightTouchdownTarget_W = Vec3<double>::Zero();
		double _leftTouchdownTargetYaw_W{0.0};
		double _rightTouchdownTargetYaw_W{0.0};
		bool _leftTouchdownTargetInitialized{false};
		bool _rightTouchdownTargetInitialized{false};

	};

#endif  // MY_CONTROLLER_H
