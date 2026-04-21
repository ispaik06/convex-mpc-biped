#ifndef MY_CONTROLLER_H
#define MY_CONTROLLER_H

#include <memory>
#include <vector>

#include "Controllers/ControlGains.h"
#include "ControlFSM.h"
#include "ConvexMPC.h"
#include "GaitScheduler.h"
#include "HorizonClock.h"
#include "MPCFormulation.h"
#include "RobotController.h"
#include "SwingFootTrajectory.h"

class MyController : public RobotController {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	MyController();
	virtual ~MyController() {}

	virtual void initializeController() override;

	virtual void runController() override;
	virtual void collectDebugVisualization(DebugVizState<double>& debugViz) const override;

private:
	struct BodyTargetState {
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		Vec3<double> position_W = Vec3<double>::Zero();
		Vec3<double> euler_W = Vec3<double>::Zero();
		bool initialized{false};
	};

	struct LegRuntimeState {
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		SwingFootTrajectory swingTrajectory;
		bool wasInStance{true};
	};

	static Mat3<double> makeDiagonal(double x, double y, double z);

	void initializeRuntimeObjects();
	int findLegIndex(Side side) const;
	Vec13<double> buildCurrentMpcState() const;
	void updateBodyTarget(const Vec13<double>& x0, double dt);
	void updateSwingTrajectories(const DesiredFootPositions& desiredFootPositions);
	void maybeUpdateMpc(const Vec13<double>& x0,
		                   const DesiredFootPositions& desiredFootPositions);
	void writeLegCommands();
	void writeStandingLegCommands();
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
	double _lastControlTime{0.0};
	Vec12<double> _stanceWrenchWorld = Vec12<double>::Zero();
	vectorAligned<LegRuntimeState> _legRuntime;
	std::unique_ptr<HorizonClock> _horizonClock;
	std::unique_ptr<GaitScheduler> _gaitScheduler;
	std::unique_ptr<ControlFSM> _controlFSM;
	std::unique_ptr<MPCFormulation> _mpcFormulation;
	std::unique_ptr<ConvexMPC> _convexMPC;
	ReferenceTrajectoryOutput _referenceTrajectoryOutput;
	MPCFormulationOutput _mpcFormulationOutput;
	Vec3<double> _swingNaturalFrequency = Vec3<double>::Zero();
	Mat3<double> _swingKd = Mat3<double>::Zero();
	double _swingHeight{0.0};
	u64 _iterationsBetweenMpc{10};
	LocomotionMode _locomotionMode{LocomotionMode::Walking};
	BodyTargetState _bodyTarget;

};

#endif  // MY_CONTROLLER_H
