#include <array>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <thread>

#include <mujoco/mujoco.h>

#include "ControllerConfig.h"
#include "MujocoCheaterStateReader.h"
#include "RobotConfig.h"
#include "SimulationConfig.h"
#include "SimulationRunner.h"
#include "Utilities/MatrixUtils.h"
#include "Utilities/Timing.h"
#include "ViewerSyncThrottle.h"
#include "setupRobotParams.h"

namespace {

bool standingKeyboardControlsFromRequest(const LegDynamicsRequest& request) {
	return request.standingFootJacobians;
}

Vec2<double> rollPitchFromQuaternion(Quat<double> quat) {
	quat.normalize();
	const double w = quat.w();
	const double x = quat.x();
	const double y = quat.y();
	const double z = quat.z();

	Vec2<double> rollPitch = Vec2<double>::Zero();
	rollPitch[0] = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
	const double sinPitch = std::clamp(2.0 * (w * y - z * x), -1.0, 1.0);
	rollPitch[1] = std::asin(sinPitch);
	return rollPitch;
}

int legControlModeCode(const LegControlMode mode) {
	switch (mode) {
		case LegControlMode::JointPd:
			return 0;
		case LegControlMode::JointTorque:
			return 1;
		case LegControlMode::SwingFoot:
			return 2;
		case LegControlMode::StanceWrench:
			return 3;
	}
	return -1;
}

void writeVec3Csv(std::ostream& out, const Vec3<double>& value) {
	out << value.x() << ',' << value.y() << ',' << value.z();
}

const ControllerContactDebugLegState* contactDebugForSide(
	const std::vector<ControllerContactDebugLegState>& states,
	const Side side) {
	for (const auto& state : states) {
		if (state.side == side) {
			return &state;
		}
	}
	return nullptr;
}

std::filesystem::path defaultHeadlessTelemetryPath() {
	return std::filesystem::path("logs") / "debug" / "headless_telemetry" /
	       "walking_telemetry.csv";
}

void applyMarkerColor(mjModel* model, const int bodyId, const DebugVizMarker<double>& marker) {
	if (model == nullptr || bodyId < 0 || !marker.hasRgba) {
		return;
	}
	if (bodyId >= model->nbody) {
		return;
	}

	const int geomStart = model->body_geomadr[bodyId];
	const int geomCount = model->body_geomnum[bodyId];
	if (geomStart < 0 || geomCount <= 0) {
		return;
	}

	for (int geomOffset = 0; geomOffset < geomCount; ++geomOffset) {
		const int geomId = geomStart + geomOffset;
		if (geomId < 0 || geomId >= model->ngeom) {
			continue;
		}

		float* geomRgba = model->geom_rgba + 4 * geomId;
		for (int channel = 0; channel < 4; ++channel) {
			geomRgba[channel] = static_cast<float>(marker.rgba[channel]);
		}
	}
}

}  // namespace

void SimulationRunner::init() {
	setActiveRobotType(_robot);
	const auto& controllerConfig = getControllerConfig(_robot);
	_keyboardCommand.setWalkingLimits(controllerConfig.userCommandFilter.xDotMax,
	                                  controllerConfig.userCommandFilter.yDotMax,
	                                  controllerConfig.userCommandFilter.psiDotMax);
	const auto& runtimeConfig = getRobotRuntimeConfig(_robot);
	_modelPath = resolveProjectPath(runtimeConfig.modelXmlPath);
	_footEndEffectorSource = runtimeConfig.footEndEffectorSource;

	if (mjVERSION_HEADER != mj_version()) {
		throw std::runtime_error("MuJoCo header/library version mismatch");
	}

	std::array<char, 1024> error{};
	model = mj_loadXML(_modelPath.c_str(), nullptr, error.data(), error.size());
	if (!model) {
		throw std::runtime_error("mj_loadXML failed for " + _modelPath + ": " + error.data());
	}

	data = mj_makeData(model);
	if (!data) {
		mj_deleteModel(model);
		throw std::runtime_error("mj_makeData failed");
	}

	configureSimulationModel(model);
	mj_forward(model, data);
	_debugMocapBindings.clear();

	std::cout << "Loaded MuJoCo model: " << _modelPath << '\n';
	std::cout << "nq=" << model->nq
			  << ", nv=" << model->nv
			  << ", nu=" << model->nu << '\n';
}

void SimulationRunner::run() {
	if (_headless) {
		// Intentionally runs until the user interrupts it. Headless auto-stop
		// criteria are deferred because this target is used for manual checking.
		runPhysicsLoop(false, false);
	} else {
		_stopRequested = false;
		_mainThread.init();

		std::thread physicsThread(&SimulationRunner::runPhysicsLoop, this, true, true);
		_mainThread.run();

		_stopRequested = true;
		physicsThread.join();
	}

	_keyboardCommand.stop();

	mj_deleteData(data);
	mj_deleteModel(model);
	data = nullptr;
	model = nullptr;
	_debugMocapBindings.clear();
}

void SimulationRunner::runPhysicsLoop(bool throttleRealtime, bool syncViewer) {
	const auto wallStart = std::chrono::steady_clock::now();
	const double simStart = data->time;
	double headlessStopTime = 0.0;
	if (_headless) {
		const char* stopEnv = std::getenv("CONVEXMPC_HEADLESS_STOP_TIME");
		if (stopEnv != nullptr && std::string(stopEnv).size() > 0) {
			const double stopTime = std::atof(stopEnv);
			if (std::isfinite(stopTime) && stopTime > 0.0) {
				headlessStopTime = stopTime;
			}
		}
	}
	const auto& simulationConfig = getSimulationConfig();
	const auto viewerSyncPeriod =
		std::chrono::duration_cast<std::chrono::steady_clock::duration>(
			std::chrono::duration<double>(1.0 / simulationConfig.viewerSyncHz));
	std::chrono::steady_clock::time_point nextViewerSync = std::chrono::steady_clock::now();

	if (syncViewer) {
		_mainThread.load(model, data, _modelPath);
		_mainThread.sync();
		nextViewerSync = std::chrono::steady_clock::now() + viewerSyncPeriod;
	}

	while (!_stopRequested && (!syncViewer || !_mainThread.exitRequested())) {
		runRobotControl();
		{
			profiling::ScopedTimer timer(_mjStepTime);
			mj_step(model, data);
		}
		++_iterations;

		if (headlessStopTime > 0.0 && data->time - simStart >= headlessStopTime) {
			_stopRequested = true;
		}

		if (syncViewer && sim::shouldSyncViewer(nextViewerSync, viewerSyncPeriod)) {
			_mainThread.sync();
		}

		if (!throttleRealtime) {
			continue;
		}

		const double simElapsed = data->time - simStart;
		const double wallElapsed =
			std::chrono::duration<double>(std::chrono::steady_clock::now() - wallStart).count();
		if (simElapsed > wallElapsed) {
			std::this_thread::sleep_for(std::chrono::duration<double>(simElapsed - wallElapsed));
		}
	}

	if (profiling::enabled()) {
		std::cout << '\n' << "[Profile] timing summary" << '\n';
		if (_robotRunner != nullptr) {
			_robotRunner->printProfilingSummary(std::cout);
		}
		std::cout << profiling::formatTimingStats("mj_step", _mjStepTime) << '\n';
	}

	std::cout << '\n' << "Simulated " << _iterations
			  << " steps, sim time=" << data->time << " sec" << "\n\n";
}

void SimulationRunner::runRobotControl() {
	if (_firstControllerRun) {

		const auto robotSetup = setupRobotParams<double>(
			_robot,
			model,
			_footEndEffectorSource);
		_params = robotSetup.params;
		_bindings = robotSetup.bindings;
		_cheaterState.resize(_params);
		_stateEstimate.resize(_params);
		_legSwingDynamicsProvider =
			std::make_unique<LegSwingDynamicsProvider>(
				_robot, model, _params, _bindings, LegSwingDynamicsProviderMode::Lazy);
		_robotRunner->init(&_params, model->opt.timestep, &_userCommand);
		const bool standingControls =
			standingKeyboardControlsFromRequest(_robotRunner->legDynamicsRequest());
		_keyboardCommand.setStandingControls(standingControls);
		_keyboardCommand.start();
		_keyboardCommand.setStandingControls(standingControls, true);
		_firstControllerRun = false;

		std::cout << "[SimulationRunner] MuJoCo physics timestep (model->opt.timestep): "
		          << model->opt.timestep << " sec" << std::endl;
	}

	updateReducedBodyMassPropertiesFromData(model, data, _bindings, _params);

	fillCheaterState(model, data, _params, _bindings, _cheaterState);
	_stateEstimator.update(_cheaterState, _stateEstimate);
	_userCommand = _keyboardCommand.getUserCommand();
	_robotRunner->prepareController(_stateEstimate);
	_keyboardCommand.setStandingControls(
		standingKeyboardControlsFromRequest(_robotRunner->legDynamicsRequest()));
	_legSwingDynamicsProvider->update(_stateEstimate, _robotRunner->legDynamicsRequest());

	// if ((_iterations % 50) == 0) {
		// 	std::cout << "[SimulationRunner] UserCommand | x_dot: " << _userCommand.x_dot
		// 			  << "  y_dot: " << _userCommand.y_dot
		// 			  << "  psi_dot: " << _userCommand.psi_dot << '\n';
		// }
	_robotRunner->run(_stateEstimate, _robotCommand);
	applyFixedJointCommands();
	updateDebugVisualization();
	applyRobotCommand();
	writeHeadlessTelemetry();
}

void SimulationRunner::writeHeadlessTelemetry() {
	if (!_headlessTelemetryInitialized) {
		_headlessTelemetryInitialized = true;
		const char* telemetryEnv = std::getenv("CONVEXMPC_HEADLESS_TELEMETRY");
		if (!_headless || telemetryEnv == nullptr || std::string(telemetryEnv).empty() ||
		    std::string(telemetryEnv) == "0") {
			return;
		}

		const char* intervalEnv = std::getenv("CONVEXMPC_HEADLESS_TELEMETRY_DT");
		if (intervalEnv != nullptr && std::string(intervalEnv).size() > 0) {
			const double interval = std::atof(intervalEnv);
			if (std::isfinite(interval) && interval > 0.0) {
				_headlessTelemetryInterval = interval;
			}
		}

		std::filesystem::path path =
			std::string(telemetryEnv) == "1" ? defaultHeadlessTelemetryPath()
			                                 : std::filesystem::path(telemetryEnv);
		if (path.is_relative()) {
			path = std::filesystem::current_path() / path;
		}
		if (path.has_parent_path()) {
			std::filesystem::create_directories(path.parent_path());
		}

		_headlessTelemetryPath = path.string();
		_headlessTelemetryOut = std::make_unique<std::ofstream>(path, std::ios::out | std::ios::trunc);
		if (!_headlessTelemetryOut->is_open()) {
			std::cerr << "[HeadlessTelemetry] failed to open " << _headlessTelemetryPath
			          << std::endl;
			_headlessTelemetryOut.reset();
			return;
		}

		auto& out = *_headlessTelemetryOut;
		out << "time,com_x,com_y,com_z,roll,pitch,yaw,vx,vy,vz,wx,wy,wz,"
		       "target_x,target_y,target_z,target_roll,target_pitch,target_yaw,"
		       "gait_recovery_hold,gait_recovery_hold_time,"
		       "total_actual_fz,total_desired_fz,torque_norm,"
		       "total_actual_mx,total_actual_my,total_actual_mz,"
		       "total_desired_mx,total_desired_my,total_desired_mz";
		for (const char* prefix : {"left", "right"}) {
			out << ',' << prefix << "_mode"
			    << ',' << prefix << "_contact"
			    << ',' << prefix << "_cm_scheduled"
			    << ',' << prefix << "_cm_estimated"
			    << ',' << prefix << "_cm_active"
			    << ',' << prefix << "_cm_early"
			    << ',' << prefix << "_cm_late"
			    << ',' << prefix << "_cm_liftoff_hold"
			    << ',' << prefix << "_cm_search"
			    << ',' << prefix << "_cm_fail"
			    << ',' << prefix << "_cm_alpha"
			    << ',' << prefix << "_cm_late_time"
			    << ',' << prefix << "_cm_liftoff_hold_time"
			    << ',' << prefix << "_fn"
			    << ',' << prefix << "_foot_x"
			    << ',' << prefix << "_foot_y"
			    << ',' << prefix << "_foot_z"
			    << ',' << prefix << "_actual_fx"
			    << ',' << prefix << "_actual_fy"
			    << ',' << prefix << "_actual_fz"
			    << ',' << prefix << "_desired_fx"
			    << ',' << prefix << "_desired_fy"
			    << ',' << prefix << "_desired_fz"
			    << ',' << prefix << "_desired_mx"
			    << ',' << prefix << "_desired_my"
			    << ',' << prefix << "_desired_mz"
			    << ',' << prefix << "_force_error_norm";
		}
		out << '\n';
		std::cout << "[HeadlessTelemetry] writing " << _headlessTelemetryPath
		          << " dt=" << _headlessTelemetryInterval << " sec" << std::endl;
	}

	if (_headlessTelemetryOut == nullptr || data == nullptr ||
	    data->time + 1e-12 < _nextHeadlessTelemetryTime) {
		return;
	}
	_nextHeadlessTelemetryTime = data->time + _headlessTelemetryInterval;

	const LegController<double>* legController =
		_robotRunner != nullptr ? _robotRunner->legController() : nullptr;
	if (legController == nullptr || _stateEstimate.legs.size() < 2 ||
	    legController->commands.size() < 2) {
		return;
	}

	const Vec3<double> com_W =
		_stateEstimate.torsoPos_W + Rz(_stateEstimate.psi) * _params.bodyComLocation;
	const Vec2<double> rollPitch = rollPitchFromQuaternion(_stateEstimate.torsoQuat_W);
	const double totalActualFz =
		_stateEstimate.legs[0].contactForce_W.z() + _stateEstimate.legs[1].contactForce_W.z();

	Vec3<double> desiredForceLeft_W = Vec3<double>::Zero();
	Vec3<double> desiredForceRight_W = Vec3<double>::Zero();
	Vec3<double> desiredMomentLeft_W = Vec3<double>::Zero();
	Vec3<double> desiredMomentRight_W = Vec3<double>::Zero();
	if (legController->commands[0].mode == LegControlMode::StanceWrench) {
		desiredForceLeft_W = -legController->commands[0].forceFeedForward_W;
		desiredMomentLeft_W = -legController->commands[0].momentFeedForward_W;
	}
	if (legController->commands[1].mode == LegControlMode::StanceWrench) {
		desiredForceRight_W = -legController->commands[1].forceFeedForward_W;
		desiredMomentRight_W = -legController->commands[1].momentFeedForward_W;
	}
	const double totalDesiredFz = desiredForceLeft_W.z() + desiredForceRight_W.z();
	const Vec3<double> totalActualMoment_W =
		(_stateEstimate.legs[0].footPos_W - com_W).cross(_stateEstimate.legs[0].contactForce_W) +
		(_stateEstimate.legs[1].footPos_W - com_W).cross(_stateEstimate.legs[1].contactForce_W);
	const Vec3<double> totalDesiredMoment_W =
		(_stateEstimate.legs[0].footPos_W - com_W).cross(desiredForceLeft_W) +
		desiredMomentLeft_W +
		(_stateEstimate.legs[1].footPos_W - com_W).cross(desiredForceRight_W) +
		desiredMomentRight_W;
	const std::vector<ControllerContactDebugLegState> contactDebugStates =
		(_robotRunner != nullptr && _robotRunner->_robot_ctrl != nullptr)
			? _robotRunner->_robot_ctrl->contactDebugLegStates()
			: std::vector<ControllerContactDebugLegState>{};
	const ControllerBodyTargetDebugState bodyTargetDebug =
		(_robotRunner != nullptr && _robotRunner->_robot_ctrl != nullptr)
			? _robotRunner->_robot_ctrl->bodyTargetDebugState()
			: ControllerBodyTargetDebugState{};

	auto& out = *_headlessTelemetryOut;
	out << std::fixed << std::setprecision(6)
	    << _stateEstimate.time << ',';
	writeVec3Csv(out, com_W);
	out << ',' << rollPitch.x()
	    << ',' << rollPitch.y()
	    << ',' << _stateEstimate.psi
	    << ',';
	writeVec3Csv(out, _stateEstimate.torsoLinVel_W);
	out << ',';
	writeVec3Csv(out, _stateEstimate.torsoAngVel_W);
	out << ',';
	writeVec3Csv(out,
	             bodyTargetDebug.initialized ? bodyTargetDebug.position_W
	                                         : Vec3<double>::Constant(std::numeric_limits<double>::quiet_NaN()));
	out << ',';
	writeVec3Csv(out,
	             bodyTargetDebug.initialized ? bodyTargetDebug.euler_W
	                                         : Vec3<double>::Constant(std::numeric_limits<double>::quiet_NaN()));
	out << ',' << static_cast<int>(bodyTargetDebug.gaitRecoveryHoldActive)
	    << ',' << bodyTargetDebug.gaitRecoveryHoldTime
	    << ',' << totalActualFz
	    << ',' << totalDesiredFz
	    << ',' << _robotCommand.tau.norm()
	    << ',';
	writeVec3Csv(out, totalActualMoment_W);
	out << ',';
	writeVec3Csv(out, totalDesiredMoment_W);

	for (std::size_t leg = 0; leg < 2; ++leg) {
		const auto& state = _stateEstimate.legs[leg];
		const auto& command = legController->commands[leg];
		const Side side = leg == 0 ? Side::Left : Side::Right;
		const ControllerContactDebugLegState* contactDebug =
			contactDebugForSide(contactDebugStates, side);
		Vec3<double> desiredForce_W = Vec3<double>::Zero();
		Vec3<double> desiredMoment_W = Vec3<double>::Zero();
		if (command.mode == LegControlMode::StanceWrench) {
			desiredForce_W = -command.forceFeedForward_W;
			desiredMoment_W = -command.momentFeedForward_W;
		}
		const double forceErrorNorm = (desiredForce_W - state.contactForce_W).norm();

		out << ',' << legControlModeCode(command.mode)
		    << ',' << static_cast<int>(state.contact)
		    << ',' << (contactDebug != nullptr ? static_cast<int>(contactDebug->scheduledContact) : -1)
		    << ',' << (contactDebug != nullptr ? static_cast<int>(contactDebug->estimatedContact) : -1)
		    << ',' << (contactDebug != nullptr ? static_cast<int>(contactDebug->activeContact) : -1)
		    << ',' << (contactDebug != nullptr ? static_cast<int>(contactDebug->earlyContact) : -1)
		    << ',' << (contactDebug != nullptr ? static_cast<int>(contactDebug->lateContact) : -1)
		    << ',' << (contactDebug != nullptr ? static_cast<int>(contactDebug->liftoffHold) : -1)
		    << ',' << (contactDebug != nullptr ? static_cast<int>(contactDebug->searchModeActive) : -1)
		    << ',' << (contactDebug != nullptr ? static_cast<int>(contactDebug->recoveryFailure) : -1)
		    << ',' << (contactDebug != nullptr ? contactDebug->contactRampAlpha : -1.0)
		    << ',' << (contactDebug != nullptr ? contactDebug->lateContactTime : -1.0)
		    << ',' << (contactDebug != nullptr ? contactDebug->liftoffHoldTime : -1.0)
		    << ',' << state.contactNormalForce
		    << ',' << state.footPos_W.x()
		    << ',' << state.footPos_W.y()
		    << ',' << state.footPos_W.z()
		    << ',';
		writeVec3Csv(out, state.contactForce_W);
		out << ',';
		writeVec3Csv(out, desiredForce_W);
		out << ',';
		writeVec3Csv(out, desiredMoment_W);
		out << ',' << forceErrorNorm;
	}
	out << '\n';
}

void SimulationRunner::applyFixedJointCommands() {
	if (model == nullptr || data == nullptr || _robotCommand.tau.size() != model->nu) {
		return;
	}

	for (const auto& joint : _params.fixedJoints) {
		const double q = static_cast<double>(data->qpos[joint.q_idx]);
		const double qd = static_cast<double>(data->qvel[joint.qd_idx]);
		_robotCommand.tau[joint.actuator_idx] =
			joint.kp * (joint.qDefault - q) - joint.kd * qd;
	}
}

void SimulationRunner::updateDebugVisualization() {
	if (model == nullptr || data == nullptr || _robotRunner == nullptr || _robotRunner->_robot_ctrl == nullptr) {
		return;
	}

	DebugVizState<double> debugViz;
	_robotRunner->_robot_ctrl->collectDebugVisualization(debugViz);

	if (debugViz.markers.empty()) {
		return;
	}

	for (const auto& marker : debugViz.markers) {
		if (!marker.active) {
			continue;
		}

		auto it = std::find_if(_debugMocapBindings.begin(),
							   _debugMocapBindings.end(),
							   [&](const DebugMocapBinding& binding) {
								   return binding.name == marker.name;
							   });

		if (it == _debugMocapBindings.end()) {
			DebugMocapBinding binding;
			binding.name = marker.name;
			binding.bodyId = mj_name2id(model, mjOBJ_BODY, marker.name.c_str());
			if (binding.bodyId < 0) {
				continue;
			}

			binding.mocapId = model->body_mocapid[binding.bodyId];
			if (binding.mocapId < 0) {
				continue;
			}

			_debugMocapBindings.push_back(binding);
			it = std::prev(_debugMocapBindings.end());
		}

		applyMarkerColor(model, it->bodyId, marker);

		const Eigen::Index posOffset = static_cast<Eigen::Index>(3 * it->mocapId);
		data->mocap_pos[posOffset + 0] = marker.position_W[0];
		data->mocap_pos[posOffset + 1] = marker.position_W[1];
		data->mocap_pos[posOffset + 2] = marker.position_W[2];

		const Eigen::Index quatOffset = static_cast<Eigen::Index>(4 * it->mocapId);
		data->mocap_quat[quatOffset + 0] = marker.orientation_W.w();
		data->mocap_quat[quatOffset + 1] = marker.orientation_W.x();
		data->mocap_quat[quatOffset + 2] = marker.orientation_W.y();
		data->mocap_quat[quatOffset + 3] = marker.orientation_W.z();
	}
}

void SimulationRunner::applyRobotCommand() {
	if (_robotCommand.tau.size() != model->nu) {
		throw std::runtime_error("RobotCommand torque dimension does not match model->nu");
	}

	for (int i = 0; i < model->nu; ++i) {
		const double tau = _robotCommand.tau[i];
		if (model->actuator_ctrllimited[i]) {
			const double lo = static_cast<double>(model->actuator_ctrlrange[2 * i + 0]);
			const double hi = static_cast<double>(model->actuator_ctrlrange[2 * i + 1]);
			data->ctrl[i] = std::clamp(tau, lo, hi);
		} else {
			data->ctrl[i] = tau;
		}
	}
}
