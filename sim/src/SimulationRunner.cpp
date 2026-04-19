#include <array>
#include <algorithm>
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <thread>

#include <mujoco/mujoco.h>

#include "MujocoCheaterStateReader.h"
#include "SimulationRunner.h"
#include "setupRobotParams.h"

void SimulationRunner::init() {
	_modelPath = "/models/mit_humanoid/scene.xml";

	_modelPath = std::string(PROJECT_ROOT_DIR) + _modelPath;

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

	mj_forward(model, data);

	model->opt.timestep = 0.002;
	model->opt.integrator = mjINT_IMPLICITFAST;

	std::cout << "Loaded MuJoCo model: " << _modelPath << '\n';
	std::cout << "nq=" << model->nq
			  << ", nv=" << model->nv
			  << ", nu=" << model->nu << '\n';
}

void SimulationRunner::run() {
	_keyboardCommand.start();

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
}

void SimulationRunner::runPhysicsLoop(bool throttleRealtime, bool syncViewer) {
	const auto wallStart = std::chrono::steady_clock::now();
	const double simStart = data->time;

	if (syncViewer) {
		_mainThread.load(model, data, _modelPath);
		_mainThread.sync();
	}

	while (!_stopRequested && (!syncViewer || !_mainThread.exitRequested())) {
		runRobotControl();
		mj_step(model, data);
		++_iterations;

		if (syncViewer) {
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

	std::cout << '\n' << "Simulated " << _iterations
			  << " steps, sim time=" << data->time << " sec" << "\n\n";
}

void SimulationRunner::runRobotControl() {
	if (_firstControllerRun) {

		const auto robotSetup = setupRobotParams<double>(_robot, model);
		_params = robotSetup.params;
		_bindings = robotSetup.bindings;
		_cheaterState.resize(_params);
		_stateEstimate.resize(_params);
		_legSwingDynamicsProvider =
			std::make_unique<LegSwingDynamicsProvider>(_robot, model, _params, _bindings);
		_robotRunner->init(&_params, model->opt.timestep, &_userCommand);
		_firstControllerRun = false;

		std::cout << model->opt.timestep << std::endl;
	}

	// 매 제어 주기마다 동적인 상체 자세(팔 스윙 등)를 반영하여 SRB 모델 파라미터 업데이트
	updateReducedBodyMassPropertiesFromData(model, data, _bindings, _params);

	fillCheaterState(model, data, _params, _bindings, _cheaterState);
	_stateEstimator.update(_cheaterState, _stateEstimate);
	if (_legSwingDynamicsProvider) {
		_legSwingDynamicsProvider->update(_stateEstimate);
	}
	_userCommand = _keyboardCommand.getUserCommand();
		// if ((_iterations % 50) == 0) {
		// 	std::cout << "[SimulationRunner] UserCommand | x_dot: " << _userCommand.x_dot
		// 			  << "  y_dot: " << _userCommand.y_dot
		// 			  << "  psi_dot: " << _userCommand.psi_dot << '\n';
		// }
	_robotRunner->run(_stateEstimate, _robotCommand);
	applyRobotCommand();
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
