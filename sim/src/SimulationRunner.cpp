#include <array>
#include <chrono>
#include <iostream>
#include <thread>
#include <stdexcept>

#include <mujoco/mujoco.h>

#include "SimulationRunner.h"


void SimulationRunner::init() {
    if(_robot == RobotType::MIT_HUMANOID) _modelPath = "/models/mit_humanoid/scene.xml";
    else if(_robot == RobotType::UNITREE_G1) _modelPath = "/models/unitree_robots/g1/scene.xml";
    else if(_robot == RobotType::UNITREE_H1) _modelPath = "/models/unitree_robots/h1/scene.xml";
    else _modelPath = "/models/mit_humanoid/scene.xml";

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

    std::cout << "Loaded MuJoCo model: " << _modelPath << '\n';
	std::cout << "nq=" << model->nq
			  << ", nv=" << model->nv
			  << ", nu=" << model->nu << '\n';

}

void SimulationRunner::run() {
	_viewer.init(model);

	const auto wallStart = std::chrono::steady_clock::now();
	const double simStart = data->time;

	while (!_viewer.isEnabled() || !_viewer.shouldClose()) {
		runRobotControl();
		mj_step(model, data);
		++_iteration;
		_viewer.render(model, data);

		if (_iteration % 50 == 0) {
			std::cout << "t=" << data->time
					  << " q=" << data->qpos[0]
					  << " qdot=" << data->qvel[0]
					  << " ctrl=" << data->ctrl[0] << "\n";
		}

		const double simElapsed = data->time - simStart;
		const double wallElapsed =
			std::chrono::duration<double>(std::chrono::steady_clock::now() - wallStart).count();
		if (simElapsed > wallElapsed) {
			std::this_thread::sleep_for(std::chrono::duration<double>(simElapsed - wallElapsed));
		}
	}

	std::cout << "Simulated " << _iteration
			  << " steps, sim time=" << data->time << " sec" << '\n';

	_viewer.shutdown();

    mj_deleteData(data);
	mj_deleteModel(model);
	data = nullptr;
	model = nullptr;
}

void SimulationRunner::runRobotControl() {
    if(_firstControllerRun) {

    }
    _robotRunner->run();
}
