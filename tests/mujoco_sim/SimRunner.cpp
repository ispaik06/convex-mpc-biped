#include "SimuRunner.h"

#include <array>
#include <iostream>
#include <stdexcept>
#include <utility>

#include <mujoco/mujoco.h>

namespace {

std::string defaultModelPath() {
	return std::string(CONVEX_MPC_ROOT) + "/models/unitree_robots/h1/scene.xml";
}

}  // namespace

SimuRunner::SimulationRunner(std::string modelPath)
	: _modelPath(std::move(modelPath)) {}

void SimuRunner::setModelPath(std::string modelPath) {
	_modelPath = std::move(modelPath);
}

void SimuRunner::run() {
	if (mjVERSION_HEADER != mj_version()) {
		throw std::runtime_error("MuJoCo header/library version mismatch");
	}

	const std::string modelPath = _modelPath.empty() ? defaultModelPath() : _modelPath;
	std::array<char, 1024> error{};

	mjModel* model = mj_loadXML(modelPath.c_str(), nullptr, error.data(), error.size());
	if (!model) {
		throw std::runtime_error("mj_loadXML failed for " + modelPath + ": " + error.data());
	}

	mjData* data = mj_makeData(model);
	if (!data) {
		mj_deleteModel(model);
		throw std::runtime_error("mj_makeData failed");
	}

	std::cout << "Loaded MuJoCo model: " << modelPath << '\n';
	std::cout << "nq=" << model->nq
			  << ", nv=" << model->nv
			  << ", nu=" << model->nu << '\n';

	constexpr int kNumSteps = 1000;
	for (int i = 0; i < kNumSteps; ++i) {
		mj_step(model, data);
		++_iteration;
	}

	std::cout << "Simulated " << _iteration
			  << " steps, sim time=" << data->time << " sec" << '\n';

	mj_deleteData(data);
	mj_deleteModel(model);
}
