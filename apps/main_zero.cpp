#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <stdexcept>

#include <mujoco/mujoco.h>

#include "RobotConfig.h"
#include "SharedMemoryPublisher.h"
#include "SharedMemoryTelemetryPublisher.h"
#include "Utilities/Timing.h"
#include "Types.h"

namespace {

std::string robotTypeName(const RobotType type) {
	switch (type) {
		case RobotType::MIT_HUMANOID:
			return "MIT Humanoid";
		case RobotType::UNITREE_G1:
			return "Unitree G1";
		case RobotType::UNITREE_H1:
			return "Unitree H1";
	}
	return "Unknown";
}

double zeroCommandHz() {
	const char* hzEnv = std::getenv("CONVEXMPC_ZERO_COMMAND_HZ");
	if (hzEnv != nullptr && hzEnv[0] != '\0') {
		const double hz = std::atof(hzEnv);
		if (std::isfinite(hz) && hz > 0.0) {
			return hz;
		}
	}
	return 60.0;
}

void printUsage() {
	std::printf(
		"Usage: robot [robot-id] Viewer [y/n]\n"
		"\t where robot-id:    m for MIT humanoid\n"
		"\t zero-command build only supports MIT humanoid\n"
		"\t Viewer argument is accepted for launch-script compatibility\n");
}

}  // namespace

int main(int argc, char** argv) {
	profiling::configureFromEnvironment();

	if (argc < 2 || argc > 3) {
		printUsage();
		return EXIT_FAILURE;
	}

	if (!(argv[1][0] == 'm' || argv[1][0] == 'M')) {
		std::fprintf(stderr, "[zero-command] only MIT humanoid is supported\n");
		return EXIT_FAILURE;
	}
	if (argc == 3) {
		const char viewer = argv[2][0];
		if (!(viewer == 'y' || viewer == 'Y' || viewer == 'n' || viewer == 'N')) {
			printUsage();
			return EXIT_FAILURE;
		}
	}

	setActiveRobotType(RobotType::MIT_HUMANOID);
	const auto& runtimeConfig = getRobotRuntimeConfig(RobotType::MIT_HUMANOID);
	const std::string modelPath = resolveProjectPath(runtimeConfig.modelXmlPath);

	if (mjVERSION_HEADER != mj_version()) {
		throw std::runtime_error("MuJoCo header/library version mismatch");
	}

	std::array<char, 1024> error{};
	mjModel* model = mj_loadXML(modelPath.c_str(), nullptr, error.data(), error.size());
	if (model == nullptr) {
		throw std::runtime_error("mj_loadXML failed for " + modelPath + ": " + error.data());
	}

	std::cout << "Loaded MuJoCo model: " << modelPath << '\n';
	std::cout << "nq=" << model->nq
	          << ", nv=" << model->nv
	          << ", nu=" << model->nu << '\n';

	DashboardSharedMemoryPublisher dashboardPublisher(robotTypeName(RobotType::MIT_HUMANOID));
	SharedMemoryTelemetryPublisher viewerPublisher(
		robotTypeName(RobotType::MIT_HUMANOID),
		modelPath);

	const std::size_t qposCount = static_cast<std::size_t>(model->nq);
	const mjtNum* qpos0 = model->qpos0;
	std::array<double, DashboardSharedMemoryPublisher::kStateDim> dashboardState{};

	const double hz = zeroCommandHz();
	const auto period = std::chrono::duration<double>(1.0 / hz);
	auto nextTick = std::chrono::steady_clock::now();
	u64 iteration = 0;
	double simTime = 0.0;

	while (true) {
		viewerPublisher.publish(iteration, simTime, qpos0, qposCount);
		dashboardPublisher.publish(iteration, simTime, dashboardState);
		++iteration;
		simTime += period.count();
		nextTick += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
		std::this_thread::sleep_until(nextTick);
	}

	mj_deleteModel(model);
	return EXIT_SUCCESS;
}
