#include "SimulationConfig.h"

#include <cmath>
#include <stdexcept>
#include <string>

#include <yaml-cpp/yaml.h>

#include "RobotConfig.h"

namespace {
std::string simulationConfigPath() {
	return resolveProjectPath("config/simulation.yaml");
}

bool isPositiveFinite(const double value) {
	return value > 0.0 && std::isfinite(value);
}

mjtIntegrator parseIntegrator(const YAML::Node& node, const std::string& configPath) {
	if (!node || !node.IsScalar()) {
		throw std::runtime_error("Simulation config " + configPath +
								 " must define physics_integrator");
	}

	const std::string value = node.as<std::string>();
	if (value == "euler") {
		return mjINT_EULER;
	}
	if (value == "rk4") {
		return mjINT_RK4;
	}
	if (value == "implicit") {
		return mjINT_IMPLICIT;
	}
	if (value == "implicitfast") {
		return mjINT_IMPLICITFAST;
	}

	throw std::runtime_error("Simulation config " + configPath +
							 " has unsupported physics_integrator '" + value +
							 "'. Expected euler, rk4, implicit, or implicitfast");
}

SimulationConfig loadSimulationConfigFromYaml() {
	const std::string configPath = simulationConfigPath();

	YAML::Node root;
	try {
		root = YAML::LoadFile(configPath);
	} catch (const YAML::Exception& exception) {
		throw std::runtime_error("Failed to load simulation config YAML from " + configPath +
								 ": " + exception.what());
	}

	const YAML::Node physicsTimestepSec = root["physics_timestep_sec"];
	if (!physicsTimestepSec || !physicsTimestepSec.IsScalar()) {
		throw std::runtime_error("Simulation config " + configPath +
								 " must define physics_timestep_sec");
	}

	const YAML::Node physicsIntegrator = root["physics_integrator"];
	if (!physicsIntegrator || !physicsIntegrator.IsScalar()) {
		throw std::runtime_error("Simulation config " + configPath +
								 " must define physics_integrator");
	}

	const YAML::Node viewerSyncHz = root["viewer_sync_hz"];
	if (!viewerSyncHz || !viewerSyncHz.IsScalar()) {
		throw std::runtime_error("Simulation config " + configPath +
								 " must define viewer_sync_hz");
	}

	SimulationConfig config;
	config.physicsTimestepSec = physicsTimestepSec.as<double>();
	if (!isPositiveFinite(config.physicsTimestepSec)) {
		throw std::runtime_error("Simulation config " + configPath +
								 " must define a finite, positive physics_timestep_sec");
	}
	config.physicsIntegrator = parseIntegrator(physicsIntegrator, configPath);
	config.viewerSyncHz = viewerSyncHz.as<double>();
	if (!isPositiveFinite(config.viewerSyncHz)) {
		throw std::runtime_error("Simulation config " + configPath +
								 " must define a finite, positive viewer_sync_hz");
	}

	return config;
}
}  // namespace

const SimulationConfig& getSimulationConfig() {
	static const SimulationConfig config = loadSimulationConfigFromYaml();
	return config;
}

void configureSimulationModel(mjModel* model) {
	if (model == nullptr) {
		throw std::runtime_error("configureSimulationModel received null model");
	}

	const auto& config = getSimulationConfig();
	model->opt.timestep = config.physicsTimestepSec;
	model->opt.integrator = static_cast<int>(config.physicsIntegrator);
}
