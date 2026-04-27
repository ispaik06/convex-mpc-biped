#include "RobotConfig.h"

#include <map>
#include <stdexcept>
#include <string>

#include <yaml-cpp/yaml.h>

namespace {
RobotType& activeRobotTypeStorage() {
    static RobotType robotType = RobotType::MIT_HUMANOID;
    return robotType;
}

const char* robotConfigRelativePath(const RobotType robotType) {
    switch (robotType) {
        case RobotType::MIT_HUMANOID:
            return "config/mit_humanoid/my_controller.yaml";
        case RobotType::UNITREE_G1:
            return "config/unitree_robots/g1/my_controller.yaml";
        case RobotType::UNITREE_H1:
            return "config/unitree_robots/h1/my_controller.yaml";
    }

    throw std::runtime_error("Unsupported robot type for config path");
}

FootEndEffectorSource parseFootEndEffectorSource(const YAML::Node& node) {
    if (!node || !node.IsScalar()) {
        throw std::runtime_error(
            "Missing or invalid model.foot_end_effector_source. Expected site or collision_geom_center");
    }

    const std::string mode = node.as<std::string>();
    if (mode == "site") {
        return FootEndEffectorSource::Site;
    }
    if (mode == "collision_geom_center") {
        return FootEndEffectorSource::CollisionGeomCenter;
    }

    throw std::runtime_error(
        "Invalid model.foot_end_effector_source. Expected site or collision_geom_center");
}

RobotRuntimeConfig loadRobotRuntimeConfig(const RobotType robotType) {
    const std::string configPath = robotConfigPath(robotType);

    YAML::Node root;
    try {
        root = YAML::LoadFile(configPath);
    } catch (const YAML::Exception& exception) {
        throw std::runtime_error("Failed to load robot config YAML from " + configPath +
                                 ": " + exception.what());
    }

    const YAML::Node model = root["model"];
    if (!model || !model["xml_path"] || !model["xml_path"].IsScalar()) {
        throw std::runtime_error("Robot config " + configPath +
                                 " must define model.xml_path");
    }

    RobotRuntimeConfig config;
    config.modelXmlPath = model["xml_path"].as<std::string>();
    if (model["auxiliary_xml_path"] && model["auxiliary_xml_path"].IsScalar()) {
        config.auxiliaryModelXmlPath = model["auxiliary_xml_path"].as<std::string>();
    } else {
        config.auxiliaryModelXmlPath = config.modelXmlPath;
    }
    config.footEndEffectorSource =
        parseFootEndEffectorSource(model["foot_end_effector_source"]);

    return config;
}
}  // namespace

void setActiveRobotType(const RobotType robotType) {
    activeRobotTypeStorage() = robotType;
}

RobotType activeRobotType() {
    return activeRobotTypeStorage();
}

std::string robotConfigPath(const RobotType robotType) {
    return resolveProjectPath(robotConfigRelativePath(robotType));
}

std::string activeRobotConfigPath() {
    return robotConfigPath(activeRobotType());
}

std::string resolveProjectPath(const std::string& path) {
    if (path.empty()) {
        return path;
    }
    if (path.front() == '/') {
        return path;
    }
    return std::string(PROJECT_ROOT_DIR) + "/" + path;
}

const RobotRuntimeConfig& getRobotRuntimeConfig(const RobotType robotType) {
    static std::map<RobotType, RobotRuntimeConfig> configs;
    auto it = configs.find(robotType);
    if (it == configs.end()) {
        it = configs.emplace(robotType, loadRobotRuntimeConfig(robotType)).first;
    }
    return it->second;
}

const RobotRuntimeConfig& getActiveRobotRuntimeConfig() {
    return getRobotRuntimeConfig(activeRobotType());
}
