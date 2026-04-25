#include "InitialPoseConfig.h"

#include <cmath>
#include <map>
#include <stdexcept>
#include <string>

#include <yaml-cpp/yaml.h>

#include "RobotConfig.h"

namespace {
template <typename T>
void readOffsetVectorIfPresent(const YAML::Node& node,
                               const char* key,
                               std::vector<T>& values) {
    if (!node || !node[key]) {
        return;
    }

    const YAML::Node valuesNode = node[key];
    if (!valuesNode.IsSequence()) {
        throw std::runtime_error(std::string("Expected YAML sequence for key initial_pose.") + key);
    }

    values.clear();
    values.reserve(valuesNode.size());
    for (std::size_t idx = 0; idx < valuesNode.size(); ++idx) {
        values.push_back(valuesNode[idx].as<T>());
    }
}

template <typename T>
void readPositiveScalarIfPresent(const YAML::Node& node, const char* key, T& value) {
    if (!node || !node[key]) {
        return;
    }

    const T parsedValue = node[key].as<T>();
    if (!(parsedValue > T(0)) || !std::isfinite(static_cast<double>(parsedValue))) {
        throw std::runtime_error(std::string("initial_pose.") + key +
                                 " must be a finite, positive scalar");
    }

    value = parsedValue;
}

InitialPoseConfig loadInitialPoseConfigFromYaml(const RobotType robotType) {
    InitialPoseConfig config;
    const std::string configPath = robotConfigPath(robotType);

    YAML::Node root;
    try {
        root = YAML::LoadFile(configPath);
    } catch (const YAML::Exception& exception) {
        throw std::runtime_error("Failed to load initial pose config YAML from " + configPath +
                                 ": " + exception.what());
    }

    const YAML::Node initialPose = root["initial_pose"];
    readOffsetVectorIfPresent(initialPose, "leg_joint_offsets", config.legJointOffsets);
    readOffsetVectorIfPresent(initialPose, "arm_joint_offsets", config.armJointOffsets);
    readPositiveScalarIfPresent(initialPose, "leg_initialization_time",
                                config.legInitializationTime);
    readPositiveScalarIfPresent(initialPose, "arm_initialization_time",
                                config.armInitializationTime);

    return config;
}
}  // namespace

const InitialPoseConfig& getInitialPoseConfig() {
    return getInitialPoseConfig(activeRobotType());
}

const InitialPoseConfig& getInitialPoseConfig(const RobotType robotType) {
    static std::map<RobotType, InitialPoseConfig> configs;
    auto it = configs.find(robotType);
    if (it == configs.end()) {
        it = configs.emplace(robotType, loadInitialPoseConfigFromYaml(robotType)).first;
    }
    return it->second;
}
