#include "JointTrackingConfig.h"

#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "RobotConfig.h"

namespace {
template <typename T>
void readVectorIfPresent(const YAML::Node& node, const char* key, std::vector<T>& values) {
    if (!node || !node[key]) {
        return;
    }

    const YAML::Node valuesNode = node[key];
    if (!valuesNode.IsSequence()) {
        throw std::runtime_error(std::string("Expected YAML sequence for key ") + key);
    }

    values.clear();
    values.reserve(valuesNode.size());
    for (std::size_t idx = 0; idx < valuesNode.size(); ++idx) {
        values.push_back(valuesNode[idx].as<T>());
    }
}

void readLimbJointGainsIfPresent(const YAML::Node& node,
                                 const char* limbName,
                                 std::vector<double>& kp,
                                 std::vector<double>& kd) {
    if (!node) {
        return;
    }

    const bool hasKp = node["kp"].IsDefined();
    const bool hasKd = node["kd"].IsDefined();
    if (hasKp != hasKd) {
        throw std::runtime_error(std::string("joint_tracking.") + limbName +
                                 " must provide both kp and kd");
    }
    if (!hasKp) {
        return;
    }

    readVectorIfPresent(node, "kp", kp);
    readVectorIfPresent(node, "kd", kd);

    if (kp.empty()) {
        throw std::runtime_error(std::string("joint_tracking.") + limbName +
                                 " kp/kd must not be empty");
    }
    if (kp.size() != kd.size()) {
        throw std::runtime_error(std::string("joint_tracking.") + limbName +
                                 " kp/kd length mismatch");
    }
}

JointTrackingConfig loadJointTrackingConfigFromYaml(const RobotType robotType) {
    JointTrackingConfig config;
    const std::string configPath = robotConfigPath(robotType);

    YAML::Node root;
    try {
        root = YAML::LoadFile(configPath);
    } catch (const YAML::Exception& exception) {
        throw std::runtime_error("Failed to load joint tracking config YAML from " + configPath +
                                 ": " + exception.what());
    }

    const YAML::Node jointTracking = root["joint_tracking"];
    readLimbJointGainsIfPresent(jointTracking["leg"], "leg", config.legKp, config.legKd);
    readLimbJointGainsIfPresent(jointTracking["arm"], "arm", config.armKp, config.armKd);
    return config;
}
}  // namespace

const JointTrackingConfig& getJointTrackingConfig() {
    return getJointTrackingConfig(activeRobotType());
}

const JointTrackingConfig& getJointTrackingConfig(const RobotType robotType) {
    static std::map<RobotType, JointTrackingConfig> configs;
    auto it = configs.find(robotType);
    if (it == configs.end()) {
        it = configs.emplace(robotType, loadJointTrackingConfigFromYaml(robotType)).first;
    }
    return it->second;
}
