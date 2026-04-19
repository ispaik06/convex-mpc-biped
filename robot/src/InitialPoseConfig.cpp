#include "InitialPoseConfig.h"

#include <stdexcept>
#include <string>

#include <yaml-cpp/yaml.h>

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

InitialPoseConfig loadInitialPoseConfigFromYaml() {
    InitialPoseConfig config;
    const std::string configPath = std::string(PROJECT_ROOT_DIR) + "/config/my_controller.yaml";

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

    return config;
}
}  // namespace

const InitialPoseConfig& getInitialPoseConfig() {
    static const InitialPoseConfig config = loadInitialPoseConfigFromYaml();
    return config;
}
