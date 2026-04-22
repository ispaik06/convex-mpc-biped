#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include <string>

#include "RobotController.h"
#include "Types.h"

struct RobotRuntimeConfig {
    std::string modelXmlPath;
    std::string auxiliaryModelXmlPath;
    FootEndEffectorSource footEndEffectorSource{FootEndEffectorSource::Site};
};

void setActiveRobotType(RobotType robotType);
RobotType activeRobotType();

std::string robotConfigPath(RobotType robotType);
std::string activeRobotConfigPath();
std::string resolveProjectPath(const std::string& path);

const RobotRuntimeConfig& getRobotRuntimeConfig(RobotType robotType);
const RobotRuntimeConfig& getActiveRobotRuntimeConfig();

#endif  // ROBOT_CONFIG_H
