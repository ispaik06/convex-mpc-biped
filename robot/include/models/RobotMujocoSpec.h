#ifndef ROBOT_MUJOCO_SPEC_H
#define ROBOT_MUJOCO_SPEC_H

#include <stdexcept>
#include <vector>

#include "RobotParams.h"

struct JointActuatorSpec {
    const char* joint = "";
    const char* actuator = "";
};

struct LimbMujocoSpec {
    Side side = Side::Left;
    const char* endBody = "";
    const char* endSite = "";
    std::vector<JointActuatorSpec> joints;
};

struct RobotMujocoSpec {
    RobotType type = RobotType::MIT_HUMANOID;
    const char* baseBody = "";
    std::vector<LimbMujocoSpec> legs;
    std::vector<LimbMujocoSpec> arms;
};

const RobotMujocoSpec& getMitHumanoidMujocoSpec();
const RobotMujocoSpec& getUnitreeG1_23MujocoSpec();
const RobotMujocoSpec& getUnitreeH1MujocoSpec();

inline const RobotMujocoSpec& getRobotMujocoSpec(RobotType type) {
    switch (type) {
        case RobotType::MIT_HUMANOID:
            return getMitHumanoidMujocoSpec();
        case RobotType::UNITREE_G1:
            return getUnitreeG1_23MujocoSpec();
        case RobotType::UNITREE_H1:
            return getUnitreeH1MujocoSpec();
        default:
            throw std::runtime_error("Unsupported RobotType in getRobotMujocoSpec");
    }
}

#endif  // ROBOT_MUJOCO_SPEC_H
