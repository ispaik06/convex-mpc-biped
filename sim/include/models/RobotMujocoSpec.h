#ifndef ROBOT_MUJOCO_SPEC_H
#define ROBOT_MUJOCO_SPEC_H

#include <string_view>
#include <stdexcept>
#include <vector>

#include "Robot/RobotParams.h"

struct JointActuatorSpec {
    std::string_view joint{};
    std::string_view actuator{};
};

struct LimbMujocoSpec {
    Side side = Side::Left;
    std::string_view endBody{};
    std::string_view endSite{};
    std::vector<JointActuatorSpec> joints;
};

struct RobotMujocoSpec {
    RobotType type = RobotType::MIT_HUMANOID;
    std::string_view baseBody{};
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
