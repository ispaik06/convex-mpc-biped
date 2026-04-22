#ifndef ROBOT_MUJOCO_SPEC_H
#define ROBOT_MUJOCO_SPEC_H

#include <string_view>
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

struct FixedJointMujocoSpec {
    std::string_view joint{};
    std::string_view actuator{};
    double kp{0.0};
    double kd{0.0};
};

struct RobotMujocoSpec {
    RobotType type = RobotType::MIT_HUMANOID;
    std::string_view baseBody{};
    std::vector<LimbMujocoSpec> legs;
    std::vector<LimbMujocoSpec> arms;
    std::vector<FixedJointMujocoSpec> fixedJoints;
};

const RobotMujocoSpec& getMitHumanoidMujocoSpec();
const RobotMujocoSpec& getUnitreeG1MujocoSpec();

const RobotMujocoSpec& getRobotMujocoSpec(RobotType type);

#endif  // ROBOT_MUJOCO_SPEC_H
