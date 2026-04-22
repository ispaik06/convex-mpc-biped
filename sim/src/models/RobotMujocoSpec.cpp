#include "models/RobotMujocoSpec.h"

#include <stdexcept>

const RobotMujocoSpec& getRobotMujocoSpec(const RobotType type) {
    switch (type) {
        case RobotType::MIT_HUMANOID:
            return getMitHumanoidMujocoSpec();
        case RobotType::UNITREE_G1:
            return getUnitreeG1MujocoSpec();
        case RobotType::UNITREE_H1:
            break;
    }

    throw std::runtime_error("No MuJoCo spec is registered for the requested robot type");
}
