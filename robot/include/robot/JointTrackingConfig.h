#ifndef JOINT_TRACKING_CONFIG_H
#define JOINT_TRACKING_CONFIG_H

#include <vector>

#include "Types.h"

struct JointTrackingConfig {
    std::vector<double> legKp;
    std::vector<double> legKd;
    std::vector<double> armKp;
    std::vector<double> armKd;

    bool hasLegGains() const {
        return !legKp.empty();
    }

    bool hasArmGains() const {
        return !armKp.empty();
    }
};

const JointTrackingConfig& getJointTrackingConfig();
const JointTrackingConfig& getJointTrackingConfig(RobotType robotType);

#endif  // JOINT_TRACKING_CONFIG_H
