#ifndef INITIAL_POSE_CONFIG_H
#define INITIAL_POSE_CONFIG_H

#include <algorithm>
#include <vector>

#include "Robot/RobotParams.h"

struct InitialPoseConfig {
    std::vector<double> legJointOffsets{0.0, 0.0, -0.65, 0.80, -0.35};
    std::vector<double> armJointOffsets{0.0, 0.0, 0.0, -0.65};
    double legInitializationTime{2.0};
    double armInitializationTime{1.0};
};

const InitialPoseConfig& getInitialPoseConfig();
const InitialPoseConfig& getInitialPoseConfig(RobotType robotType);

template <typename T, typename LimbParamsT>
void applyConfiguredJointOffsets(std::vector<T>& target,
                                 std::vector<T>& midpoint,
                                 const vectorAligned<LimbParamsT>& limbs,
                                 const std::vector<double>& offsets) {
    std::size_t flatIdx = 0;
    for (const auto& limb : limbs) {
        const std::size_t dof = limb.joints.q_idx.size();
        const std::size_t configuredCount = std::min(dof, offsets.size());

        for (std::size_t joint = 0; joint < configuredCount; ++joint) {
            target[flatIdx + joint] += static_cast<T>(offsets[joint]);
            midpoint[flatIdx + joint] = target[flatIdx + joint];
        }

        flatIdx += dof;
    }
}

#endif  // INITIAL_POSE_CONFIG_H
