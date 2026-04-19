#ifndef MUJOCO_ROBOT_BINDINGS_H
#define MUJOCO_ROBOT_BINDINGS_H

#include <vector>

struct MujocoEndEffectorBinding {
    int rootBodyId{-1};
    int bodyId{-1};
    int siteId{-1};
};

struct MujocoRobotBindings {
    int torsoBodyId{-1};
    std::vector<MujocoEndEffectorBinding> feet;
    std::vector<MujocoEndEffectorBinding> hands;
};

#endif  // MUJOCO_ROBOT_BINDINGS_H
