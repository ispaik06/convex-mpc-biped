#include "RobotMujocoSpec.h"

const RobotMujocoSpec& getMitHumanoidMujocoSpec() {
    static const RobotMujocoSpec spec{
        RobotType::MIT_HUMANOID,
        "torso",
        {
            {Side::Left,
             "left_foot_link",
             "",
             {
                 {"left_hip_yaw_joint", "left_hip_yaw"},
                 {"left_hip_abad_joint", "left_hip_abad"},
                 {"left_hip_pitch_joint", "left_hip_pitch"},
                 {"left_knee_joint", "left_knee"},
                 {"left_ankle_joint", "left_ankle"},
             }},
            {Side::Right,
             "right_foot_link",
             "",
             {
                 {"right_hip_yaw_joint", "right_hip_yaw"},
                 {"right_hip_abad_joint", "right_hip_abad"},
                 {"right_hip_pitch_joint", "right_hip_pitch"},
                 {"right_knee_joint", "right_knee"},
                 {"right_ankle_joint", "right_ankle"},
             }},
        },
        {
            {Side::Left,
             "left_forearm_link",
             "",
             {
                 {"left_shoulder_pitch_joint", "left_shoulder_pitch"},
                 {"left_shoulder_abad_joint", "left_shoulder_abad"},
                 {"left_shoulder_yaw_joint", "left_shoulder_yaw"},
                 {"left_elbow_joint", "left_elbow"},
             }},
            {Side::Right,
             "right_forearm_link",
             "",
             {
                 {"right_shoulder_pitch_joint", "right_shoulder_pitch"},
                 {"right_shoulder_abad_joint", "right_shoulder_abad"},
                 {"right_shoulder_yaw_joint", "right_shoulder_yaw"},
                 {"right_elbow_joint", "right_elbow"},
             }},
        }};

    return spec;
}
