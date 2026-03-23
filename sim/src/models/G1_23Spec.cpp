#include "RobotMujocoSpec.h"

const RobotMujocoSpec& getUnitreeG1_23MujocoSpec() {
    static const RobotMujocoSpec spec{
        RobotType::UNITREE_G1,
        "pelvis",
        {
            {Side::Left,
             "left_ankle_roll_link",
             "",
             {
                 {"left_hip_pitch_joint", "left_hip_pitch"},
                 {"left_hip_roll_joint", "left_hip_roll"},
                 {"left_hip_yaw_joint", "left_hip_yaw"},
                 {"left_knee_joint", "left_knee"},
                 {"left_ankle_pitch_joint", "left_ankle_pitch"},
                 {"left_ankle_roll_joint", "left_ankle_roll"},
             }},
            {Side::Right,
             "right_ankle_roll_link",
             "",
             {
                 {"right_hip_pitch_joint", "right_hip_pitch"},
                 {"right_hip_roll_joint", "right_hip_roll"},
                 {"right_hip_yaw_joint", "right_hip_yaw"},
                 {"right_knee_joint", "right_knee"},
                 {"right_ankle_pitch_joint", "right_ankle_pitch"},
                 {"right_ankle_roll_joint", "right_ankle_roll"},
             }},
        },
        {
            {Side::Left,
             "left_wrist_roll_rubber_hand",
             "",
             {
                 {"left_shoulder_pitch_joint", "left_shoulder_pitch"},
                 {"left_shoulder_roll_joint", "left_shoulder_roll"},
                 {"left_shoulder_yaw_joint", "left_shoulder_yaw"},
                 {"left_elbow_joint", "left_elbow"},
                 {"left_wrist_roll_joint", "left_wrist_roll"},
             }},
            {Side::Right,
             "right_wrist_roll_rubber_hand",
             "",
             {
                 {"right_shoulder_pitch_joint", "right_shoulder_pitch"},
                 {"right_shoulder_roll_joint", "right_shoulder_roll"},
                 {"right_shoulder_yaw_joint", "right_shoulder_yaw"},
                 {"right_elbow_joint", "right_elbow"},
                 {"right_wrist_roll_joint", "right_wrist_roll"},
             }},
        }};

    return spec;
}
