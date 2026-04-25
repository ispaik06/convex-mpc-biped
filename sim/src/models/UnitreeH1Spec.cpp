#include "models/RobotMujocoSpec.h"

const RobotMujocoSpec& getUnitreeH1MujocoSpec() {
    static const RobotMujocoSpec spec{
        RobotType::UNITREE_H1,
        "pelvis",
        {
            {Side::Left,
             "left_ankle_link",
             "left_foot_contact_site",
             {
                 {"left_hip_yaw_joint", "left_hip_yaw_joint"},
                 {"left_hip_roll_joint", "left_hip_roll_joint"},
                 {"left_hip_pitch_joint", "left_hip_pitch_joint"},
                 {"left_knee_joint", "left_knee_joint"},
                 {"left_ankle_joint", "left_ankle_joint"},
             }},
            {Side::Right,
             "right_ankle_link",
             "right_foot_contact_site",
             {
                 {"right_hip_yaw_joint", "right_hip_yaw_joint"},
                 {"right_hip_roll_joint", "right_hip_roll_joint"},
                 {"right_hip_pitch_joint", "right_hip_pitch_joint"},
                 {"right_knee_joint", "right_knee_joint"},
                 {"right_ankle_joint", "right_ankle_joint"},
             }},
        },
        {
            {Side::Left,
             "left_elbow_link_ball_hand",
             "",
             {
                 {"left_shoulder_pitch_joint", "left_shoulder_pitch_joint"},
                 {"left_shoulder_roll_joint", "left_shoulder_roll_joint"},
                 {"left_shoulder_yaw_joint", "left_shoulder_yaw_joint"},
                 {"left_elbow_joint", "left_elbow_joint"},
             }},
            {Side::Right,
             "right_elbow_link_ball_hand",
             "",
             {
                 {"right_shoulder_pitch_joint", "right_shoulder_pitch_joint"},
                 {"right_shoulder_roll_joint", "right_shoulder_roll_joint"},
                 {"right_shoulder_yaw_joint", "right_shoulder_yaw_joint"},
                 {"right_elbow_joint", "right_elbow_joint"},
             }},
        },
        {
            {"torso_joint", "torso_joint", 300.0, 30.0},
            {"not_use_joint", "not_use_joint", 10.0, 1.0},
        }};

    return spec;
}
