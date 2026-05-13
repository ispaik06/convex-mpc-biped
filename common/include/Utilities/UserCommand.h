#ifndef USER_COMMAND_H
#define USER_COMMAND_H

struct UserCommand {
    double x_dot{0.0};
    double y_dot{0.0};
    double psi_dot{0.0};
    double body_height_offset_m{0.0};
    double standing_roll_offset_rad{0.0};
    double standing_pitch_offset_rad{0.0};
    unsigned long long standing_mpc_debug_log_request{0};
    unsigned long long locomotion_mode_toggle_request{0};
};

#endif  // USER_COMMAND_H
