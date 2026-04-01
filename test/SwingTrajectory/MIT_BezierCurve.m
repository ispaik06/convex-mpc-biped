%% plot_mit_swing_frames.m
% Reproduce the MIT Cheetah open-source swing-foot trajectory logic from:
%   - ConvexMPCLocomotion.cpp
%   - FootSwingTrajectory.cpp
%
% What this script shows:
%   1) Desired swing-foot trajectory in the world frame.
%   2) The same desired trajectory expressed in the current body frame B(t),
%      with origin at the current COM.
%   3) How the touchdown target Pf is re-planned at every control tick.
%
% Notes:
%   - The exact swing shape in the MIT code is:
%       * x/y: straight-line interpolation with a cubic time-scaling
%       * z: two-piece cubic interpolation (up, then down)
%   - The touchdown target Pf is updated online during swing.
%   - This script uses the same coordinate-rotation convention as the MIT code.

clear; clc; close all;

cfg = default_mit_swing_config();
out = simulate_mit_swing(cfg);
plot_mit_swing(out, cfg);


function cfg = default_mit_swing_config()
  % Leg ordering in MIT Cheetah code:
  %   1 = FR, 2 = FL, 3 = HR, 4 = HL
  cfg.leg_names = {'FR', 'FL', 'HR', 'HL'};
  cfg.leg = 1;

  % Mini Cheetah defaults from the open-source tree:
  % controller_dt = 0.002
  % iterationsBetweenMPC = int(27 / (1000 * controller_dt)) = 13
  % dtMPC = 0.026
  % For trotting: stance = 5 segments, swing = 5 segments
  cfg.controller_dt = 0.002;
  cfg.iterations_between_mpc = floor(27 / (1000 * cfg.controller_dt));
  cfg.dtMPC = cfg.controller_dt * cfg.iterations_between_mpc;
  cfg.horizon_segments = 10;
  cfg.stance_segments = 5;
  cfg.swing_segments = cfg.horizon_segments - cfg.stance_segments;
  cfg.T_stance = cfg.dtMPC * cfg.stance_segments;
  cfg.T_swing = cfg.dtMPC * cfg.swing_segments;

  % Hard-coded swing settings used in ConvexMPCLocomotion.cpp
  cfg.swing_height = 0.06;
  cfg.cmpc_bonus_swing = 0.0;
  cfg.p_rel_max = 0.3;
  cfg.g = 9.81;
  cfg.ground_z = -0.003;

  % Mini Cheetah geometry from MiniCheetah.h
  body_length = 0.19 * 2;
  body_width = 0.049 * 2;
  cfg.body_height = 0.29;
  cfg.hip_locations_body = [...
     body_length/2, -body_width/2, 0;  % FR
     body_length/2,  body_width/2, 0;  % FL
    -body_length/2, -body_width/2, 0;  % HR
    -body_length/2,  body_width/2, 0]; % HL

  % Foot placement constants from ConvexMPCLocomotion.cpp
  cfg.side_sign = [-1, 1, -1, 1];
  cfg.interleave_y = [-0.08, 0.08, 0.02, -0.02];
  cfg.interleave_gain = -0.2;
  cfg.nominal_lateral_offset = 0.065;

  % Example state and command history for one swing.
  % You can replace these with your own measured/estimated values.
  cfg.p_com0_world = [0; 0; cfg.body_height];
  cfg.yaw0 = deg2rad(10);
  cfg.yaw_rate = 0.8;                 % _yaw_turn_rate
  cfg.v_des_body = [0.60; 0.00; 0.0]; % [_x_vel_des; _y_vel_des; 0]
  cfg.v_body_actual = [0.55; 0.03; 0.0];

  % Initial foot location at swing start.
  % p_leg is in the leg frame, which is parallel to the body frame.
  leg = cfg.leg;
  side = cfg.side_sign(leg);
  cfg.p_foot0_leg = [0.00; side * 0.02; cfg.ground_z - cfg.body_height];

  % Plot options
  cfg.axis_equal = true;
end


function out = simulate_mit_swing(cfg)
  leg = cfg.leg;
  dt = cfg.controller_dt;
  t = 0:dt:cfg.T_swing;
  if abs(t(end) - cfg.T_swing) > 1e-12
    t = [t, cfg.T_swing];
  end

  n = numel(t);

  p_com_world = zeros(3, n);
  yaw = zeros(1, n);
  R_BW = zeros(3, 3, n);
  R_WB = zeros(3, 3, n);
  v_world = zeros(3, n);

  p_foot_world_now = zeros(3, n);
  p_touchdown_world = zeros(3, n);
  p_des_world = zeros(3, n);
  p_des_body = zeros(3, n);
  p_des_leg = zeros(3, n);
  p_touchdown_body = zeros(3, n);

  % Current body state at swing start
  p_com_world(:, 1) = cfg.p_com0_world;
  yaw(1) = cfg.yaw0;
  R_BW(:, :, 1) = coord_rot_z_mit(yaw(1));
  R_WB(:, :, 1) = R_BW(:, :, 1).';
  v_world(:, 1) = R_WB(:, :, 1) * cfg.v_body_actual;

  p_hip_body = cfg.hip_locations_body(leg, :).';
  p0_world = p_com_world(:, 1) + R_WB(:, :, 1) * (p_hip_body + cfg.p_foot0_leg);

  for k = 1:n
    if k > 1
      yaw(k) = yaw(k - 1) + cfg.yaw_rate * dt;
      R_BW(:, :, k) = coord_rot_z_mit(yaw(k));
      R_WB(:, :, k) = R_BW(:, :, k).';
      v_world(:, k) = R_WB(:, :, k) * cfg.v_body_actual;
      p_com_world(:, k) = p_com_world(:, k - 1) + v_world(:, k - 1) * dt;
    end

    swing_state = t(k) / cfg.T_swing;
    swing_state = min(max(swing_state, 0), 1);
    swing_time_remaining = max(cfg.T_swing - t(k), 0);

    % Current foot position if the foot were still on the ground at p0_world.
    % This is only used here for visualization, not by the swing generator.
    p_foot_world_now(:, k) = p0_world;

    % Desired velocity in world frame, exactly as used in ConvexMPCLocomotion
    v_des_world = R_WB(:, :, k) * cfg.v_des_body;
    v_abs = abs(cfg.v_des_body(1));

    % Nominal foot placement in the body frame
    offset = [0; cfg.side_sign(leg) * cfg.nominal_lateral_offset; 0];
    p_robot_frame = p_hip_body + offset;
    p_robot_frame(2) = p_robot_frame(2) + ...
      cfg.interleave_y(leg) * v_abs * cfg.interleave_gain;

    % MIT coordinateRotation convention:
    % coordinateRotation(Z, theta) is a coordinate transform, not an active rotation.
    delta_yaw = cfg.yaw_rate * cfg.T_stance / 2;
    p_yaw_corrected = coord_rot_z_mit(-delta_yaw) * p_robot_frame;

    % Base touchdown point before velocity feedback
    p_touchdown = p_com_world(:, k) + R_WB(:, :, k) * ...
      (p_yaw_corrected + cfg.v_des_body * swing_time_remaining);

    % Velocity feedback / yaw-coupling terms from the MIT source
    delta_x = v_world(1, k) * (0.5 + cfg.cmpc_bonus_swing) * cfg.T_stance + ...
              0.03 * (v_world(1, k) - v_des_world(1)) + ...
              (0.5 * p_com_world(3, k) / cfg.g) * (v_world(2, k) * cfg.yaw_rate);

    delta_y = v_world(2, k) * 0.5 * cfg.T_stance * cfg.dtMPC + ...
              0.03 * (v_world(2, k) - v_des_world(2)) + ...
              (0.5 * p_com_world(3, k) / cfg.g) * (-v_world(1, k) * cfg.yaw_rate);

    delta_x = clamp_scalar(delta_x, -cfg.p_rel_max, cfg.p_rel_max);
    delta_y = clamp_scalar(delta_y, -cfg.p_rel_max, cfg.p_rel_max);

    p_touchdown(1) = p_touchdown(1) + delta_x;
    p_touchdown(2) = p_touchdown(2) + delta_y;
    p_touchdown(3) = cfg.ground_z;
    p_touchdown_world(:, k) = p_touchdown;

    % MIT swing trajectory:
    %   - x/y: straight-line interpolation with cubic time scaling
    %   - z: piecewise cubic interpolation
    [p_cmd_world, ~, ~] = foot_swing_bezier_mit( ...
      p0_world, p_touchdown, cfg.swing_height, swing_state, cfg.T_swing);

    p_des_world(:, k) = p_cmd_world;
    p_des_body(:, k) = R_BW(:, :, k) * (p_des_world(:, k) - p_com_world(:, k));
    p_touchdown_body(:, k) = R_BW(:, :, k) * (p_touchdown_world(:, k) - p_com_world(:, k));
    p_des_leg(:, k) = p_des_body(:, k) - p_hip_body;
  end

  out.t = t;
  out.p_com_world = p_com_world;
  out.yaw = yaw;
  out.R_BW = R_BW;
  out.R_WB = R_WB;
  out.v_world = v_world;
  out.p0_world = p0_world;
  out.p_foot_world_now = p_foot_world_now;
  out.p_touchdown_world = p_touchdown_world;
  out.p_touchdown_body = p_touchdown_body;
  out.p_des_world = p_des_world;
  out.p_des_body = p_des_body;
  out.p_des_leg = p_des_leg;
end


function plot_mit_swing(out, cfg)
  leg_name = cfg.leg_names{cfg.leg};

  fig1 = figure('Name', 'MIT Swing Trajectory: World vs Body', 'Color', 'w');
  tiledlayout(fig1, 2, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

  nexttile;
  hold on; grid on; box on;
  plot3(out.p_des_world(1, :), out.p_des_world(2, :), out.p_des_world(3, :), ...
    'b-', 'LineWidth', 2);
  plot3(out.p_touchdown_world(1, :), out.p_touchdown_world(2, :), out.p_touchdown_world(3, :), ...
    'r--', 'LineWidth', 1.5);
  plot3(out.p_com_world(1, :), out.p_com_world(2, :), out.p_com_world(3, :), ...
    'k-.', 'LineWidth', 1.2);
  plot3(out.p0_world(1), out.p0_world(2), out.p0_world(3), 'ko', ...
    'MarkerFaceColor', 'g', 'MarkerSize', 7);
  plot3(out.p_des_world(1, end), out.p_des_world(2, end), out.p_des_world(3, end), 'ks', ...
    'MarkerFaceColor', 'b', 'MarkerSize', 7);
  xlabel('x_W [m]'); ylabel('y_W [m]'); zlabel('z_W [m]');
  title(sprintf('World Frame: %s Swing', leg_name));
  legend({'p_{des}^W', 'P_f^W (replanned)', 'p_{COM}^W', 'P_0^W', 'p_{des}^W(T_{sw})'}, ...
    'Location', 'best');
  view(3);
  if cfg.axis_equal
    axis equal;
  end

  nexttile;
  hold on; grid on; box on;
  plot3(out.p_des_body(1, :), out.p_des_body(2, :), out.p_des_body(3, :), ...
    'b-', 'LineWidth', 2);
  plot3(out.p_touchdown_body(1, :), out.p_touchdown_body(2, :), out.p_touchdown_body(3, :), ...
    'r--', 'LineWidth', 1.5);
  plot3(0, 0, 0, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 7);
  xlabel('x_B [m]'); ylabel('y_B [m]'); zlabel('z_B [m]');
  title('Current Body Frame B(t), origin at COM');
  legend({'p_{des}^{B(t)}', 'P_f^{B(t)}', 'COM'}, 'Location', 'best');
  view(3);
  if cfg.axis_equal
    axis equal;
  end

  nexttile;
  hold on; grid on; box on;
  plot(out.t, out.p_des_world(1, :), 'r-', 'LineWidth', 1.5);
  plot(out.t, out.p_des_world(2, :), 'g-', 'LineWidth', 1.5);
  plot(out.t, out.p_des_world(3, :), 'b-', 'LineWidth', 1.5);
  plot(out.t, out.p_touchdown_world(1, :), 'r--', 'LineWidth', 1.0);
  plot(out.t, out.p_touchdown_world(2, :), 'g--', 'LineWidth', 1.0);
  plot(out.t, out.p_touchdown_world(3, :), 'b--', 'LineWidth', 1.0);
  xlabel('time [s]');
  ylabel('world coordinates [m]');
  title('World Coordinates vs Time');
  legend({'p_{des,x}^W', 'p_{des,y}^W', 'p_{des,z}^W', ...
          'P_{f,x}^W', 'P_{f,y}^W', 'P_{f,z}^W'}, 'Location', 'best');

  nexttile;
  hold on; grid on; box on;
  plot(out.t, out.p_des_body(1, :), 'r-', 'LineWidth', 1.5);
  plot(out.t, out.p_des_body(2, :), 'g-', 'LineWidth', 1.5);
  plot(out.t, out.p_des_body(3, :), 'b-', 'LineWidth', 1.5);
  plot(out.t, out.p_touchdown_body(1, :), 'r--', 'LineWidth', 1.0);
  plot(out.t, out.p_touchdown_body(2, :), 'g--', 'LineWidth', 1.0);
  plot(out.t, out.p_touchdown_body(3, :), 'b--', 'LineWidth', 1.0);
  xlabel('time [s]');
  ylabel('body coordinates [m]');
  title('Current Body-Frame Coordinates vs Time');
  legend({'p_{des,x}^{B(t)}', 'p_{des,y}^{B(t)}', 'p_{des,z}^{B(t)}', ...
          'P_{f,x}^{B(t)}', 'P_{f,y}^{B(t)}', 'P_{f,z}^{B(t)}'}, 'Location', 'best');

  fig2 = figure('Name', 'MIT Swing: Leg Frame', 'Color', 'w');
  tiledlayout(fig2, 1, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

  nexttile;
  hold on; grid on; box on;
  plot3(out.p_des_leg(1, :), out.p_des_leg(2, :), out.p_des_leg(3, :), ...
    'm-', 'LineWidth', 2);
  xlabel('x_H [m]'); ylabel('y_H [m]'); zlabel('z_H [m]');
  title(sprintf('Leg Frame H_%s (hip-centered)', leg_name));
  view(3);
  if cfg.axis_equal
    axis equal;
  end

  nexttile;
  hold on; grid on; box on;
  plot(out.t, out.p_des_leg(1, :), 'r-', 'LineWidth', 1.5);
  plot(out.t, out.p_des_leg(2, :), 'g-', 'LineWidth', 1.5);
  plot(out.t, out.p_des_leg(3, :), 'b-', 'LineWidth', 1.5);
  xlabel('time [s]');
  ylabel('leg-frame coordinates [m]');
  title('Leg-Frame Desired Position vs Time');
  legend({'x_H', 'y_H', 'z_H'}, 'Location', 'best');

  fprintf('\nMIT swing demo configuration\n');
  fprintf('  leg                     : %s\n', leg_name);
  fprintf('  controller_dt           : %.4f s\n', cfg.controller_dt);
  fprintf('  iterationsBetweenMPC    : %d\n', cfg.iterations_between_mpc);
  fprintf('  dtMPC                   : %.4f s\n', cfg.dtMPC);
  fprintf('  T_stance                : %.4f s\n', cfg.T_stance);
  fprintf('  T_swing                 : %.4f s\n', cfg.T_swing);
  fprintf('  swing_height            : %.4f m\n', cfg.swing_height);
  fprintf('  v_des_body              : [%.3f %.3f %.3f] m/s\n', cfg.v_des_body);
  fprintf('  v_body_actual           : [%.3f %.3f %.3f] m/s\n', cfg.v_body_actual);
  fprintf('  yaw_rate                : %.3f rad/s\n', cfg.yaw_rate);
  fprintf('  P0_world                : [%.3f %.3f %.3f] m\n', out.p0_world);
  fprintf('  Pf_world at swing start : [%.3f %.3f %.3f] m\n', out.p_touchdown_world(:, 1));
  fprintf('  Pf_world at swing end   : [%.3f %.3f %.3f] m\n', out.p_touchdown_world(:, end));
end


function [p, v, a] = foot_swing_bezier_mit(p0, pf, height, phase, swing_time)
  % Exact MIT open-source logic:
  %   _p = cubicBezier(_p0, _pf, phase)
  %   z is replaced by a two-piece cubic trajectory
  p = cubic_bezier_interp(p0, pf, phase);
  v = cubic_bezier_first_derivative(p0, pf, phase) / swing_time;
  a = cubic_bezier_second_derivative(p0, pf, phase) / (swing_time^2);

  if phase < 0.5
    u = 2 * phase;
    z0 = p0(3);
    z1 = p0(3) + height;
    zp = cubic_bezier_interp(z0, z1, u);
    zv = cubic_bezier_first_derivative(z0, z1, u) * 2 / swing_time;
    za = cubic_bezier_second_derivative(z0, z1, u) * 4 / (swing_time^2);
  else
    u = 2 * phase - 1;
    z0 = p0(3) + height;
    z1 = pf(3);
    zp = cubic_bezier_interp(z0, z1, u);
    zv = cubic_bezier_first_derivative(z0, z1, u) * 2 / swing_time;
    za = cubic_bezier_second_derivative(z0, z1, u) * 4 / (swing_time^2);
  end

  p(3) = zp;
  v(3) = zv;
  a(3) = za;
end


function y = cubic_bezier_interp(y0, yf, x)
  b = x^3 + 3 * x^2 * (1 - x);
  y = y0 + b * (yf - y0);
end


function y = cubic_bezier_first_derivative(y0, yf, x)
  b = 6 * x * (1 - x);
  y = b * (yf - y0);
end


function y = cubic_bezier_second_derivative(y0, yf, x)
  b = 6 - 12 * x;
  y = b * (yf - y0);
end


function R = coord_rot_z_mit(theta)
  % Same as coordinateRotation(CoordinateAxis::Z, theta) in the MIT code.
  % This is a coordinate transform, not an active vector rotation.
  c = cos(theta);
  s = sin(theta);
  R = [c,  s, 0;
      -s,  c, 0;
       0,  0, 1];
end


function y = clamp_scalar(x, lo, hi)
  y = min(max(x, lo), hi);
end
