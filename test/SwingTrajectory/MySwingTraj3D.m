clc; clear; close all;

%% =========================
% User inputs
% ==========================
P0_w = [ 0.5;  1.0;  0.0];   % 3D start point
Pf_w = [10.2; -0.3;  0.0];   % 3D final point

h = 5;       % nominal height parameter
N = 200;       % number of samples per segment

% optional reference "up" direction in world frame
worldUp = [0; 0; 1];

%% =========================
% Basic quantities
% ==========================
d = Pf_w - P0_w;
l = norm(d);

if l < 1e-10
    error('P0_w and Pf_w are too close or identical.');
end

H = 0.7 * h;

%% =========================
% Build local frame
% We want local coordinates to match your 2D design:
% P0_local = [0, 0, 0]
% Pf_local = [-l, 0, 0]
%
% So local x-axis points from Pf to P0
% ==========================
ex = (P0_w - Pf_w) / l;   % local x-axis in world frame

% avoid singularity if ex is parallel to worldUp
if norm(cross(worldUp, ex)) < 1e-8
    worldUp = [0; 1; 0];
end

ey = cross(worldUp, ex);
ey = ey / norm(ey);

ez = cross(ex, ey);
ez = ez / norm(ez);

% rotation matrix: local -> world
R = [ex, ey, ez];

%% =========================
% Local control points
% 2D (x,y) shape is embedded into local 3D as (x,0,z)
% so the "height" goes along local z-axis
% ==========================
Q0 = [ 0.0;     0.0; 0.0];
Q1 = [ 0.9*h;   0.0; 0.0];
Q2 = [ 1.7*h;   0.0; 1.1*H];
Q3 = [ 0.3*h;   0.0; H];

R0 = Q3;
R1 = [-1.1*h;   0.0; 0.9*H];
R2 = [-1.25*l;  0.0; 0.0];
R3 = [-l;       0.0; 0.0];

%% =========================
% Parameter
% ==========================
t = linspace(0, 1, N);

%% =========================
% Evaluate in local frame
% ==========================
B1_local = cubicBezier3D(Q0, Q1, Q2, Q3, t);
B2_local = cubicBezier3D(R0, R1, R2, R3, t);

%% =========================
% Transform local curve to world frame
% p_world = P0_w + R * p_local
% ==========================
B1_w = (R * B1_local')' + P0_w.';
B2_w = (R * B2_local')' + P0_w.';

Q0_w = (R * Q0 + P0_w).';
Q1_w = (R * Q1 + P0_w).';
Q2_w = (R * Q2 + P0_w).';
Q3_w = (R * Q3 + P0_w).';

R0_w = (R * R0 + P0_w).';
R1_w = (R * R1 + P0_w).';
R2_w = (R * R2 + P0_w).';
R3_w = (R * R3 + P0_w).';

%% =========================
% Plot 3D
% ==========================
figure; hold on; grid on; axis equal;

% curves
plot3(B1_w(:,1), B1_w(:,2), B1_w(:,3), 'b', 'LineWidth', 2);
plot3(B2_w(:,1), B2_w(:,2), B2_w(:,3), 'r', 'LineWidth', 2);

% control polygons
plot3([Q0_w(1) Q1_w(1) Q2_w(1) Q3_w(1)], ...
      [Q0_w(2) Q1_w(2) Q2_w(2) Q3_w(2)], ...
      [Q0_w(3) Q1_w(3) Q2_w(3) Q3_w(3)], ...
      'bo--', 'LineWidth', 1);

plot3([R0_w(1) R1_w(1) R2_w(1) R3_w(1)], ...
      [R0_w(2) R1_w(2) R2_w(2) R3_w(2)], ...
      [R0_w(3) R1_w(3) R2_w(3) R3_w(3)], ...
      'ro--', 'LineWidth', 1);

% key points
Pm_w = Q3_w;

plot3(P0_w(1), P0_w(2), P0_w(3), 'ks', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
plot3(Pm_w(1), Pm_w(2), Pm_w(3), 'ks', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
plot3(Pf_w(1), Pf_w(2), Pf_w(3), 'ks', 'MarkerSize', 8, 'MarkerFaceColor', 'k');

% line between endpoints
plot3([P0_w(1) Pf_w(1)], [P0_w(2) Pf_w(2)], [P0_w(3) Pf_w(3)], 'k:', 'LineWidth', 1);

xlabel('X');
ylabel('Y');
zlabel('Z');
title('3D two-piece cubic Bezier spline');
legend('Bezier 1', 'Bezier 2', 'Ctrl poly 1', 'Ctrl poly 2', ...
       'Start', 'Middle', 'End', 'P0-Pf line', ...
       'Location', 'best');

view(3);

%% =========================
% Local function
% ==========================
function B = cubicBezier3D(P0, P1, P2, P3, t)
    t = t(:);
    B = (1-t).^3 .* P0' + ...
        3*(1-t).^2 .* t .* P1' + ...
        3*(1-t) .* t.^2 .* P2' + ...
        t.^3 .* P3';
end