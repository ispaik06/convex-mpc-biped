clc; clear; close all;

%% Parameters
h = 1.0;      % height scale
l = 2.0;      % distance scale

H = 0.7*h;

% End points
P0 = [0, 0];
Pf = [-l, 0];

% Middle joint point
Pm = [0.15*l, H];

%% First cubic Bezier control points
Q0 = P0;
Q1 = [0.45*l, 0];
Q2 = [0.85*l, 1.1*H];
Q3 = Pm;

%% Second cubic Bezier control points
R0 = Pm;
R1 = [-0.55*l, 0.9*H];
R2 = [-1.25*l, 0];
R3 = Pf;

%% Parameter
N = 200;
t = linspace(0, 1, N);

%% Evaluate curves
B1 = cubicBezier(Q0, Q1, Q2, Q3, t);
B2 = cubicBezier(R0, R1, R2, R3, t);

%% Plot
figure;
plot(B1(:,1), B1(:,2), 'b', 'LineWidth', 2); hold on;
plot(B2(:,1), B2(:,2), 'r', 'LineWidth', 2);

% Plot control polygons
plot([Q0(1) Q1(1) Q2(1) Q3(1)], [Q0(2) Q1(2) Q2(2) Q3(2)], 'bo--', 'LineWidth', 1);
plot([R0(1) R1(1) R2(1) R3(1)], [R0(2) R1(2) R2(2) R3(2)], 'ro--', 'LineWidth', 1);

% Mark points
plot(P0(1), P0(2), 'ks', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
plot(Pm(1), Pm(2), 'ks', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
plot(Pf(1), Pf(2), 'ks', 'MarkerSize', 8, 'MarkerFaceColor', 'k');

grid on; axis equal;
xlabel('x'); ylabel('y');
title('Bezier spline from two cubic Bezier curves');
legend('Bezier 1', 'Bezier 2', 'Ctrl poly 1', 'Ctrl poly 2', 'Location', 'best');

%% Endpoint velocity / acceleration check
v_start = 3*(Q1 - Q0);
a_start = 6*(Q0 - 2*Q1 + Q2);

v_end   = 3*(R3 - R2);
a_end   = 6*(R3 - 2*R2 + R1);

disp('Start velocity B1''(0) ='); disp(v_start);
disp('Start acceleration B1''''(0) ='); disp(a_start);

disp('End velocity B2''(1) ='); disp(v_end);
disp('End acceleration B2''''(1) ='); disp(a_end);

%% -------- Local function --------
function B = cubicBezier(P0, P1, P2, P3, t)
    t = t(:);
    B = (1-t).^3 .* P0 + ...
        3*(1-t).^2.*t .* P1 + ...
        3*(1-t).*t.^2 .* P2 + ...
        t.^3 .* P3;
end