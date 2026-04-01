clear; clc; close all;

%% Parameters
p0 = [0.0; 0.0; 0.0];      % start foot position [x0; y0; z0]
pf = [0.3; 0.15; 0.0];     % final foot position [xf; yf; zf]
h  = 0.12;                 % swing height
N  = 400;                  % number of samples

s = linspace(0,1,N);

%% Cubic interpolation function B(s) = 3s^2 - 2s^3
B = @(x) 3*x.^2 - 2*x.^3;

%% Preallocate
x = zeros(1,N);
y = zeros(1,N);
z = zeros(1,N);

%% Compute trajectory
for i = 1:N
    si = s(i);

    % x, y : straight-line interpolation
    x(i) = (1 - B(si))*p0(1) + B(si)*pf(1);
    y(i) = (1 - B(si))*p0(2) + B(si)*pf(2);

    % z : piecewise cubic lift
    if si < 0.5
        u = 2*si;
        z(i) = (1 - B(u))*p0(3) + B(u)*(p0(3) + h);
    else
        u = 2*si - 1;
        z(i) = (1 - B(u))*(p0(3) + h) + B(u)*pf(3);
    end
end

%% 3D plot
figure;
plot3(x, y, z, 'LineWidth', 2); hold on;
plot3(p0(1), p0(2), p0(3), 'o', 'MarkerSize', 8, 'LineWidth', 2);
plot3(pf(1), pf(2), pf(3), 's', 'MarkerSize', 8, 'LineWidth', 2);
grid on; axis equal;
xlabel('x'); ylabel('y'); zlabel('z');
title('MIT-style Foot Swing Trajectory (3D)');

%% XY projection
figure;
plot(x, y, 'LineWidth', 2); hold on;
plot(p0(1), p0(2), 'o', 'MarkerSize', 8, 'LineWidth', 2);
plot(pf(1), pf(2), 's', 'MarkerSize', 8, 'LineWidth', 2);
grid on; axis equal;
xlabel('x'); ylabel('y');
title('XY Projection: Straight Line');

%% XZ projection
figure;
plot(x, z, 'LineWidth', 2); hold on;
plot(p0(1), p0(3), 'o', 'MarkerSize', 8, 'LineWidth', 2);
plot(pf(1), pf(3), 's', 'MarkerSize', 8, 'LineWidth', 2);
grid on;
xlabel('x'); ylabel('z');
title('XZ Projection: Lifted Swing Profile');

%% Also plot x(s), y(s), z(s)
figure;
plot(s, x, 'LineWidth', 2); hold on;
plot(s, y, 'LineWidth', 2);
plot(s, z, 'LineWidth', 2);
grid on;
xlabel('s');
ylabel('position');
title('Position Profiles vs s');
legend('x(s)', 'y(s)', 'z(s)', 'Location', 'best');