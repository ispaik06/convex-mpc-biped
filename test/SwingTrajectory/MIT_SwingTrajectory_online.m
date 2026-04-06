%% mit_swing_one_figure.m
clear; clc; close all;

%% ===================== SETTINGS =====================
dt = 0.002;
Tsw = 0.13;
Tstance = 0.13;
dtMPC = 0.026;
h = 0.06;

playback_speed = 0.45;
curve_npts = 120;

force_touchdown_jump = false;
jump_time = 0.078;
jump_offset_world = [-0.22; 0.08; 0.0];

cmpc_bonus_swing = 0.0;
p_rel_max = 0.3;
g = 9.81;
ground_z = -0.003;

%% ===================== ROBOT / LEG =====================
leg_name = 'FR';

body_length = 0.19 * 2;
body_width  = 0.049 * 2;
body_height = 0.29;

p_hip_body = [ body_length/2; -body_width/2; 0 ];
p_foot0_leg = [0.00; -0.02; ground_z - body_height];

side_sign = -1;
interleave_y = -0.08;
interleave_gain = -0.2;
nominal_lateral_offset = 0.065;

%% ===================== TIME =====================
t = 0:dt:Tsw;
N = numel(t);
s = min(max(t / Tsw, 0), 1);

%% ===================== RAW COMMANDS =====================
x_cmd_raw = 0.60 * ones(1, N);
y_cmd_raw = 0.00 * ones(1, N);
psi_cmd_raw = 3.00 * ones(1, N);

% x_cmd_raw(t >= 0.040) = 0.20;
% x_cmd_raw(t >= 0.085) = 0.75;
% 
% y_cmd_raw(t >= 0.055) = 0.10;
% y_cmd_raw(t >= 0.095) = -0.08;

psi_cmd_raw(t >= 0.060) = -0.35;
psi_cmd_raw(t >= 0.100) = 1.00;

%% ===================== INTERNAL DES (MIT STYLE) =====================
alpha = 0.1;
x_des = zeros(1, N);
y_des = zeros(1, N);
psi_des = psi_cmd_raw;

x_des(1) = x_cmd_raw(1);
y_des(1) = y_cmd_raw(1);

for k = 2:N
    x_des(k) = (1 - alpha) * x_des(k-1) + alpha * x_cmd_raw(k);
    y_des(k) = (1 - alpha) * y_des(k-1) + alpha * y_cmd_raw(k);
end

%% ===================== ACTUAL BODY MOTION FOR DEMO =====================
v_body_actual = zeros(3, N);
v_body_actual(:,1) = [0.45; 0.00; 0];

for k = 2:N
    v_body_actual(1,k) = 0.95 * v_body_actual(1,k-1) + 0.05 * x_des(k-1);
    v_body_actual(2,k) = 0.95 * v_body_actual(2,k-1) + 0.05 * y_des(k-1);
    v_body_actual(3,k) = 0.0;
end

yaw = zeros(1, N);
yaw(1) = deg2rad(10);

R_BW = zeros(3,3,N);
R_WB = zeros(3,3,N);

p_com_act_world = zeros(3, N);
v_world_actual = zeros(3, N);

R_BW(:,:,1) = coordRotZ_MIT(yaw(1));
R_WB(:,:,1) = R_BW(:,:,1).';
p_com_act_world(:,1) = [0; 0; body_height];
v_world_actual(:,1) = R_WB(:,:,1) * v_body_actual(:,1);

for k = 2:N
    yaw(k) = yaw(k-1) + psi_des(k-1) * dt;
    R_BW(:,:,k) = coordRotZ_MIT(yaw(k));
    R_WB(:,:,k) = R_BW(:,:,k).';
    v_world_actual(:,k) = R_WB(:,:,k) * v_body_actual(:,k);
    p_com_act_world(:,k) = p_com_act_world(:,k-1) + v_world_actual(:,k-1) * dt;
end

%% ===================== DESIRED COM PATH =====================
p_com_des_world = zeros(3, N);
p_com_des_world(:,1) = p_com_act_world(:,1);

for k = 2:N
    v_des_body_prev = [x_des(k-1); y_des(k-1); 0];
    v_des_world_prev = R_WB(:,:,k-1) * v_des_body_prev;
    p_com_des_world(:,k) = p_com_des_world(:,k-1) + dt * [v_des_world_prev(1); v_des_world_prev(2); 0];
    p_com_des_world(3,k) = body_height;
end

%% ===================== SWING START POINT P0 =====================
P0_world = p_com_act_world(:,1) + R_WB(:,:,1) * (p_hip_body + p_foot0_leg);

%% ===================== REPLAN Pf AND GENERATE p_cmd =====================
Pf_world = zeros(3, N);
Pf_body  = zeros(3, N);

p_cmd_world = zeros(3, N);
p_cmd_body  = zeros(3, N);

for k = 1:N
    T_rem = max(Tsw - t(k), 0);

    v_des_body_k = [x_des(k); y_des(k); 0];
    v_des_world_k = R_WB(:,:,k) * v_des_body_k;
    v_abs = abs(x_des(k));

    offset = [0; side_sign * nominal_lateral_offset; 0];
    pRobotFrame = p_hip_body + offset;
    pRobotFrame(2) = pRobotFrame(2) + interleave_y * v_abs * interleave_gain;

    delta_yaw = psi_des(k) * Tstance / 2;
    pYawCorrected = coordRotZ_MIT(-delta_yaw) * pRobotFrame;

    Pf = p_com_act_world(:,k) + R_WB(:,:,k) * (pYawCorrected + v_des_body_k * T_rem);

    pfx_rel = v_world_actual(1,k) * (0.5 + cmpc_bonus_swing) * Tstance + ...
              0.03 * (v_world_actual(1,k) - v_des_world_k(1)) + ...
              (0.5 * p_com_act_world(3,k) / g) * (v_world_actual(2,k) * psi_des(k));

    pfy_rel = v_world_actual(2,k) * 0.5 * Tstance * dtMPC + ...
              0.03 * (v_world_actual(2,k) - v_des_world_k(2)) + ...
              (0.5 * p_com_act_world(3,k) / g) * (-v_world_actual(1,k) * psi_des(k));

    pfx_rel = clampScalar(pfx_rel, -p_rel_max, p_rel_max);
    pfy_rel = clampScalar(pfy_rel, -p_rel_max, p_rel_max);

    Pf(1) = Pf(1) + pfx_rel;
    Pf(2) = Pf(2) + pfy_rel;
    Pf(3) = ground_z;

    if force_touchdown_jump && t(k) >= jump_time
        Pf = Pf + jump_offset_world;
    end

    Pf_world(:,k) = Pf;
    Pf_body(:,k) = R_BW(:,:,k) * (Pf - p_com_act_world(:,k));

    p_cmd_world(:,k) = footSwingMIT(P0_world, Pf, h, s(k));
    p_cmd_body(:,k)  = R_BW(:,:,k) * (p_cmd_world(:,k) - p_com_act_world(:,k));
end

%% ===================== ONE FIGURE / MULTI AXES =====================
% fig = figure('Color','w','Position',[40 40 1650 920]);
fig = figure('Color','w', 'WindowState','maximized');


% top row: bigger foot plots
axW   = axes('Parent',fig,'Position',[0.05 0.40 0.42 0.55]); hold(axW,'on'); grid(axW,'on'); box(axW,'on');
axB   = axes('Parent',fig,'Position',[0.53 0.40 0.42 0.55]); hold(axB,'on'); grid(axB,'on'); box(axB,'on');

% bottom-left: COM
axCOM = axes('Parent',fig,'Position',[0.05 0.08 0.42 0.22]); hold(axCOM,'on'); grid(axCOM,'on'); box(axCOM,'on');

% bottom-right: commands stacked
axX   = axes('Parent',fig,'Position',[0.53 0.22 0.42 0.08]); hold(axX,'on'); grid(axX,'on'); box(axX,'on');
axY   = axes('Parent',fig,'Position',[0.53 0.14 0.42 0.08]); hold(axY,'on'); grid(axY,'on'); box(axY,'on');
axPsi = axes('Parent',fig,'Position',[0.53 0.06 0.42 0.08]); hold(axPsi,'on'); grid(axPsi,'on'); box(axPsi,'on');

title(axW, 'Foot Motion in World Frame');
xlabel(axW, 'x_W [m]'); ylabel(axW, 'y_W [m]'); zlabel(axW, 'z_W [m]');
view(axW, 3);

title(axB, 'Foot Motion in Current Body Frame');
xlabel(axB, 'x_B [m]'); ylabel(axB, 'y_B [m]'); zlabel(axB, 'z_B [m]');
view(axB, 3);

title(axCOM, 'Body COM Path in World XY');
xlabel(axCOM, 'x_W [m]'); ylabel(axCOM, 'y_W [m]');
axis(axCOM, 'equal');

title(axX, 'x velocity command');
ylabel(axX, '[m/s]');

title(axY, 'y velocity command');
ylabel(axY, '[m/s]');

title(axPsi, 'yaw rate command');
xlabel(axPsi, 'time [s]');
ylabel(axPsi, '[rad/s]');

% bounds
allW = [p_cmd_world, Pf_world, P0_world];
mW = 0.05;
xlim(axW, [min(allW(1,:))-mW, max(allW(1,:))+mW]);
ylim(axW, [min(allW(2,:))-mW, max(allW(2,:))+mW]);
zlim(axW, [min(allW(3,:))-mW, max(allW(3,:))+mW]);
axis(axW, 'equal');

allB = [p_cmd_body, Pf_body, zeros(3,1)];
mB = 0.05;
xlim(axB, [min(allB(1,:))-mB, max(allB(1,:))+mB]);
ylim(axB, [min(allB(2,:))-mB, max(allB(2,:))+mB]);
zlim(axB, [min(allB(3,:))-mB, max(allB(3,:))+mB]);
axis(axB, 'equal');

allCOM = [p_com_des_world, p_com_act_world, Pf_world];
mC = 0.05;
xlim(axCOM, [min(allCOM(1,:))-mC, max(allCOM(1,:))+mC]);
ylim(axCOM, [min(allCOM(2,:))-mC, max(allCOM(2,:))+mC]);

xlim(axX, [t(1), t(end)]);
xlim(axY, [t(1), t(end)]);
xlim(axPsi, [t(1), t(end)]);

% static markers
plot3(axW, P0_world(1), P0_world(2), P0_world(3), 'gs', 'MarkerFaceColor','g', 'MarkerSize',8);

h_body_x_B = quiver3(axB, 0,0,0, 0.06,0,0, 0, 'k', 'LineWidth', 1.8, 'MaxHeadSize', 0.8);
text(axB, 0.07, 0, 0, 'x_B', 'FontSize', 11, 'Color', 'k');

% world handles
hFootHistW = plot3(axW, nan, nan, nan, 'b-', 'LineWidth', 2.4);
hPfHistW   = plot3(axW, nan, nan, nan, 'r:', 'LineWidth', 2.0);
hCurveW    = plot3(axW, nan, nan, nan, 'c--', 'LineWidth', 1.6);
hPfNowW    = plot3(axW, nan, nan, nan, 'ro', 'MarkerFaceColor','r', 'MarkerSize',8);
hFootNowW  = plot3(axW, nan, nan, nan, 'mo', 'MarkerFaceColor','m', 'MarkerSize',8);

% body handles
hFootHistB = plot3(axB, nan, nan, nan, 'b-', 'LineWidth', 2.4);
hPfHistB   = plot3(axB, nan, nan, nan, 'r:', 'LineWidth', 2.0);
hCurveB    = plot3(axB, nan, nan, nan, 'c--', 'LineWidth', 1.6);
hPfNowB    = plot3(axB, nan, nan, nan, 'ro', 'MarkerFaceColor','r', 'MarkerSize',8);
hFootNowB  = plot3(axB, nan, nan, nan, 'mo', 'MarkerFaceColor','m', 'MarkerSize',8);

% COM handles
hComDesPath = plot(axCOM, nan, nan, 'k--', 'LineWidth', 1.8);
hComActPath = plot(axCOM, nan, nan, 'b-',  'LineWidth', 2.0);
hPfHistCOM  = plot(axCOM, nan, nan, 'r:', 'LineWidth', 1.8);
hComDesNow  = plot(axCOM, nan, nan, 'ks', 'MarkerFaceColor','k', 'MarkerSize',8);
hComActNow  = plot(axCOM, nan, nan, 'bo', 'MarkerFaceColor','b', 'MarkerSize',8);
hPfNowCOM   = plot(axCOM, nan, nan, 'ro', 'MarkerFaceColor','r', 'MarkerSize',8);

hBodyArrowCOM = quiver(axCOM, nan, nan, nan, nan, 0, ...
    'Color',[0.1 0.5 0.1], 'LineWidth', 2.0, 'MaxHeadSize', 2.0);

% command handles
hXraw = plot(axX, t, x_cmd_raw, 'k--', 'LineWidth', 1.2);
hXdes = plot(axX, t, x_des, 'b-', 'LineWidth', 1.8);

hYraw = plot(axY, t, y_cmd_raw, 'k--', 'LineWidth', 1.2);
hYdes = plot(axY, t, y_des, 'b-', 'LineWidth', 1.8);

hPsiraw = plot(axPsi, t, psi_cmd_raw, 'k--', 'LineWidth', 1.2);
hPsides = plot(axPsi, t, psi_des, 'm-', 'LineWidth', 1.8);

yl1 = ylim(axX);
yl2 = ylim(axY);
yl3 = ylim(axPsi);

hCursor1 = plot(axX, [t(1) t(1)], yl1, 'r-', 'LineWidth', 1.2);
hCursor2 = plot(axY, [t(1) t(1)], yl2, 'r-', 'LineWidth', 1.2);
hCursor3 = plot(axPsi, [t(1) t(1)], yl3, 'r-', 'LineWidth', 1.2);

legend(axW, [hFootHistW, hPfHistW, hCurveW, hPfNowW, hFootNowW], ...
    {'foot command history (world)', ...
     'touchdown target history (world)', ...
     'current replanned swing curve (world)', ...
     'current touchdown target P_f (world)', ...
     'current foot command point (world)'}, ...
    'Location', 'best');

legend(axB, [hFootHistB, hPfHistB, hCurveB, hPfNowB, hFootNowB, h_body_x_B], ...
    {'foot coordinate history (body)', ...
     'touchdown target history (body)', ...
     'current replanned swing curve (body)', ...
     'current touchdown target P_f (body)', ...
     'current foot command point (body)', ...
     'body x-axis'}, ...
    'Location', 'best');

lgd = legend(axCOM, [hComDesPath, hComActPath, hPfHistCOM, hComDesNow, hComActNow, hPfNowCOM, hBodyArrowCOM], ...
    {'desired COM path', ...
     'actual COM path', ...
     'touchdown target history (world XY)', ...
     'current desired COM', ...
     'current actual COM', ...
     'current touchdown target P_f', ...
     'body x-axis now'}, ...
    'Location','southwest');
lgd.Units = 'normalized';
pos = lgd.Position;
pos(1) = pos(1) - 0.09;
pos(2) = pos(2) - 0.07;
lgd.Position = pos;

legend(axX, [hXraw, hXdes], {'raw x command', 'internal _x_vel_des'}, 'Location', 'northeast');
legend(axY, [hYraw, hYdes], {'raw y command', 'internal _y_vel_des'}, 'Location', 'northeast');
legend(axPsi, [hPsiraw, hPsides], {'raw yaw-rate command', 'internal _yaw_turn_rate'}, 'Location', 'northeast');

%% ===================== ANIMATION LOOP =====================
s_curve = linspace(0, 1, curve_npts);

while isgraphics(fig)
    for k = 1:N
        curveW = zeros(3, curve_npts);
        for i = 1:curve_npts
            curveW(:,i) = footSwingMIT(P0_world, Pf_world(:,k), h, s_curve(i));
        end

        curveB = R_BW(:,:,k) * (curveW - p_com_act_world(:,k) * ones(1, curve_npts));

        set(hFootHistW, 'XData', p_cmd_world(1,1:k), 'YData', p_cmd_world(2,1:k), 'ZData', p_cmd_world(3,1:k));
        set(hPfHistW,   'XData', Pf_world(1,1:k),   'YData', Pf_world(2,1:k),   'ZData', Pf_world(3,1:k));
        set(hCurveW,    'XData', curveW(1,:),       'YData', curveW(2,:),       'ZData', curveW(3,:));
        set(hPfNowW,    'XData', Pf_world(1,k),     'YData', Pf_world(2,k),     'ZData', Pf_world(3,k));
        set(hFootNowW,  'XData', p_cmd_world(1,k),  'YData', p_cmd_world(2,k),  'ZData', p_cmd_world(3,k));

        set(hFootHistB, 'XData', p_cmd_body(1,1:k), 'YData', p_cmd_body(2,1:k), 'ZData', p_cmd_body(3,1:k));
        set(hPfHistB,   'XData', Pf_body(1,1:k),    'YData', Pf_body(2,1:k),    'ZData', Pf_body(3,1:k));
        set(hCurveB,    'XData', curveB(1,:),       'YData', curveB(2,:),       'ZData', curveB(3,:));
        set(hPfNowB,    'XData', Pf_body(1,k),      'YData', Pf_body(2,k),      'ZData', Pf_body(3,k));
        set(hFootNowB,  'XData', p_cmd_body(1,k),   'YData', p_cmd_body(2,k),   'ZData', p_cmd_body(3,k));

        set(hComDesPath, 'XData', p_com_des_world(1,1:k), 'YData', p_com_des_world(2,1:k));
        set(hComActPath, 'XData', p_com_act_world(1,1:k), 'YData', p_com_act_world(2,1:k));
        set(hPfHistCOM,  'XData', Pf_world(1,1:k),        'YData', Pf_world(2,1:k));
        set(hComDesNow,  'XData', p_com_des_world(1,k),   'YData', p_com_des_world(2,k));
        set(hComActNow,  'XData', p_com_act_world(1,k),   'YData', p_com_act_world(2,k));
        set(hPfNowCOM,   'XData', Pf_world(1,k),          'YData', Pf_world(2,k));

        body_x_world = R_WB(:,:,k) * [1;0;0];
        arrow_len = 0.04;
        set(hBodyArrowCOM, 'XData', p_com_act_world(1,k), ...
                           'YData', p_com_act_world(2,k), ...
                           'UData', arrow_len * body_x_world(1), ...
                           'VData', arrow_len * body_x_world(2));

        set(hCursor1, 'XData', [t(k) t(k)], 'YData', yl1);
        set(hCursor2, 'XData', [t(k) t(k)], 'YData', yl2);
        set(hCursor3, 'XData', [t(k) t(k)], 'YData', yl3);

        title(axW, sprintf('Foot Motion in World Frame | t = %.3f s | s = %.3f', t(k), s(k)));
        title(axB, sprintf('Foot Motion in Current Body Frame | yaw = %.1f deg', rad2deg(yaw(k))));

        drawnow;

        if k < N
            pause(dt / playback_speed);
        end
    end

    if ~isgraphics(fig)
        break;
    end

    pause(2);
end

%% ===================== LOCAL FUNCTIONS =====================
function p = footSwingMIT(P0, Pf, h, s)
    p = cubicBezierInterp(P0, Pf, s);
    if s < 0.5
        u = 2*s;
        p(3) = cubicBezierInterp(P0(3), P0(3)+h, u);
    else
        u = 2*s - 1;
        p(3) = cubicBezierInterp(P0(3)+h, Pf(3), u);
    end
end

function y = cubicBezierInterp(y0, yf, x)
    B = x^3 + 3*x^2*(1-x);
    y = y0 + B*(yf - y0);
end

function R = coordRotZ_MIT(theta)
    c = cos(theta);
    s = sin(theta);
    R = [ c,  s, 0;
         -s,  c, 0;
          0,  0, 1 ];
end

function y = clampScalar(x, lo, hi)
    y = min(max(x, lo), hi);
end
