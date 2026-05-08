%% Stage 1: Forward Kinematics of 2-DOF Planar Leg
% This script computes where the foot is when you give it hip/knee angles.
clear; clc; close all;

%% 1. Define leg parameters (in cm)
L1 = 10;   % Thigh length
L2 = 10;   % Shin length

%% 2. Define joint angles (in degrees, then convert to radians)
theta1_deg = -60;   % Hip angle (negative = leg points down)
theta2_deg = 30;    % Knee angle (bent forward)

theta1 = deg2rad(theta1_deg);
theta2 = deg2rad(theta2_deg);

%% 3. Forward Kinematics equations (from your project doc)
% Hip joint is at origin (0,0)
% Knee joint position:
x_knee = L1 * cos(theta1);
y_knee = L1 * sin(theta1);

% Foot position:
x_foot = L1*cos(theta1) + L2*cos(theta1 + theta2);
y_foot = L1*sin(theta1) + L2*sin(theta1 + theta2);

fprintf('Knee position: (%.2f, %.2f) cm\n', x_knee, y_knee);
fprintf('Foot position: (%.2f, %.2f) cm\n', x_foot, y_foot);

%% 4. Plot the leg
figure('Color','w','Name','2-DOF Leg FK');
hold on; grid on; axis equal;

% Draw thigh (hip → knee)
plot([0 x_knee], [0 y_knee], 'b-', 'LineWidth', 4);

% Draw shin (knee → foot)
plot([x_knee x_foot], [y_knee y_foot], 'r-', 'LineWidth', 4);

% Draw joints
plot(0, 0, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');          % Hip
plot(x_knee, y_knee, 'ko', 'MarkerSize', 10, 'MarkerFaceColor','y'); % Knee
plot(x_foot, y_foot, 'ko', 'MarkerSize', 10, 'MarkerFaceColor','g'); % Foot

% Labels
text(0, 1, 'Hip', 'FontSize', 12);
text(x_knee+0.5, y_knee, 'Knee', 'FontSize', 12);
text(x_foot+0.5, y_foot, 'Foot', 'FontSize', 12);

xlabel('X (cm)'); ylabel('Y (cm)');
title(sprintf('Leg Pose: \\theta_1=%d°, \\theta_2=%d°', theta1_deg, theta2_deg));
xlim([-25 25]); ylim([-25 5]);
legend('Thigh','Shin','Location','northwest');