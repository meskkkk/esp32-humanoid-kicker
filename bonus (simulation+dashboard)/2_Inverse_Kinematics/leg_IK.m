%% Stage 2: Inverse Kinematics of 2-DOF Planar Leg
% Given a target foot position (x, y), compute the required hip and knee angles.
clear; clc; close all;

%% 1. Leg parameters (must match Stage 1)
L1 = 10;   % Thigh length (cm)
L2 = 10;   % Shin length  (cm)

%% 2. Target foot position (this would come from the camera in real life)
x_target = 12;   % cm forward from hip
y_target = -14;  % cm below hip (negative Y = down)

%% 3. Inverse Kinematics using Law of Cosines
% Step A: Check if target is reachable
distance = sqrt(x_target^2 + y_target^2);
if distance > (L1 + L2)
    error('Target is too far! Max reach = %.2f cm, requested = %.2f cm', ...
          L1+L2, distance);
end
if distance < abs(L1 - L2)
    error('Target is too close! Min reach = %.2f cm', abs(L1-L2));
end

% Step B: Compute knee angle theta2 using law of cosines
cos_theta2 = (x_target^2 + y_target^2 - L1^2 - L2^2) / (2 * L1 * L2);

% There are 2 solutions: "knee-up" and "knee-down". We pick knee-down (more natural for kicking).
theta2 = -acos(cos_theta2);   % Negative = knee bends forward (kicking pose)

% Step C: Compute hip angle theta1
theta1 = atan2(y_target, x_target) - atan2(L2*sin(theta2), L1 + L2*cos(theta2));

%% 4. Convert to degrees for display / servo commands
theta1_deg = rad2deg(theta1);
theta2_deg = rad2deg(theta2);

fprintf('=== IK Solution ===\n');
fprintf('Target foot position: (%.2f, %.2f) cm\n', x_target, y_target);
fprintf('Required Hip angle  theta1 = %.2f deg\n', theta1_deg);
fprintf('Required Knee angle theta2 = %.2f deg\n', theta2_deg);

%% 5. VERIFY using Forward Kinematics (very important — catches errors!)
x_check = L1*cos(theta1) + L2*cos(theta1 + theta2);
y_check = L1*sin(theta1) + L2*sin(theta1 + theta2);
fprintf('\n=== FK Verification ===\n');
fprintf('Computed foot position: (%.2f, %.2f) cm\n', x_check, y_check);
fprintf('Error: %.4f cm (should be ~0)\n', sqrt((x_check-x_target)^2 + (y_check-y_target)^2));

%% 6. Plot the solution
figure('Color','w','Name','2-DOF Leg IK');
hold on; grid on; axis equal;

x_knee = L1 * cos(theta1);
y_knee = L1 * sin(theta1);

plot([0 x_knee], [0 y_knee], 'b-', 'LineWidth', 4);
plot([x_knee x_check], [y_knee y_check], 'r-', 'LineWidth', 4);
plot(0, 0, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
plot(x_knee, y_knee, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'y');
plot(x_check, y_check, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot(x_target, y_target, 'rx', 'MarkerSize', 15, 'LineWidth', 3);  % Red X = target

text(0, 1, 'Hip', 'FontSize', 12);
text(x_knee+0.5, y_knee, 'Knee', 'FontSize', 12);
text(x_target+0.5, y_target-1, 'Target', 'FontSize', 12, 'Color','r');

xlabel('X (cm)'); ylabel('Y (cm)');
title(sprintf('IK: target (%.1f, %.1f) → \\theta_1=%.1f°, \\theta_2=%.1f°', ...
      x_target, y_target, theta1_deg, theta2_deg));
xlim([-25 25]); ylim([-25 5]);
legend('Thigh','Shin','Location','northwest');

%% 7. Map to servo PWM (SG90 servo: 0°=500us, 180°=2500us)
% Most hobby servos accept angles 0-180°. We need to offset our angles to fit.
servo1_angle = theta1_deg + 90;    % Shift hip so -90° → 0°, 90° → 180°
servo2_angle = theta2_deg + 90;    % Shift knee similarly

PWM_min = 500;   % microseconds at 0°
PWM_max = 2500;  % microseconds at 180°
servo1_PWM = PWM_min + (servo1_angle/180) * (PWM_max - PWM_min);
servo2_PWM = PWM_min + (servo2_angle/180) * (PWM_max - PWM_min);

fprintf('\n=== Servo Commands ===\n');
fprintf('Hip  servo: %.1f° → PWM %.0f us\n', servo1_angle, servo1_PWM);
fprintf('Knee servo: %.1f° → PWM %.0f us\n', servo2_angle, servo2_PWM);