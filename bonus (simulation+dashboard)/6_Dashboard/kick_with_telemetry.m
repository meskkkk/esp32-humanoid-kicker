%% LIVE Dashboard Mode — writes telemetry.json each frame while animating
% How to run this:
%   1. Start Python server in this folder:  python -m http.server 8000
%   2. Open http://localhost:8000/dashboard.html in Chrome
%   3. Run THIS script in MATLAB — dashboard updates LIVE
clear; clc; close all;

%% Parameters
L1 = 10; L2 = 10;
hip_width = 8; torso_h = 15; torso_r = 3; head_r = 3;

%% Kick keyframes
keyframes = [
     0,  -19;   % STAND
    -8,  -16;   % WINDUP
     0,  -19;   % PRE-STRIKE
    14,  -12;   % STRIKE
    17,   -5;   % FOLLOW-THROUGH
     0,  -19;   % RETURN
];
phase_names = {'STAND','WINDUP','PRE-STRIKE','STRIKE','FOLLOW-THROUGH','RETURN'};

%% Compute IK and interpolate
N = size(keyframes,1);
thetas = zeros(N,2);
for i = 1:N
    [t1, t2] = inverseKinematics(keyframes(i,1), keyframes(i,2), L1, L2);
    thetas(i,:) = [t1, t2];
end

frames_per_phase = 30;
total_frames = (N-1) * frames_per_phase;
traj = zeros(total_frames, 2);
phase_label = strings(1, total_frames);
for i = 1:(N-1)
    seg1 = linspace(thetas(i,1), thetas(i+1,1), frames_per_phase);
    seg2 = linspace(thetas(i,2), thetas(i+1,2), frames_per_phase);
    rng = (i-1)*frames_per_phase + 1 : i*frames_per_phase;
    traj(rng,1) = seg1;
    traj(rng,2) = seg2;
    phase_label(rng) = phase_names{i+1};
end

%% Figure
figure('Color',[0.85 0.92 1],'Name','🔴 LIVE 3D Kick','Position',[100 100 900 600]);
[Xs, Ys, Zs] = sphere(20);

fprintf('\n🔴 LIVE MODE — Dashboard should be open at http://localhost:8000/dashboard.html\n');
fprintf('   Starting animation in 3 seconds...\n\n');
pause(3);

%% Animate + stream telemetry
for k = 1:total_frames
    clf; hold on; axis equal; grid on;
    xlabel('X (cm)'); ylabel('Y (cm)'); zlabel('Z (cm)');
    xlim([-25 55]); ylim([-20 20]); zlim([-22 25]);
    view(40 + k*0.25, 18);

    [Xg, Yg] = meshgrid(-25:5:55, -20:5:20);
    surf(Xg, Yg, ones(size(Xg))*(-L1-L2), 'FaceColor',[0.4 0.8 0.4], ...
         'EdgeColor',[0.3 0.6 0.3], 'FaceAlpha',0.6);

    t1 = traj(k,1); t2 = traj(k,2);

    Rhip  = [0, -hip_width/2, 0];
    Rknee = Rhip  + [L1*cos(t1), 0, L1*sin(t1)];
    Rfoot = Rknee + [L2*cos(t1+t2), 0, L2*sin(t1+t2)];
    plot3([Rhip(1) Rknee(1)], [Rhip(2) Rknee(2)], [Rhip(3) Rknee(3)], 'Color',[0.2 0.3 0.9], 'LineWidth', 12);
    plot3([Rknee(1) Rfoot(1)], [Rknee(2) Rfoot(2)], [Rknee(3) Rfoot(3)], 'Color',[0.9 0.2 0.2], 'LineWidth', 12);

    Lhip = [0, hip_width/2, 0]; Lknee = Lhip + [0,0,-L1]; Lfoot = Lknee + [0,0,-L2];
    plot3([Lhip(1) Lknee(1)], [Lhip(2) Lknee(2)], [Lhip(3) Lknee(3)], 'Color',[0.4 0.4 0.4], 'LineWidth', 12);
    plot3([Lknee(1) Lfoot(1)], [Lknee(2) Lfoot(2)], [Lknee(3) Lfoot(3)], 'Color',[0.4 0.4 0.4], 'LineWidth', 12);

    plotSphere(Rhip, 1.2, [0.1 0.1 0.1], Xs, Ys, Zs);
    plotSphere(Rknee, 1.2, [0.9 0.9 0.1], Xs, Ys, Zs);
    plotSphere(Rfoot, 1.5, [0.1 0.8 0.1], Xs, Ys, Zs);
    plotSphere(Lhip, 1.2, [0.1 0.1 0.1], Xs, Ys, Zs);
    plotSphere(Lknee, 1.2, [0.4 0.4 0.4], Xs, Ys, Zs);
    plotSphere(Lfoot, 1.5, [0.4 0.4 0.4], Xs, Ys, Zs);

    [Xt, Yt, Zt] = cylinder(torso_r, 20);
    surf(Xt, Yt, Zt*torso_h, 'FaceColor',[0.2 0.4 0.8], 'EdgeColor','none');
    surf(head_r*Xs, head_r*Ys, head_r*Zs + torso_h + head_r + 0.5, ...
         'FaceColor',[0.95 0.75 0.6], 'EdgeColor','none');

    ball_x0 = 17; ball_y0 = -18;
    strike_frame = 3.5 * frames_per_phase;
    if k > strike_frame
        dt = k - strike_frame;
        ball_x_curr = ball_x0 + dt*0.8;
        ball_y_curr = ball_y0 + dt*0.35 - 0.012*dt^2;
    else
        ball_x_curr = ball_x0; ball_y_curr = ball_y0;
    end
    surf(2*Xs + ball_x_curr, 2*Ys, 2*Zs + ball_y_curr, 'FaceColor',[0.1 0.8 0.1], 'EdgeColor','k');

    lighting gouraud; camlight('headlight');
    title(sprintf('🔴 LIVE | Phase: %s | \\theta_1=%.1f° \\theta_2=%.1f°', ...
          phase_label(k), rad2deg(t1), rad2deg(t2)), 'FontSize', 13, 'FontWeight', 'bold');

    % --- STREAM TELEMETRY LIVE TO JSON ---
    hip_servo_angle  = rad2deg(t1) + 90;
    knee_servo_angle = rad2deg(t2) + 90;
    hip_pwm  = 500 + (hip_servo_angle  / 180) * 2000;
    knee_pwm = 500 + (knee_servo_angle / 180) * 2000;

    telemetry = struct( ...
        'phase',        char(phase_label(k)), ...
        'frame',        k, ...
        'total_frames', total_frames, ...
        'theta1',       rad2deg(t1), ...
        'theta2',       rad2deg(t2), ...
        'hip_pwm',      hip_pwm, ...
        'knee_pwm',     knee_pwm, ...
        'foot_x',       Rfoot(1), ...
        'foot_y',       Rfoot(3), ...
        'ball_x',       ball_x_curr, ...
        'ball_y',       ball_y_curr);
    
    json_str = jsonencode(telemetry);
    fid = fopen('telemetry.json','w');
    fprintf(fid, '%s', json_str);
    fclose(fid);

    drawnow;
    pause(0.05);
end

fprintf('✅ Live stream complete!\n');

%% Helpers
function [theta1, theta2] = inverseKinematics(x, y, L1, L2)
    cos_t2 = (x^2 + y^2 - L1^2 - L2^2) / (2*L1*L2);
    theta2 = -acos(cos_t2);
    theta1 = atan2(y, x) - atan2(L2*sin(theta2), L1 + L2*cos(theta2));
end

function plotSphere(center, radius, color, Xs, Ys, Zs)
    surf(radius*Xs + center(1), radius*Ys + center(2), radius*Zs + center(3), ...
         'FaceColor', color, 'EdgeColor','none');
end