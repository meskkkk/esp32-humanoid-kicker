%% Stage 4 v4: Full 3D Humanoid Football-Kicking Robot (WITH ARMS)
% Complete humanoid with torso, head, two legs, AND two arms that swing.
% Saves output as humanoid_3D_kick.mp4
clear; clc; close all;

%% 1. Robot dimensions (cm)
L1 = 10;        % Thigh length
L2 = 10;        % Shin length
hip_width = 8;  % Distance between hips
torso_h = 15;   % Torso height
torso_r = 3;    % Torso radius
head_r = 3;     % Head radius
leg_r = 1.5;    % Leg segment radius
arm_upper = 6;  % Upper arm length
arm_lower = 5;  % Forearm length

%% 2. Kick keyframes: [foot_x, foot_z] in sagittal plane
keyframes = [
     0,  -19;   % STAND
    -8,  -16;   % WINDUP
     0,  -19;   % PRE-STRIKE
    14,  -12;   % STRIKE
    17,   -5;   % FOLLOW-THROUGH
     0,  -19;   % RETURN
];
phase_names = {'STAND','WINDUP','PRE-STRIKE','STRIKE','FOLLOW-THROUGH','RETURN'};

%% 3. Compute IK for each keyframe
N = size(keyframes,1);
thetas = zeros(N,2);
for i = 1:N
    [t1, t2] = inverseKinematics(keyframes(i,1), keyframes(i,2), L1, L2);
    thetas(i,:) = [t1, t2];
    fprintf('Keyframe %d (%-15s): theta1=%6.1f°   theta2=%6.1f°\n', ...
            i, phase_names{i}, rad2deg(t1), rad2deg(t2));
end

%% 4. Interpolate smoothly between keyframes
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

%% 5. Set up video writer
video = VideoWriter('humanoid_3D_kick.mp4','MPEG-4');
video.FrameRate = 30;
video.Quality = 95;
open(video);
fprintf('\n🎥 Recording 3D animation ...\n');

%% 6. Animate — clear and redraw the ENTIRE scene each frame
figure('Color',[0.85 0.92 1],'Name','3D Humanoid Kick','Position',[100 100 1000 700]);
[Xs, Ys, Zs] = sphere(20);

for k = 1:total_frames
    clf;    % Clear everything
    hold on; axis equal; grid on;
    xlabel('X (cm)'); ylabel('Y (cm)'); zlabel('Z (cm)');
    xlim([-25 55]); ylim([-20 20]); zlim([-22 25]);
    view(40 + k*0.25, 18);
    
    % --- GROUND ---
    [Xg, Yg] = meshgrid(-25:5:55, -20:5:20);
    Zg = ones(size(Xg)) * (-L1 - L2);
    surf(Xg, Yg, Zg, 'FaceColor',[0.4 0.8 0.4], 'EdgeColor',[0.3 0.6 0.3], ...
         'FaceAlpha',0.6);
    
    t1 = traj(k,1);
    t2 = traj(k,2);
    
    % --- RIGHT LEG (kicking) ---
    Rhip  = [0,           -hip_width/2, 0];
    Rknee = Rhip  + [L1*cos(t1), 0, L1*sin(t1)];
    Rfoot = Rknee + [L2*cos(t1+t2), 0, L2*sin(t1+t2)];
    
    plot3([Rhip(1) Rknee(1)], [Rhip(2) Rknee(2)], [Rhip(3) Rknee(3)], ...
          'Color',[0.2 0.3 0.9], 'LineWidth', 12);      % blue thigh
    plot3([Rknee(1) Rfoot(1)], [Rknee(2) Rfoot(2)], [Rknee(3) Rfoot(3)], ...
          'Color',[0.9 0.2 0.2], 'LineWidth', 12);      % red shin
    
    % --- LEFT LEG (support — straight down) ---
    Lhip  = [0,  hip_width/2, 0];
    Lknee = Lhip  + [0, 0, -L1];
    Lfoot = Lknee + [0, 0, -L2];
    plot3([Lhip(1) Lknee(1)], [Lhip(2) Lknee(2)], [Lhip(3) Lknee(3)], ...
          'Color',[0.4 0.4 0.4], 'LineWidth', 12);      % gray
    plot3([Lknee(1) Lfoot(1)], [Lknee(2) Lfoot(2)], [Lknee(3) Lfoot(3)], ...
          'Color',[0.4 0.4 0.4], 'LineWidth', 12);
    
    % --- Leg Joints (spheres) ---
    plotSphere(Rhip,  1.2, [0.1 0.1 0.1], Xs, Ys, Zs);
    plotSphere(Rknee, 1.2, [0.9 0.9 0.1], Xs, Ys, Zs);
    plotSphere(Rfoot, 1.5, [0.1 0.8 0.1], Xs, Ys, Zs);
    plotSphere(Lhip,  1.2, [0.1 0.1 0.1], Xs, Ys, Zs);
    plotSphere(Lknee, 1.2, [0.4 0.4 0.4], Xs, Ys, Zs);
    plotSphere(Lfoot, 1.5, [0.4 0.4 0.4], Xs, Ys, Zs);
    
    % --- TORSO (blue cylinder) ---
    [Xt, Yt, Zt] = cylinder(torso_r, 20);
    Zt = Zt * torso_h;
    surf(Xt, Yt, Zt, 'FaceColor',[0.2 0.4 0.8], 'EdgeColor','none');
    
    % --- ARMS (swing naturally — opposite to the kicking leg) ---
    shoulder_z = torso_h - 1;          % just below top of torso
    shoulder_offset = torso_r + 0.3;   % sticks out sideways
    arm_swing = -0.4 * sin(t1 + pi/2); % counter-rotates with leg
    
    % Right arm
    Rshoulder = [0,  shoulder_offset, shoulder_z];
    Relbow = Rshoulder + [arm_upper*sin(arm_swing), 0, -arm_upper*cos(arm_swing)];
    Rhand  = Relbow    + [arm_lower*sin(arm_swing*1.5), 0, -arm_lower*cos(arm_swing*1.5)];
    plot3([Rshoulder(1) Relbow(1)], [Rshoulder(2) Relbow(2)], [Rshoulder(3) Relbow(3)], ...
          'Color',[0.2 0.4 0.8], 'LineWidth', 10);      % blue upper arm
    plot3([Relbow(1) Rhand(1)], [Relbow(2) Rhand(2)], [Relbow(3) Rhand(3)], ...
          'Color',[0.95 0.75 0.6], 'LineWidth', 10);    % skin forearm
    plotSphere(Rshoulder, 1.0, [0.1 0.1 0.1], Xs, Ys, Zs);
    plotSphere(Relbow,    0.9, [0.9 0.9 0.1], Xs, Ys, Zs);
    plotSphere(Rhand,     1.1, [0.95 0.75 0.6], Xs, Ys, Zs);
    
    % Left arm (mirrored — swings the opposite way)
    Lshoulder = [0, -shoulder_offset, shoulder_z];
    Lelbow = Lshoulder + [arm_upper*sin(-arm_swing), 0, -arm_upper*cos(-arm_swing)];
    Lhand  = Lelbow    + [arm_lower*sin(-arm_swing*1.5), 0, -arm_lower*cos(-arm_swing*1.5)];
    plot3([Lshoulder(1) Lelbow(1)], [Lshoulder(2) Lelbow(2)], [Lshoulder(3) Lelbow(3)], ...
          'Color',[0.2 0.4 0.8], 'LineWidth', 10);
    plot3([Lelbow(1) Lhand(1)], [Lelbow(2) Lhand(2)], [Lelbow(3) Lhand(3)], ...
          'Color',[0.95 0.75 0.6], 'LineWidth', 10);
    plotSphere(Lshoulder, 1.0, [0.1 0.1 0.1], Xs, Ys, Zs);
    plotSphere(Lelbow,    0.9, [0.9 0.9 0.1], Xs, Ys, Zs);
    plotSphere(Lhand,     1.1, [0.95 0.75 0.6], Xs, Ys, Zs);
    
    % --- HEAD (sphere) ---
    head_z = torso_h + head_r + 0.5;
    surf(head_r*Xs, head_r*Ys, head_r*Zs + head_z, ...
         'FaceColor',[0.95 0.75 0.6], 'EdgeColor','none');
    
    % --- BALL physics ---
    ball_x0 = 22; ball_y0 = 0; ball_z0 = -L1-L2+2;
    strike_frame = 3.5 * frames_per_phase;
    if k > strike_frame
        dt = k - strike_frame;
        bx = ball_x0 + dt*0.8;
        bz = ball_z0 + dt*0.35 - 0.012*dt^2;
    else
        bx = ball_x0; bz = ball_z0;
    end
    surf(2*Xs + bx, 2*Ys + ball_y0, 2*Zs + bz, ...
         'FaceColor',[0.1 0.8 0.1], 'EdgeColor','k');
    
    % --- Lighting ---
    lighting gouraud; camlight('headlight');
    
    title(sprintf('Phase: %s   |   \\theta_1=%.1f°   \\theta_2=%.1f°', ...
          phase_label(k), rad2deg(t1), rad2deg(t2)), 'FontSize',14,'FontWeight','bold');
    
    drawnow;
    frame = getframe(gcf);
    writeVideo(video, frame);
end

close(video);
fprintf('✅ Video saved as humanoid_3D_kick.mp4\n');
fprintf('✅ 3D animation complete!\n');

%% ============ HELPER FUNCTIONS ============

function [theta1, theta2] = inverseKinematics(x, y, L1, L2)
    distance = sqrt(x^2 + y^2);
    if distance > (L1+L2) || distance < abs(L1-L2)
        error('Target (%.1f, %.1f) unreachable', x, y);
    end
    cos_t2 = (x^2 + y^2 - L1^2 - L2^2) / (2*L1*L2);
    theta2 = -acos(cos_t2);
    theta1 = atan2(y, x) - atan2(L2*sin(theta2), L1 + L2*cos(theta2));
end

function plotSphere(center, radius, color, Xs, Ys, Zs)
    surf(radius*Xs + center(1), radius*Ys + center(2), radius*Zs + center(3), ...
         'FaceColor', color, 'EdgeColor','none');
end