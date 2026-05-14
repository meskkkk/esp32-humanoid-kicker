%% Stage 3: Animated Kicking Motion (with video recording)
% Defines kick keyframes as foot targets, uses IK to get angles,
% then animates the leg smoothly through the full kick.
% Saves output as kick_animation.mp4 — no screen recorder needed!
clear; clc; close all;

%% 1. Leg parameters
L1 = 10;   % Thigh length (cm)
L2 = 10;   % Shin length  (cm)

%% 2. Define the kick keyframes as foot positions (x, y in cm)
% Each row: [x, y]  — foot target at that moment of the kick
keyframes = [
     0,  -19;   % Phase 1: STAND (leg straight down, slightly bent)
    -8,  -16;   % Phase 2: WINDUP (leg pulled back)
     0,  -19;   % Phase 3: PRE-STRIKE (back to neutral, building speed)
    14,  -12;   % Phase 4: STRIKE (foot forward, contact with ball!)
    17,   -5;   % Phase 5: FOLLOW-THROUGH (leg extended high)
     0,  -19;   % Phase 6: RETURN (back to stand)
];

phase_names = {'STAND','WINDUP','PRE-STRIKE','STRIKE','FOLLOW-THROUGH','RETURN'};

%% 3. Compute IK for each keyframe
N_keys = size(keyframes, 1);
thetas = zeros(N_keys, 2);   % Will hold [theta1, theta2] for each keyframe

for i = 1:N_keys
    x = keyframes(i,1);
    y = keyframes(i,2);
    [t1, t2] = inverseKinematics(x, y, L1, L2);
    thetas(i,:) = [t1, t2];
    fprintf('Keyframe %d (%s): target=(%.1f,%.1f) → theta1=%.1f°, theta2=%.1f°\n', ...
            i, phase_names{i}, x, y, rad2deg(t1), rad2deg(t2));
end

%% 4. Interpolate smoothly between keyframes
frames_per_phase = 30;                       % More = smoother/slower
total_frames = (N_keys-1) * frames_per_phase;

theta1_traj = [];
theta2_traj = [];
phase_label = strings(1, total_frames);

for i = 1:(N_keys-1)
    % Linear interpolation of angles between keyframe i and i+1
    t1_seg = linspace(thetas(i,1), thetas(i+1,1), frames_per_phase);
    t2_seg = linspace(thetas(i,2), thetas(i+1,2), frames_per_phase);
    theta1_traj = [theta1_traj, t1_seg];
    theta2_traj = [theta2_traj, t2_seg];
    phase_label((i-1)*frames_per_phase + 1 : i*frames_per_phase) = phase_names{i+1};
end

%% 5. Animate
figure('Color','w','Name','Kicking Animation','Position',[200 200 800 600]);

% Draw the ball (static, where foot hits during STRIKE phase)
ball_x = 17; ball_y = -18;
ball_radius = 2;

% Set up video writer — saves the animation as an MP4 file
video = VideoWriter('kick_animation.mp4','MPEG-4');
video.FrameRate = 30;        % playback speed (frames per second)
video.Quality = 95;          % 0-100, higher = better quality
open(video);
fprintf('\n🎥 Recording animation to kick_animation.mp4 ...\n');

for k = 1:total_frames
    clf; hold on; grid on; axis equal;
    
    t1 = theta1_traj(k);
    t2 = theta2_traj(k);
    
    % Forward kinematics to get knee and foot positions
    x_knee = L1*cos(t1);
    y_knee = L1*sin(t1);
    x_foot = L1*cos(t1) + L2*cos(t1+t2);
    y_foot = L1*sin(t1) + L2*sin(t1+t2);
    
    % Ground line
    plot([-25 25], [-20 -20], 'k-', 'LineWidth', 2);
    
    % Ball (green circle) — moves away if "kicked"
    bx = ball_x; by = ball_y;
    if k > 3.5*frames_per_phase   % After STRIKE phase, ball flies off
        bx = ball_x + (k - 3.5*frames_per_phase)*1.2;
    end
    theta_c = linspace(0, 2*pi, 30);
    fill(bx + ball_radius*cos(theta_c), by + ball_radius*sin(theta_c), ...
         [0.2 0.8 0.2], 'EdgeColor','k');
    
    % Leg segments
    plot([0 x_knee], [0 y_knee], 'b-', 'LineWidth', 5);
    plot([x_knee x_foot], [y_knee y_foot], 'r-', 'LineWidth', 5);
    
    % Joints
    plot(0, 0, 'ko', 'MarkerSize', 12, 'MarkerFaceColor','k');
    plot(x_knee, y_knee, 'ko', 'MarkerSize', 10, 'MarkerFaceColor','y');
    plot(x_foot, y_foot, 'ko', 'MarkerSize', 10, 'MarkerFaceColor','g');
    
    % Labels
    xlabel('X (cm)'); ylabel('Y (cm)');
    title(sprintf('Phase: %s   |   \\theta_1=%.1f°  \\theta_2=%.1f°', ...
          phase_label(k), rad2deg(t1), rad2deg(t2)), 'FontSize', 14);
    xlim([-25 45]); ylim([-25 8]);
    
    drawnow;
    
    % Capture the current frame and write it to the video file
    frame = getframe(gcf);
    writeVideo(video, frame);
    
    pause(0.02);
end

% Close the video file — this finalizes the MP4
close(video);
fprintf('✅ Video saved as kick_animation.mp4 in your current folder!\n');
fprintf('✅ Animation complete!\n');

%% --- Helper function: Inverse Kinematics ---
function [theta1, theta2] = inverseKinematics(x, y, L1, L2)
    distance = sqrt(x^2 + y^2);
    if distance > (L1 + L2) || distance < abs(L1 - L2)
        error('Target (%.1f, %.1f) unreachable', x, y);
    end
    cos_theta2 = (x^2 + y^2 - L1^2 - L2^2) / (2*L1*L2);
    theta2 = -acos(cos_theta2);   % knee-down (kicking pose)
    theta1 = atan2(y, x) - atan2(L2*sin(theta2), L1 + L2*cos(theta2));
end