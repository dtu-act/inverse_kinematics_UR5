%% OBTAIN JOINT COORDINATES FOR UR5 ROBOT WITH ROD AND BASE ATTACHED
% Each element of the output vector represents the joint angle (in radians)
% for one of the UR5's 6 joints:
% Index	| Joint	        | Meaning
% ------------------------------------
%   1	| Shoulder pan	| Base rotation
%   2	| Shoulder lift	| Arm up/down
%   3	| Elbow	        | Arm extend
%   4	| Wrist 1	    | Wrist pitch
%   5	| Wrist 2	    | Wrist yaw
%   6	| Wrist 3	    | Tool roll

%% ==== Clear workspace ====
clear, clc, close all

%% ==== Parameters ====
% Metallic rod
L0 = 0.57; % Length of the metallic rod [meters]
R0 = 0.01; % Radius of the metallic rod [meters]

% Robot base
Lx = 0.08;   % length in X [m]
Ly = 0.08;   % width in Y [m]
Lz = 1;   % height in Z [m]
baseDim = [Lx Ly Lz];

% Target positions
load('Array configurations\positions_line.mat','pos_line');
r = pos_line;
numPositions = size(r,1);
targetPositions = r(:,1:3)*1e-3; % Desired end-effector position (XYZ)

% Note: Quaternions
% Matlab notation is q = [qx, qy, qz, qw]
% It defines a rotation about an axis by a given angle
% q = axang2quat([ux uy uz theta]);

%% ==== Load UR5 Robot ====
robot = buildUR5WithRod(L0, R0, baseDim);

% Visualise robot
% show(robot, 'Collisions', 'on', 'Visuals', 'on');
% hold on
% plot3(targetPositions(:,1), targetPositions(:,2), targetPositions(:,3), 'ro', 'MarkerSize', 1, 'LineWidth', 2)
% title('UR5+Rod+Base and Target Position')

%% ==== Plan sequence of configurations ====
initialPosition = homeConfiguration(robot);
configs = planPoseSequence(robot, initialPosition, targetPositions);

%% ==== VISUALISE FULL PATH ====
if ~isempty(configs)
    figure
    ax = show(robot, configs(1,:), 'Collisions', 'on', 'Visuals', 'on');
    view(135, 20);
    hold on;
    plot3(targetPositions(:,1), targetPositions(:,2), targetPositions(:,3), 'ro', 'MarkerSize', 1, 'LineWidth', 2)
    title('UR5 with Rod: Animated Path');

    for i = 1:size(configs,1)
        show(robot, configs(i,:), 'PreservePlot', false, 'Parent', ax);
        pause(0.1); % Adjust speed of animation
    end
end
