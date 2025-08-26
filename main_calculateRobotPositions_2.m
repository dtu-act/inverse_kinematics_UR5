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
L0 = 0.42; % Length of the metallic rod [meters]
R0 = 0.01; % Radius of the metallic rod [meters]

% Robot base
Lx = 0.08;   % length in X [m]
Ly = 0.08;   % width in Y [m]
Lz = 1;   % height in Z [m]
baseDim = [Lx Ly Lz];

% Target positions
load('Array configurations\positions_line.mat','pos_line');
r = pos_line(1:10:end,:);
numPositions = size(r,1);
targetPositions = r(end:-1:1,1:3)*1e-3;  % mm → m
targetPositions(:,2) = -targetPositions(:,2);
targetPositions(:,2) = targetPositions(:,2) - 0.25;

% Note: Quaternions
% Matlab notation is q = [qx, qy, qz, qw]
% It defines a rotation about an axis by a given angle
% q = axang2quat([ux uy uz theta]);

clear pos_line

%% ==== Load UR5 Robot ====
robot = buildUR5WithRod(L0, R0, baseDim);

% Environment
env = {};

% Visualise robot
show(robot, 'Collisions', 'on', 'Visuals', 'on');
hold on
plot3(targetPositions(:,1), targetPositions(:,2), targetPositions(:,3), 'ro', 'MarkerSize', 1, 'LineWidth', 2)
text(targetPositions(1,1), targetPositions(1,2), targetPositions(1,3),'Ini')
text(targetPositions(end,1), targetPositions(end,2), targetPositions(end,3),'End')
title('UR5+Rod+Base and Target Position')

%% ==== Creat RRT planner ====
rrt = manipulatorRRT(robot,env);
rrt.SkippedSelfCollisions = "parent";
% rrt.ValidationDistance = 0.01;

%% ==== Plan sequence of configurations ====
% startConfig = homeConfiguration(robot);

% Inverse Kinematics
maxTries = 20;
rng(0)
ik = inverseKinematics("RigidBodyTree",robot);
weights = [1 1 1 0 0 0];

% Start pose
initialPosition = robot.homeConfiguration;      % Used later when calling planPoseSequence

% ------------------------------------------------------------------
% Main loop over target points
% ------------------------------------------------------------------
currentConfig = robot.homeConfiguration;   % initial

for iPos = 1:numPositions
    rng(0)
    disp(['---- Position ' num2str(iPos) ' -----'])
    % ---- IK ------------------------------------------------------------
    maxTriesIK   = 100;     % from 20 → 200  (orientation samples)
    maxRandGuess = 10;      % 10 random initial guesses per orientation
    [goalCfg,info] = ikPositionCollisionAware(robot,'rodTip', ...
                   targetPositions(iPos,:), currentConfig, ...
                   maxTriesIK, maxRandGuess);

    % How good was the IK solution?
    if ~info.success     % custom flag we set inside ikPositionCollisionAware
        warning('IK failed for point %d', iPos);
        continue
    end

    % Re-compute FK to check residual error
    eeTform = getTransform(robot, goalCfg, 'rodTip');
    posErr  = norm(eeTform(1:3,4).' - targetPositions(iPos,:), 2);

    workspaceRadius = 1.24;      % or use the measured maxReach
    if norm(targetPositions(iPos,:)) > workspaceRadius + 0.01
        warning('Point %d outside measured workspace – skipping', iPos); continue
    end

    if posErr > 1e-3      % 1 mm threshold (adjust to taste)
        warning('Point %d unreachable within 1 mm (error %.1f mm) – skipping', ...
            iPos, posErr*1e3);
        continue
    end

    % ---- path planning -------------------------------------------------
    path = plan(rrt, currentConfig, goalCfg);
    if isempty(path)
        warning('RRT failed at point %d; skipping.', iPos);
        continue
    end

    % Visualise
    interpPath = interpolate(rrt,path);
    clf
    for i = 1:20:size(interpPath,1)
        show(robot,interpPath(i,:)); hold on
        if i == 1
            plot3(targetPositions(:,1), targetPositions(:,2), targetPositions(:,3), 'ro', 'MarkerSize', 1, 'LineWidth', 2)
            text(targetPositions(1,1), targetPositions(1,2), targetPositions(1,3),'Ini')
            text(targetPositions(end,1), targetPositions(end,2), targetPositions(end,3),'End')
        end
        view(2)
        drawnow
        pause(0.1)
    end
    hold off
    pause
    
    % ---- advance -------------------------------------------------------
    currentConfig = goalCfg;            % ready for next point
end
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

%% PLOT MAX REACH
max_reach = 1240e-3;
figure, hold on, grid on
plot3(targetPositions(:,1),targetPositions(:,2),targetPositions(:,3),'r.','MarkerSize',20)
[x,y,z] = sphere(30);                           % UR5 reach ≈ 0.85 m
surf(max_reach*x,max_reach*y,max_reach*z,'FaceAlpha',0.05,'EdgeColor','none')
xlabel('X'), ylabel('Y'), zlabel('Z')
axis equal
title('Red = targets  |  Transparent sphere ≈ UR5 reach')
view(3)

% Sample 50 000 random joint vectors, compute rod-tip XYZ
N = 5e4;
qSamples = zeros(N, numel(homeConfiguration(robot))); % pre-allocate
for ii = 1:N
    qSamples(ii,:) = randomConfiguration(robot);
end
Ttip  = getTransform(robot,qSamples,'rodTip');
xyz   = squeeze(Ttip(1:3,4,:)).';

figure, scatter3(xyz(:,1),xyz(:,2),xyz(:,3),2,'.'), axis equal
xlabel X, ylabel Y, zlabel Z, title('Empirical workspace with rod attached')
hold on
plot3(targetPositions(:,1),targetPositions(:,2),targetPositions(:,3),'ro')