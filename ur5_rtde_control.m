%UR5_ROD_RRT_IK  Control a real UR5 with attached rod to visit a 3-D grid of target points.
%
%   This script connects to a Universal Robots UR5 via RTDE, builds a
%   simulation model of the UR5 with a metallic rod and base, and uses
%   collision-aware inverse kinematics (IK) plus a manipulator RRT planner
%   to move the real robot through a sequence of Cartesian target positions.
%
%   -------------------------------------------------------------------------
%   WORKFLOW
%   -------------------------------------------------------------------------
%   1) **Clear Workspace** – reset variables, figures, and command window.
%   2) **Parameter Setup**
%        • Metallic rod: length (L0) and radius (R0).
%        • Base dimensions: [Lx Ly Lz].
%        • IK tolerance (posTol), number of random orientation samples
%          (maxTriesIK), and random initial guesses per orientation
%          (maxRandGuess).
%   3) **Target Grid** – load a 3-D grid from grid_cuboid.mat, decimate it,
%      and translate it by a user-defined centroid.
%   4) **Connect to Real UR5** – establish an RTDE client, read current joint
%      configuration, and display the live robot state.
%   5) **Simulation Model** – create a UR5 with rod and rectangular base
%      using BUILDUR5WITHROD for collision checking and visualization.
%   6) **RRT Planner** – create a manipulatorRRT object, ignoring
%      parent–child self-collisions.
%   7) **Move to Home Pose** – plan a collision-free path from the current
%      robot pose to the UR5 home configuration and execute it on the real
%      robot with FOLLOWJOINTWAYPOINTS.
%   8) **Iterative Motion Planning**
%        • For each target point:
%            – Solve IK with IKPOSITIONCOLLISIONAWARE to reach the position.
%            – Validate residual pose error.
%            – Plan a collision-free path from the previous configuration.
%            – Animate the path in simulation.
%            – Command the UR5 to follow the path in hardware.
%
%   -------------------------------------------------------------------------
%   KEY VARIABLES
%   -------------------------------------------------------------------------
%   L0, R0         : Rod length and radius [m].
%   baseDim        : [Lx Ly Lz] base block dimensions [m].
%   posTol         : Allowed Cartesian position error [m].
%   maxTriesIK     : Random orientation samples per target.
%   maxRandGuess   : Random initial guesses per orientation.
%   centroid       : [x y z] translation applied to the loaded grid.
%   targetPositions: N×3 matrix of target Cartesian points [m].
%   qs             : N×6 matrix of solved joint configurations.
%   posError       : 1×N vector of final Cartesian errors [m].
%
%   -------------------------------------------------------------------------
%   REQUIREMENTS
%   -------------------------------------------------------------------------
%   • MATLAB Robotics System Toolbox.
%   • Auxiliary functions:
%        – BUILDUR5WITHROD(L0,R0,baseDim): returns a rigidBodyTree model
%          of the UR5 with attached rod and base.
%        – IKPOSITIONCOLLISIONAWARE: collision-aware inverse kinematics.
%   • Data file: grid_cuboid.mat containing variable grid_cuboid.
%   • Hardware: Universal Robots UR5 reachable at the specified IP address
%     (default '10.59.33.149') and configured for RTDE.
%
%   -------------------------------------------------------------------------
%   USAGE
%   -------------------------------------------------------------------------
%   1. Make sure the UR5 is powered on and in a safe, cleared workspace.
%   2. Adjust IP address and parameters as needed.
%   3. Run this script:
%
%        >> UR5_Rod_RRT_IK
%
% Author: Antonio Figueroa-Duran
% Contact: anfig@dtu.dk

%% ==== Clear workspace ====
clear, clc, close all

%% ==== Parameters ====
% Robot: Metallic rod
L0 = 0.42;      % Length of the metallic rod [meters]
R0 = 0.01;      % Radius of the metallic rod [meters]

% Robot: Base
Lx = 0.08;      % length in X [m]
Ly = 0.08;      % width in Y [m]
Lz = 1;         % height in Z [m]
baseDim = [Lx Ly Lz];

% IK: position tolerance
posTol = 1e-3;          % Position tolerance [meters]
maxTriesIK   = 100;     % IK: orientation samples
maxRandGuess = 10;      % IK: 10 random initial guesses per orientation

% Target positions: centred array grid + centroid
centroid = [1 0 0];     % New custom array centre
load('grid_cuboid.mat');

targetPositions = grid_cuboid(1:2:end,:) + centroid;
numPositions = size(targetPositions,1);

clear grid_cuboid

%% ==== Connect to UR5 ====
ur = urRTDEClient('10.59.33.149','CobotName','universalUR5');

% Current robot state
jointAngles = readJointConfiguration(ur);
figure
show(ur.RigidBodyTree,jointAngles)
title('Real robot: current configuration')

%% ==== Load UR5 Robot ====
robot = buildUR5WithRod(L0, R0, baseDim);
% showdetails(robot)

% Environment
env = {};

% Visualise robot
figure
show(robot, 'Collisions', 'on', 'Visuals', 'on');
hold on
plot3(targetPositions(:,1), targetPositions(:,2), targetPositions(:,3), 'ro', 'MarkerSize', 1, 'LineWidth', 2)
plot3(mean(targetPositions(:,1)), mean(targetPositions(:,2)), mean(targetPositions(:,3)), 'ro', 'MarkerSize', 1, 'LineWidth', 2)
text(targetPositions(1,1), targetPositions(1,2), targetPositions(1,3),'Ini')
text(targetPositions(end,1), targetPositions(end,2), targetPositions(end,3),'End')
hold off
title('UR5+Rod+Base and Target Position')

%% ==== Creat RRT planner ====
rrt = manipulatorRRT(robot,env);
rrt.SkippedSelfCollisions = "parent";

%% Send robot to home configuration
qInitial = homeConfiguration(robot);    % Home config as initial guess

path = plan(rrt, jointAngles, qInitial);

disp('Ready to move the robot?...'), pause
% [result,state] = sendJointConfigurationAndWait(ur,qInitial,'EndTime',5);
followJointWaypoints(ur, path', 'BlendRadius', 0.02)

%% ==== Plan sequence of configurations ====
ndof = length(qInitial);
qs = nan(numPositions, ndof);

endEffector = 'rodTip';

rng(0)  % Ensure reproducibility in IK calculations
posError = nan(1,numPositions);
for iPos = 1:numPositions
    disp(['---- Position ' num2str(iPos) ' -----'])

    % ---- IK ------------------------------------------------------------
    % Solve for the configuration satisfying the desired end effector position
    point = targetPositions(iPos,:);
    [qSol,info] = ikPositionCollisionAware(robot,endEffector, ...
                   point, qInitial, ...
                   maxTriesIK, maxRandGuess);

    % If IK fails to reach the point
    if ~info.success
        warning('IK failed for point %d. Skipping pose.', iPos);
        continue
    end

    % Re-compute FK to check residual error
    eeTform = getTransform(robot, qSol, endEffector);
    posError(iPos) = norm(eeTform(1:3,4).' - point, 2);
    if (posError(iPos) > posTol)
        disp(['ERROR: Pos ' num2str(iPos) ' not reachable by ' num2str(posError(iPos)*1e2) ' cm'])
    end

    % ---- path planning -------------------------------------------------
    path = plan(rrt, qInitial, qSol);
    if isempty(path)
        warning('RRT failed at point %d; skipping.', iPos);
        continue
    end

    % Start from prior solution
    qs(iPos,:) = qSol;      % Store the configuration
    qInitial = qSol;

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

    % Send path to UR5
    disp('Ready to move the robot?...'), pause
    followJointWaypoints(ur, path', 'BlendRadius', 0.02)
end
