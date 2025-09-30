%UR5_ROD_RRT_IK  Control a real UR5 with attached rod to visit grid targets inside a scenario environment.
%
%   This script connects to a Universal Robots UR5 via RTDE, builds a
%   simulation model of the UR5 with a metallic rod and base, and uses
%   collision-aware inverse kinematics (IK) plus a manipulator RRT planner
%   to move the real robot through a sequence of Cartesian target positions
%   defined within a scenario environment.
%
%   -------------------------------------------------------------------------
%   WORKFLOW
%   -------------------------------------------------------------------------
%   1) **Clear Workspace** – reset variables, figures, and command window.
%   2) **Parameter Setup**
%        • Metallic rod: length (L0) and radius (R0).
%        • Base dimensions: [Lx Ly Lz].
%        • IK tolerance (posTol).
%   3) **Scenario & Target Grid**
%        • Load a JSON environment description (e.g. 'Kitchen') with
%          `loadScenario`.
%        • Load a 3-D grid of points from grid_cuboid.mat, decimate it, and
%          translate by a user-defined centroid.
%   4) **Connect to Real UR5**
%        • Establish an RTDE client (`urRTDEClient`).
%        • Read current joint configuration and display the live robot state.
%   5) **Simulation Model**
%        • Build a UR5 with attached rod and rectangular base
%          (`buildUR5WithRod`).
%        • Place robot at a global offset (posRobotGlobal).
%        • Insert collision objects from the scenario with a safety margin.
%   6) **RRT Planner**
%        • Create a manipulatorRRT object with the robot and environment.
%        • Ignore parent–child self-collisions.
%   7) **Move to Home Pose**
%        • Plan a collision-free path from the current robot pose to the
%          UR5 home configuration and execute it on the real robot with
%          `followJointWaypoints`.
%   8) **Iterative Motion Planning**
%        • For each target point:
%            – Solve IK with `ikPositionCollisionAware`.
%            – Check residual Cartesian error against tolerance.
%            – Plan a collision-free path from the previous configuration.
%            – Animate the path in simulation.
%            – Send the path to the UR5 and verify motion.
%
%   -------------------------------------------------------------------------
%   KEY VARIABLES
%   -------------------------------------------------------------------------
%   L0, R0         : Rod length and radius [m].
%   baseDim        : [Lx Ly Lz] base block dimensions [m].
%   posTol         : Allowed Cartesian position error [m].
%   centroid       : [x y z] translation applied to the loaded grid.
%   targetPositions: N×3 matrix of target Cartesian points [m].
%   qs             : N×6 matrix of solved joint configurations.
%   posError       : 1×N vector of final Cartesian errors [m].
%   posRobotGlobal : [x y z] offset of robot base in world frame.
%   env            : Cell array of collisionBox objects from the scenario.
%
%   -------------------------------------------------------------------------
%   REQUIREMENTS
%   -------------------------------------------------------------------------
%   • MATLAB Robotics System Toolbox.
%   • RTDE client utilities (`urRTDEClient`, `readJointConfiguration`,
%     `followJointWaypoints`, etc.).
%   • Auxiliary functions:
%        – buildUR5WithRod(L0,R0,baseDim): returns a rigidBodyTree model
%          of the UR5 with attached rod and base.
%        – ikPositionCollisionAware: collision-aware inverse kinematics.
%   • Data files:
%        – grid_cuboid.mat containing variable `grid_cuboid`.
%        – JSON scenario files (e.g. Environment/kitchen.json).
%   • Hardware: Universal Robots UR5 reachable at the specified IP address
%     (default '10.59.33.149') and configured for RTDE.
%
%   -------------------------------------------------------------------------
%   USAGE
%   -------------------------------------------------------------------------
%   1. Make sure the UR5 is powered on and the workspace is clear.
%   2. Adjust IP address, scenario name, and parameters as needed.
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
L0 = 0.59;      % Length of the metallic rod [meters]
R0 = 0.01;      % Radius of the metallic rod [meters]

% Robot: Base
Lx = 0.08;      % length in X [m]
Ly = 0.08;      % width in Y [m]
Lz = 1;         % height in Z [m]
baseDim = [Lx Ly Lz];

% IK: position tolerance
posTol = 1e-3;          % Position tolerance [meters]

% Load scenario
scenarioName = 'Kitchen';
scenario = loadScenario(['Environment/' lower(scenarioName) '.json']);

% Target positions: centred array grid + centroid
centroid = [1 0 0.35];     % New custom array centre (aligned with Lspk)
load('grid_cuboid.mat');

% Sample nPosSample positions
nPosSample = 100;
posIdx = sort(randperm(size(grid_cuboid,1),nPosSample));

targetPositions = grid_cuboid(posIdx,:) + centroid;
numPositions = size(targetPositions,1);

clear grid_cuboid

%% ==== Connect to UR5 ====
ur = urRTDEClient('10.59.33.149','CobotName','universalUR5');

%% Current robot state
% IMPORTANT NOTE:
% The real robot's reference is the opposite to the simulations. When
% visualising the real robot (i.e. using ur.RigidBodyTree), the X-axis is
% pointing towards the opposite direction

jointAngles = readJointConfiguration(ur);
figure(1)
show(ur.RigidBodyTree,jointAngles,'collision','on');
title('Real robot: current configuration')

%% ==== Load UR5 Robot & define environment ====
robot = buildUR5WithRod(L0, R0, baseDim);
% showdetails(robot)

% Pos robot wrt global world
posRobotGlobal = [1.31 1.86 1];

% Environment: kitchen
margin = 5e-2;     % Distance margin to collision boxes [m]
nObjects = numel(scenario.objects);

env = cell(1,nObjects);
for iObj = 1:nObjects
    dimsPlusMargin = scenario.objects(iObj).dimensions + margin;
    env{iObj} = collisionBox(dimsPlusMargin(1), dimsPlusMargin(2), dimsPlusMargin(3));   % (Lx, Ly, Lz)
    env{iObj}.Pose = trvec2tform(scenario.objects(iObj).position'-posRobotGlobal);
end

% Visualise robot
jointAngles = readJointConfiguration(ur);
figure(2)
show(robot, jointAngles, 'Collisions', 'on', 'Visuals', 'on');
hold on
for iObj = 1:nObjects, show(env{iObj}); end
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
jointAngles = readJointConfiguration(ur);

path = plan(rrt, jointAngles, qInitial);

% Visualise
interpPath = interpolate(rrt,path);
figure(2)
clf
for i = 1:20:size(interpPath,1)
    show(robot,interpPath(i,:)); hold on
    if i == 1
        plot3(targetPositions(:,1), targetPositions(:,2), targetPositions(:,3), 'ro', 'MarkerSize', 1, 'LineWidth', 2)
        text(targetPositions(1,1), targetPositions(1,2), targetPositions(1,3),'Ini')
        text(targetPositions(end,1), targetPositions(end,2), targetPositions(end,3),'End')
        for iObj = 1:nObjects, show(env{iObj}); end
    end
    view(2)
    drawnow
    pause(0.1)
end
show(robot,interpPath(end,:));
view(2)
drawnow
hold off

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
                   point, qInitial);

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
    figure(2)
    clf
    for i = 1:20:size(interpPath,1)
        show(robot,interpPath(i,:)); hold on
        if i == 1
            plot3(targetPositions(:,1), targetPositions(:,2), targetPositions(:,3), 'ro', 'MarkerSize', 1, 'LineWidth', 2)
            text(targetPositions(1,1), targetPositions(1,2), targetPositions(1,3),'Ini')
            text(targetPositions(end,1), targetPositions(end,2), targetPositions(end,3),'End')
            for iObj = 1:nObjects, show(env{iObj}); end
        end
        view(2)
        drawnow
        pause(0.1)
    end
    show(robot,interpPath(end,:));
    view(2)
    drawnow
    hold off

    % Send path to UR5
    % disp('Ready to move the robot?...'), pause
    followJointWaypoints(ur, path', 'BlendRadius', 0.02)

    % Wait for the robot to move
    jointVelocity = readJointVelocity(ur);
    while sum(abs(jointVelocity)) > 0
        jointVelocity = readJointVelocity(ur);
    end
    pause(0.05)

    % Verify connection is still operative by comparing target vs real
    % joint configuration
    jointAngles = readJointConfiguration(ur);
    if rad2deg(norm(jointAngles-qSol)) > 3
        error(['Connection lost at position ' num2str(iPos)])
    end
end