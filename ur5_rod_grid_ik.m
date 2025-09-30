%UR5_ROD_GRID_IK  Plan and visualise UR5 robot motions to reach a 3-D target grid in a loaded environment.
%
%   This script builds a UR5 manipulator with a metallic rod attached to its
%   tool flange, loads an environment description from a JSON scenario file,
%   and solves a sequence of collision-aware inverse kinematics (IK) problems
%   so that the rod tip visits every point in a 3-D target grid. It checks
%   reachability, stores the joint configurations, and animates the resulting
%   motion.
%
%   -------------------------------------------------------------------------
%   WORKFLOW
%   -------------------------------------------------------------------------
%   1) Clear the MATLAB workspace, command window, and figures.
%   2) Define physical parameters:
%        • Metallic rod length (L0) and radius (R0).
%        • Rectangular base dimensions (Lx, Ly, Lz).
%   3) Set IK solver parameters:
%        • Position tolerance (posTol).
%   4) Load an environment scenario (JSON) and specify robot placement in the
%      global world (posRobotGlobal).
%   5) Load a precomputed grid of Cartesian points (grid_cuboid.mat), decimate
%      it, and offset by a user-defined centroid.
%   6) Build a UR5 robot model with the rod and base using BUILDUR5WITHROD.
%   7) Visualise the robot, environment, and target grid in 3-D.
%   8) Loop through every target point:
%        • Call IKPOSITIONCOLLISIONAWARE to find a collision-free joint
%          configuration for the rod tip.
%        • Recompute forward kinematics to confirm the position error.
%        • Accumulate all valid joint solutions.
%   9) Verify that all points are reachable within tolerance.
%  10) Animate the robot moving through the stored configurations.
%
%   -------------------------------------------------------------------------
%   KEY VARIABLES
%   -------------------------------------------------------------------------
%   L0, R0         : Metallic rod length [m] and radius [m].
%   baseDim        : [Lx Ly Lz] base block dimensions [m].
%   posTol         : Maximum allowed Cartesian position error [m].
%   centroid       : [x y z] translation applied to the grid points.
%   targetPositions: N×3 array of Cartesian points to visit [m].
%   qs             : N×nDOF matrix of joint configurations returned by IK.
%   posError       : 1×N vector of final position errors [m].
%   posRobotGlobal : [x y z] offset of the robot base in world coordinates.
%   env            : Cell array of collisionBox objects loaded from scenario.
%
%   -------------------------------------------------------------------------
%   REQUIREMENTS
%   -------------------------------------------------------------------------
%   • MATLAB Robotics System Toolbox.
%   • Auxiliary functions:
%        – buildUR5WithRod(L0,R0,baseDim): returns a rigidBodyTree model
%          of the UR5 with attached rod and rectangular base.
%        – ikPositionCollisionAware: collision-aware inverse kinematics
%          routine used to solve each pose.
%   • Data files:
%        – grid_cuboid.mat containing variable `grid_cuboid`.
%        – JSON scenario files (e.g. Environment/kitchen.json).
%
%   -------------------------------------------------------------------------
%   USAGE
%   -------------------------------------------------------------------------
%   Save this file (e.g., as UR5_Rod_Grid_IK.m) and run:
%
%       >> UR5_Rod_Grid_IK
%
%   The script will:
%       • Display the robot, environment, and target grid.
%       • Solve IK for each target point.
%       • Report progress and any unreachable points.
%       • Animate the robot visiting all reachable targets.
%
%   -------------------------------------------------------------------------
%   OUTPUT
%   -------------------------------------------------------------------------
%   • 3-D figures showing the UR5+rod, environment, and motion sequence.
%   • Command-window messages reporting IK progress and success/failure.
%   • An error if any target grid point is unreachable within tolerance.
%
%   See also BUILDUR5WITHROD, IKPOSITIONCOLLISIONAWARE, LOADSCENARIO, SHOW, GETTRANSFORM.
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

% Load scenario
scenarioName = 'Kitchen';
scenario = loadScenario(['Environment/' lower(scenarioName) '.json']);

% Pos robot wrt global world
posRobotGlobal = [1.31 1.86 1];

% Target positions: centred array grid + centroid
centroid = [1 0 0];     % New custom array centre
load('grid_cuboid.mat');

targetPositions = grid_cuboid(1:2:end,:) + centroid;
numPositions = size(targetPositions,1);

clear grid_cuboid

%% ==== Load UR5 Robot ====
robot = buildUR5WithRod(L0, R0, baseDim);
% showdetails(robot)

% Environment: load from JSON file
margin = 5e-2;     % Distance margin to collision boxes [m]
nObjects = numel(scenario.objects);

env = cell(1,nObjects);
for iObj = 1:nObjects
    dimsPlusMargin = scenario.objects(iObj).dimensions + margin;
    env{iObj} = collisionBox(dimsPlusMargin(1), dimsPlusMargin(2), dimsPlusMargin(3));   % (Lx, Ly, Lz)
    env{iObj}.Pose = trvec2tform(scenario.objects(iObj).position'-posRobotGlobal);
end

% Visualise robot
figure
show(robot, 'Collisions', 'on', 'Visuals', 'on');
hold on
for iObj = 1:nObjects, show(env{iObj}); end
plot3(targetPositions(:,1), targetPositions(:,2), targetPositions(:,3), 'ro', 'MarkerSize', 1, 'LineWidth', 2)
plot3(mean(targetPositions(:,1)), mean(targetPositions(:,2)), mean(targetPositions(:,3)), 'ro', 'MarkerSize', 1, 'LineWidth', 2)
text(targetPositions(1,1), targetPositions(1,2), targetPositions(1,3),'Ini')
text(targetPositions(end,1), targetPositions(end,2), targetPositions(end,3),'End')
title('UR5+Rod+Base and Target Position')

%% ==== Plan sequence of configurations ====
qInitial = homeConfiguration(robot);    % Home config as initial guess
nDOF = length(qInitial);
qs = nan(numPositions, nDOF);

endEffector = 'rodTip';

rng(0)  % Ensure reproducibility in IK calculations
posError = nan(1,numPositions);
for iPos = 1:numPositions
    % Progress tracking (every 50 positions)
    if ~mod(iPos,50)
        disp(['IK solver... ' num2str(100*iPos/numPositions,'%.2f') ' %'])
    end

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

    % Store the configuration
    qs(iPos,:) = qSol;
    % Start from prior solution
    qInitial = qSol;
end

% If error is too large or IK fails at some position: error
if sum(posError > posTol,'all') + sum(isnan(qs),'all') ~= 0
    error('--- Array configuration not reachable! ---')
else
    disp('--- All array positions are reachable! ---')
end

%% ==== VISUALISE ALL CONFIGURATIONS ====
figure
show(robot,qs(1,:), 'Collisions', 'on', 'Visuals', 'on');
ax = gca;
ax.Projection = 'orthographic';
hold on
for iObj = 1:nObjects, show(env{iObj}); end
plot3(targetPositions(:,1), targetPositions(:,2), targetPositions(:,3), 'ro', 'MarkerSize', 1, 'LineWidth', 2)
text(targetPositions(1,1), targetPositions(1,2), targetPositions(1,3),'Ini')
text(targetPositions(end,1), targetPositions(end,2), targetPositions(end,3),'End')

framesPerSecond = 15;
r = rateControl(framesPerSecond);
for iPos = 1:numPositions
    show(robot,qs(iPos,:), 'Collisions', 'on', 'Visuals', 'on','PreservePlot',false);
    drawnow
    waitfor(r);
end
