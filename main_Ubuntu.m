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

% IK position tolerance
posTol = 1e-3; % Position tolerance [meters]

% Target positions
load('grid_cuboid.mat');
r = grid_cuboid(1:1e2:end,:);
numPositions = size(r,1);
targetPositions = r(:,1:3); % Only position
targetPositions(:,3) = targetPositions(:,3)-0.8;

clear grid_cuboid

%% ==== Load UR5 Robot ====
robot = buildUR5WithRod(L0, R0, baseDim);
% showdetails(robot)

% Environment
env = {};

% Visualise robot
figure(1)
show(robot, 'Collisions', 'on', 'Visuals', 'on');
hold on
plot3(targetPositions(:,1), targetPositions(:,2), targetPositions(:,3), 'ro', 'MarkerSize', 1, 'LineWidth', 2)
plot3(mean(targetPositions(:,1)), mean(targetPositions(:,2)), mean(targetPositions(:,3)), 'ro', 'MarkerSize', 1, 'LineWidth', 2)
text(targetPositions(1,1), targetPositions(1,2), targetPositions(1,3),'Ini')
text(targetPositions(end,1), targetPositions(end,2), targetPositions(end,3),'End')
title('UR5+Rod+Base and Target Position')

%% ==== Plan sequence of configurations ====
q0 = homeConfiguration(robot);
ndof = length(q0);
qs = zeros(numPositions, ndof);
maxTriesIK   = 100;     % IK: orientation samples
maxRandGuess = 10;      % IK: 10 random initial guesses per orientation

ik = inverseKinematics('RigidBodyTree', robot);
weights = [0, 0, 0, 1, 1, 1];   % weights = [roll pitch yaw x y z]
endEffector = 'rodTip';

qInitial = q0; % Use home configuration as the initial guess

posError = nan(1,numPositions);
for iPos = 1:numPositions
    disp(['---- Position ' num2str(iPos) ' -----'])
    % ---- IK ------------------------------------------------------------
    % Solve for the configuration satisfying the desired end effector position
    point = targetPositions(iPos,:);
    [qSol,info] = ikPositionCollisionAware(robot,endEffector, ...
                   point, qInitial, ...
                   maxTriesIK, maxRandGuess);

    if ~info.success     % custom flag we set inside ikPositionCollisionAware
        warning('IK failed for point %d. Skipping pose.', iPos);
        continue
    end

    % Store the configuration
    qs(iPos,:) = qSol;
    % Start from prior solution
    qInitial = qSol;

    % Re-compute FK to check residual error
    eeTform = getTransform(robot, qSol, endEffector);

    posError(iPos) = norm(eeTform(1:3,4).' - targetPositions(iPos,:), 2);

    if (posError(iPos) > posTol)
        disp(['ERROR: Pos ' num2str(iPos) ' not reachable by ' num2str(posError(iPos)*1e2) ' cm'])
    end

    isColliding = checkCollision(robot, qSol, SkippedSelfCollisions="parent");
    if isColliding
        figure(2)
        show(robot,qSol,Collisions="on",Visuals="off");
        title(["Collision Detected at pos " num2str(iPos)])
        pause
    end

end

if any(posError > posTol)
    error('--- Array configuration not reachable! ---')
end

%% ==== VISUALISE FULL PATH ====
figure(3)
show(robot,qs(1,:), 'Collisions', 'on', 'Visuals', 'on');
ax = gca;
ax.Projection = 'orthographic';
hold on
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
