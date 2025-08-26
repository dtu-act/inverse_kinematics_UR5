%% UR5(e) grid-scan, position-only IK, collision-checked planning
clear; clc; rng(1);

%% ---- Parameters (your specs) ----
rodR = 0.01;                 % [m]
rodL = 0.42;                 % [m]
baseSize = [0.08 0.08 1.0];  % [Lx Ly Lz] m
basePose = trvec2tform([0 0 baseSize(3)/2]);   % centered at z = Lz/2

pA = [ 1.00  0.595 0.20];
pB = [-1.00  0.595 0.20];
numPts = 21;

%% ---- Robot + tool collision ----
robot = loadrobot("universalUR5","DataFormat","row","Gravity",[0 0 -9.81]);
eeName = robot.BodyNames{end};                        % end-effector body

rodBody = rigidBody("rod");
setFixedTransform(rodBody.Joint, trvec2tform([0 0 0.01]));
addCollision(rodBody, collisionCylinder(rodR, rodL), trvec2tform([0 0 rodL/2]));
addBody(robot, rodBody, eeName);

% Attach the rod along +Z of the tool, starting at tool flange
addCollision(robot.Bodies{end}, ...
    collisionCylinder(rodR, rodL), trvec2tform([0 0 rodL/2]));

% World obstacle: the slim base pillar
env = {collisionBox(baseSize(1), baseSize(2), baseSize(3))};
env{1}.Pose = basePose;

%% ---- Build scan line (21 positions from pA -> pB) ----
gridXYZ = [linspace(pA(1),pB(1),numPts)' ...
           linspace(pA(2),pB(2),numPts)' ...
           linspace(pA(3),pB(3),numPts)'];

%% ---- Position-only IK, multiple seeds per point ----
ik = inverseKinematics("RigidBodyTree", robot);
w  = [1 1 1 0 0 0];   % ignore orientation entirely
NS = 32;              % seeds tried per point

% Seed pool: home + randoms
homeCfg = homeConfiguration(robot);
seedPool = zeros(NS, numel(homeCfg));
seedPool(1,:) = homeCfg;
for k=2:NS, seedPool(k,:) = randomConfiguration(robot); end

feasible = cell(numPts,1);
for i = 1:numPts
    T = trvec2tform(gridXYZ(i,:));   % position-only target
    Qi = [];
    for s = 1:NS
        [q, ~] = ik(eeName, T, w, seedPool(s,:));
        if isFree(robot, q, env)   % self + world collision check
            Qi = [Qi; q]; %#ok<AGROW>
        end
    end
    if isempty(Qi)
        error("No collision-free IK found at point %d. Try increasing NS or adjusting reach.", i);
    end
    feasible{i} = unique(round(Qi,6),'rows');  % dedup numerically
end

%% ---- Greedy choice of configs (min joint distance) ----
pathQ = zeros(numPts, numel(homeCfg));
% Start near home to reduce first move
[~, idx0] = min(vecnorm(feasible{1} - homeCfg, 2, 2));
pathQ(1,:) = feasible{1}(idx0,:);

for i = 2:numPts
    Qprev = pathQ(i-1,:);
    D = vecnorm(feasible{i} - Qprev, 2, 2);
    [~, j] = min(D);
    pathQ(i,:) = feasible{i}(j,:);
end

%% ---- Plan between successive waypoints with manipulatorRRT ----
planner = manipulatorRRT(robot, env);
planner.ValidationDistance   = 0.05;
planner.MaxConnectionDistance= 0.2;
planner.EnableConnectHeuristic = true;

plannedQ = pathQ(1,:);   % accumulate full joint path
for i = 1:numPts-1
    q1 = pathQ(i,:); q2 = pathQ(i+1,:);
    % Try quick joint-space interpolation first
    if segmentCollisionFree(robot, q1, q2, env, 40)
        qseg = [q1; q2];
    else
        qseg = plan(planner, q1, q2);
        if isempty(qseg)
            error("Planner failed between points %d and %d", i, i+1);
        end
    end
    % Append (avoid duplicating the start)
    plannedQ = [plannedQ; qseg(2:end,:)]; %#ok<AGROW>
end

%% ---- Time parameterize (trapezoidal velocity) ----
% Sample count proportional to path length
segdiff = vecnorm(diff(plannedQ),2,2);
N = max(200, ceil(200 * sum(segdiff)));   % increase for smoother tracking
[qTraj, qdTraj, qddTraj, t] = trapveltraj(plannedQ', N);
qTraj = qTraj';  % rows = samples

fprintf("Planned %d waypoints -> %d trajectory samples.\n", size(plannedQ,1), size(qTraj,1));

%% ---- (Optional) Visualize ----
show(robot, plannedQ(1,:), "Frames","off","PreservePlot",false); hold on
% show(robot, plannedQ(end,:), "Frames","off","PreservePlot",false);
% show(world); axis equal; view(130,20); title('Start (blue) and End (yellow)')

%% ---- (Optional) Stream to UR via RTDE (dry-run skeleton) ----
% urt = urRTDEClient("192.168.0.10");   % <-- set your UR IP
% connect(urt);
% jointNames = {robot.Bodies{[robot.PositionDoFMap]}.Name}; %# illustrative
% for k = 1:size(qTraj,1)
%     sendrtde(urt, "setQ", qTraj(k,:));   % use the appropriate RTDE setter
%     pause(diff([0 t(k)]));               % crude timing; replace with monotonic clock
% end
% disconnect(urt);

%% ---- Helpers ----
function ok = isFree(robot, q, env)
    % Self + world collision check
    ok = ~checkCollision(robot, q, env, "Exhaustive","on", "IgnoreSelfCollision","off");
end

function ok = segmentCollisionFree(robot, q1, q2, env, steps)
    ok = true;
    for s = 0:steps
        a = s/steps;
        q = (1-a)*q1 + a*q2;
        if ~isFree(robot, q, env)
            ok = false; return;
        end
    end
end
