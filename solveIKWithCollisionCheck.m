function [config, success] = solveIKWithCollisionCheck(robot, targetPos, prevPos, maxTries, initialGuess, weights)
%[config, success] = solveIKWithCollisionCheck(robot, targetPos,...
%prevPos, maxTries, initialGuess, weights)
%   Solves Inverse Kinematics for a single position considering the
%   previous position

%% ==== Set Up IK Solver ====
ik = inverseKinematics('RigidBodyTree', robot);
ik.SolverParameters.SolutionTolerance = 1e-6;
endEffector = 'rodTip';
% endEffector = 'tool0';

%% ==== Solve IK with Collision Filtering ====
success = false;
config = [];

% % Optimises the orientation based on the gradient of the grid
% if isempty(prevPos)
%     % Default pointing direction if no previous point
%     zAxis = [0 0 -1];  % e.g., rod pointing upwards
% else
%     % Compute direction vector
%     direction = targetPos - prevPos;
%     zAxis = direction / norm(direction);
% end

zAxis = [0 0 1];

% Build consistent orthonormal frame (Z points along rod)
temp = [1 0 0];
if abs(dot(zAxis, temp)) > 0.9
    temp = [0 1 0];
end
xAxis = cross(temp, zAxis); xAxis = xAxis / norm(xAxis);
yAxis = cross(zAxis, xAxis);
R = [xAxis; yAxis; zAxis]';
baseQuat = rotm2quat(R);

for attempt = 1:maxTries
    % Randomize orientation if not first attempt
    if attempt > 1
        angleRnd = (rand-0.5) * pi/4; % ±45°
        axisRnd = zAxis;
        perturbQuat = axang2quat([axisRnd, angleRnd]);
        targetQuat = quatmultiply(baseQuat, perturbQuat);
    else
        targetQuat = baseQuat;
    end

    % Build target pose
    targetPose = trvec2tform(targetPos) * quat2tform(targetQuat);

    % Solver Inverse Kinematics
    [configSol, ~] = ik(endEffector, targetPose, weights, initialGuess);

    show(robot,configSol)

    % Check for collision
    inCollision = checkCollision(robot, configSol, ...
        'IgnoreSelfCollision', 'off', ...
        'SkippedSelfCollisions', 'parent');
    % inCollision = false;

    if ~inCollision
        success = true;
        config = configSol;
        return;
    end
end

end