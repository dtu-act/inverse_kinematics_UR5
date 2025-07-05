function fullPath = planPoseSequence(robot, initialPosition, targetPositions)
%fullPath = planPoseSequence(robot, initialPosition, targetPositions) Plans
%the robot poses for a grid of points

%% ==== CALCULATE POSES FOR GRID OF POINTS ====
maxTries = 20; % Max tries at each position
weights = [1 1 1 0.2 0.2 0.2]; % Priority to position over orientation
fullPath = [];
prevTarget = [];
currentConfig = initialPosition;

for iPos = 1:size(targetPositions,1)
    [goalConfig, success] = solveIKWithCollisionCheck(robot, targetPositions(iPos,:), prevTarget, maxTries, currentConfig, weights);

    if ~success
        warning("Target %d unreachable. Skipping.", iPos);
        continue;
    end

    % Plan safe path from current to goal
    % segmentPath = planSafePathBetweenConfigs(robot, currentConfig, goalConfig);
    segmentPath = planSafePathWithPlanner(robot, currentConfig, goalConfig);
    fullPath = [fullPath; segmentPath];
    prevTarget = targetPositions(iPos,:);
    currentConfig = goalConfig;
end

end
