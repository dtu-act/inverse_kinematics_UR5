function path = planSafePathBetweenConfigs(robot, env, startConfig, goalConfig)
%path = planSafePathBetweenConfigs(robot, startConfig, goalConfig)
%Calculates path between two robot configurations

%% ==== CALCULATE PATH USING MANIPULATOR STATE SPACE ====
planner = manipulatorRRT(robot, env);
planner.ValidationDistance = 0.1;
planner.MaxConnectionDistance = 0.2;
planner.MaxIterations = 200;
planner.SkippedSelfCollisions = 'parent';

[pth, ~] = plan(planner, startConfig, goalConfig);

if isempty(pth)
    warning("RRT could not find a path between poses. Using linear fallback.");
    % Fallback: Linear interpolation
    numSteps = 10;
    path = interp1([1 numSteps], [startConfig; goalConfig], 1:numSteps);
else
    path = pth;
end
end
