function path = planSafePathBetweenConfigs(robot, startConfig, goalConfig)
%path = planSafePathBetweenConfigs(robot, startConfig, goalConfig)
%Calculates path between two robot configurations

%% ==== CALCULATE PATH USING MANIPULATOR STATE SPACE ====
ss = manipulatorStateSpace(robot);
sv = manipulatorCollisionBodyValidator(ss);
sv.ValidationDistance = 0.01;
sv.Environment = {}; % No external collision objects

planner = manipulatorRRT(ss, sv);
planner.MaxConnectionDistance = 0.2;
planner.MaxIterations = 200;

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
