function path = planSafePathWithPlanner(robot, startConfig, goalConfig)
%path = planSafePathWithPlanner(robot, startConfig, goalConfig)
%Calculates path between two robot configurations

%% ==== CALCULATE PATH USING MANIPULATOR STATE SPACE ====
% Define task space (SE3)
ss = stateSpaceSE3;
sv = validatorOccupancyMap3D(ss);   % Dummy validator for now
% sv.MapName = 'empty';               % No obstacles

% Create planner
planner = plannerRRT(ss, sv);
planner.MaxConnectionDistance = 0.1;
planner.MaxIterations = 500;

% Plan path
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
