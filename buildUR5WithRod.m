function robot = buildUR5WithRod(L0, R0, baseDim)
%robot = buildUR5WithRod(L0, R0, baseDim) Generates robot with rod and base
%   Contact:
%       Antonio Figueroa-Duran
%       anfig@dtu.dk

%% ==== LOAD ROBOT ====
robot = loadrobot('universalUR5', 'DataFormat', 'row', 'Gravity', [0 0 -9.81]);

% %% ==== ATTACH ROD ====
% % Define rod as rigid body with fixed joint
% rod = rigidBody('rod');
% rodJoint = rigidBodyJoint('rodJoint', 'fixed');
% 
% % Position rod along Z axis of tool0 (e.g., extruding straight out)
% setFixedTransform(rodJoint, trvec2tform([0 0 0]));
% rod.Joint = rodJoint;
% 
% % Add collision geometry: cylinder centered along Z axis
% rodCollisionTform = trvec2tform([0, 0, L0/2]); % Center of cylinder
% addCollision(rod, collisionCylinder(R0, L0), rodCollisionTform);
% 
% % Attach rod to tool0
% addBody(robot, rod, 'tool0');
% 
% % Define rod tip
% tipBody = rigidBody('rodTip');
% tipJoint = rigidBodyJoint('rodTipJoint', 'fixed');
% setFixedTransform(tipJoint, trvec2tform([0 0 L0]));  % place at tip
% tipBody.Joint = tipJoint;
% addBody(robot, tipBody, 'rod');
% 
% %% ==== ATTACH BASE ====
% % Define base as rigid body with fixed joint
% robotBase = rigidBody('robotBase');
% baseJoint = rigidBodyJoint('robotBaseJoint', 'fixed');
% 
% % Position it under the robot
% setFixedTransform(baseJoint, trvec2tform([0 0 -baseDim(3)/2]));  % base is below origin
% robotBase.Joint = baseJoint;
% 
% % Add collision geometry: box centered at origin
% addCollision(robotBase, collisionBox(baseDim(1), baseDim(2), baseDim(3)));
% 
% % Attach to main robot tree
% addBody(robot, robotBase, robot.BaseName);

end