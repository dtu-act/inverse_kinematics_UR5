function env = buildBase(baseDim)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

base = collisionBox(baseDim(1), baseDim(2), baseDim(3));
base.Pose = trvec2tform([0, 0, -baseDim(3)/2-1e-2]);

env = {base};

end