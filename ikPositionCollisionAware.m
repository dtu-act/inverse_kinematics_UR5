function [configSol,info] = ikPositionCollisionAware(robot,eeName, ...
                                targetPos,prevConfig,oriSamples,randGuesses)
%IKPOSITIONCOLLISIONAWARE  Solve collision-aware inverse kinematics (IK) for a target position.
%
%   [CONFIGSOL,INFO] = IKPOSITIONCOLLISIONAWARE(ROBOT,EENAME,TARGETPOS, ...
%                      PREVCONFIG,ORISAMPLES,RANDGUESSES) attempts to find a
%   joint configuration CONFIGSOL of the rigid body tree ROBOT such that
%   the end effector named EENAME reaches the Cartesian position TARGETPOS
%   while avoiding collisions.  The function searches over randomly sampled
%   orientations and random initial guesses to improve robustness.
%
%   INPUTS
%     ROBOT       – rigidBodyTree object describing the robot.
%     EENAME      – name (char or string) of the end effector body.
%     TARGETPOS   – 1×3 vector [x y z] specifying desired Cartesian position
%                   in the base frame (meters).
%     PREVCONFIG  – n×1 vector of joint positions used as the preferred
%                   “seed” for continuity.
%     ORISAMPLES  – (optional) number of random orientation samples to try.
%                   Each sample is a uniform random quaternion.  Required.
%     RANDGUESSES – (optional) number of random configuration guesses to try
%                   for each orientation sample (default = 1).  The first
%                   guess is always PREVCONFIG.
%
%   OUTPUTS
%     CONFIGSOL   – n×1 vector of joint positions for a collision-free IK
%                   solution closest (in 2-norm) to PREVCONFIG.  Returns []
%                   if no solution is found.
%     INFO        – struct with diagnostic fields:
%          success        : logical true if a feasible solution was found.
%          attempts       : total number of IK attempts performed.
%          IKStatus       : status string from the last IK solver call.
%          PoseErrorNorm  : pose error norm from the last IK solver call.
%
%   ALGORITHM
%     1) Create an inverseKinematics solver for ROBOT with a position-only
%        objective (weights = [0 0 0 1 1 1]).
%     2) Loop over ORISAMPLES random orientations.
%     3) For each orientation, run the IK solver RANDGUESSES times using
%        PREVCONFIG first, then random joint guesses.
%     4) Discard solutions that fail IK convergence or are in collision
%        (self-collisions are ignored by setting
%        'SkippedSelfCollisions'="parent").
%     5) Return the feasible configuration with minimum joint-space
%        distance from PREVCONFIG.
%
%   NOTES
%     • The random orientation is drawn from a uniform distribution on SO(3).
%     • Collision checking requires that collision geometries are defined
%       in the ROBOT model.
%     • If no solution is found, CONFIGSOL is empty and INFO.success = false.
%
%   EXAMPLE
%     robot = loadrobot('kinovaGen3','DataFormat','column');
%     prevCfg = homeConfiguration(robot);
%     target  = [0.4 0.2 0.3];
%     [qSol,info] = ikPositionCollisionAware(robot,'EndEffector_Link', ...
%                                            target,prevCfg,8,3);
%     if info.success
%         show(robot,qSol); hold on;
%         plot3(target(1),target(2),target(3),'rx','MarkerSize',10);
%     else
%         warning('No collision-free IK solution found.');
%     end
%
%   See also INVERSEKINEMATICS, CHECKCOLLISION, TRVEC2TFORM, QUAT2TFORM.
%
% Author: Antonio Figueroa-Duran
% Contact: anfig@dtu.dk


% 1. IK object
ik = inverseKinematics('RigidBodyTree',robot);
ik.SolverParameters.SolutionTolerance = 1e-3;  % default tolerance
weights = [0 0 0 1 1 1];     % weights = [roll pitch yaw x y z]

% Initialise info structure
info            = struct();
info.success    = false;       % overall result of this routine
info.attempts   = 0;           % how many IK attempts performed
info.IKStatus   = "";          % Status string from last IK call
info.PoseErrorNorm = [];       % Pose error from last IK call

% Provide default for randGuesses if omitted
if nargin < 6 || isempty(randGuesses)
    randGuesses = 1;
end

% 2. Search loop
configSol = [];
attemptCnt = 0;
found = false;
bestCost = inf;
for k = 1:oriSamples
    % -- 2a) choose a random orientation (uniform quaternion)
    qOri = randomQuaternion();
    targetTform = trvec2tform(targetPos) * quat2tform(qOri);

    for g = 1:randGuesses
        attemptCnt = attemptCnt + 1;

        % Initial guess: first one is prevConfig for continuity
        if g == 1
            qInit = prevConfig;
        else
            qInit = randomConfiguration(robot);
        end

        % -- Solve IK
        [cfg,solInfo] = ik(eeName,targetTform,weights,qInit);

        % Record last IK info (overwritten each attempt; ok for diagnostics)
        info.IKStatus       = solInfo.Status;
        info.PoseErrorNorm  = solInfo.PoseErrorNorm;

        % -- 2c) reject if IK did not converge; accept anything but 'failure'
        if strcmp(solInfo.Status,'failure')
            continue
        end

        % -- 2d) reject if resulting posture is in collision
        % Check collision but ignore self-collisions to widen feasible set
        isColl = checkCollision(robot,cfg,SkippedSelfCollisions="parent");
        if isColl
            continue
        end

        % -- 2e) compute joint-change cost
        cost = norm(cfg - prevConfig,2);
        if cost < 1e-3 % practically identical to previous pose
            configSol = cfg;
            info.success = true;
            info.attempts = attemptCnt;
            return          % early exit for speed
        end
        % keep best in case early exit criterion not met
        if ~found || cost < bestCost
            bestCost = cost;
            configSol = cfg;
            found = true;
            info.success = true;
        end
    end
    if found
        break   % break outer loop too
    end
end

info.attempts = attemptCnt;

% 3. Fallback if nothing found
if ~info.success
    configSol = [];
end
end

% ---------- helper: uniform random quaternion ---------------------------
function q = randomQuaternion()
u1 = rand; u2 = rand; u3 = rand;
q  = [ sqrt(1-u1)*sin(2*pi*u2) , ...
       sqrt(1-u1)*cos(2*pi*u2) , ...
       sqrt(u1)  *sin(2*pi*u3) , ...
       sqrt(u1)  *cos(2*pi*u3) ];
end