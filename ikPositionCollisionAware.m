function [configSol,info] = ikPositionCollisionAware(robot,eeName, ...
                                targetPos,prevConfig,oriSamples,randGuesses)

% 1. IK object
ik = inverseKinematics('RigidBodyTree',robot);
ik.SolverParameters.SolutionTolerance = 1e-3;  % default tolerance
weights            = [0 0 0 1 1 1];    % give a small but non-zero weight

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