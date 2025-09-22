%% ==== Clear workspace ====
clear, clc, close all

try % Wrap to send log over email
    logFile = fullfile(tempdir, 'matlab_log.txt');
    diary(logFile);
    diary on;


%% ==== Parameters ====
% RIR: measurement parameters
T = 3;          % Sweep length [s]
Toff = 1;       % Silence length = RIR length [s]
fs = 48e3;      % Sampling frequency [Hz]
Fband = [20 20E3];  % Sweep bandwidth
Nsamples = fs*(T+Toff); % A-priori number of samples
gain_nexus = 1;    % Nexus gain [V/Pa]
gain_sweep = -7;        % Sweep gain
frameSize = 512;        % Frame size to minimise latency
maxRep = 3;             % Max repetitions in case of samples over/underrun

% RIR: folder and file structure
folderData = 'Data/SFControlRoom/';
fileNamePrefix = 'cuboid_RIR_pos_';

% Room conditions
roomDimensions = [6.08 5.76 3.08]; % Room dimensions [m x m x m]
tempC = 18.8;       % Temperature [C]
humidityRH = 73.2;  % Relative humidity [%RH]

% Robot: Metallic rod (plastic + rod + microphone)
L0 = 0.59;      % Length of the metallic rod [meters]
R0 = 0.01;      % Radius of the metallic rod [meters]

% Robot: Base
Lx = 0.08;      % length in X [m]
Ly = 0.08;      % width in Y [m]
Lz = 1;         % height in Z [m]
baseDim = [Lx Ly Lz];

% IK: position tolerance
posTol = 1e-3;          % Position tolerance [meters]

% Target positions: centred array grid + centroid
centroid = [1 0 0.35];     % New custom array centre
load('grid_cuboid.mat');

targetPositions = grid_cuboid + centroid;
numPositions = size(targetPositions,1);

clear grid_cuboid

%% ==== Connect to SOUNDCARD ====
infoDev = audiodevinfo;
nameDev = 'ASIO Fireface USB';
% infoDev.output.Name

playRec = audioPlayerRecorder(fs,"SupportVariableSize",true,...
    Device=nameDev,...
    BufferSize=frameSize,...
    BitDepth="24-bit integer");

% Mic Channel - 5
playRec.RecorderChannelMapping = 5;

% Loudspeaker Channel - 5
playRec.PlayerChannelMapping = 1;

%% ==== Connect to UR5 ====
ur = urRTDEClient('10.59.33.149','CobotName','universalUR5');

% Current robot state
% IMPORTANT NOTE:
% The real robot's reference is the opposite to the simulations. When
% visualising the real robot (i.e. using ur.RigidBodyTree), the X-axis is
% pointing towards the opposite direction
jointAngles = readJointConfiguration(ur);
figure(1)
show(ur.RigidBodyTree,jointAngles);
title('Real robot: current configuration. X-axis is flipped')

%% ==== Load UR5 Robot ====
robot = buildUR5WithRod(L0, R0, baseDim);
% showdetails(robot)

% Pos robot wrt global world
posRobotGlobal = [1.31 3.22 1];

% Pos source wrt global world
posSourceGlobal = [4.81 1.37 1.35];

% Environment: kitchen
margin = 5e-2;     % Distance margin to collision boxes [m]
backWall  = collisionBox(0.2 + margin, 2 + margin, 3 + margin);   % (Lx, Ly, Lz)
desktop = collisionBox(0.8 + margin, 1.2 + margin, 1.4 + margin);

backWallPosition = [0.1, 3.22, 1.5];
desktopPosition = [0.6, 4.4, 0.7];

backWall.Pose  = trvec2tform(backWallPosition-posRobotGlobal); % behind robot
desktop.Pose = trvec2tform(desktopPosition-posRobotGlobal); % to robot’s right

env = {backWall, desktop};

% Visualise robot
figure(2)
show(robot, 'Collisions', 'on', 'Visuals', 'on');
hold on
show(backWall);
show(desktop);
plot3(targetPositions(:,1), targetPositions(:,2), targetPositions(:,3), 'ro', 'MarkerSize', 1, 'LineWidth', 2)
plot3(mean(targetPositions(:,1)), mean(targetPositions(:,2)), mean(targetPositions(:,3)), 'ro', 'MarkerSize', 1, 'LineWidth', 2)
text(targetPositions(1,1), targetPositions(1,2), targetPositions(1,3),'Ini')
text(targetPositions(end,1), targetPositions(end,2), targetPositions(end,3),'End')
hold off
title('UR5+Rod+Base and Target Position')

%% ==== Create RRT planner ====
rrt = manipulatorRRT(robot,env);
rrt.SkippedSelfCollisions = "parent";

%% Send robot to home configuration
qInitial = homeConfiguration(robot);    % Home config as initial guess

path = plan(rrt, jointAngles, qInitial);

% Visualise
interpPath = interpolate(rrt,path);
figure(2)
clf
for i = 1:20:size(interpPath,1)
    show(robot,interpPath(i,:),'Collisions','on'); hold on
    if i == 1
        show(backWall);
        show(desktop);
        plot3(targetPositions(:,1), targetPositions(:,2), targetPositions(:,3), 'ro', 'MarkerSize', 1, 'LineWidth', 2)
        text(targetPositions(1,1), targetPositions(1,2), targetPositions(1,3),'Ini')
        text(targetPositions(end,1), targetPositions(end,2), targetPositions(end,3),'End')
    end
    view(2)
    drawnow
    pause(0.1)
end
show(robot,interpPath(i,:),'Collisions','on');
hold off

disp('Ready to move the robot?...'), pause
% [result,state] = sendJointConfigurationAndWait(ur,qInitial,'EndTime',5);
followJointWaypoints(ur, path', 'BlendRadius', 0.02)

%% ==== Create exponential sweep ====
% Adapt sweep length to obtain full frames (initial RIR length is modified)
nFrames = ceil(Nsamples/frameSize);
Nsamples = nFrames*frameSize;
Toff = Nsamples/fs - T;
Nh = fs*Toff;
t = 0:1/fs:Toff-1/fs;

% Gen sweep
sweep = sweeptone(T,Toff,fs,'SweepFrequencyRange',Fband,'ExcitationLevel',gain_sweep);

% Slice sweep
audioToPlay = reshape(sweep,frameSize,Nsamples/frameSize);

%% ==== Plan sequence of configurations & measure RIR ====
ndof = length(qInitial);
qs = nan(numPositions, ndof);

endEffector = 'rodTip';

% Check for folder
if ~exist(folderData,'dir')
    mkdir(folderData)
end

% Save metadata
save([folderData 'metadata'])

disp('Leave the room now!...')
pause(45)

tic;
rng(0)  % Ensure reproducibility in IK calculations
posError = nan(1,numPositions);
for iPos = 1:numPositions
    disp(['---- Position ' num2str(iPos) ' -----'])

    % ---- IK ------------------------------------------------------------
    % Solve for the configuration satisfying the desired end effector position
    point = targetPositions(iPos,:);
    [qSol,info] = ikPositionCollisionAware(robot,endEffector, ...
                   point, qInitial);

    % If IK fails to reach the point
    if ~info.success
        warning('IK failed for point %d. Skipping pose.', iPos);
        continue
    end

    % Re-compute FK to check residual error
    eeTform = getTransform(robot, qSol, endEffector);
    posError(iPos) = norm(eeTform(1:3,4).' - point, 2);
    if (posError(iPos) > posTol)
        disp(['ERROR: Pos ' num2str(iPos) ' not reachable by ' num2str(posError(iPos)*1e2) ' cm'])
    end

    % ---- path planning -------------------------------------------------
    path = plan(rrt, qInitial, qSol);
    if isempty(path)
        warning('RRT failed at point %d; skipping.', iPos);
        continue
    end

    % Visualise
    interpPath = interpolate(rrt,path);
    figure(2)
    clf
    for i = 1:20:size(interpPath,1)
        show(robot,interpPath(i,:),'Collisions','on'); hold on
        if i == 1
            show(backWall);
            show(desktop);
            plot3(targetPositions(:,1), targetPositions(:,2), targetPositions(:,3), 'ro', 'MarkerSize', 1, 'LineWidth', 2)
            text(targetPositions(1,1), targetPositions(1,2), targetPositions(1,3),'Ini')
            text(targetPositions(end,1), targetPositions(end,2), targetPositions(end,3),'End')
        end
        view(2)
        drawnow
        pause(0.1)
    end
    show(robot,interpPath(i,:),'Collisions','on');
    hold off

    % Send path to UR5
    % disp('Ready to move the robot?...'), pause
    followJointWaypoints(ur, path', 'BlendRadius', 0.02)
    
    % Wait for the robot to move
    jointVelocity = readJointVelocity(ur);
    while sum(abs(jointVelocity)) > 0
        jointVelocity = readJointVelocity(ur);
    end
    pause(1)

    % Verify connection is still operative (not confirmed this works)
    jointAngles = readJointConfiguration(ur);

    % Next position: Start from prior solution
    qs(iPos,:) = qSol;      % Store the configuration
    qInitial = qSol;

    % ---- measure RIR ---------------------------------------------------
    % Play + record signal
    [p_meas,flagRep] = get_dataRME(playRec,audioToPlay,maxRep);
    if flagRep == 1, disp(['ERROR: Max repetition number reached for position ' num2str(i)]); end

    % Gain Nexus + RME
    p_meas = p_meas/gain_nexus;

    % Calculate RIR
    rir = impzest(sweep,p_meas);

    % Plot RIR
    if iPos == 1, figure(3), hold on, end
    if iPos == 1 || mod(iPos, 25) == 0
        plot(t,rir), grid on
        xlabel('Time /s'), title('Impulse response')
    end

    % Save pressure & RIR
    position = targetPositions(iPos,:);
    fileName = [folderData fileNamePrefix num2str(iPos,'%04.f')];
    save(fileName,'rir','p_meas','position');
    disp(['Data saved to ' fileName])
end

%% ==== Optional: send email when finished ====
% End logging
fprintf('Elapsed time: %s\n', datetime(0,0,0,0,0,toc,'Format','HH:mm:ss'))
diary off;

sendmail('anfig@dtu.dk','ARMando has finished', ...
    'ARMando, your favorite cobot, has finished measuring. Come and collect your IRs!',{logFile});

%%
catch ME
    % Save the crash log
    logFile = fullfile(tempdir, 'crash_log.txt');
    fid = fopen(logFile,'w');
    fprintf(fid, '%s\n', getReport(ME,'extended','hyperlinks','off'));
    fclose(fid);

    % Send the email with the log attached
    sendmail('anfig@dtu.dk','ARMando had trouble :(', ...
             'An error occurred. Log attached.', {logFile});

    rethrow(ME);  % optional: still show the error in MATLAB
end