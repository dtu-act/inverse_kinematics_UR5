%% ==== Clear workspace ====
clear, clc, close all

%% Load responses
% RIR: measurement parameters
T = 15;          % Sweep length [s]
Toff = 5;       % Silence length = RIR length [s]
fs = 48e3;      % Sampling frequency [Hz]
Fband = [20 20E3];  % Sweep bandwidth
Nsamples = fs*(T+Toff); % A-priori number of samples
Nh = fs*Toff;
t = 0:1/fs:Toff-1/fs;

% RIR: folder and file structure
folderData = 'Data/ReverbChamber/CuboidData/';
fileNamePrefix = 'cuboid_RIR_pos_';

load('grid_cuboid.mat');
targetPositions = grid_cuboid;
numPositions = size(targetPositions,1);

clear grid_cuboid

%% Load files
posIdx = 1;
rirs = nan(Nh, 50);
figure, hold on
for iPosSet = 1:floor(numPositions/50)
    for iPos = 1:50
        fileName = [folderData fileNamePrefix num2str(posIdx,'%04.f')];
        load(fileName,'rir');
        rirs(:,iPos) = rir;
        posIdx = posIdx + 1;
    end

    disp(posIdx)
    clf
    plot(t,rirs), grid on
    xlim([0 100e-3])
    pause
end

