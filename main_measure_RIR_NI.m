%% ==== Clear workspace ====
clear, clc, close all

%% ==== Parameters ====
% RIR: measurement parameters
T = 3;          % Sweep length [s]
Toff = 1;       % Silence length = RIR length [s]
fs = 48e3;      % Sampling frequency [Hz]
Fband = [20 20E3];  % Sweep bandwidth
Nsamples = fs*(T+Toff); % A-priori number of samples
gain_nexus = 1;    % Nexus gain [V/Pa]
gain_sweep = -7;        % Sweep gain
maxRep = 2;             % Max repetitions in case of samples over/underrun
numMicPos = 6;          % Num microphone positions per source position
numSourcePos = 3;       % Num source positions

% RIR: folder and file structure
folderData = 'Data/Kitchen/RT/';
fileNamePrefix = 'single_RIR_';

% Room conditions
roomDimensions = [6.7 6.25 3.0]; % Room dimensions [m x m x m]
tempC = 24.2;       % Temperature [C]
humidityRH = 61.4;  % Relative humidity [%RH]

%% ==== Connect to SOUNDCARD ====
% Configure NI USB4431
out_range    = [-3.5 3.5];
in_range     = [-3.5 3.5];

% d = daqlist('ni')
ni = daq('ni');
ni.Rate = fs;

% Output
ni.addoutput('Dev5',0,'Voltage');
ni.Channels(1).Range = out_range;

% Inputs
ni.addinput('Dev5',0,'Voltage');
ni.Channels(2).Range = in_range;

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

%% ==== Run measurements ====
% Check for folder
if ~exist(folderData,'dir')
    mkdir(folderData)
end

% Save metadata
save([folderData 'metadata'])

figure, hold on
for sPos = 1:numSourcePos
    disp(['---- Source Position ' num2str(sPos) ' -----'])

    for mPos = 1:numMicPos
        disp(['Mic Position ' num2str(mPos) ' -----'])

        disp('Ready?'), pause

        % ---- measure RIR ---------------------------------------------------
        % Play + record signal
        s.queueOutputData(sweep);
        p_meas = s.startForeground;

        % Gain Nexus + RME
        p_meas = p_meas/gain_nexus;

        % Calculate RIR
        rir = impzest(sweep,p_meas);

        % Plot RIR
        plot(t,rir), grid on
        xlabel('Time /s'), title('Impulse response')

        % Save pressure & RIR
        fileName = [folderData fileNamePrefix 's' num2str(sPos,'%02.f') '_m' num2str(mPos,'%02.f')];
        save(fileName,'rir','p_meas');
        disp(['Data saved to ' fileName])
    end

    disp(['Done with measurements for Source Position ' num2str(sPos)])
end