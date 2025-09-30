%% ========================================================================
% SCRIPT: Single-position RIR measurement with RME Fireface
%
% DESCRIPTION:
%   This script measures Room Impulse Responses (RIRs) using an exponential
%   sweep played through an RME Fireface USB sound card (ASIO driver).
%   The sweep is recorded with one or more microphones placed at defined
%   positions. Both time-domain impulse responses and frequency responses
%   are obtained, with results saved for later analysis.
%
% WORKFLOW:
%   1) Define measurement parameters (sweep, sample rate, repetitions).
%   2) Load the acoustic scenario from a JSON file.
%   3) Configure the audio I/O interface:
%        • Player: loudspeaker channel
%        • Recorder: microphone channel
%   4) Generate and slice exponential sweep into frames.
%   5) For each source and microphone position:
%        • Play the sweep and record microphone signal
%        • Estimate impulse response (RIR)
%        • Compute and plot impulse & frequency responses
%        • Save data to disk
%
% INPUTS:
%   - Scenario JSON file: Environment/<scenarioName>.json
%
% OUTPUTS:
%   - Saved RIRs and raw signals in: Data/<ScenarioName>/RT/
%   - Figures:
%       • Impulse response and frequency response (Figure 1)
%       • Spectrogram of recorded signal (Figure 2)
%
% REQUIREMENTS:
%   • MATLAB Audio Toolbox
%   • RME Fireface USB with ASIO driver
%   • Custom function: get_dataRME.m
%
% AUTHOR: Antonio Figueroa-Duran
% CONTACT: anfig@dtu.dk
% ========================================================================


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
frameSize = 512;        % Frame size to minimise latency
maxRep = 2;             % Max repetitions in case of samples over/underrun
numMicPos = 1;          % Num microphone positions per source position
numSourcePos = 1;       % Num source positions

% Load scenario
scenarioName = 'Kitchen';
scenario = loadScenario(['Environment/' lower(scenarioName) '.json']);

% Room conditions
roomDimensions = scenario.room_dimensions; % Room dimensions [m x m x m]
tempC = 22.7;       % Temperature [C]
humidityRH = 52.2;  % Relative humidity [%RH]

% RIR: folder and file structure
folderData = ['Data/' scenarioName '/RT/'];
fileNamePrefix = 'single_RIR_';

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

%% ==== Create exponential sweep ====
% Adapt sweep length to obtain full frames (initial RIR length is modified)
nFrames = ceil(Nsamples/frameSize);
Nsamples = nFrames*frameSize;
Toff = Nsamples/fs - T;
Nh = fs*Toff;
t = 0:1/fs:Toff-1/fs;
f = (0:Nh/2-1)*fs/Nh;

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
% save([folderData 'metadata'])

figure(1), hold on
figure(2)
for sPos = 1:numSourcePos
    disp(['---- Source Position ' num2str(sPos) ' -----'])

    for mPos = 1:numMicPos
        disp(['Mic Position ' num2str(mPos) ' -----'])

        disp('Ready?'), pause

        % ---- measure RIR ---------------------------------------------------
        % Play + record signal
        [p_meas,flagRep] = get_dataRME(playRec,audioToPlay,maxRep);
        if flagRep == 1, disp(['ERROR: Max repetition number reached for position (s' num2str(sPos) ',m' num2str(mPos) ')']); end

        % Gain Nexus + RME
        p_meas = p_meas/gain_nexus;

        % Calculate RIR
        rir = impzest(sweep,p_meas);

        rtf = fft(rir)/Nh;
        rtf = 2*rtf(1:Nh/2);

        % Plot RIR
        figure(1)
        subplot(211), hold on
        plot(t,rir), grid on
        xlim([0 100e-3])
        xlabel('Time / s'), title('Impulse response')

        subplot(212), hold on
        plot(f,20*log10(abs(rtf))), grid on
        xlabel('Frequency / Hz'), title('Frequency response')

        % Plot spectrogram
        figure(2)
        spectrogram(p_meas,512)

        % Save pressure & RIR
        fileName = [folderData fileNamePrefix 's' num2str(sPos,'%02.f') '_m' num2str(mPos,'%02.f')];
        save(fileName,'rir','p_meas');
        disp(['Data saved to ' fileName])
    end

    disp(['Done with measurements for Source Position ' num2str(sPos)])
end