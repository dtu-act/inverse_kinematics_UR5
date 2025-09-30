%% ========================================================================
% SCRIPT: Single-position RIR measurement with NI USB4431
%
% DESCRIPTION:
%   This script measures Room Impulse Responses (RIRs) using an exponential
%   sweep played and recorded via a National Instruments USB-4431 DAQ card.
%   The sweep is played through a loudspeaker and recorded by one or more
%   microphones at defined positions. Impulse responses and corresponding
%   frequency responses are calculated and saved for later analysis.
%
% WORKFLOW:
%   1) Define measurement parameters (sweep, sample rate, repetitions).
%   2) Load acoustic scenario from a JSON file.
%   3) Configure NI USB4431 DAQ (output and input voltage channels).
%   4) Generate exponential sweep signal.
%   5) For each source and microphone position:
%        • Play sweep and record microphone signal
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
%   • MATLAB Data Acquisition Toolbox
%   • NI USB-4431 DAQ with configured drivers
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
gain_sweep = -15;        % Sweep gain
maxRep = 2;             % Max repetitions in case of samples over/underrun
numMicPos = 6;          % Num microphone positions per source position
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
% Configure NI USB4431
out_range    = [-3.5 3.5];

d = daqlist('ni');
devName = d.DeviceID;  % Assume only one card is connected
ni = daq('ni');
ni.Rate = fs;

% Output
ni.addoutput(devName,0,'Voltage');
ni.Channels(1).Range = out_range;

% Inputs
ni.addinput(devName,0,'Voltage');

% Check channels:
% ni.Channels

%% ==== Create exponential sweep ====
Nh = fs*Toff;
t = 0:1/fs:Toff-1/fs;
f = (0:Nh/2-1)*fs/Nh;

% Gen sweep
sweep = sweeptone(T,Toff,fs,'SweepFrequencyRange',Fband,'ExcitationLevel',gain_sweep);

%% ==== Run measurements ====
% Check for folder
if ~exist(folderData,'dir')
    mkdir(folderData)
end

% Save metadata
% save([folderData 'metadata'])

figure(1)
figure(2)
for sPos = 1:numSourcePos
    disp(['---- Source Position ' num2str(sPos) ' -----'])

    for mPos = 1:numMicPos
        disp(['Mic Position ' num2str(mPos) ' -----'])

        disp('Ready?'), pause

        % ---- measure RIR ---------------------------------------------------
        % Play + record signal
        p_meas = readwrite(ni,sweep,'OutputFormat','Matrix');

        % Gain Nexus
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