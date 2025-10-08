%% ==== Clear workspace ====
clear, clc, close all

addpath(genpath('../tools'))
loadPlotParams

%% Load responses
% RIR: folder and file structure
folderData = '../../Data/Office/CuboidData/';
fileNamePrefix = 'cuboid_RIR_pos_';

% Load metadata
load([folderData 'metadata.mat'],'Toff','fs','Fband','Nsamples','Nh', ...
    't','numPositions','posRobotGlobal','posSourceGlobal','roomDimensions', ...
    'targetPositions')

posArrayGlobal = targetPositions + posRobotGlobal;

%% Plot setup
figure
scatter3(posSourceGlobal(1), posSourceGlobal(2), posSourceGlobal(3), 200, 'filled'), hold on
scatter3(posArrayGlobal(:,1), posArrayGlobal(:,2), posArrayGlobal(:,3))
scatter3(posRobotGlobal(1), posRobotGlobal(2), posRobotGlobal(3))
drawRoom(roomDimensions(1), roomDimensions(2), roomDimensions(3))
grid on, axis equal
xlabel('$x$ / m'), ylabel('$y$ / m'), zlabel('$z$ / m')
legend('Source', 'Cuboid array', 'Robot centre')
applyAxisProperties(gca)
applyLegendProperties(gcf)

%% Plot RIRs: all together
% Load responses
RIRs = nan(Nh, size(posArrayGlobal,1));
for iPos = 1:size(posArrayGlobal,1)
    fileName = [folderData fileNamePrefix num2str(iPos,'%04.f')];
    load(fileName,'rir');
    RIRs(:,iPos) = rir;
end
clear rir

%% Filter responses
buttFilter = designfilt('bandpassfir','filterorder',4,'CutoffFrequency1',200,'CutoffFrequency2',7000,'samplerate',fs);
RIRs = filtfilt(buttFilter,RIRs);

%% Normalise
maxRIR = max(abs(RIRs),[],'all');
RIRs = RIRs/maxRIR;

%% Plot
% Time constraints
T = [6 7.5]*1e-3;

dataPlot = RIRs(t>=T(1) & t<=T(2),:);
timePlot = t(t>=T(1) & t<=T(2));

%% Combine plots in scatter3
figure
% v = VideoWriter('office.mp4','MPEG-4'); % Create video file
% v.FrameRate = 10;                            % Adjust playback speed (frames per second)
% open(v)
% pause
for iT = 1:5:numel(timePlot)
    scatter3(targetPositions(:,1), targetPositions(:,2), targetPositions(:,3), 100, 5*dataPlot(iT,:), 'filled', 'MarkerEdgeColor','k')
    clim([-1 1])
    axis equal
    title(['Time = ' num2str(timePlot(iT)*1e3) ' ms'])
    colorbarpwn(-1,1)
    c = colorbar;
    xlabel('X-axis'), ylabel('Y-axis'), zlabel('Z-axis')
    set(gca,'Xdir','reverse')
    applyColorbarProperties(c,'Normalised $h(t)$')
    applyAxisProperties(gca)
    drawnow

    % % Capture the frame and write to video
    % frame = getframe(gcf);
    % writeVideo(v, frame);
end

% close(v)  % Finish writing