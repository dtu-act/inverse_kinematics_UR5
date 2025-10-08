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

%% Plot RIRs: Horizontal plane
% Find mid-plane indices
tol = 1e-6;
zPlane = median(targetPositions);
zPlaneIdx = find(abs(targetPositions(:,3) - zPlane(:,3)) < tol);

% Load responses
horAxisRIRs = nan(Nh, numel(zPlaneIdx));
for iPos = 1:numel(zPlaneIdx)
    fileName = [folderData fileNamePrefix num2str(zPlaneIdx(iPos),'%04.f')];
    load(fileName,'rir');
    horAxisRIRs(:,iPos) = rir;
end
clear rir

%% Plot RIRs: Vertical plane
% Find vert-plane indices
xPlane = median(targetPositions);
xPlaneIdx = find(abs(targetPositions(:,1) - xPlane(:,1)) < tol);

% Load responses
vertAxisRIRs = nan(Nh, numel(xPlaneIdx));
for iPos = 1:numel(xPlaneIdx)
    fileName = [folderData fileNamePrefix num2str(xPlaneIdx(iPos),'%04.f')];
    load(fileName,'rir');
    vertAxisRIRs(:,iPos) = rir;
end
clear rir

%% Filter responses
buttFilter = designfilt('bandpassfir','filterorder',4,'CutoffFrequency1',200,'CutoffFrequency2',7000,'samplerate',fs);
horAxisRIRs = filtfilt(buttFilter,horAxisRIRs);
vertAxisRIRs = filtfilt(buttFilter,vertAxisRIRs);

%% Normalise
maxRIR = max(abs([horAxisRIRs vertAxisRIRs]),[],'all');
horAxisRIRs = horAxisRIRs/maxRIR;
vertAxisRIRs = vertAxisRIRs/maxRIR;

%% Plot
% Time constraints
T = [5 20]*1e-3;

dataHorPlot = horAxisRIRs(t>=T(1) & t<=T(2),:);
dataVertPlot = vertAxisRIRs(t>=T(1) & t<=T(2),:);
timePlot = t(t>=T(1) & t<=T(2));

%% Plot hor
figure
for iT = 1:10:numel(timePlot)
    scatter(posArrayGlobal(zPlaneIdx,2),posArrayGlobal(zPlaneIdx,1), 100, dataHorPlot(iT,:).', 'filled')
    clim([-1 1])
    axis equal
    title(['Time = ' num2str(timePlot(iT)*1e3) ' ms'])
    colorbarpwn(-1,1)
    c = colorbar;
    applyColorbarProperties(c,'Normalised $h(t)$')
    applyAxisProperties(gca)
    drawnow
end

%% Plot vert
figure
for iT = 1:numel(timePlot)
    scatter(posArrayGlobal(xPlaneIdx,2),posArrayGlobal(xPlaneIdx,3), 100, dataVertPlot(iT,:), 'filled')
    clim([-1 1])
    axis equal
    title(['Time = ' num2str(timePlot(iT)*1e3) ' ms'])
    colorbarpwn(-1,1)
    c = colorbar;
    applyColorbarProperties(c,'Normalised $h(t)$')
    applyAxisProperties(gca)
    drawnow
end
