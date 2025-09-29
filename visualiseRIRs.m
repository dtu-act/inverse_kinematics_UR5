%% ==== Clear workspace ====
clear, clc, close all

addpath(genpath('../tools'))
loadPlotParams

%% Load responses
% RIR: folder and file structure
folderData = 'Data/SFControlRoom/CuboidData/';
fileNamePrefix = 'cuboid_RIR_pos_';

% Load metadata
load([folderData 'metadata.mat'],'Toff','fs','Fband','Nsamples','Nh', ...
    't','numPositions','posRobotGlobal','posSourceGlobal','roomDimensions', ...
    'targetPositions','centroid')

if contains(folderData,'Kitchen')
    posSourceGlobal = posSourceGlobal([2 1 3]);
end
posArrayGlobal = targetPositions + posRobotGlobal;

% % Load all responses
% rirs = nan(Nh, numPositions);
% for iPos = 1:numPositions
%     fileName = [folderData fileNamePrefix num2str(iPos,'%04.f')];
%     load(fileName,'rir');
%     rirs(:,iPos) = rir;
% end
% 
% clear rir

% Array settings:
% fAlias ~ 5.5 kHz
% dMin ~ 3.12 cm

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

%% Plot RIRs: X-axis
% Find mid-plane indices
tol = 1e-6;
zPlane = centroid(3);
zPlaneIdx = find(abs(targetPositions(:,3) - zPlane) < tol);

% Find x-axis
xAxisIdx = zPlaneIdx(abs(targetPositions(zPlaneIdx,2) - centroid(2)) < tol);

% Load responses
xAxisRIRs = nan(Nh, numel(xAxisIdx));
for iPos = 1:numel(xAxisIdx)
    fileName = [folderData fileNamePrefix num2str(xAxisIdx(iPos),'%04.f')];
    load(fileName,'rir');
    xAxisRIRs(:,iPos) = rir;
end
clear rir

% Normalise
maxRIR = max(abs(xAxisRIRs),[],'all');
xAxisRIRs = xAxisRIRs/maxRIR;

% Time constraints
T = [0 60]*1e-3;

dataPlot = xAxisRIRs(t>T(1) & t<T(2),:);
timePlot = t(t>T(1) & t<T(2));

% Plot
figure, hold on
s = pcolor(posArrayGlobal(xAxisIdx,1),timePlot*1e3,dataPlot);
set(s,'edgecolor','none')
xlabel('$x$ / m'), ylabel('Time / ms')
% axis([0 Data.D(1) T(1)*1e3 T(2)*1e3])
ylim([T(1)*1e3 T(2)*1e3])
DR = colormapDR(dataPlot);
colorbarpwn(-DR,DR)
c = colorbar;
clim([-1 1])
applyColorbarProperties(c,'Normalised $h(t)$')
applyAxisProperties(gca)

%% Plot RIRs: Y-axis
% Find mid-plane indices
tol = 1e-6;
zPlane = centroid(3);
zPlaneIdx = find(abs(targetPositions(:,3) - zPlane) < tol);

% Find x-axis
yAxisIdx = zPlaneIdx(abs(targetPositions(zPlaneIdx,1) - centroid(1)) < tol);

% Load responses
yAxisRIRs = nan(Nh, numel(yAxisIdx));
for iPos = 1:numel(yAxisIdx)
    fileName = [folderData fileNamePrefix num2str(yAxisIdx(iPos),'%04.f')];
    load(fileName,'rir');
    yAxisRIRs(:,iPos) = rir;
end
clear rir

% Normalise
maxRIR = max(abs(yAxisRIRs),[],'all');
yAxisRIRs = yAxisRIRs/maxRIR;

% Time constraints
T = [0 60]*1e-3;

dataPlot = yAxisRIRs(t>T(1) & t<T(2),:);
timePlot = t(t>T(1) & t<T(2));

% Plot
figure, hold on
s = pcolor(posArrayGlobal(yAxisIdx,2),timePlot*1e3,dataPlot);
set(s,'edgecolor','none')
set(gca,'Xdir','reverse')
xlabel('$y$ / m'), ylabel('Time / ms')
% axis([0 Data.D(1) T(1)*1e3 T(2)*1e3])
ylim([T(1)*1e3 T(2)*1e3])
DR = colormapDR(dataPlot);
colorbarpwn(-DR,DR)
c = colorbar;
clim([-1 1])
applyColorbarProperties(c,'Normalised $h(t)$')
applyAxisProperties(gca)

%% Plot RIRs: Plane
% Find mid-plane indices
tol = 1e-6;
zPlane = centroid(3);
zPlaneIdx = find(abs(targetPositions(:,3) - zPlane) < tol);

% Load responses
zPlaneRIRs = nan(Nh, numel(zPlaneIdx));
for iPos = 1:numel(zPlaneIdx)
    fileName = [folderData fileNamePrefix num2str(zPlaneIdx(iPos),'%04.f')];
    load(fileName,'rir');
    zPlaneRIRs(:,iPos) = rir;
end
clear rir

% Normalise
maxRIR = max(abs(zPlaneRIRs),[],'all');
zPlaneRIRs = zPlaneRIRs/maxRIR;

% Animation: Plot plane
figure
for iT = 1:10:Nh
    scatter(posArrayGlobal(zPlaneIdx,1),posArrayGlobal(zPlaneIdx,2), 200, zPlaneRIRs(iT,:), 'filled')
    clim([-1 1])
    axis equal
    title(['Time = ' num2str(t(iT)*1e3) ' ms'])
    colorbarpwn(-1,1)
    c = colorbar;
    applyColorbarProperties(c,'Normalised $h(t)$')
    applyAxisProperties(gca)
    drawnow
end
