function pointsOptimised = findOptimalPath(D, points)
% findOptimalPath Finds an optimized path through a set of points.
%
% Inputs:
%   D      - n x n distance matrix (D(i,j) = distance from point i to j)
%   points - n x m coordinates of points
%
% Output:
%   pointsOptimised - points reordered according to a greedy nearest-neighbor path
%
% Author: Antonio Figueroa-Duran
% Contact: anfig@dtu.dk

% For future implementations: consider minimising velocity or overall
% distance to obtain less jerky trajectories. The reader is referred to
% documentation dealing with the Traveling Salesman Path and the
% Hamiltonian Path problems.

numPoints = size(points, 1);

% Replace zeros with NaN to ignore self-distances
D(D == 0) = NaN;

idxOptimised = zeros(numPoints,1);  % Preallocate index array
idxOptimised(1) = 1;  % Start at the first point

for i = 2:numPoints
    % Find the nearest unvisited point
    [~, idx] = min(D(:, idxOptimised(i-1)));
    idxOptimised(i) = idx;

    % Mark the previous point as visited by setting its row and column to NaN
    D(idxOptimised(i-1), :) = NaN;
    D(:, idxOptimised(i-1)) = NaN;
end

pointsOptimised = points(idxOptimised, :);

end