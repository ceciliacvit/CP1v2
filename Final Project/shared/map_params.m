function [mapResolution, maxLidarRange] = map_params()
% Shared occupancy-grid parameters. mapResolution MUST match the resolution
% explored_map.mat was saved at, so the navigation planning map and the
% saved/MCL map line up.
    mapResolution = 25;
    maxLidarRange = 8;
end
