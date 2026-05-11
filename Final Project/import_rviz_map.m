function map = import_rviz_map(pgm_file_path, resolution_meters_per_cell)
% IMPORT_RVIZ_MAP Reads a standard ROS 2 .pgm map and converts it to a MATLAB occupancy map.
% 
% Inputs:
%   pgm_file_path: String path to the map.pgm file saved by nav2_map_server
%   resolution_meters_per_cell: Typically 0.05 for standard TurtleBot mapping
%
% Output:
%   map: A binaryOccupancyMap object ready for PRM or display

    % 1. Read the image file saved by ROS 2
    img = imread(pgm_file_path);
    
    % ROS .pgm files typically use:
    % 205 or 254 for Free Space (White/Light Gray)
    % 205/128 for Unknown Space (Gray)
    % 0 for Occupied Space (Black)
    
    % 2. Create a logical matrix where obstacles are true (1) and free space is false (0)
    % We assume anything darker than a threshold (e.g., 100) is a solid obstacle.
    occ_matrix = img < 100;
    
    % 3. Convert image coordinates to MATLAB's occupancy grid format
    % The resolution in ROS is meters/cell. MATLAB uses cells/meter.
    cells_per_meter = 1 / resolution_meters_per_cell;
    
    % 4. Create the occupancy map
    map = binaryOccupancyMap(occ_matrix, cells_per_meter);
end