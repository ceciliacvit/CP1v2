function path = createPath(start, goal, map)
    % Try to generate a safe path keeping away from walls (inflated map)
    safeMap = copy(map);
    inflate(safeMap, 0.2);
    
    prm = mobileRobotPRM(safeMap, 1000);
    prm.ConnectionDistance = 5;
    path = findpath(prm, start, goal);
    
    % If the robot or goal is too close to a wall, the inflated map swallows them 
    % and the safe path fails. In that case, fall back to the uninflated map.
    if isempty(path)
        prm = mobileRobotPRM(map, 1000);
        prm.ConnectionDistance = 5;
        path = findpath(prm, start, goal);
    end
end
