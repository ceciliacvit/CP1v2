function path = createPath(start, goal, map)
% Find path using PRM


updateOccupancy(map,max(checkOccupancy(map),0));


prm = mobileRobotPRM(map); % create PRM planner



path = prm.findpath(start, goal); % query planner for path

end