function path = createPath(start, goal, map)
% Find path using PRM


updateOccupancy(map,max(checkOccupancy(map),0));
inflate(map,0.15);

prm = mobileRobotPRM(map,500,500); % create PRM planner



path = prm.findpath(start, goal); % query planner for path

end