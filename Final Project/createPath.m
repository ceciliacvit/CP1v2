function path = createPath(start, goal, map)
% Find path using PRM


mapCopy = copy(map);
updateOccupancy(mapCopy, max(checkOccupancy(mapCopy), 0));
inflate(mapCopy, 0.15);

prm = mobileRobotPRM(mapCopy, 500); % create PRM planner



path = prm.findpath(start, goal);

end