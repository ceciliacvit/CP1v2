function path = createPath(start, goal, map)
% Find path using PRM

rng(0)

prm = mobileRobotPRM(map); % create PRM planner

path = prm.findpath(start, goal); % query planner for path

figure
prm.show()
hold on
plot(start(1), start(2), 'r*', 'MarkerSize', 12)
text(start(1), start(2), ' START')
plot(goal(1), goal(2), 'ro', 'MarkerSize', 12)
text(goal(1), goal(2), ' GOAL')
end