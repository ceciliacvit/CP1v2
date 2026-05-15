function plot_all(figure1, map, poses, path, lookahead_point, target)
arguments
    figure1
    map
    poses
    path = [0,0]
    lookahead_point = []
    target = [0,0]
end
figure(figure1);
show(map);
hold on;
position = poses(end,1:2);

scatter(target(1),target(2));
plot(path(:,1), path(:,2), 'r-', 'LineWidth', 2);
plot(path(:,1), path(:,2), 'bo', 'MarkerSize', 4, 'MarkerFaceColor', 'b');
if ~isempty(lookahead_point)
    plot(lookahead_point(1), lookahead_point(2), 'g*', 'MarkerSize', 12, 'LineWidth', 2);
end

% Plot current robot position
plot(position(1), position(2), 'mo', 'MarkerSize', 8, 'MarkerFaceColor', 'm');

hold off;
drawnow;
end

