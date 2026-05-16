function plot_all(figureHandle, map, poses, path)
arguments
    figureHandle
    map
    poses
    path = []
end
figure(figureHandle);
show(map);
hold on;
position = poses(end,1:2);

if ~isempty(path)
    plot(path(:,1), path(:,2), 'r-', 'LineWidth', 2);
    plot(path(:,1), path(:,2), 'bo', 'MarkerSize', 4, 'MarkerFaceColor', 'b');
end

plot(position(1), position(2), 'mo', 'MarkerSize', 8, 'MarkerFaceColor', 'm');

hold off;
drawnow;
end
