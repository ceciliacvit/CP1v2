function plot_all(figure1, map, poses, path, target)
arguments
    figure1
    map
    poses
    path = [0,0]
    target = [0,0]
end
figure(figure1);
show(map);
hold on;
position = poses(end,1:2);
angle = poses(end,3);

global visualise;
visualise = updatePose(visualise, position , angle);
scatter(target(1),target(2));
plot(path(:,1), path(:,2), 'r-', 'LineWidth', 2);
plot(path(:,1), path(:,2), 'bo', 'MarkerSize', 4, 'MarkerFaceColor', 'b');
hold off;
drawnow;
end

