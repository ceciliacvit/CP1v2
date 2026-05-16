function points = plot_and_point(slamAlg)
    [mapResolution, maxLidarRange] = map_params();

    [scans, optimizedPoses]  = scansAndPoses(slamAlg);
    map = buildMap(scans, optimizedPoses, mapResolution, maxLidarRange);

    show(map);
    title('Click 3 points: B1, B2, C');

    [x_clicks, y_clicks] = ginput(3);

    hold on
    plot (x_clicks,y_clicks, 'ro', 'MarkerSize', 8, 'LineWidth', 2);
    hold off

    points = [x_clicks,y_clicks];
    close(gcf);
end
