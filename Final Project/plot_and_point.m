function points = plot_and_point(slamAlg)
    maxLidarRange = 8;
    mapResolution = 20;

    [scans, optimizedPoses]  = scansAndPoses(slamAlg);
    map = buildMap(scans, optimizedPoses, mapResolution, maxLidarRange);

    show(map);
    title('Click 2 points: 1st for destination B, 2nd for destination C');

    [x_clicks, y_clicks] = ginput(2);

    hold on
    plot (x_clicks,y_clicks, 'ro', 'MarkerSize', 8, 'LineWidth', 2);
    hold off

    points = [x_clicks,y_clicks];
    close(gcf);
end
