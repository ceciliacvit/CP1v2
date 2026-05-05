function points = plot_and_box(slamAlg)
    maxLidarRange = 8;
    mapResolution = 20;
           
    [scans, optimizedPoses]  = scansAndPoses(slamAlg);
    map = buildMap(scans, optimizedPoses, mapResolution, maxLidarRange);
    
    show(map);

    [x_clicks, y_clicks] = ginput(8);

    hold on
    plot (x_clicks,y_clicks, 'ro');
    hold off

    points = [x_clicks,y_clicks];
end