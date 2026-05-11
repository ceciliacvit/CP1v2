function slamAlg = move_to_box(scanSub,odomSub,cmdPub,box,slamAlg,initialPose)
    arguments
        scanSub
        odomSub
        cmdPub
        box
        slamAlg
        initialPose = []
    end
'botox started'
maxLidarRange = 8;
mapResolution = 20;
global visualise
visualise=TurtleBotVisualise();
figure1 = figure;

% Starting position: use MCL result when provided (loaded map case),
% otherwise trust the last SLAM pose from exploration.
[scans, optimizedPoses] = scansAndPoses(slamAlg);
map = buildMap(scans, optimizedPoses, mapResolution, maxLidarRange);
if ~isempty(initialPose)
    position = initialPose(1:2);
else
    position = optimizedPoses(end,1:2);
end

while true
    target = findNextTarget(box);

    path = createPath(position,target,map);
    path

    [slamAlg,fail] = MoveRobot(path,slamAlg,scanSub,odomSub,cmdPub,figure1,0.8);

    % MoveRobot has updated slamAlg — get fresh position and map from it.
    [scans, optimizedPoses] = scansAndPoses(slamAlg);
    map = buildMap(scans, optimizedPoses, mapResolution, maxLidarRange);
    position = optimizedPoses(end,1:2);

    plot_all(figure1,map,optimizedPoses);
    drawnow;
end

end


function target = findNextTarget(box)
    box
    p1 = box(1,:);
    p2 = box(2,:);
    p3 = box(3,:);
    p4 = box(4,:);

    m1 = p1 + (p2-p1) .* rand();
    m2 = p3 + (p4-p3) .* rand();

    target = m1 + (m2-m1) .* rand();
end
