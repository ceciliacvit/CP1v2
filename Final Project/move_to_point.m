function final_pose = move_to_point(scanSub,odomSub,cmdPub,target,slamAlg,initialPose)
    arguments
        scanSub
        odomSub
        cmdPub
        target
        slamAlg
        initialPose = []
    end
'move_to_point started'
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
    current_pose = initialPose;
    use_slam = false;
else
    current_pose = optimizedPoses(end,:);
    use_slam = true;
end

while true
    path = createPath(current_pose(1:2),target,map);
    path

    [slamAlg,fail,final_pose] = MoveRobot(path,slamAlg,scanSub,odomSub,cmdPub,figure1,0.8,0.20,current_pose,use_slam);
    current_pose = final_pose;

    if use_slam
        % MoveRobot has updated slamAlg — get fresh map from it.
        [scans, optimizedPoses] = scansAndPoses(slamAlg);
        map = buildMap(scans, optimizedPoses, mapResolution, maxLidarRange);
        plot_all(figure1,map,optimizedPoses);
    else
        plot_all(figure1,map,current_pose);
    end
    
    drawnow;

    if ~fail
        break;
    end
end
close(figure1);
end