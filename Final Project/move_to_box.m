function slamAlg = move_to_box(scanSub,odomSub,cmdPub,box,slamAlg)
'botox started'
maxLidarRange = 8;
mapResolution = 20;
global visualise
visualise=TurtleBotVisualise();
figure1 = figure;
figure2 = figure;
path=[0,0];
target = [0,0];
i = 0;
initialized=false;

while true
    scan = receive(scanSub);
    try % catch if scan is empty
        lidarScan=rosReadLidarScan(scan);
    catch
        continue;
    end    

    addScan(slamAlg, lidarScan);
    
    %create map
    [scans, optimizedPoses]  = scansAndPoses(slamAlg);
    map = buildMap(scans, optimizedPoses, mapResolution, maxLidarRange);
    
    position = optimizedPoses(end,1:2);

    %plot map
    plot_all(figure1,map,optimizedPoses);
    

    grid_pos = world2grid(map,position);

    target = findNextTarget(box);
    
    path = createPath(position,target,map);
    path
    % Plot path
       
    [slamAlg,fail] = MoveRobot(path,slamAlg,scanSub,odomSub,cmdPub,figure1,0.8);
    
    hold off;
    drawnow;
    i = i+1;

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