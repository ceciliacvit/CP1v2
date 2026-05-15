function slamAlg = explore(scanSub,cmdPub,odomSub)
'explore started'
maxLidarRange = 8;
mapResolution = 20;
slamAlg = lidarSLAM(mapResolution, maxLidarRange);

slamAlg.OptimizationInterval = 1;
slamAlg.LoopClosureMaxAttempts = 10;
slamAlg.LoopClosureThreshold = 100;  
slamAlg.LoopClosureSearchRadius = 8;
% global visualise
% if isempty(visualise) || ~isvalid(visualise.fig)
%     visualise=TurtleBotVisualise();
% end
figure1 = figure;
figure2 = figure;
path=[0,0];
target = [0,0];
i = 0;
initialized=false;
position = [0, 0];
last_scan_position = [Inf, Inf];
last_odom_pos   = [0, 0];
last_odom_angle = 0;
odom_msg = odomSub.LatestMessage;
if ~isempty(odom_msg)
    last_odom_pos   = [odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y];
    last_odom_angle = quat_to_yaw(odom_msg.pose.pose.orientation);
end
duds=[];
fails=0;
patience = 3; % How many times to try to go to target before new target is created
start_explore = tic;
explore_time = 60*10;

while true
    scan = receive(scanSub);
    if isempty(scan)
        continue;
    end
    try % catch if scan is empty
        lidarScan=rosReadLidarScan(scan);
    catch
        continue;
    end    

    if norm(position - last_scan_position) > 0.1
        odom_msg = odomSub.LatestMessage;
        if ~isempty(odom_msg)
            curr_odom_pos   = [odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y];
            curr_odom_angle = quat_to_yaw(odom_msg.pose.pose.orientation);
            d_odom  = curr_odom_pos - last_odom_pos;
            d_theta = wrapToPi(curr_odom_angle - last_odom_angle);
            c = cos(last_odom_angle); s = sin(last_odom_angle);
            relPoseEst = [c*d_odom(1)+s*d_odom(2), -s*d_odom(1)+c*d_odom(2), d_theta];
            addScan(slamAlg, lidarScan, relPoseEst);
            last_odom_pos   = curr_odom_pos;
            last_odom_angle = curr_odom_angle;
        else
            addScan(slamAlg, lidarScan);
        end
        last_scan_position = position;

        %create map
        [scans, optimizedPoses]  = scansAndPoses(slamAlg);
        map = buildMap(scans, optimizedPoses, mapResolution, maxLidarRange);

        position = optimizedPoses(end,1:2);
    end

    %plot map
    plot_all(figure1,map,optimizedPoses);
    

    grid_pos = world2grid(map,position);

    if i == 20
        initialized=true;
    end

    % Find new target
    if initialized==true
        target = findNextTarget(map, grid_pos,figure2,duds);
        target=grid2world(map,target);
       
        path = createPath(position,target,map);
        
        [slamAlg,fail] = MoveRobot(path,slamAlg,scanSub,odomSub,cmdPub,figure1,1);

        % Resync after navigation so the scan threshold doesn't deadlock
        [scans, optimizedPoses] = scansAndPoses(slamAlg);
        map = buildMap(scans, optimizedPoses, mapResolution, maxLidarRange);
        position = optimizedPoses(end,1:2);
        last_scan_position = position;
        odom_msg = odomSub.LatestMessage;
        if ~isempty(odom_msg)
            last_odom_pos   = [odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y];
            last_odom_angle = quat_to_yaw(odom_msg.pose.pose.orientation);
        end

        if fail
            fails=fails+1;
            if fails>patience
                duds(end+1,:)=target;
            end
        end
    end


    
    hold off;
    drawnow;
    i = i+1;

    time_explored = toc(start_explore);
    if(time_explored > explore_time)
        "timeout"
        [scans, optimizedPoses] = scansAndPoses(slamAlg);
        map = buildMap(scans, optimizedPoses, mapResolution, maxLidarRange);

        occ_matrix    = map.occupancyMatrix;
        resolution    = map.Resolution;
        grid_location = map.GridLocationInWorld;
        save('explored_map.mat', 'occ_matrix', 'resolution', 'grid_location', 'slamAlg');

        img = flipud(1 - occ_matrix);
        imwrite(img, 'explored_map.png');

        return
    end


end

end

function target = findNextTarget(map, grid_pos,fig,duds)
    probabilities = map.occupancyMatrix;

    BW = edge(probabilities,'sobel');
    
    inflation = occupancyMap(probabilities);
    inflate(inflation,0.4);
    BW = BW .* (inflation.occupancyMatrix *-1 + 1);
    size(BW)
    size(radialLinearMatrix(size(BW),grid_pos,0.2))
    BW = BW .* radialLinearMatrix(size(BW),grid_pos,0.2);
    BW = imfilter(BW, ones(3,3)/9, 'replicate');
    
    radius = 4; % tune this
    P = ones(size(BW));

    [X,Y] = meshgrid(1:size(BW,2), 1:size(BW,1));
    
    for i = 1:size(duds,1)
        dud = duds(i,:);
        duds;
        dud;

        D = sqrt((X - dud(1)).^2 + (Y - dud(2)).^2);
        mask = D > radius;

        % inside radius → penalize
        localPenalty = mask + (~mask)*0.2;  % 0.2 = strong suppression
        
        P = P .* localPenalty;
    end

    BW = BW .* P;
    [~, sorted_idx] = sort(BW(:), 'descend');
    k = 10; % top candidates
    if length(sorted_idx)>k
        candidates = sorted_idx(1:k);        
    else
        candidates = sorted_idx;
    end

    chosen = candidates(randi(k));
    [X,Y] = ind2sub(size(BW), chosen);



    figure(fig)
    hold on

    axis on
    imshow(BW)
    plot(X,Y,'ro')
    hold off

    target=[X,Y];
end

function M = radialLinearMatrix(size, grid_pos,minVal)
    [X,Y] = meshgrid(1:size(2),1:size(1));

    D = sqrt((Y-grid_pos(2)).^2 + (X-grid_pos(1)).^2);   % distance from chosen point
    Dmax = max(D(:));                  % farthest point in matrix

    M = max(minVal, 1 - 0.8*(D/Dmax));
end

function yaw = quat_to_yaw(q)
    eul = quat2eul([q.w, q.x, q.y, q.z]);
    yaw = eul(1);
end