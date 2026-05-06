function slamAlg = explore(scanSub,cmdPub)
'explore started'
maxLidarRange = 8;
mapResolution = 20;
slamAlg = lidarSLAM(mapResolution, maxLidarRange);

slamAlg.LoopClosureThreshold = 210;  
slamAlg.LoopClosureSearchRadius = 8;
global visualise
visualise=TurtleBotVisualise();
figure1 = figure;
figure2 = figure;
path=[0,0];
target = [0,0];
i = 0;
initialized=false;
duds=[];
fails=0;
patience = 3; % How many times to try to go to target before new target is created

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

    addScan(slamAlg, lidarScan);
    
    %create map
    [scans, optimizedPoses]  = scansAndPoses(slamAlg);
    map = buildMap(scans, optimizedPoses, mapResolution, maxLidarRange);
    
    position = optimizedPoses(end,1:2);

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
        
        [slamAlg,fail] = MoveRobot(path,slamAlg,scanSub,cmdPub,figure1,1);
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

    if(i > 30)
        "Finished searching area"
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
        duds
        dud

        D = sqrt((X - dud(1)).^2 + (Y - dud(2)).^2);
        mask = D > radius;

        % inside radius → penalize
        localPenalty = mask + (~mask)*0.2;  % 0.2 = strong suppression
        
        P = P .* localPenalty;
    end

    BW = BW .* P;
    BW = BW.*P;
    [~, sorted_idx] = sort(BW, 'descend');
    
    k = 10; % top candidates
    if length(sorted_idx)>k
        candidates = sorted_idx(1:k);
    else
        candidates = sorted_idx;
    end
    
    chosen = candidates(randi(k));
    [X,Y] = ind2sub(size(BW), chosen);
    target=[X,Y];
end

function M = radialLinearMatrix(size, grid_pos,minVal)
    [X,Y] = meshgrid(1:size(2),1:size(1));
    
    D = sqrt((Y-grid_pos(2)).^2 + (X-grid_pos(1)).^2);   % distance from chosen point
    Dmax = max(D(:));                  % farthest point in matrix
    
    M = max(minVal, 1 - 0.8*(D/Dmax));
end