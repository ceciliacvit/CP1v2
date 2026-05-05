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

    % If close to target find another target
    if initialized==true
        target = findNextTarget(map, grid_pos,figure2);
        target=grid2world(map,target);
       
        path = createPath(position,target,map);
        path
        % Plot path
           
        slamAlg = MoveRobot(path,slamAlg,scanSub,cmdPub,figure1,1);
    end
    
    hold off;
    drawnow;
    i = i+1;

    if(i > 19)
        "aaaaaaaaaaaa"
        return
    end
end

end

function target = findNextTarget(map, grid_pos,fig)
    probabilities = map.occupancyMatrix;

    BW = edge(probabilities,'sobel');
    
    inflation = occupancyMap(probabilities);
    inflate(inflation,0.4);
    BW = BW .* (inflation.occupancyMatrix *-1 + 1);
    size(BW)
    size(radialLinearMatrix(size(BW),grid_pos,0.2))
    BW = BW .* radialLinearMatrix(size(BW),grid_pos,0.2);
    BW = imfilter(BW, ones(3,3)/9, 'replicate');
    %figure(fig);
    %imshow(BW)
    %drawnow;
    [~,index] = max(BW,[],'all',"linear"); %finds the maximum over all elements of z
    [X,Y] = ind2sub(size(BW),index);
    target=[X,Y];
end

function M = radialLinearMatrix(size, grid_pos,minVal)
    [X,Y] = meshgrid(1:size(2),1:size(1));
    
    D = sqrt((Y-grid_pos(2)).^2 + (X-grid_pos(1)).^2);   % distance from chosen point
    Dmax = max(D(:));                  % farthest point in matrix
    
    M = max(minVal, 1 - 0.8*(D/Dmax));
end