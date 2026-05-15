function slamAlg = teleop_map(scanSub, odomSub, imuSub, slamAlg)
% Passive SLAM map builder.
% Drive the robot externally (e.g. `ros2 run turtlebot3_teleop teleop_keyboard`).
% This function listens to /scan, /odom, /imu, builds the map, and saves on Q.
% Uses IMU yaw for heading (more accurate than wheel-encoder yaw) and
% /odom for translation. Falls back to /odom yaw if no IMU messages arrive.
% Pass an existing slamAlg to resume an interrupted mapping run.
arguments
    scanSub
    odomSub
    imuSub
    slamAlg = []
end
'teleop_map started — drive from teleop_keyboard, press Q in figure to save'

    % SLAM config — tuned for accuracy in a 30x30m corridor environment.
    % Controls run in a separate process, so MATLAB loop speed doesn't matter.
    maxLidarRange = 8;
    mapResolution = 25;  % 4 cm cells — finer detail in the saved map
    if isempty(slamAlg)
        slamAlg = lidarSLAM(mapResolution, maxLidarRange);
    else
        'teleop_map resuming with existing slamAlg'
    end
    % Apply tuning to both fresh and resumed slamAlg objects
    slamAlg.OptimizationInterval     = 1;   % optimize after every scan
    slamAlg.LoopClosureMaxAttempts   = 10;  % more chances to find closures
    slamAlg.LoopClosureThreshold     = 180; % strict — corridors alias
    slamAlg.LoopClosureSearchRadius  = 15;  % wide enough for cross-room loops

    scan_spacing = 0.15;

    % Visualization (plot_all expects this global)
    global visualise
    visualise = TurtleBotVisualise();

    quit_flag = false;

    figure1 = figure('Name','Map Viewer (press Q to save and exit)', ...
                     'NumberTitle','off', ...
                     'KeyPressFcn',@onKey);

    % Initial odom reference (translation only)
    last_odom_pos = [0, 0];
    odom_msg = odomSub.LatestMessage;
    if ~isempty(odom_msg)
        last_odom_pos = [odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y];
    end

    % Initial heading reference — prefer IMU, fall back to odom yaw.
    % We track yaw relative to the initial reading so it starts at 0
    % (matches /odom convention and keeps the SLAM frame natural).
    imu_msg = imuSub.LatestMessage;
    if ~isempty(imu_msg)
        use_imu = true;
        yaw_origin = quat_to_yaw(imu_msg.orientation);
        'teleop_map: using /imu for heading'
    elseif ~isempty(odom_msg)
        use_imu = false;
        yaw_origin = quat_to_yaw(odom_msg.pose.pose.orientation);
        'teleop_map: /imu unavailable, falling back to /odom yaw'
    else
        use_imu = false;
        yaw_origin = 0;
    end
    last_yaw = 0;

    rotation_gate = 0.15;  % rad — tighter so corners get well-covered
    first_scan = true;
    map_rebuild_interval = 2;  % only rebuild map / plot every N addScans
    scans_since_rebuild = 0;

    while ~quit_flag && isvalid(figure1)

        % Receive a scan (1s timeout so the loop doesn't hang on stalled /scan)
        try
            scan = receive(scanSub, 1.0);
        catch
            drawnow;
            continue
        end
        if isempty(scan)
            drawnow;
            continue
        end
        try
            lidarScan = rosReadLidarScan(scan);
        catch
            continue
        end

        % Read latest odom for position
        odom_msg = odomSub.LatestMessage;
        if isempty(odom_msg)
            drawnow limitrate;
            continue
        end
        curr_odom_pos = [odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y];

        % Read latest heading source (IMU if available, else /odom yaw),
        % expressed relative to the initial reading so it starts at 0.
        if use_imu
            imu_msg = imuSub.LatestMessage;
            if ~isempty(imu_msg)
                curr_yaw_abs = quat_to_yaw(imu_msg.orientation);
            else
                curr_yaw_abs = quat_to_yaw(odom_msg.pose.pose.orientation);
            end
        else
            curr_yaw_abs = quat_to_yaw(odom_msg.pose.pose.orientation);
        end
        curr_yaw = wrapToPi(curr_yaw_abs - yaw_origin);

        d_pos = norm(curr_odom_pos - last_odom_pos);
        d_rot = abs(wrapToPi(curr_yaw - last_yaw));

        % Add a scan on first frame, or once motion exceeds the gate
        if first_scan || d_pos > scan_spacing || d_rot > rotation_gate
            d_odom = curr_odom_pos - last_odom_pos;
            d_theta = wrapToPi(curr_yaw - last_yaw);
            c = cos(last_yaw); s = sin(last_yaw);
            relPoseEst = [c*d_odom(1)+s*d_odom(2), -s*d_odom(1)+c*d_odom(2), d_theta];
            addScan(slamAlg, lidarScan, relPoseEst);

            last_odom_pos = curr_odom_pos;
            last_yaw      = curr_yaw;
            first_scan = false;
            scans_since_rebuild = scans_since_rebuild + 1;

            % Only rebuild + plot every N scans — buildMap is O(graph size)
            % and blocks key callbacks while it runs.
            if scans_since_rebuild >= map_rebuild_interval
                [scans, optimizedPoses] = scansAndPoses(slamAlg);
                map = buildMap(scans, optimizedPoses, mapResolution, maxLidarRange);
                if ~isempty(map) && isvalid(figure1)
                    plot_all(figure1, map, optimizedPoses);
                end
                scans_since_rebuild = 0;
            end
        end

        drawnow limitrate;
    end

    % Final map
    [scans, optimizedPoses] = scansAndPoses(slamAlg);
    if isempty(scans)
        'teleop_map: no scans recorded, nothing to save'
        return
    end
    map = buildMap(scans, optimizedPoses, mapResolution, maxLidarRange);

    occ_matrix    = map.occupancyMatrix;
    resolution    = map.Resolution;
    grid_location = map.GridLocationInWorld;
    save('explored_map.mat', 'occ_matrix', 'resolution', 'grid_location', 'slamAlg');

    img = flipud(1 - occ_matrix);
    imwrite(img, 'explored_map.png');

    'teleop_map saved explored_map.mat'

    function onKey(~, event)
        if strcmp(event.Key, 'q')
            quit_flag = true;
        end
    end
end

function yaw = quat_to_yaw(q)
    eul = quat2eul([q.w, q.x, q.y, q.z]);
    yaw = eul(1);
end
