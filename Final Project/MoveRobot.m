function MoveRobot(path,percentage_threshold,tolerance)
    arguments
        path;
        percentage_threshold = 1;
        tolerance = 0.15;
    end

%% for finding position
maxLidarRange = 8;
mapResolution = 20;
slamAlg = lidarSLAM(mapResolution, maxLidarRange);

slamAlg.LoopClosureThreshold = 210;  
slamAlg.LoopClosureSearchRadius = 8;

%% for navigation

time_previous = tic;


path_index = 1;
desired_position = path(path_index,:);

%% PID initialization

headingErrorInt = 0;
headingErrorPrev = 0;

distanceErrorInt = 0;
distanceErrorPrev = 0;

% Heading PID gains
Kp_h = 1.2;
Ki_h = 0.0;
Kd_h = 0.02;

% Distance PID gains
Kp_d = 0.4;
Ki_d = 0.0;
Kd_d = 0.0;

% for stopping
drive = true;

%% naw dawg
while true
    try
        scanSub = ros2subscriber(controlNode, '/scan', {}, 'Reliability', 'besteffort'); % laser scan topic
        cmdPub = ros2publisher(controlNode, '/cmd_vel', 'geometry_msgs/Twist');
        break
    catch

    end
end

while drive

    %% find position and angle
    scan = receive(scanSub);
    try % catch if scan is empty
        lidarScan=rosReadLidarScan(scan);
    catch
        continue;
    end    

    addScan(slamAlg, lidarScan);
    
    [~, optimizedPoses]  = scansAndPoses(slamAlg);

    position = optimizedPoses(end,1:2);
    angle = optimizedPoses(end,3);

    %% setup PID
    position_delta = desired_position - position;
    desired_heading = atan2(position_delta(2),position_delta(1));
    distance_to_target = norm(position_delta);

    dt = toc(time_previous);
    time_previous = tic;

    if dt <= 0
        dt = 0.01;
    end

    heading_error = atan2(sin(desired_heading-angle),cos(desired_heading-angle));
    headingErrorInt = headingErrorInt + headingError * dt;
    headingErrorDer = (headingError - headingErrorPrev) / dt;
    headingErrorPrev = heading_error;

    distanceError = distanceToTarget;
    distanceErrorInt = distanceErrorInt + distanceError * dt;
    distanceErrorDer = (distanceError - distanceErrorPrev) / dt;
    distanceErrorPrev = distanceError;

    %% PID

    angularVelocity = Kp_h * headingError ...
                    + Ki_h * headingErrorInt ...
                    + Kd_h * headingErrorDer;

    linearVelocity = Kp_d * distanceError ...
                   + Ki_d * distanceErrorInt ...
                   + Kd_d * distanceErrorDer;

    if distance_to_target < tolerance
        path_index = path_index + 1;
        path_size = size(path,1);
        percentage = path_index /path_size;
        if(path_index < path_size && percentage < percentage_threshold)
            desired_position = path(path_index,:);
        else
            drive = false;
        end
    end

    cmdMsg = ros2message('geometry_msgs/Twist');
    cmdMsg.linear.x = clip(linearVelocity, -0.2, 0.2);
    cmdMsg.angular.z = clip(angularVelocity, -2.0, 2.0);
    send(cmdPub, cmdMsg);
end


end