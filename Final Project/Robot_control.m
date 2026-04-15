%% Clear workspace, command window, and close all figures
clear all
clc
close all

%% Declare global variables for robot pose and laser scan data
global pose scan tf
pose = [];
scan = [];
tf = [];

%% Set the ROS domain ID for communication
setenv('ROS_DOMAIN_ID', '30');

%% Display available ROS2 topics (for debug)
ros2 topic list

%% Create a ROS2 node for communication
controlNode = ros2node('/base_station');

%% Define subscribers
odomSub = ros2subscriber(controlNode, '/odom', @odomCallback); % odometry topic
tfSub = ros2subscriber(controlNode,'/tf',@tfCallback);
scanSub = ros2subscriber(controlNode, '/scan', @scanCallback, 'Reliability', 'besteffort'); % laser scan topic

% Pause to allow ROS subscriptions to initialize
pause(0.5);

try
    %% Define publishers
    cmdPub = ros2publisher(controlNode, '/cmd_vel', 'geometry_msgs/Twist');

    %% Create figure for TurtleBot's data
    visualise = TurtleBotVisualise();

    

    waypoint_idx = 1;
    waypoint_tolerance = 0.15;  % meters
   

    %% Optional live plots
    timeData = [];
    headingData = [];
    desiredHeadingData = [];
    distanceData = [];

    t0 = tic;

    %% ADDED: Occupancy map figure
    figMap = figure;
    axMap = axes(figMap);

    odomTrail = [];
    slamTrail = [];
    %% PID gains
    % Heading PID gains
    Kp_h = 0.6;
    Ki_h = 0.0;
    Kd_h = 0.2;

    % Distance PID gains
    Kp_d = 0.2;
    Ki_d = 0.0;
    Kd_d = 0.0;

    %% PID memory
    headingErrorInt = 0;
    headingErrorPrev = 0;

    distanceErrorInt = 0;
    distanceErrorPrev = 0;
    tPrev = tic;

    %% creating SLAM object

    cellsize = 0.05;
    res = 1 / cellsize;
    slamObj = lidarSLAM(res,8);

    %% Infinite loop for real-time visualization, until the figure is closed
    while true
        %% Wait until pose and scan have arrived
        if isempty(scan) || isempty(pose)
            pause(0.01)
            continue
        end

        Current_pos = [pose.position.x ... 
            pose.position.y];

        



        %% Read scan values
        ranges = double(scan.ranges); 
        angle_min = double(scan.angle_min);
        angle_increment = double(scan.angle_increment);

        dt = toc(tPrev);
        tPrev = tic;

        if dt <= 0
            dt = 0.01;
        end
 
        t = tf.transforms(1);

        tf_pos = [
            t.transform.translation.x
            t.transform.translation.y
        ];

        tf_rot = [
            t.transform.rotation.w
            t.transform.rotation.x
            t.transform.rotation.y
            t.transform.rotation.z
        ];
  
        %% setup slam scan.
        angles = angle_min + (0:length(ranges)-1) * angle_increment;

        ScanLidar = lidarScan(ranges, angles);

        %rotation 
        qw = tf_rot(1);
        qx = tf_rot(2);
        qy = tf_rot(3);
        qz = tf_rot(4);

        tf_angle = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy^2 + qz^2));

        odom_pos = [tf_pos(1),tf_pos(2)];
        relPoseEst = [tf_pos(1),tf_pos(2),tf_angle];

        [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamObj, ScanLidar, relPoseEst);

        [scans,poses] = scansAndPoses(slamObj);

        occMap = buildMap(scans, poses, res, 8);
        %% ADDED: Build occupancy map every 2 scans
        if mod(numel(scans),2) == 0
            show(occMap,'Parent',axMap);
            title(axMap,'Occupancy Map');
        end

        xLimits = occMap.XWorldLimits;
        yLimits = occMap.YWorldLimits;
        
        valid = false;
        
        while ~valid
            end_point = [
                rand() * diff(xLimits) + xLimits(1), ...
                rand() * diff(yLimits) + yLimits(1)
            ];
        
            occ = getOccupancy(occMap, end_point);
        
            valid = ~isnan(occ) && occ < 0.5;
        end

        startOcc = getOccupancy(occMap, Current_pos);

        if isnan(startOcc) || startOcc > 0.5
            continue;
        end
                
        path = createPath(Current_pos,end_point,occMap)

        
        %% Current waypoint from path
        position_desired = path(1, :);

        %% Visualise desired position
        % visualise = updatePositionDesired(visualise, position_desired);

        slam_pos = [poses(1), poses(2)];
        %heading = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy^2 + qz^2));
        slam_angle = poses(3);
        visualise = updatePose(visualise, slam_pos, slam_angle);


        %% Process and plot laser scan data
        cart = rosReadCartesian(scan);  % Convert scan to Cartesian coordinates
        cart = cart * [cos(slam_angle), -sin(slam_angle); sin(slam_angle), cos(slam_angle)]' + slam_pos;
        
        visualise = updateScan(visualise, cart);

        %% Desired heading and distance to current waypoint
        delta = position_desired - slam_pos;
        desiredHeading = atan2(delta(2), delta(1));
        distanceToTarget = norm(delta);

        %% Live plots
        t = toc(t0);

        timeData(end+1) = t;
        headingData(end+1) = slam_angle;
        desiredHeadingData(end+1) = desiredHeading;
        distanceData(end+1) = distanceToTarget;

        drawnow limitrate;

        %% PID controller for heading
       

        headingError = atan2(sin(desiredHeading - slam_angle), cos(desiredHeading - slam_angle));
        headingErrorInt = headingErrorInt + headingError * dt;
        headingErrorDer = (headingError - headingErrorPrev) / dt;

        angularVelocity = Kp_h * headingError ...
                        + Ki_h * headingErrorInt ...
                        + Kd_h * headingErrorDer;

        headingErrorPrev = headingError;

        %% PID controller for position
        distanceError = distanceToTarget;
        distanceErrorInt = distanceErrorInt + distanceError * dt;
        distanceErrorDer = (distanceError - distanceErrorPrev) / dt;

        linearVelocity = Kp_d * distanceError ...
                       + Ki_d * distanceErrorInt ...
                       + Kd_d * distanceErrorDer;

        distanceErrorPrev = distanceError;

        %% Reduce forward motion if heading is too far off
        if abs(headingError) > 0.2
            linearVelocity = linearVelocity * max(0, cos(headingError));
        end

        %% Reduce speed near waypoint for smoother turns
        if distanceToTarget < 0.05
            linearVelocity = 0.5 * linearVelocity;
        end

        %% Move to next waypoint or stop at final goal
        if distanceToTarget < waypoint_tolerance
            xLimits = occMap.XWorldLimits;
            yLimits = occMap.YWorldLimits;
            
            valid = false;
            
            while ~valid
                end_point = [
                    rand() * diff(xLimits) + xLimits(1), ...
                    rand() * diff(yLimits) + yLimits(1)
                ];
            
                occ = getOccupancy(occMap, end_point);
            
                valid = ~isnan(occ) && occ < 0.3;
            end
        end

            % linearVelocity = 0.0;
            % angularVelocity = 0.0;

        %% MOVE
        cmdMsg = ros2message('geometry_msgs/Twist');
        cmdMsg.linear.x = clip(linearVelocity, -0.2, 0.2);
        cmdMsg.angular.z = clip(angularVelocity, -2.0, 2.0);
        send(cmdPub, cmdMsg);

        %% Pause to visualize and delete old plots
        pause(0.01)

        %% Exit the loop if the figure is closed
        if size(findobj(visualise.fig)) == 0
            ME = MException('NonExeption:EndProgram', 'The program was closed.');
            throw(ME)
        end
    end
catch ME

    % Stop the robot
    cmdMsg = ros2message('geometry_msgs/Twist');
    cmdMsg.Linear.X = 0;
    cmdMsg.Angular.Z = 0;
    send(cmdPub, cmdMsg);

    % Clean up ROS subscriptions
    clear odomSub scanSub

    % Show the error
    if ~strcmp(ME.identifier, 'NonExeption:EndProgram')
        rethrow(ME)
    end
end

%% Callback functions
function odomCallback(message)
    global pose
    pose = message.pose.pose;
end


function scanCallback(message)
    global scan
    scan = message;
end

function tfCallback(message)
    global tf
    tf = message;
end