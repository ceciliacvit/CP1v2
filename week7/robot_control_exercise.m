%% Clear workspace, command window, and close all figures
clear all
clc
close all

%% Declare global variables for robot pose and laser scan data
global pose scan imu tf
pose = [];
scan = [];
imu = [];
tf = [];

%% Set the ROS domain ID for communication
setenv('ROS_DOMAIN_ID', '30');

%% Display available ROS2 topics (for debug)
ros2 topic list

%% Create a ROS2 node for communication
controlNode = ros2node('/base_station');

%% Define subscribers
odomSub = ros2subscriber(controlNode, '/odom', @odomCallback); % odometry topic
imuSub = ros2subscriber(controlNode,'/imu',@imuCallback);
tfSub = ros2subscriber(controlNode,'/tf',@tfCallback);
scanSub = ros2subscriber(controlNode, '/scan', @scanCallback, 'Reliability', 'besteffort'); % laser scan topic

% Pause to allow ROS subscriptions to initialize
pause(0.5);

try
    %% Define publishers
    cmdPub = ros2publisher(controlNode, '/cmd_vel', 'geometry_msgs/Twist');

    %% Create figure for TurtleBot's data
    visualise = TurtleBotVisualise();

    path = [pose.position.x ... 
            pose.position.y];
    %% PRM path waypoints
    % path = [
    %     0         0
    %     0.1786    1.2714
    %     0.7881    3.7563
    %     2.7344    1.7019
    %     2.5000    1.5000
    % ];

    %%path = [
    %%    0.1000    0.1000
    %%    0.1623    0.2899
    %%    1.1705    0.5538
    %%    1.5583    1.5904
    %%    1.5000    1.5000
    %%];

    waypoint_idx = 1;
    waypoint_tolerance = 0.15;  % meters
    position_desired = path(waypoint_idx, :);

    %% Optional live plots
    timeData = [];
    headingData = [];
    desiredHeadingData = [];
    distanceData = [];

    t0 = tic;
    
    % fig2 = figure;
    % ax2 = axes(fig2);
    % 
    % fig3 = figure;
    % ax3 = axes(fig3);

    fig4 = figure;
    ax4 = axes(fig4);
    
    fig5 = figure;
    ax5 = axes(fig5);

    fig6 = figure;
    ax6 = axes(fig6);

    %% ADDED: Occupancy map figure
    figMap = figure;
    axMap = axes(figMap);

    odomTrail = [];
    drTrail = [];
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

    dr_vel = [0 0 0];
    dr_pos = [0 0 0];

    tPrev = tic;

    %% creating SLAM object

    cellsize = 0.1;
    res = 1 / cellsize;
    slamObj = lidarSLAM(res,8);

    %% Infinite loop for real-time visualization, until the figure is closed
    while true
        %% Wait until pose and scan have arrived
        if isempty(scan) || isempty(pose) || isempty(imu)
            pause(0.01)
            continue
        end

        %% Current waypoint from path
        position_desired = path(waypoint_idx, :);

        %% Read scan values
        ranges = double(scan.ranges); %#ok<NASGU>
        angle_min = double(scan.angle_min); %#ok<NASGU>
        angle_increment = double(scan.angle_increment); %#ok<NASGU>

        linear_accel = imu.linear_acceleration;

        orient = imu.orientation;

        linear_accel.z = linear_accel.z - 9.8200;

        cutoff = 0.0;
        %cutoff to combat position drift
        if(abs(linear_accel.x) < cutoff)
            linear_accel.x = 0;
        end
        if(abs(linear_accel.y) < cutoff)
            linear_accel.y = 0;
        end
        if(abs(linear_accel.z) < cutoff)
            linear_accel.z = 0;
        end

        Orient = [
            orient.w ...
            orient.x ...
            orient.y ...
            orient.z
        ];
        Acc_body = [
            linear_accel.x ...
            linear_accel.y ...
            linear_accel.z 
        ];
        dt = toc(tPrev);
        tPrev = tic;

        if dt <= 0
            dt = 0.01;
        end

        Acc_world = quatrotate(Orient,Acc_body);

        dr_vel = dr_vel + Acc_world * dt;
        dr_vel = dr_vel * 0.98;
        dr_pos = dr_pos + dr_vel * dt
        
        qw = Orient(1);
        qx = Orient(2);
        qy = Orient(3);
        qz = Orient(4);


        dr_angle = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy^2 + qz^2));

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
        %visualise = updateDeadReckoning(visualise, dr_pos(1:2));


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

        %% ADDED: Build occupancy map every 10 scans
        if mod(numel(scans),10) == 0
            occMap = buildMap(scans, poses, res, 8);
            show(occMap,'Parent',axMap);
            title(axMap,'Occupancy Map');
        end

        %% Visualise desired position
        visualise = updatePositionDesired(visualise, position_desired);

        slam_pos = [poses(1), poses(2)];

        qx = pose.orientation.x;
        qy = pose.orientation.y;
        qz = pose.orientation.z;
        qw = pose.orientation.w;

        %heading = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy^2 + qz^2));
        slam_angle = poses(3);
        visualise = updatePose(visualise, slam_pos, slam_angle);


        %% Process and plot laser scan data
        cart = rosReadCartesian(scan);  % Convert scan to Cartesian coordinates
        cart = cart * [cos(slam_angle), -sin(slam_angle); sin(slam_angle), cos(slam_angle)]' + slam_pos;
        
        visualise = updateScan(visualise, cart);

        %% plot dead reckoning
        drTrail(end+1,:) = dr_pos(1:2);
        plot(ax4, drTrail(:,1), drTrail(:,2), 'r--', 'LineWidth', 1.5);
        xlabel(ax4, 'X [m]'); ylabel(ax4, 'Y [m]');
        legend(ax4, 'Dead Reckoning');
        grid(ax4, 'on');

        %% Compare odom (tf) with slam
        odomTrail(end+1,:) = odom_pos;
        slamTrail(end+1,:) = slam_pos;
        plot(ax5, odomTrail(:,1), odomTrail(:,2), 'b--', slamTrail(:,1), slamTrail(:,2), 'r--', 'LineWidth', 1.5);
        xlabel(ax5, 'X [m]'); ylabel(ax5, 'Y [m]');
        legend(ax5, 'Odometry', 'Slam');
        grid(ax5, 'on');

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

        % plot(ax2, timeData, headingData, 'b', timeData, desiredHeadingData, 'r--', 'LineWidth', 1.5);
        % xlabel(ax2, 'Time [s]');
        % ylabel(ax2, 'Heading [rad]');
        % legend(ax2, 'Actual heading', 'Desired heading');
        % grid(ax2, 'on');
        % 
        % plot(ax3, timeData, distanceData, 'k', 'LineWidth', 1.5);
        % xlabel(ax3, 'Time [s]');
        % ylabel(ax3, 'Distance to current waypoint [m]');
        % legend(ax3, 'Distance to waypoint');
        % grid(ax3, 'on');

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
            if waypoint_idx < 0
                waypoint_idx = waypoint_idx + 1;
                position_desired = path(waypoint_idx, :);
            else
                linearVelocity = 0.0;
                angularVelocity = 0.0;
            end
        end

        %% Publish velocity commands
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

function imuCallback(message)
    global imu
    imu = message;
end

function scanCallback(message)
    global scan
    scan = message;
end

function tfCallback(message)
    global tf
    tf = message;
end