%% Clear workspace, command window, and close all figures
clear all
clc
close all

%% Declare global variables for robot pose and laser scan data
global pose scan
pose = [];
scan = [];

%% Set the ROS domain ID for communication
setenv('ROS_DOMAIN_ID', '30');

%% Display available ROS2 topics (for debug)
ros2 topic list

%% Create a ROS2 node for communication
controlNode = ros2node('/base_station');

%% Define subscribers
odomSub = ros2subscriber(controlNode, '/odom', @odomCallback); % odometry topic
scanSub = ros2subscriber(controlNode, '/scan', @scanCallback, 'Reliability', 'besteffort'); % laser scan topic

% Pause to allow ROS subscriptions to initialize
pause(0.5);

try
    %% Define publishers
    cmdPub = ros2publisher(controlNode, '/cmd_vel', 'geometry_msgs/Twist');

    %% Create figure for TurtleBot's data
    visualise = TurtleBotVisualise();

    %% PRM path waypoints
    path = [
        0.1000    0.1000
        0.2899    0.1623
        0.5538    1.1705
        1.5904    1.5583
        1.5000    1.5000
    ];

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

    fig2 = figure;
    ax2 = axes(fig2);

    fig3 = figure;
    ax3 = axes(fig3);

    %% PID gains
    % Heading PID gains
    Kp_h = 0.9;
    Ki_h = 0.02;
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

    %% Infinite loop for real-time visualization, until the figure is closed
    while true
        %% Wait until pose and scan have arrived
        if isempty(scan) || isempty(pose)
            pause(0.01)
            continue
        end

        %% Current waypoint from path
        position_desired = path(waypoint_idx, :);

        %% Read scan values
        ranges = double(scan.ranges); %#ok<NASGU>
        angle_min = double(scan.angle_min); %#ok<NASGU>
        angle_increment = double(scan.angle_increment); %#ok<NASGU>

        %% Visualise desired position
        visualise = updatePositionDesired(visualise, position_desired);

        %% Get the robot's current position and heading
        position = [pose.position.x pose.position.y];

        qx = pose.orientation.x;
        qy = pose.orientation.y;
        qz = pose.orientation.z;
        qw = pose.orientation.w;

        heading = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy^2 + qz^2));
        visualise = updatePose(visualise, position, heading);

        %% Process and plot laser scan data
        cart = rosReadCartesian(scan);  % Convert scan to Cartesian coordinates
        cart = cart * [cos(heading), -sin(heading); sin(heading), cos(heading)]' + position;
        visualise = updateScan(visualise, cart);

        %% Desired heading and distance to current waypoint
        delta = position_desired - position;
        desiredHeading = atan2(delta(2), delta(1));
        distanceToTarget = norm(delta);

        %% Live plots
        t = toc(t0);

        timeData(end+1) = t;
        headingData(end+1) = heading;
        desiredHeadingData(end+1) = desiredHeading;
        distanceData(end+1) = distanceToTarget;

        plot(ax2, timeData, headingData, 'b', timeData, desiredHeadingData, 'r--', 'LineWidth', 1.5);
        xlabel(ax2, 'Time [s]');
        ylabel(ax2, 'Heading [rad]');
        legend(ax2, 'Actual heading', 'Desired heading');
        grid(ax2, 'on');

        plot(ax3, timeData, distanceData, 'k', 'LineWidth', 1.5);
        xlabel(ax3, 'Time [s]');
        ylabel(ax3, 'Distance to current waypoint [m]');
        legend(ax3, 'Distance to waypoint');
        grid(ax3, 'on');

        drawnow limitrate;

        %% PID controller for heading
        dt = toc(tPrev);
        tPrev = tic;

        if dt <= 0
            dt = 0.01;
        end

        headingError = atan2(sin(desiredHeading - heading), cos(desiredHeading - heading));
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
        if abs(headingError) > 0.4
            linearVelocity = linearVelocity * max(0, cos(headingError));
        end

        %% Reduce speed near waypoint for smoother turns
        if distanceToTarget < 0.15
            linearVelocity = 0.5 * linearVelocity;
        end

        %% Move to next waypoint or stop at final goal
        if distanceToTarget < waypoint_tolerance
            if waypoint_idx < size(path, 1)
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
    % Use global variable to store the robot's position and orientation
    global pose

    % Extract position and orientation data from the ROS message
    pose = message.pose.pose;
end

function scanCallback(message)
    % Use global variable to store laser scan data
    global scan

    % Save the laser scan message
    scan = message;
end