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

    %% Desired position
    position_desired = [0, 2];

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

    %% Obstacle avoidance parameters
    d0 = 0.5;
    beta = 0.01;

    % Front rays around 0 rad
    front_indices = [1:10, 230:239];

    % Back rays around pi rad
    back_indices = 111:130;

    % Use both front and back as requested
    selected_indices = [front_indices, back_indices];

    %% Infinite loop for real-time visualization, until the figure is closed
    while true
        %% Wait until pose and scan have arrived
        if isempty(scan) || isempty(pose)
            pause(0.01)
            continue
        end

        %% Read scan values
        ranges = double(scan.ranges);
        angle_min = double(scan.angle_min);
        angle_increment = double(scan.angle_increment);

        %% Visialise desired position
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

        %% Detect collision
        collision = false;
        if collision
            ME = MException('NonExeption:CollisionDetected', 'Collision detected.');
            throw(ME)
        end

        %% Desired heading and distance to target
        delta = position_desired - position;
        desiredHeading = atan2(delta(2), delta(1));
        distanceToTarget = norm(delta);

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

        %% Repulsive forces from selected rays
        Fx_rays = [];
        Fy_rays = [];

        for k = 1:length(selected_indices)
            i = selected_indices(k);

            % Safety in case number of rays differs
            if i < 1 || i > length(ranges)
                continue
            end

            d = ranges(i);

            if ~isnan(d) && isfinite(d) && d > 0 && d < d0
                theta = angle_min + (i-1) * angle_increment;

                % Repulsive force magnitude
                F = (2 * beta / d^2) * (1/d - 1/d0);

                % Repulsive force vector components
                Fx = -F * cos(theta);
                Fy = -F * sin(theta);

                Fx_rays(end+1) = Fx;
                Fy_rays(end+1) = Fy;
            end
        end

        %% Total repulsive force
        repulsiveX = sum(Fx_rays);
        repulsiveY = sum(Fy_rays);

        %% Obstacle Avoidance
        linearVelocity = linearVelocity + repulsiveX;
        angularVelocity = angularVelocity + repulsiveY;

        %% Obstacle avoidance but do not move backwards
        linearVelocity = max(0, linearVelocity);
        %%angularVelocity = max(0, angularVelocity);

        %% Stop at target
        if distanceToTarget < 0.05
            linearVelocity = 0.0;
            angularVelocity = 0.0;
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