%% Clear workspace, command window, and close all figures
clear all
clc
close all

%% Declare global variables for robot pose and laser scan data
global pose scan

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
    
    %% Initialize array for desired positions
    %%position_desired = [-1, 1];

    %%Week 3 exercise 1
    %%position_desired = [0 1]; %% venstre
    %%position_desired = [0 -1]; %% højre

    %%Week 3 exercise 2
    %% Circle trajectory parameters
    c = [-0.5, 0];
    r = 0.5;
    w = 0.2;

    %% Exercise 5
    timeData = [];
    headingData = [];
    desiredHeadingData = [];
    distanceData = [];
    
    t0 = tic;
    
    fig2 = figure;
    ax2 = axes(fig2);
    
    fig3 = figure;
    ax3 = axes(fig3);

    %% Exercise 6
    % Heading PID gains
    Kp_h = 0.9;
    Ki_h = 0.02;
    Kd_h = 0.2;

    % Distance PID gains
    Kp_d = 0.2;
    Ki_d = 0.0;
    Kd_d = 0.0;
    
    % PID memory
    headingErrorInt = 0;
    headingErrorPrev = 0;
    
    distanceErrorInt = 0;
    distanceErrorPrev = 0;
    
    tPrev = tic;
        
    %% Infinite loop for real-time visualization, until the figure is closed
    while true

        %% Exercise 2.2 kør i ring
        t = toc(t0);
        position_desired = [r*cos(w*t) + c(1), r*sin(w*t) + c(2)];

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

        %% Exercise 5
        delta = position_desired - position;
        desiredHeading = atan2(delta(2), delta(1));
        distanceToTarget = norm(delta);
        
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
        ylabel(ax3, 'Distance to target [m]');
        legend(ax3, 'Distance to target');
        grid(ax3, 'on');
        
        drawnow limitrate;

        %% Process and plot laser scan data
        cart = rosReadCartesian(scan);  % Convert scan to Cartesian coordinates
        cart = cart * [cos(heading), -sin(heading); sin(heading), cos(heading)]' + position; % Transform based on robot position and heading
        visualise = updateScan(visualise, cart);

        %% PID controller for heading
        dt = toc(tPrev);
        tPrev = tic;

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

        %% Stop if heading is too far off
        if abs(headingError) > 0.4
            linearVelocity = linearVelocity * max(0, cos(headingError));
        end

        distanceErrorPrev = distanceError;

        %% Stop at target
        %%if distanceToTarget < 0.05
        %%    linearVelocity = 0.0;
        %%    angularVelocity = 0.0;
        %%end
            
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