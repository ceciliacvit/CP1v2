%% Clear workspace, command window, and close all figures
clear all
clc
close all

%% Declare global variables for robot pose and laser scan data
global pose poseOffset scan image

%% Set the ROS domain ID for communication
setenv('ROS_DOMAIN_ID', '30');

%% Display available ROS2 topics (for debug)
ros2 topic list

%% Create a ROS2 node for communication
controlNode = ros2node('/base_station');

%% Define subscribers
odomSub = ros2subscriber(controlNode, '/odom', @odomCallback); % odometry topic
scanSub = ros2subscriber(controlNode, '/scan', @scanCallback, 'Reliability', 'besteffort'); % laser scan topic
imageSub = ros2subscriber(controlNode, '/camera/image_raw/compressed', @imageCallback); % image topic

% Pause to allow ROS subscriptions to initialize
pause(0.5);
    
try
    %% Define publishers
    cmdPub = ros2publisher(controlNode, '/cmd_vel', 'geometry_msgs/Twist');
    
    %% Create figure for TurtleBot's data
    visualise = TurtleBotVisualise();
    
    %% Initialize array for desired positions
    positionDesired = [1; 1];

    %% Calculate offset
    quatOffset = [poseOffset.orientation.x poseOffset.orientation.y poseOffset.orientation.z poseOffset.orientation.w];
    orientationOffset = quat2eul(quatOffset);  % Convert offset quaternion to Euler angles
    headingOffset = orientationOffset(3); % Extract offset heading (yaw)

    %% Calculate transformations for offset
    positionOffset = [poseOffset.position.x; poseOffset.position.y];
    R_W2R = [cos(-headingOffset), -sin(-headingOffset); sin(-headingOffset), cos(-headingOffset)];
    t_R2V = -R_W2R * positionOffset;
    R_R2V = [cos(headingOffset), -sin(headingOffset); sin(headingOffset), cos(headingOffset)]';
    
    %% Infinite loop for real-time visualization, until the figure is closed
    while true
        %% Visialise desired position
        visualise = updatePositionDesired(visualise, positionDesired);

        %% Get the robot's current position and heading
        position = [pose.position.x; pose.position.y];
        quat = [pose.orientation.x pose.orientation.y pose.orientation.z pose.orientation.w];
        orientation = quat2eul(quat);  % Convert quaternion to Euler angles
        heading = orientation(3); % Extract heading (yaw)

        %% Apply offset
        position = R_R2V * position + t_R2V;
        heading = heading - headingOffset; % Offset heading

        %% Visualise the robot
        visualise = updatePose(visualise, position, heading);
    
        %% Process and plot laser scan data
        cart = rosReadCartesian(scan);  % Convert scan to Cartesian coordinates
        cart = cart * [cos(heading), -sin(heading); sin(heading), cos(heading)]' + position'; % Transform based on robot position and heading
        visualise = updateScan(visualise, cart);

        %% Visualise image
        visualise = updateImage(visualise, image);

        %% PID controller for heading
        angularVelocity = 0.0;

        %% PID controller for position
        linearVelocity = 0.0;
    
        %% Publish velocity commands
        cmdMsg = ros2message('geometry_msgs/Twist');
        cmdMsg.linear.x = clip(linearVelocity, -0.1, 0.1);
        cmdMsg.angular.z = clip(angularVelocity, -1.0, 1.0);
        % send(cmdPub, cmdMsg);
    
        %% Pause to visualize and delete old plots
        pause(0.1)
    
        %% Exit the loop if the figure is closed
        if size(findobj(visualise.figAvatar)) == 0 | size(findobj(visualise.figImage)) == 0
            ME = MException('NonExeption:EndProgram', 'The program was closed.');
            throw(ME)
        end
    end
catch ME
    % Stop the robot
    cmdMsg = ros2message('geometry_msgs/Twist');
    cmdMsg.Linear.x = 0;
    cmdMsg.Angular.z = 0;
    send(cmdPub, cmdMsg);

    % Close all figures
    close all
    
    % Clean up ROS subscriptions
    clear odomSub scanSub imageSub

    % Show the error
    if ~strcmp(ME.identifier, 'NonExeption:EndProgram')
        rethrow(ME)
    end
end 

% %% Callback functions
function odomCallback(message)
    % Use global variable to store the robot's position and orientation
    global pose poseOffset

    % Extract position and orientation data from the ROS message
    pose = message.pose.pose;

    if isempty(poseOffset)
        poseOffset = message.pose.pose;
    end
end

function scanCallback(message)
    % Use global variable to store laser scan data
    global scan

    % Save the laser scan message
    scan = message;
end

function imageCallback(message)
    % Use global variable to store laser scan data
    global image

    % Save the laser scan message
    image = rosReadImage(message);
end