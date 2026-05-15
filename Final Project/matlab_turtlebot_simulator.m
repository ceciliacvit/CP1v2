% matlab_turtlebot_simulator.m
% A MATLAB-based mock ROS 2 Simulator for the TurtleBot
% 
% Instructions: 
% 1. Open a SECOND instance of MATLAB.
% 2. Run this script in the second instance.
% 3. Run your main.m in your first MATLAB instance.
%
% This simulator loads your map, listens for /cmd_vel commands, 
% and publishes /odom, /scan, and /camera/image_raw/compressed.

clear; clc; close all;

%% 1. Configuration & Setup
setenv('ROS_DOMAIN_ID', '30'); % Match the domain ID of the base station
simNode = ros2node('/turtlebot_simulator');

% Load the environment map
load('explored_map.mat', 'occ_matrix', 'resolution', 'grid_location');
map = occupancyMap(occ_matrix, resolution);
map.GridLocationInWorld = grid_location;

disp('Starting MATLAB ROS 2 TurtleBot Simulator...');

%% 2. Publishers and Subscribers
% Publishers
odomPub = ros2publisher(simNode, '/odom', 'nav_msgs/Odometry');
scanPub = ros2publisher(simNode, '/scan', 'sensor_msgs/LaserScan');
camPub  = ros2publisher(simNode, '/camera/image_raw/compressed', 'sensor_msgs/CompressedImage');

% Subscriber for commands
cmdSub = ros2subscriber(simNode, '/cmd_vel', 'geometry_msgs/Twist', 'Reliability', 'besteffort');

%% 3. Pre-allocate messages
odomMsg = ros2message(odomPub);
scanMsg = ros2message(scanPub);
imgMsg  = ros2message(camPub);

%% 4. Initialize Robot State
% We will start the robot at 0,0
x = 0.0; 
y = 0.0;
theta = 0;
v = 0;
omega = 0;

% LiDAR Setup
lidar = rangeSensor('RangeNoise', 0.01, 'Range', [0.1, 8], 'HorizontalAngleResolution', 2*pi/179);

%% 5. Visualization Setup
fig = figure('Name', 'MATLAB ROS 2 Simulator');
ax = axes(fig);
show(map, 'Parent', ax);
hold(ax, 'on');

robotPlot = plot(ax, x, y, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
headingPlot = quiver(ax, x, y, 0.5*cos(theta), 0.5*sin(theta), 'r', 'LineWidth', 2, 'MaxHeadSize', 2);
title(ax, 'Live Simulator View (Keep this window open)');

%% 6. Dummy Camera Image Generation
% Create a dummy image with blue and orange circles so detectCircle won't hang.
% Radius of 63.3 pixels gives a simulated distance of ~1.0 meter 
% (based on: distance = 1266 * 0.1 / (2 * r))
dummy_img = zeros(480, 640, 3, 'uint8');
[X, Y] = meshgrid(1:640, 1:480);

% Blue circle at x=240, y=240
blueMask = (X - 240).^2 + (Y - 240).^2 <= 63.3^2;
% Orange circle at x=400, y=240
orangeMask = (X - 400).^2 + (Y - 240).^2 <= 63.3^2;

% Extract channels
rChannel = dummy_img(:,:,1);
gChannel = dummy_img(:,:,2);
bChannel = dummy_img(:,:,3);

% Set blue color for blue circle (RGB: 0, 150, 255)
bChannel(blueMask) = 255;
gChannel(blueMask) = 150; 

% Set orange color for orange circle (RGB: 255, 150, 0)
rChannel(orangeMask) = 255;
gChannel(orangeMask) = 150;

% Re-assign channels
dummy_img(:,:,1) = rChannel;
dummy_img(:,:,2) = gChannel;
dummy_img(:,:,3) = bChannel;

fid = fopen('sim_cam_dummy.jpg', 'w');
imwrite(dummy_img, 'sim_cam_dummy.jpg');
fclose(fid);

fid = fopen('sim_cam_dummy.jpg', 'r');
imgData = fread(fid, inf, '*uint8');
fclose(fid);
imgMsg.format = 'jpeg';
imgMsg.data = imgData;

%% 7. Simulation Loop
rate = rateControl(20); % 20 Hz loop
disp('Simulator is running. Press Ctrl+C to stop.');

while true
    % Read commanded velocity
    cmd = cmdSub.LatestMessage;
    if ~isempty(cmd)
        v = cmd.linear.x;
        omega = cmd.angular.z;
    end
    
    % Update Kinematics (Unicycle Model)
    dt = rate.DesiredPeriod;
    x = x + v * cos(theta) * dt;
    y = y + v * sin(theta) * dt;
    theta = theta + omega * dt;
    theta = atan2(sin(theta), cos(theta)); % Normalize angle
    
    % Prepare and Publish Odometry
    odomMsg.pose.pose.position.x = x;
    odomMsg.pose.pose.position.y = y;
    
    % Simple Euler to Quaternion mapping [yaw, pitch, roll] = [theta, 0, 0]
    cy = cos(theta * 0.5);
    sy = sin(theta * 0.5);
    odomMsg.pose.pose.orientation.w = cy;
    odomMsg.pose.pose.orientation.z = sy;
    odomMsg.pose.pose.orientation.x = 0;
    odomMsg.pose.pose.orientation.y = 0;
    
    send(odomPub, odomMsg);
    
    % Prepare and Publish Laser Scan
    pose = [x, y, theta];
    [ranges, angles] = lidar(pose, map);
    
    scanMsg.header.frame_id = 'base_scan';
    scanMsg.angle_min = single(min(angles));
    scanMsg.angle_max = single(max(angles));
    scanMsg.angle_increment = single(angles(2) - angles(1));
    scanMsg.range_min = single(0.1);
    scanMsg.range_max = single(8.0);
    scanMsg.ranges = single(ranges);
    
    send(scanPub, scanMsg);
    
    % Publish Dummy Image continuously so subscribers get fresh stamps
    send(camPub, imgMsg);
    
    % Update Visualization
    if isvalid(robotPlot) && isvalid(headingPlot)
        set(robotPlot, 'XData', x, 'YData', y);
        set(headingPlot, 'XData', x, 'YData', y, 'UData', 0.5*cos(theta), 'VData', 0.5*sin(theta));
        drawnow limitrate;
    else
        % If user closes the figure, stop simulation
        disp('Simulator window closed. Stopping.');
        break;
    end
    
    waitfor(rate);
end
