%% Passive SLAM map builder — run this alongside an external teleop node.
%
% Driving is handled by, e.g. (in a separate terminal):
%   ros2 run turtlebot3_teleop teleop_keyboard
%
% This script only listens to /scan and /odom, builds the map, and saves
% explored_map.mat when you press Q on the figure window.

clear all
clc
close all

addpath('pipeline','teleop','shared');

setenv('ROS_DOMAIN_ID', '30');
ros2 topic list

controlNode = ros2node('/base_station');

while true
    try
        scanSub = ros2subscriber(controlNode, '/scan', {}, 'Reliability', 'besteffort');
        odomSub = ros2subscriber(controlNode, '/odom', 'nav_msgs/Odometry', 'Reliability', 'besteffort');
        imuSub  = ros2subscriber(controlNode, '/imu',  'sensor_msgs/Imu',  'Reliability', 'besteffort');
        break
    catch
    end
end

% Resume from previous run if a saved map exists, else start fresh
if isfile('explored_map.mat')
    load('explored_map.mat', 'slamAlg');
else
    slamAlg = [];
end

slamAlg = teleop_map(scanSub, odomSub, imuSub, slamAlg);
