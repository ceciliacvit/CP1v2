% Passive SLAM map builder. Drive the robot from a separate terminal with
%   ros2 run turtlebot3_teleop teleop_keyboard
% then press Q on the figure to save explored_map.mat.

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

% resume a previous map if one exists
if isfile('explored_map.mat')
    load('explored_map.mat', 'slamAlg');
else
    slamAlg = [];
end

slamAlg = teleop_map(scanSub, odomSub, imuSub, slamAlg);
