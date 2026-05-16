clear all
clc
close all

% run this from the Final Project folder
addpath('pipeline','teleop','shared');

setenv('ROS_DOMAIN_ID', '30');
ros2 topic list

controlNode = ros2node('/base_station');

% retry until ROS2 discovery finds the topics
while true
    try
        scanSub = ros2subscriber(controlNode, '/scan', {}, 'Reliability', 'besteffort');
        odomSub = ros2subscriber(controlNode, '/odom', 'nav_msgs/Odometry', 'Reliability', 'besteffort');
        break
    catch
    end
end

try
    cmdPub = ros2publisher(controlNode, '/cmd_vel', 'geometry_msgs/Twist');

    load('explored_map.mat', 'occ_matrix', 'resolution', 'grid_location', 'slamAlg');
    map = occupancyMap(occ_matrix, resolution);
    map.GridLocationInWorld = grid_location;
    initialPose = localize_robot(map, scanSub, odomSub, cmdPub);

    points = plot_and_point(slamAlg);

    % trajectory and circle state persist across legs
    odom_trajectory = struct('scans', {{}}, 'poses', zeros(0,3));
    circle_state = struct('orange', false, 'blue', false);

    % A -> B1
    [final_pose_B1, odom_trajectory, circle_state] = move_to_point( ...
        scanSub,odomSub,cmdPub,points(1,:),slamAlg,initialPose,odom_trajectory,circle_state,false);

    % relocalize, then re-approach B1 before the scan leg
    final_pose_B1 = localize_robot(map, scanSub, odomSub, cmdPub, final_pose_B1);
    [final_pose_B1, odom_trajectory, circle_state] = move_to_point( ...
        scanSub,odomSub,cmdPub,points(1,:),slamAlg,final_pose_B1,odom_trajectory,circle_state,false);

    % B1 -> B2, scanning for circles on this leg
    [final_pose_B2, odom_trajectory, circle_state] = move_to_point( ...
        scanSub,odomSub,cmdPub,points(2,:),slamAlg,final_pose_B1,odom_trajectory,circle_state,true);

    % relocalize before B2->C
    final_pose_B2 = localize_robot(map, scanSub, odomSub, cmdPub, final_pose_B2);

    % B2 -> C
    [~, odom_trajectory, circle_state] = move_to_point( ...
        scanSub,odomSub,cmdPub,points(3,:),slamAlg,final_pose_B2,odom_trajectory,circle_state,false);
catch ME
    stop_robot(cmdPub);
    clear scanSub
    rethrow(ME)
end
