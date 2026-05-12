%% Clear workspace, command window, and close all figures
clear all
clc
close all

%% Declare global variables for     robot pose and laser scan data


%% Set the ROS domain ID for communication
setenv('ROS_DOMAIN_ID', '30');

%% Display available ROS2 topics (for debug)
ros2 topic list

%% Create a ROS2 node for communication
controlNode = ros2node('/base_station');

%% Define subscribers
while true
    try
    scanSub = ros2subscriber(controlNode, '/scan', {}, 'Reliability', 'besteffort'); % laser scan topic
    odomSub = ros2subscriber(controlNode, '/odom', 'nav_msgs/Odometry', 'Reliability', 'besteffort');
    break

    catch
    end

end



try
    %% Define publishers
    cmdPub = ros2publisher(controlNode, '/cmd_vel', 'geometry_msgs/Twist');

    if isfile('explored_map.mat')
        load('explored_map.mat', 'occ_matrix', 'resolution', 'grid_location', 'slamAlg');
        map = occupancyMap(occ_matrix, resolution);
        map.GridLocationInWorld = grid_location;
        initialPose = localize_robot(map, scanSub, odomSub, cmdPub);
    else
        slamAlg = explore(scanSub,cmdPub,odomSub);
        initialPose = [];
    end

    points = plot_and_point(slamAlg)

    % Accumulated MCL trajectory across legs (empty in SLAM mode).
    mcl_history = struct('scans', {{}}, 'poses', zeros(0,3));
    % Circle-detection state persists across legs so we don't re-search.
    circle_state = struct('orange', false, 'blue', false);

    %% move to B1 (no circle scanning)
    [final_pose_B1, mcl_history, circle_state] = move_to_point( ...
        scanSub,odomSub,cmdPub,points(1,:),slamAlg,initialPose,mcl_history,circle_state,false);

    %% Re-localize at B1 (MCL mode only). A->B1 was pure dead-reckoning, so
    % correct accumulated drift, then drive from the true pose to actual B1
    % before starting the scan leg.
    if isfile('explored_map.mat')
        final_pose_B1 = localize_robot(map, scanSub, odomSub, cmdPub, final_pose_B1);
        [final_pose_B1, mcl_history, circle_state] = move_to_point( ...
            scanSub,odomSub,cmdPub,points(1,:),slamAlg,final_pose_B1,mcl_history,circle_state,false);
    end

    %% move to B2 (circle scanning enabled on this leg only)
    [final_pose_B2, mcl_history, circle_state] = move_to_point( ...
        scanSub,odomSub,cmdPub,points(2,:),slamAlg,final_pose_B1,mcl_history,circle_state,true);

    %% move to C (no circle scanning)
    [~, mcl_history, circle_state] = move_to_point( ...
        scanSub,odomSub,cmdPub,points(3,:),slamAlg,final_pose_B2,mcl_history,circle_state,false);
catch ME
    %% Catching potential errors
    % Stop the robot
    cmdMsg = ros2message('geometry_msgs/Twist');
    cmdMsg.Linear.X = 0;
    cmdMsg.Angular.Z = 0;
    send(cmdPub, cmdMsg);

    % Clean up ROS subscriptions
    clear scanSub

    % Show the error
    if ~strcmp(ME.identifier, 'NonExeption:EndProgram')
        rethrow(ME)
    end
end