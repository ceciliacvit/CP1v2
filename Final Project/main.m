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

    points = plot_and_box(slamAlg)

    %% move to B
    move_to_box(scanSub,odomSub,cmdPub,points(1:4,:),slamAlg,initialPose);

    %% find circles
    find_circles(scanSub,cmdPub);

    %% move to C
    move_to_box(scanSub,odomSub,cmdPub,points(5:end,:),slamAlg,initialPose);
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