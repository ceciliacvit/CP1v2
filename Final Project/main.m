%% Clear workspace, command window, and close all figures
clear all
clc
close all

%% Declare global variables for robot pose and laser scan data


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
    break

    catch
    end

end

fig2 = figure;
ax2 = axes(fig2);

fig3 = figure;
ax3 = axes(fig3);


try
    %% Define publishers
    cmdPub = ros2publisher(controlNode, '/cmd_vel', 'geometry_msgs/Twist');

    %% explote
    slamAlg = explore(scanSub,cmdPub);

    points = plot_and_box(slamAlg)

    %% move to B
    move_to_box(scanSub,cmdPub,points(1:4,:),slamAlg);

    %% find circles
    find_circles(scanSub,cmdPub);

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
