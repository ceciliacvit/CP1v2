%% Wall Following — maintains 1m from wall on robot's LEFT side
clear all
clc
close all

%% Globals
global pose scan
pose = [];
scan = [];

%% ROS2 setup
setenv('ROS_DOMAIN_ID', '30');
controlNode = ros2node('/base_station');

odomSub = ros2subscriber(controlNode, '/odom', @odomCallback);
scanSub = ros2subscriber(controlNode, '/scan', @scanCallback, 'Reliability', 'besteffort');

pause(0.5);

try
    cmdPub = ros2publisher(controlNode, '/cmd_vel', 'geometry_msgs/Twist');

    %% Controller parameters
    TARGET_DIST  = 1.0;   % desired distance from wall (m)
    LINEAR_SPEED = 0.15;  % forward speed (m/s)
    Kp_dist      = 0.6;   % gain on distance error
    Kp_align     = 0.8;   % gain on wall alignment error

    %% Main loop
    while true
        if isempty(scan) || isempty(pose)
            pause(0.01);
            continue
        end

        angles = double(scan.angle_min) + ...
                 (0:numel(scan.ranges)-1) * double(scan.angle_increment);
        ranges = double(scan.ranges(:))';

        %% Sample two rays on the left side
        [~, i90] = min(abs(angles - deg2rad(90)));
        [~, i60] = min(abs(angles - deg2rad(60)));

        d90 = ranges(i90);
        d60 = ranges(i60);

        %% Wall points in robot Cartesian frame
        p90 = [d90 * cos(deg2rad(90)); d90 * sin(deg2rad(90))];
        p60 = [d60 * cos(deg2rad(60)); d60 * sin(deg2rad(60))];

        %% Errors
        dist_error  = d90 - TARGET_DIST;

        wall_vec    = p90 - p60;
        align_error = atan2(wall_vec(2), wall_vec(1)) - deg2rad(90);

        %% Control
        angular_vel = Kp_dist * dist_error + Kp_align * align_error;

        cmdMsg = ros2message('geometry_msgs/Twist');
        cmdMsg.linear.x  = clip(LINEAR_SPEED,  -0.2, 0.2);
        cmdMsg.angular.z = clip(angular_vel,   -2.0, 2.0);
        send(cmdPub, cmdMsg);

        pause(0.01);
    end

catch ME
    %% Stop the robot on exit
    cmdMsg = ros2message('geometry_msgs/Twist');
    cmdMsg.linear.x  = 0;
    cmdMsg.angular.z = 0;
    send(cmdPub, cmdMsg);

    clear odomSub scanSub

    if ~strcmp(ME.identifier, 'NonExeption:EndProgram')
        rethrow(ME)
    end
end

%% Callbacks
function odomCallback(message)
    global pose
    pose = message.pose.pose;
end

function scanCallback(message)
    global scan
    scan = message;
end
