function [updated_slam, fail] = MoveRobot(path,slamAlg,scanSub,odomSub,cmdPub,figure1,percentage_threshold,tolerance)
    arguments
        path;
        slamAlg;
        scanSub;
        odomSub;
        cmdPub;
        figure1;
        percentage_threshold = 1;
        tolerance = 0.20;
    end

%% for finding position
maxLidarRange = 8;
mapResolution = 20;

%% for navigation
lookahead_dist = 0.3;      % pure pursuit lookahead distance [m]
map_rebuild_interval = 5;  % rebuild occupancy map every N scans

time_previous = tic;
fail = false;

path_index = 1;
if isempty(path)
    updated_slam = slamAlg;
    return
end
total_waypoints = size(path, 1);
max_index = round(total_waypoints * percentage_threshold);

%% For plots
t0 = tic;

global timeData headingData desiredHeadingData distanceData ax2 ax3

timeData = [];
headingData = [];
desiredHeadingData = [];
distanceData = [];

figure;
ax2 = subplot(2,1,1);
ax3 = subplot(2,1,2);

hold(ax2, 'on');
hold(ax3, 'on');

%% PID initialization

headingErrorInt = 0;
headingErrorPrev = 0;

distanceErrorInt = 0;
distanceErrorPrev = 0;

% Heading PID gains
Kp_h = 0.8;
Ki_h = 0.2;
Kd_h = 0.2;

% Distance PID gains
Kp_d = 0.2;
Ki_d = 0.0;
Kd_d = 0.0;

drive = true;

% Build initial map before entering the loop
scan = receive(scanSub);

patience = 5;
time_not_moving = 0;
prev_position = [0,0];
prev_heading = 0;

try
    lidarScan = rosReadLidarScan(scan);
    addScan(slamAlg, lidarScan);
catch
end
[scans, optimizedPoses] = scansAndPoses(slamAlg);
map = buildMap(scans, optimizedPoses, mapResolution, maxLidarRange);
map_rebuild_counter = 0;
last_scan_position = optimizedPoses(end,1:2);
last_scan_angle    = optimizedPoses(end,3);
position = optimizedPoses(end,1:2);
angle    = optimizedPoses(end,3);

% Odom anchors — dead reckon from here between SLAM updates
odom_anchor_pos   = [0, 0];
odom_anchor_angle = 0;
odom_msg = odomSub.LatestMessage;
if ~isempty(odom_msg)
    odom_anchor_pos   = [odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y];
    odom_anchor_angle = quat_to_yaw(odom_msg.pose.pose.orientation);
end
slam_anchor_pos   = position;
slam_anchor_angle = angle;
curr_odom_pos     = odom_anchor_pos;
curr_odom_angle   = odom_anchor_angle;

while drive

    %% find position and angle
    scan = receive(scanSub);
    if isempty(scan)
        continue;
    end

    try % catch if scan is empty
        lidarScan = rosReadLidarScan(scan);
    catch
        continue;
    end

    %% Dead reckoning from odometry between SLAM updates
    curr_odom_pos   = odom_anchor_pos;
    curr_odom_angle = odom_anchor_angle;
    odom_msg = odomSub.LatestMessage;
    if ~isempty(odom_msg)
        curr_odom_pos   = [odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y];
        curr_odom_angle = quat_to_yaw(odom_msg.pose.pose.orientation);
        position = slam_anchor_pos + (curr_odom_pos - odom_anchor_pos);
        angle    = slam_anchor_angle + atan2(sin(curr_odom_angle - odom_anchor_angle), ...
                                             cos(curr_odom_angle - odom_anchor_angle));
    end

    %% Add scan and resync SLAM when robot has moved enough
    dist_moved  = norm(position - last_scan_position);
    angle_moved = abs(atan2(sin(angle - last_scan_angle), cos(angle - last_scan_angle)));
    if dist_moved > 0.05 || angle_moved > 0.1
        addScan(slamAlg, lidarScan);
        last_scan_position = position;
        last_scan_angle    = angle;

        [scans, optimizedPoses] = scansAndPoses(slamAlg);

        map_rebuild_counter = map_rebuild_counter + 1;
        if map_rebuild_counter >= map_rebuild_interval
            map = buildMap(scans, optimizedPoses, mapResolution, maxLidarRange);
            map_rebuild_counter = 0;
        end

        % Resync: treat SLAM-corrected pose as the new anchor
        slam_anchor_pos   = optimizedPoses(end,1:2);
        slam_anchor_angle = optimizedPoses(end,3);
        position = slam_anchor_pos;
        angle    = slam_anchor_angle;
        odom_anchor_pos   = curr_odom_pos;
        odom_anchor_angle = curr_odom_angle;
    end

    dt = toc(time_previous);
    time_previous = tic;
    if dt <= 0
        dt = 0.01;
    end

    %% Stuck detection
    position_diff = norm(position - prev_position);
    angle_diff = atan2(sin(prev_heading - angle), cos(prev_heading - angle));
    if position_diff < 0.02 && abs(angle_diff) < 0.05
        time_not_moving = time_not_moving + dt;
        if time_not_moving > patience
            'aaaaaaaa copy'
            stop_robot(cmdPub);
            updated_slam = slamAlg;
            time_not_moving = 0;
            fail = true;
            return;
        end
    else
        time_not_moving = 0;
    end

    prev_heading  = angle;
    prev_position = position;

    % Only validate the remaining path segment from current position
    if (~validate_path([position; path(path_index:end,:)], map))
        "invalid path :(((("
        stop_robot(cmdPub);
        updated_slam = slamAlg;
        return
    end

    %% Advance waypoint index when within tolerance
    if norm(position - path(path_index,:)) < tolerance
        path_index = path_index + 1;
        if path_index > max_index || path_index > total_waypoints
            "finished, no i'm danish"
            drive = false;
            break;
        end
    end

    %% Pure pursuit: aim at a lookahead point along the remaining path
    lookahead_point = find_lookahead(path, path_index, position, lookahead_dist);

    plot_all(figure1, map, optimizedPoses, path, lookahead_point);

    position_delta   = lookahead_point - position;
    desired_heading  = atan2(position_delta(2), position_delta(1));
    distance_to_target = norm(path(path_index,:) - position);

    headingError    = atan2(sin(desired_heading - angle), cos(desired_heading - angle));
    headingErrorInt = headingErrorInt + headingError * dt;
    headingErrorDer = (headingError - headingErrorPrev) / dt;
    headingErrorPrev = headingError;

    distanceError    = distance_to_target;
    distanceErrorInt = distanceErrorInt + distanceError * dt;
    distanceErrorDer = (distanceError - distanceErrorPrev) / dt;
    distanceErrorPrev = distanceError;

    plot_error(angle, desired_heading, distance_to_target, t0);

    %% PID

    angularVelocity = Kp_h * headingError ...
                    + Ki_h * headingErrorInt ...
                    + Kd_h * headingErrorDer;

    linearVelocity  = Kp_d * distanceError ...
                    + Ki_d * distanceErrorInt ...
                    + Kd_d * distanceErrorDer;

    linearVelocity = linearVelocity * max(0, cos(headingError) .^ 8);

    % Gentle deceleration close to the current waypoint
    if distance_to_target < 0.05
        linearVelocity = linearVelocity * 0.5;
    end

    cmdMsg = ros2message('geometry_msgs/Twist');
    cmdMsg.linear.x  = clip(linearVelocity,  -0.4, 0.4);
    cmdMsg.angular.z = clip(angularVelocity, -4.0, 4.0);

    % send(cmdPub, cmdMsg);
end

stop_robot(cmdPub);
updated_slam = slamAlg;
end


function lookahead = find_lookahead(path, path_index, position, L_d)
% Project position onto the current path segment, then walk L_d arc-length
% forward along the path so the lookahead point is always on the path.
    n = size(path, 1);

    % Current segment: path(path_index-1) -> path(path_index)
    if path_index > 1
        p1 = path(path_index-1,:);
        p2 = path(path_index,:);
        d  = p2 - p1;
        t  = dot(position - p1, d) / dot(d, d);
        t  = max(0, min(1, t));
        start_point = p1 + t * d;
        start_seg   = path_index - 1;
    else
        start_point = path(1,:);
        start_seg   = 1;
    end

    remaining = L_d;
    for i = start_seg : n-1
        if i == start_seg
            p_start = start_point;
        else
            p_start = path(i,:);
        end
        p_end   = path(i+1,:);
        seg_len = norm(p_end - p_start);
        if seg_len < 1e-9
            continue;
        end
        if remaining <= seg_len
            lookahead = p_start + (remaining / seg_len) * (p_end - p_start);
            return;
        end
        remaining = remaining - seg_len;
    end

    lookahead = path(end,:);
end


function stop_robot(cmdPub)
    cmdMsg = ros2message('geometry_msgs/Twist');
    cmdMsg.linear.x  = 0;
    cmdMsg.angular.z = 0;
    send(cmdPub, cmdMsg);
end


function plot_error(heading, desiredHeading, distanceToTarget, t0)
    global timeData headingData desiredHeadingData distanceData ax2 ax3

    t = toc(t0);

    timeData(end+1)           = t;
    headingData(end+1)        = heading;
    desiredHeadingData(end+1) = desiredHeading;
    distanceData(end+1)       = distanceToTarget;

    plot(ax2, timeData, headingData, 'b', 'LineWidth', 1.5);
    plot(ax2, timeData, desiredHeadingData, 'r--', 'LineWidth', 1.5);
    xlabel(ax2, 'Time [s]');
    ylabel(ax2, 'Heading [rad]');
    legend(ax2, 'Actual', 'Desired');
    grid(ax2, 'on');

    plot(ax3, timeData, distanceData, 'k', 'LineWidth', 1.5);
    xlabel(ax3, 'Time [s]');
    ylabel(ax3, 'Distance [m]');
    grid(ax3, 'on');

    drawnow limitrate;
end


function yaw = quat_to_yaw(q)
    yaw = atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y^2 + q.z^2));
end
