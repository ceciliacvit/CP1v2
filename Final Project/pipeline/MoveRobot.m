function [updated_slam, fail, final_pose, mcl_history, circle_scan_needed] = MoveRobot(path,slamAlg,scanSub,odomSub,cmdPub,figure1,percentage_threshold,tolerance,current_pose,mcl_history,max_distance)
% Drive along path with pure pursuit + VFH, dead-reckoning the pose from odom.
    arguments
        path;
        slamAlg;
        scanSub;
        odomSub;
        cmdPub;
        figure1 = 0;
        percentage_threshold = 1;
        tolerance = 0.20;
        current_pose = [];
        mcl_history = struct('scans', {{}}, 'poses', zeros(0,3));
        max_distance = Inf;
    end

    circle_scan_needed = false;

    if ~isstruct(mcl_history) || ~isfield(mcl_history,'scans') || ~isfield(mcl_history,'poses')
        mcl_history = struct('scans', {{}}, 'poses', zeros(0,3));
    end

maxLidarRange = 8;
mapResolution = 20;

time_previous = tic;
fail = false;
final_pose = current_pose;

if isempty(path)
    updated_slam = slamAlg;
    fail = true;
    return
end
total_waypoints = size(path, 1);
max_index = round(total_waypoints * percentage_threshold);
goal = path(max_index, :);

controller = controllerPurePursuit;
controller.LookaheadDistance     = 0.3;
controller.MaxAngularVelocity    = 1.84;
controller.DesiredLinearVelocity = 0.2;
controller.Waypoints             = path(1:max_index, :);

vfh = controllerVFH;
vfh.UseLidarScan = true;
vfh.RobotRadius = 0.15;
vfh.SafetyDistance = 0.05;
vfh.DistanceLimits = [0.12 3];
vfh.MinTurningRadius = 0.05;
vfh.HistogramThresholds = [3 10];

patience = 5;
time_not_moving = 0;
prev_position = [0,0];
prev_heading  = 0;
cum_dist = 0;
cum_dist_started = false;

[base_scans, base_poses] = scansAndPoses(slamAlg);
map = buildMap(base_scans, base_poses, mapResolution, maxLidarRange);

position = current_pose(1:2);
angle = current_pose(3);
if isempty(mcl_history.poses)
    optimizedPoses = current_pose;
else
    optimizedPoses = [base_poses; mcl_history.poses];
end

last_scan_position = position;
last_scan_angle    = angle;

% dead reckon from these odom/pose anchors
odom_anchor_pos   = [0, 0];
odom_anchor_angle = 0;
odom_msg = odomSub.LatestMessage;
if ~isempty(odom_msg)
    odom_anchor_pos   = [odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y];
    eul = quat2eul([odom_msg.pose.pose.orientation.w, odom_msg.pose.pose.orientation.x, ...
                    odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z]);
    odom_anchor_angle = eul(1);
end
start_anchor_pos   = position;
start_anchor_angle = angle;
curr_odom_pos     = odom_anchor_pos;
curr_odom_angle   = odom_anchor_angle;

drive = true;

while drive

    scan = receive(scanSub);
    if isempty(scan)
        continue;
    end

    try
        lidarScan = rosReadLidarScan(scan);
    catch
        continue;   % skip a malformed scan message
    end

    curr_odom_pos   = odom_anchor_pos;
    curr_odom_angle = odom_anchor_angle;
    odom_msg = odomSub.LatestMessage;
    if ~isempty(odom_msg)
        curr_odom_pos   = [odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y];
        eul = quat2eul([odom_msg.pose.pose.orientation.w, odom_msg.pose.pose.orientation.x, ...
                        odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z]);
        curr_odom_angle = eul(1);
        d_odom = curr_odom_pos - odom_anchor_pos;
        theta_diff = start_anchor_angle - odom_anchor_angle;
        c = cos(theta_diff); s = sin(theta_diff);
        position = start_anchor_pos + [c*d_odom(1) - s*d_odom(2), s*d_odom(1) + c*d_odom(2)];
        angle    = start_anchor_angle + wrapToPi(curr_odom_angle - odom_anchor_angle);
    end

    dist_moved  = norm(position - last_scan_position);
    angle_moved = abs(wrapToPi(angle - last_scan_angle));
    if isempty(mcl_history.poses) || dist_moved > 0.05 || angle_moved > 0.1
        mcl_history.scans{end+1}   = lidarScan;
        mcl_history.poses(end+1,:) = [position, angle];
        last_scan_position = position;
        last_scan_angle    = angle;
    end
    if isempty(mcl_history.poses)
        optimizedPoses = [position, angle];
    else
        optimizedPoses = [base_poses; mcl_history.poses];
    end

    dt = toc(time_previous);
    time_previous = tic;
    if dt <= 0
        dt = 0.01;
    end

    position_diff = norm(position - prev_position);
    angle_diff = wrapToPi(prev_heading - angle);
    if position_diff < 0.02 && abs(angle_diff) < 0.05
        time_not_moving = time_not_moving + dt;
        if time_not_moving > patience
            stop_robot(cmdPub);
            updated_slam = slamAlg;
            time_not_moving = 0;
            fail = true;
            final_pose = [position, angle];
            return;
        end
    else
        time_not_moving = 0;
    end

    % skip the first iteration so the [0,0] prev_position doesn't add a huge delta
    if cum_dist_started
        cum_dist = cum_dist + position_diff;
    else
        cum_dist_started = true;
    end

    prev_heading  = angle;
    prev_position = position;

    if cum_dist >= max_distance
        stop_robot(cmdPub);
        updated_slam = slamAlg;
        fail = false;
        circle_scan_needed = true;
        final_pose = [position, angle];
        return;
    end

    if (~validate_path([position; path(1:max_index,:)], map))
        stop_robot(cmdPub);
        updated_slam = slamAlg;
        fail = true;
        final_pose = [position, angle];
        return
    end

    if norm(position - goal) < tolerance
        drive = false;
        break;
    end

    if(figure1 ~= 0)
        plot_all(figure1, map, optimizedPoses, path(1:max_index,:));
    end

    currentPose = [position, angle];
    [v_desired, ~, lookaheadPt] = controller(currentPose);

    targetDir = wrapToPi(atan2(lookaheadPt(2) - position(2), ...
                               lookaheadPt(1) - position(1)) - angle);

    steeringDir = vfh(lidarScan, targetDir);

    if isnan(steeringDir)
        v = 0;
        omega = 0;
    else
        omega = 2.0 * steeringDir;
        v = v_desired * max(0.1, cos(steeringDir) .^ 8);
    end

    cmdMsg = ros2message('geometry_msgs/Twist');
    cmdMsg.linear.x  = max(min(v,    0.22), -0.22);
    cmdMsg.angular.z = max(min(omega, 2.84), -2.84);
    send(cmdPub, cmdMsg);
end

stop_robot(cmdPub);
updated_slam = slamAlg;
final_pose = [position, angle];
end


function stop_robot(cmdPub)
    cmdMsg = ros2message('geometry_msgs/Twist');
    cmdMsg.linear.x  = 0;
    cmdMsg.angular.z = 0;
    send(cmdPub, cmdMsg);
end
