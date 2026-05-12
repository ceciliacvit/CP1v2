function [circle_state, final_pose] = find_circles(scanSub, cmdPub, slamAlg, current_pose, circle_state, odomSub)
    arguments
        scanSub
        cmdPub
        slamAlg
        current_pose = []
        circle_state = struct('orange', false, 'blue', false)
        odomSub = []
    end

    % Skip immediately if both colors already found.
    if circle_state.orange && circle_state.blue
        if isempty(current_pose)
            final_pose = [0, 0, 0];
        else
            final_pose = current_pose;
        end
        return;
    end

    scan = receive(scanSub);
    try
    lidarScan = rosReadLidarScan(scan);
    if isempty(current_pose)
        addScan(slamAlg, lidarScan);
    end
    catch
    end

    maxLidarRange = 8;
    mapResolution = 20;

    if isempty(current_pose)
        [scans, optimizedPoses] = scansAndPoses(slamAlg);
        position = optimizedPoses(end,1:2);
        angle    = optimizedPoses(end,3);
    else
        position = current_pose(1:2);
        angle    = current_pose(3);
    end

    % Record odom anchor so we can dead-reckon back to a map-frame pose at exit.
    odom_anchor_pos   = [];
    odom_anchor_angle = 0;
    if ~isempty(odomSub)
        odom_msg = odomSub.LatestMessage;
        if ~isempty(odom_msg)
            odom_anchor_pos = [odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y];
            eul = quat2eul([odom_msg.pose.pose.orientation.w, odom_msg.pose.pose.orientation.x, ...
                            odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z]);
            odom_anchor_angle = eul(1);
        end
    end

    global run_speed
    run_speed = 0.3;
    global scan_speed
    scan_speed = 0.05;

    cmdMsg = ros2message('geometry_msgs/Twist');
    cmdMsg.angular.z = -run_speed;
    cmdMsg.linear.x  = 0;
    pause(1.0)
    send(cmdPub, cmdMsg)

    global Detects
    Detects = false(1, 10);
    global detect_idx
    detect_idx = 0;
    global orange_circle_found
    orange_circle_found = circle_state.orange;
    global blue_circle_found
    blue_circle_found   = circle_state.blue;
    global slowing
    slowing = false;

    while ~(blue_circle_found && orange_circle_found)
        circle = detectCircle(0.75);
        detect_idx = mod(detect_idx, 10) + 1;
        Detects(detect_idx) = circle.found;
        recent_sum = sum(Detects);

        % Slow down on first sight, speed back up when lost
        if circle.found && ~slowing
            cmdMsg.angular.z = -scan_speed;
            send(cmdPub, cmdMsg);
            slowing = true;
        elseif ~circle.found && slowing
            cmdMsg.angular.z = -run_speed;
            send(cmdPub, cmdMsg);
            slowing = false;
        end

        % Not confirmed by enough recent frames, or color already found
        if recent_sum <= 3
            continue;
        end
        if (circle.color == "orange" && orange_circle_found) || ...
           (circle.color == "blue"   && blue_circle_found)
            continue;
        end

        % Stop spinning
        cmdMsg.angular.z = 0;
        cmdMsg.linear.x  = 0;
        send(cmdPub, cmdMsg);
        pause(0.4);

        if circle.color == "orange"
            orange_circle_found = true;
            disp("orange found");
        elseif circle.color == "blue"
            blue_circle_found = true;
            disp("blue found");
        end

        % Approach until 0.9–1.1m away
        while true
            circle = detectCircle(0.7);
            if ~circle.found
                cmdMsg.linear.x = 0;
                send(cmdPub, cmdMsg);
                continue;
            end
            if circle.distance > 1.1
                cmdMsg.linear.x = 0.05;
            elseif circle.distance < 0.9
                cmdMsg.linear.x = -0.05;
            else
                cmdMsg.linear.x = 0;
                send(cmdPub, cmdMsg);
                break;
            end
            send(cmdPub, cmdMsg);
        end

        captureImage();
        Detects = false(1, 10);  % reset so next circle starts fresh
        detect_idx = 0;
        slowing = false;

        % Resume spinning to look for the next color
        cmdMsg.angular.z = -run_speed;
        cmdMsg.linear.x  = 0;
        send(cmdPub, cmdMsg);
    end

    % Final stop
    cmdMsg.angular.z = 0;
    cmdMsg.linear.x  = 0;
    send(cmdPub, cmdMsg)

    % Read back persistent flags into the return struct.
    circle_state.orange = orange_circle_found;
    circle_state.blue   = blue_circle_found;

    % Estimate final pose by integrating odometry delta in the map frame.
    final_pose = [position, angle];
    if ~isempty(odomSub) && ~isempty(odom_anchor_pos)
        odom_msg = odomSub.LatestMessage;
        if ~isempty(odom_msg)
            curr_odom_pos = [odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y];
            eul = quat2eul([odom_msg.pose.pose.orientation.w, odom_msg.pose.pose.orientation.x, ...
                            odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z]);
            curr_odom_angle = eul(1);
            d_odom = curr_odom_pos - odom_anchor_pos;
            theta_diff = angle - odom_anchor_angle;
            c = cos(theta_diff); s = sin(theta_diff);
            new_xy = [position(1) + c*d_odom(1) - s*d_odom(2), ...
                      position(2) + s*d_odom(1) + c*d_odom(2)];
            final_pose = [new_xy, angle + wrapToPi(curr_odom_angle - odom_anchor_angle)];
        end
    end

end
