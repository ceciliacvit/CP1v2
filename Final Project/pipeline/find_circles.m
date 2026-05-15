function [circle_state, final_pose] = find_circles(cmdPub, current_pose, circle_state, odomSub)
    arguments
        cmdPub
        current_pose
        circle_state = struct('orange', false, 'blue', false)
        odomSub = []
    end

    if circle_state.orange && circle_state.blue
        final_pose = current_pose;
        return;
    end

    position = current_pose(1:2);
    angle    = current_pose(3);

    % anchor odom so we can dead-reckon back to a map-frame pose on exit
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

    run_speed  = 0.3;
    scan_speed = 0.05;

    cmdMsg = ros2message('geometry_msgs/Twist');
    cmdMsg.angular.z = -run_speed;
    cmdMsg.linear.x  = 0;
    pause(1.0)
    send(cmdPub, cmdMsg)

    % lock a color only when the whole window agrees, to reject false positives
    detect_window = 3;
    Detects = false(1, detect_window);
    detect_idx = 0;
    orange_circle_found = circle_state.orange;
    blue_circle_found   = circle_state.blue;
    slowing = false;

    % stop after roughly one full turn so the caller can move on
    last_yaw = read_yaw(odomSub);
    total_rotation = 0;
    full_rotation  = 2*pi;

    while ~(blue_circle_found && orange_circle_found) && total_rotation < full_rotation
        circle = detectCircle(0.8, orange_circle_found, blue_circle_found);
        detect_idx = mod(detect_idx, detect_window) + 1;
        Detects(detect_idx) = circle.found;
        recent_sum = sum(Detects);

        % slow down on first sight, speed back up when lost
        if circle.found && ~slowing
            slowing = true;
        elseif ~circle.found && slowing
            slowing = false;
        end
        if slowing
            cmdMsg.angular.z = -scan_speed;
        else
            cmdMsg.angular.z = -run_speed;
        end
        cmdMsg.linear.x = 0;
        send(cmdPub, cmdMsg);  % republish so the cmd_vel watchdog keeps the motors alive

        curr_yaw = read_yaw(odomSub);
        if ~isnan(curr_yaw) && ~isnan(last_yaw)
            total_rotation = total_rotation + abs(wrapToPi(curr_yaw - last_yaw));
        end
        last_yaw = curr_yaw;

        if recent_sum < detect_window
            continue;
        end
        if circle.color == ""
            continue;
        end
        if (circle.color == "orange" && orange_circle_found) || ...
           (circle.color == "blue"   && blue_circle_found)
            continue;
        end

        cmdMsg.angular.z = 0;
        cmdMsg.linear.x  = 0;
        send(cmdPub, cmdMsg);
        pause(0.4);

        target_color = circle.color;

        % approach until 0.9-1.1 m away
        approach_timer = tic;
        success = false;
        while true
            if toc(approach_timer) > 20
                disp("Approach timed out.");
                cmdMsg.linear.x = 0;
                cmdMsg.angular.z = 0;
                send(cmdPub, cmdMsg);
                break;
            end

            circle = detectCircle(0.8, orange_circle_found, blue_circle_found);
            if ~circle.found
                cmdMsg.linear.x = 0;
                cmdMsg.angular.z = 0;
                send(cmdPub, cmdMsg);
                continue;
            end

            cmdMsg.angular.z = -0.5 * circle.angle_error;

            if circle.distance > 1.1
                cmdMsg.linear.x = 0.05;
            elseif circle.distance < 0.9
                cmdMsg.linear.x = -0.05;
            else
                cmdMsg.linear.x = 0;
                cmdMsg.angular.z = 0;
                send(cmdPub, cmdMsg);
                success = true;
                break;
            end
            send(cmdPub, cmdMsg);
        end

        if success
            if target_color == "orange"
                orange_circle_found = true;
                disp("orange found");
            elseif target_color == "blue"
                blue_circle_found = true;
                disp("blue found");
            end
            captureImage();
        else
            disp("Failed to reach circle, resuming search.");
        end
        Detects = false(1, detect_window);
        detect_idx = 0;
        slowing = false;

        % allow another full turn from the new vantage
        cmdMsg.angular.z = -run_speed;
        cmdMsg.linear.x  = 0;
        send(cmdPub, cmdMsg);
        last_yaw = read_yaw(odomSub);
        total_rotation = 0;
    end

    cmdMsg.angular.z = 0;
    cmdMsg.linear.x  = 0;
    send(cmdPub, cmdMsg)

    circle_state.orange = orange_circle_found;
    circle_state.blue   = blue_circle_found;

    % integrate the odom delta to report the exit pose in the map frame
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


function yaw = read_yaw(odomSub)
    yaw = NaN;
    if isempty(odomSub)
        return;
    end
    odom_msg = odomSub.LatestMessage;
    if isempty(odom_msg)
        return;
    end
    eul = quat2eul([odom_msg.pose.pose.orientation.w, odom_msg.pose.pose.orientation.x, ...
                    odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z]);
    yaw = eul(1);
end
