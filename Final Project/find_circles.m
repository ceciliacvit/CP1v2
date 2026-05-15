function [circle_state, final_pose] = find_circles(cmdPub, current_pose, circle_state, odomSub)
    arguments
        cmdPub
        current_pose
        circle_state = struct('orange', false, 'blue', false)
        odomSub = []
    end

    % Skip immediately if both colors already found.
    if circle_state.orange && circle_state.blue
        final_pose = current_pose;
        return;
    end

    position = current_pose(1:2);
    angle    = current_pose(3);

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

    run_speed  = 0.3;
    scan_speed = 0.05;

    cmdMsg = ros2message('geometry_msgs/Twist');
    cmdMsg.angular.z = -run_speed;
    cmdMsg.linear.x  = 0;
    pause(1.0)
    send(cmdPub, cmdMsg)

    % Rolling window of recent detections; a color is only locked onto once
    % the whole window agrees, to reject single-frame false positives.
    detect_window = 3;
    Detects = false(1, detect_window);
    detect_idx = 0;
    orange_circle_found = circle_state.orange;
    blue_circle_found   = circle_state.blue;
    slowing = false;

    % One spin per find_circles call. Track cumulative rotation via odom yaw
    % so the function returns after ~360 degrees regardless of how many
    % circles were found — the next drive chunk gets a fresh vantage point.
    last_yaw = read_yaw(odomSub);
    total_rotation = 0;
    full_rotation  = 2*pi;

    while ~(blue_circle_found && orange_circle_found) && total_rotation < full_rotation
        circle = detectCircle(0.8, orange_circle_found, blue_circle_found);
        detect_idx = mod(detect_idx, detect_window) + 1;
        Detects(detect_idx) = circle.found;
        recent_sum = sum(Detects);

        % Slow down on first sight, speed back up when lost
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
        send(cmdPub, cmdMsg);  % republish every iter so cmd_vel watchdog doesn't stop the motors

        % Accumulate rotation from odom yaw deltas.
        curr_yaw = read_yaw(odomSub);
        if ~isnan(curr_yaw) && ~isnan(last_yaw)
            total_rotation = total_rotation + abs(wrapToPi(curr_yaw - last_yaw));
        end
        last_yaw = curr_yaw;

        % Not confirmed by enough recent frames, or color already found
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

        % Stop spinning
        cmdMsg.angular.z = 0;
        cmdMsg.linear.x  = 0;
        send(cmdPub, cmdMsg);
        pause(0.4);

        target_color = circle.color;

        % Approach until 0.9–1.1m away
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
            
            % Realign the circle to the middle of the image
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
        Detects = false(1, detect_window);  % reset so next circle starts fresh
        detect_idx = 0;
        slowing = false;

        % Resume spinning to look for the next color. Reset the rotation
        % counter so we still allow a full 360° from this new vantage.
        cmdMsg.angular.z = -run_speed;
        cmdMsg.linear.x  = 0;
        send(cmdPub, cmdMsg);
        last_yaw = read_yaw(odomSub);
        total_rotation = 0;
    end

    % Final stop
    cmdMsg.angular.z = 0;
    cmdMsg.linear.x  = 0;
    send(cmdPub, cmdMsg)

    % Read back persistent flags into the return struct.
    circle_state.orange = orange_circle_found;
    circle_state.blue   = blue_circle_found;

    % NB: rotation cap is reset after each circle approach (above), so
    % find_circles still completes both circles in one call when both are
    % visible from the same vantage. When only one is visible, the function
    % returns after a single rotation so the caller can advance.

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
