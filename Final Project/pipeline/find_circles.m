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

    % anchor odom so we can dead-reckon back to a map-frame pose on exit
    odom_anchor = read_odom(odomSub);

    run_speed  = 0.3;
    scan_speed = 0.05;

    cmdMsg = ros2message('geometry_msgs/Twist');
    cmdMsg.angular.z = -run_speed;
    cmdMsg.linear.x  = 0;
    pause(1.0)
    send(cmdPub, cmdMsg)

    detect_window = 3;
    Detects = false(1, detect_window);
    detect_idx = 0;

    last_yaw = odom_anchor(3);
    total_rotation = 0;

    while ~(circle_state.blue && circle_state.orange) && total_rotation < 2*pi
        circle = detectCircle(0.8, circle_state.orange, circle_state.blue);
        detect_idx = mod(detect_idx, detect_window) + 1;
        Detects(detect_idx) = circle.found;

        % slow down on first sight, speed back up when lost
        if circle.found
            cmdMsg.angular.z = -scan_speed;
        else
            cmdMsg.angular.z = -run_speed;
        end
        cmdMsg.linear.x = 0;
        send(cmdPub, cmdMsg);  

        curr_odom = read_odom(odomSub);
        total_rotation = total_rotation + abs(wrapToPi(curr_odom(3) - last_yaw));
        last_yaw = curr_odom(3);

        if sum(Detects) < detect_window || circle.color == "" || ...
           (circle.color == "orange" && circle_state.orange) || ...
           (circle.color == "blue"   && circle_state.blue)
            continue;
        end

        cmdMsg.angular.z = 0;
        cmdMsg.linear.x  = 0;
        send(cmdPub, cmdMsg);
        pause(0.4);

        target_color = circle.color;
        approach_timer = tic;
        success = false;

        % approach until 0.9-1.1 m away
        while toc(approach_timer) <= 20
            circle = detectCircle(0.8, circle_state.orange, circle_state.blue);
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
            circle_state.(target_color) = true;
            disp(target_color + " found");
            captureImage();
        else
            disp("Failed to reach circle, resuming search.");
        end
        
        Detects = false(1, detect_window);

        % allow another full turn from the new vantage
        cmdMsg.angular.z = -run_speed;
        cmdMsg.linear.x  = 0;
        send(cmdPub, cmdMsg);
        last_odom = read_odom(odomSub);
        last_yaw = last_odom(3);
        total_rotation = 0;
    end

    cmdMsg.angular.z = 0;
    cmdMsg.linear.x  = 0;
    send(cmdPub, cmdMsg)

    % integrate the odom delta to report the exit pose in the map frame
    curr_odom = read_odom(odomSub);
    d_odom = curr_odom(1:2) - odom_anchor(1:2);
    theta_diff = current_pose(3) - odom_anchor(3);
    
    final_pose = [current_pose(1) + cos(theta_diff)*d_odom(1) - sin(theta_diff)*d_odom(2), ...
                  current_pose(2) + sin(theta_diff)*d_odom(1) + cos(theta_diff)*d_odom(2), ...
                  current_pose(3) + wrapToPi(curr_odom(3) - odom_anchor(3))];
end


function odom = read_odom(odomSub)
    msg = odomSub.LatestMessage;
    eul = quat2eul([msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, ...
                    msg.pose.pose.orientation.y, msg.pose.pose.orientation.z]);
    odom = [msg.pose.pose.position.x, msg.pose.pose.position.y, eul(1)];
end