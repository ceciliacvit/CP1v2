function [circle_state, final_pose] = find_circles(cmdPub, current_pose, odomSub, circle_state)
    arguments
        cmdPub
        current_pose
        odomSub
        circle_state = struct('orange', false, 'blue', false)
    end

    if circle_state.orange && circle_state.blue
        final_pose = current_pose;
        return;
    end

    % odom anchor for exit dead-reckoning
    odom_anchor = read_odom(odomSub);

    run_speed  = 0.3;
    scan_speed = 0.05;

    cmdMsg = ros2message('geometry_msgs/Twist');
    cmdMsg.angular.z = -run_speed;
    cmdMsg.linear.x  = 0;
    pause(1.0)
    send(cmdPub, cmdMsg)

    min_hits = 3;
    hits = 0;

    last_yaw = odom_anchor(3);
    total_rotation = 0;

    while ~(circle_state.blue && circle_state.orange) && total_rotation < 2*pi
        circle = detectCircle(0.8, circle_state.orange, circle_state.blue);
        if circle.found
            hits = hits + 1;
        else
            hits = 0;
        end

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

        if hits < min_hits || circle.color == "" || ...
           (circle.color == "orange" && circle_state.orange) || ...
           (circle.color == "blue"   && circle_state.blue)
            continue;
        end

        stop_robot(cmdPub);
        pause(0.4);

        target_color = circle.color;
        approach_timer = tic;
        success = false;

        % approach until 0.9-1.1 m away
        while toc(approach_timer) <= 20
            circle = detectCircle(0.8, circle_state.orange, circle_state.blue);
            if ~circle.found
                stop_robot(cmdPub);
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
            captureImage(circle.image);
        else
            disp("Failed to reach circle, resuming search.");
        end

        hits = 0;

        % allow another full turn from the new vantage
        cmdMsg.angular.z = -run_speed;
        cmdMsg.linear.x  = 0;
        send(cmdPub, cmdMsg);
        last_odom = read_odom(odomSub);
        last_yaw = last_odom(3);
        total_rotation = 0;
    end

    stop_robot(cmdPub);

    % exit pose in map frame
    curr_odom = read_odom(odomSub);
    d_odom = curr_odom(1:2) - odom_anchor(1:2);
    theta_diff = current_pose(3) - odom_anchor(3);

    final_pose = [current_pose(1) + cos(theta_diff)*d_odom(1) - sin(theta_diff)*d_odom(2), ...
                  current_pose(2) + sin(theta_diff)*d_odom(1) + cos(theta_diff)*d_odom(2), ...
                  current_pose(3) + wrapToPi(curr_odom(3) - odom_anchor(3))];
end