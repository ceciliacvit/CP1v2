function find_circles(~, cmdPub)

    run_speed  = 0.3;
    scan_speed = 0.05;

    cmdMsg = ros2message('geometry_msgs/Twist');
    cmdMsg.angular.z = -run_speed;
    cmdMsg.linear.x  = 0;
    pause(1.0)
    send(cmdPub, cmdMsg)

    Detects = false(1, 10);
    detect_idx = 0;
    orange_circle_found = false;
    blue_circle_found   = false;
    slowing = false;

    while ~(blue_circle_found && orange_circle_found)

        circle = detectCircle(0.75);
        detect_idx = mod(detect_idx, 10) + 1;
        Detects(detect_idx) = circle.found;
        recent_sum = sum(Detects);

        % Slow down on first sight, speed back up when lost
        if circle.found && ~slowing
            cmdMsg.angular.z = -scan_speed;
            send(cmdPub, cmdMsg)
            slowing = true;
        elseif ~circle.found && slowing
            cmdMsg.angular.z = -run_speed;
            send(cmdPub, cmdMsg)
            slowing = false;
        end

        % Skip if not confirmed, or already found this color
        if recent_sum <= 3
            continue
        end
        if (circle.color == "orange" && orange_circle_found) || ...
           (circle.color == "blue"   && blue_circle_found)
            continue
        end

        % Stop
        cmdMsg.angular.z = 0;
        cmdMsg.linear.x  = 0;
        send(cmdPub, cmdMsg)
        pause(0.4)

        if circle.color == "orange"
            orange_circle_found = true;
            disp("orange found")
        elseif circle.color == "blue"
            blue_circle_found = true;
            disp("blue found")
        end

        % Approach until 1m away
        while true
            circle = detectCircle(0.7);
            if ~circle.found
                cmdMsg.linear.x = 0;
                send(cmdPub, cmdMsg)
                continue
            end
            if circle.distance > 1.1
                cmdMsg.linear.x = 0.05;
            elseif circle.distance < 0.9
                cmdMsg.linear.x = -0.05;
            else
                cmdMsg.linear.x = 0;
                send(cmdPub, cmdMsg)
                break
            end
            send(cmdPub, cmdMsg)
        end

        captureImage();
        Detects = false(1, 10);  % reset so next circle starts fresh
        detect_idx = 0;
        slowing = false;

        % Resume spinning
        cmdMsg.angular.z = -run_speed;
        cmdMsg.linear.x  = 0;
        send(cmdPub, cmdMsg)

    end

    % Final stop
    cmdMsg.angular.z = 0;
    cmdMsg.linear.x  = 0;
    send(cmdPub, cmdMsg)

end
