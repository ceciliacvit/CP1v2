function find_circles(scanSub,cmdPub)

    slamAlg = lidarSLAM(mapResolution, maxLidarRange);

    slamAlg.LoopClosureThreshold = 210;  
    slamAlg.LoopClosureSearchRadius = 8;
    circles = [];
    running = true;
    while running
        scan = receive(scanSub);
        if isempty(scan)
            continue;
        end
        try % catch if scan is empty
            lidarScan=rosReadLidarScan(scan);
        catch
            continue;
        end    

        addScan(slamAlg, lidarScan);
        
        %% we want to explore here. 




        circle = detectCircle();

        if(circle.found)
            

            [scans, optimizedPoses] = scansAndPoses(slamAlg);
            map = buildMap(scans, optimizedPoses, mapResolution, maxLidarRange);

            position = optimizedPoses(end,1:2);
            angle    = optimizedPoses(end,3);
            headingError = 10000;
    
            bearing  = angle - circle.angle_error;
            target_x = position.x + (circle.distance - 1.0) * cos(bearing);
            target_y = position.y + (circle.distance - 1.0) * sin(bearing);
            circle_pos = [target_x,target_y];
            
            if(~ismember(circle_pos,circles))
                headingErrorPrev = 0;
                headingErrorInt = 0;
                circles(end+1,:) = circle_pos;
                path = createPath(position,circles(end,:),map);
                [slamAlg,~] = MoveRobot(path,slamAlg,scanSub,cmdPub);
                
                position_delta = circle_pos - position;
                desired_heading = atan2(position_delta(2), position_delta(1));

                while (headingError > 0.1)
                    cmdMsg = ros2message('geometry_msgs/Twist');
                    [cmdMsg.angular.z,headingErrorInt,headingErrorPrev,headingError] = look_at(angle,desired_heading,headingErrorInt,headingErrorPrev);
                    send(cmdPub, cmdMsg);
                end
                cmdMsg = ros2message('geometry_msgs/Twist');
                cmdMsg.angular.z = 0;
                send(cmdPub,cmdMsg)
                captureImage()
            end
        end
        
        if(size(circles,1) >= 2)
            running = false;
        end

        pause(0.05);
    end
    

end