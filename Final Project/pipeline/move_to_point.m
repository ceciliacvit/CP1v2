function [current_pose, odom_trajectory, circle_state] = move_to_point(scanSub, odomSub, cmdPub, target, slamAlg, initialPose, odom_trajectory, circle_state, scan_enabled)
    arguments
        scanSub
        odomSub
        cmdPub
        target
        slamAlg
        initialPose = []
        odom_trajectory = struct('scans', {{}}, 'poses', zeros(0,3))
        circle_state = struct('orange', false, 'blue', false)
        scan_enabled = false
    end

    fig = figure('Name', 'Navigation');
    [base_scans, base_poses] = scansAndPoses(slamAlg);
    map = buildMap(base_scans, base_poses, 20, 8);
    current_pose = initialPose;

    while true
        if scan_enabled
            path = [current_pose(1:2); target];
        else
            path = createPath(current_pose(1:2), target, map);
        end

        if scan_enabled && ~(circle_state.orange && circle_state.blue)
            chunk = 0.75;
        else
            chunk = Inf;
        end

        [slamAlg, fail, current_pose, odom_trajectory, circle_scan] = MoveRobot( ...
            path, slamAlg, scanSub, odomSub, cmdPub, fig, 0.8, 0.20, ...
            current_pose, odom_trajectory, chunk);
            
        refresh_plot(fig, map, current_pose, odom_trajectory, base_poses);

        if fail
            disp('fail: relocalizing');
            current_pose = localize_robot(map, scanSub, odomSub, cmdPub, current_pose);
            continue; 
        end

        if circle_scan
            pre_scan_pose = current_pose;
            [circle_state, current_pose] = find_circles(cmdPub, current_pose, circle_state, odomSub);
            
            if norm(current_pose(1:2) - pre_scan_pose(1:2)) > 0.05 && ~isempty(path)
                [current_pose, odom_trajectory] = backtrack_to_path( ...
                    current_pose, path, map, slamAlg, scanSub, odomSub, cmdPub, fig, odom_trajectory, base_poses);
            end
            continue; 
        end

        break;
    end
    
    close(fig);
end

function refresh_plot(fig, map, current_pose, odom_trajectory, base_poses)
    if ~isempty(odom_trajectory.poses)
        plot_all(fig, map, [base_poses; odom_trajectory.poses]);
    else
        plot_all(fig, map, current_pose);
    end
    drawnow;
end

function [current_pose, odom_trajectory] = backtrack_to_path(current_pose, path, map, slamAlg, scanSub, odomSub, cmdPub, fig, odom_trajectory, base_poses)
    [~, idx] = min(vecnorm(path - current_pose(1:2), 2, 2));
    back_path = createPath(current_pose(1:2), path(idx, :), map);
    
    if ~isempty(back_path)
        [~, ~, current_pose, odom_trajectory] = MoveRobot( ...
            back_path, slamAlg, scanSub, odomSub, cmdPub, fig, ...
            1.0, 0.20, current_pose, odom_trajectory);
        refresh_plot(fig, map, current_pose, odom_trajectory, base_poses);
    end
end