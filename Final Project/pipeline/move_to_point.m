function [final_pose, mcl_history, circle_state] = move_to_point(scanSub,odomSub,cmdPub,target,slamAlg,initialPose,mcl_history,circle_state,scan_enabled)
    arguments
        scanSub
        odomSub
        cmdPub
        target
        slamAlg
        initialPose = []
        mcl_history = struct('scans', {{}}, 'poses', zeros(0,3))
        circle_state = struct('orange', false, 'blue', false)
        scan_enabled = false
    end

maxLidarRange = 8;
mapResolution = 20;
figure1 = figure;

[base_scans, base_poses] = scansAndPoses(slamAlg);
current_pose = initialPose;
map = buildMap(base_scans, base_poses, mapResolution, maxLidarRange);

while true
    % only chunk the drive while still looking for circles
    if scan_enabled && ~(circle_state.orange && circle_state.blue)
        chunk_distance = 0.75;
    else
        chunk_distance = Inf;
    end

    if scan_enabled
        path = [current_pose(1:2); target];
    else
        path = createPath(current_pose(1:2),target,map);
    end

    [slamAlg,fail,final_pose,mcl_history,circle_scan_needed] = MoveRobot( ...
        path,slamAlg,scanSub,odomSub,cmdPub,figure1,0.8,0.20, ...
        current_pose,mcl_history,chunk_distance);
    current_pose = final_pose;

    refresh_map_and_plot();

    % a failed leg means the believed pose is unreliable, so relocalize
    if fail
        disp('fail: relocalizing');
        current_pose = localize_robot(map, scanSub, odomSub, cmdPub, current_pose);
    end

    if circle_scan_needed
        pre_scan_pose = current_pose;

        [circle_state, post_scan_pose] = find_circles( ...
            cmdPub, current_pose, circle_state, odomSub);
        current_pose = post_scan_pose;

        % only backtrack if the scan actually moved the robot off the path
        moved_during_scan = norm(post_scan_pose(1:2) - pre_scan_pose(1:2)) > 0.05;
        if moved_during_scan && ~isempty(path) && size(path,1) > 0
            idx = nearest_path_idx(path, post_scan_pose);
            backtrack_target = path(idx,:);
            back_path = createPath(post_scan_pose(1:2), backtrack_target, map);
            if ~isempty(back_path)
                [slamAlg,~,final_pose,mcl_history] = MoveRobot( ...
                    back_path,slamAlg,scanSub,odomSub,cmdPub,figure1,1.0,0.20, ...
                    current_pose,mcl_history);
                current_pose = final_pose;
                refresh_map_and_plot();
            end
        end

        continue;
    end

    if ~fail
        break;
    end
end
close(figure1);

    function refresh_map_and_plot()
        if ~isempty(mcl_history.poses)
            plot_all(figure1, map, [base_poses; mcl_history.poses]);
        else
            plot_all(figure1, map, current_pose);
        end
        drawnow;
    end
end


function idx = nearest_path_idx(path, pose)
    d = vecnorm(path - pose(1:2), 2, 2);
    [~, idx] = min(d);
end
