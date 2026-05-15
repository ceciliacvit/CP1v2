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

'move_to_point started'
maxLidarRange = 8;
mapResolution = 20;
% global visualise
% if isempty(visualise) || ~isvalid(visualise.fig)
%     visualise=TurtleBotVisualise();
% end
figure1 = figure;

% Starting position is always the MCL result (loaded-map workflow).
[base_scans, base_poses] = scansAndPoses(slamAlg);
current_pose = initialPose;
% Map is static: the loaded map is the source of truth.
map = buildMap(base_scans, base_poses, mapResolution, maxLidarRange);

while true
    % Only fire the 50cm circle-scan trigger if scanning is enabled AND
    % we haven't found both circles yet.
    if scan_enabled && ~(circle_state.orange && circle_state.blue)
        chunk_distance = 0.75;
    else
        chunk_distance = Inf;
    end

    if scan_enabled
        % B1->B2 leg only: straight line from current pose to target, no PRM.
        % validate_path inside MoveRobot still checks the segment against the map.
        path = [current_pose(1:2); target];
    else
        path = createPath(current_pose(1:2),target,map);
    end
    path

    [slamAlg,fail,final_pose,mcl_history,circle_scan_needed] = MoveRobot( ...
        path,slamAlg,scanSub,odomSub,cmdPub,figure1,0.8,0.20, ...
        current_pose,mcl_history,chunk_distance);
    current_pose = final_pose;

    refresh_map_and_plot();

    % Recovery: if MoveRobot bailed out (stuck or its path went invalid), the
    % believed pose is likely wrong (e.g. inside a wall). Re-run MCL before
    % replanning so the next path starts from a corrected pose.
    if fail
        'fail detected — running MCL to re-localize'
        current_pose = localize_robot(map, scanSub, odomSub, cmdPub, current_pose);
    end

    if circle_scan_needed
        pre_scan_pose = current_pose;

        % Time to scan for circles. find_circles returns immediately if both
        % already found, so this is cheap once the run is complete.
        [circle_state, post_scan_pose] = find_circles( ...
            cmdPub, current_pose, circle_state, odomSub);
        current_pose = post_scan_pose;

        % Only backtrack if the robot actually moved off the path during the
        % scan (i.e. find_circles did real work). Skip when both colors were
        % already found at entry and the function returned a no-op.
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

        % Loop continues, next iteration replans toward target and drives
        % another 20cm chunk.
        continue;
    end

    if ~fail
        break;
    end
end
close(figure1);

    % --- nested helpers (share workspace with outer function) -----------------
    function refresh_map_and_plot()
        % Map stays at the loaded base map; only the trajectory updates.
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