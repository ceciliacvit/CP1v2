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

    % Only fire the 20cm circle-scan trigger on legs where scanning is enabled.
    if scan_enabled
        chunk_distance = 0.20;
    else
        chunk_distance = Inf;
    end
'move_to_point started'
maxLidarRange = 8;
mapResolution = 20;
global visualise
visualise=TurtleBotVisualise();
figure1 = figure;

% Starting position: use MCL result when provided (loaded map case),
% otherwise trust the last SLAM pose from exploration.
[scans, optimizedPoses] = scansAndPoses(slamAlg);
base_scans = scans;
base_poses = optimizedPoses;
if ~isempty(initialPose)
    current_pose = initialPose;
    use_slam = false;
else
    current_pose = optimizedPoses(end,:);
    use_slam = true;
end
% Map is static in MCL mode (loaded map is the source of truth) and is
% rebuilt only from slamAlg in SLAM mode.
map = buildMap(base_scans, base_poses, mapResolution, maxLidarRange);

while true
    path = createPath(current_pose(1:2),target,map);
    path

    [slamAlg,fail,final_pose,mcl_history,circle_scan_needed] = MoveRobot( ...
        path,slamAlg,scanSub,odomSub,cmdPub,figure1,0.8,0.20, ...
        current_pose,use_slam,mcl_history,chunk_distance);
    current_pose = final_pose;

    refresh_map_and_plot();

    % Recovery: if MoveRobot bailed out (stuck or its path went invalid), the
    % believed pose is likely wrong (e.g. inside a wall). Re-run MCL before
    % replanning so the next path starts from a corrected pose.
    if fail && ~use_slam
        'fail detected — running MCL to re-localize'
        current_pose = localize_robot(map, scanSub, odomSub, cmdPub, current_pose);
    end

    if circle_scan_needed
        pre_scan_pose = current_pose;

        % Time to scan for circles. find_circles returns immediately if both
        % already found, so this is cheap once the run is complete.
        [circle_state, post_scan_pose] = find_circles( ...
            scanSub, cmdPub, slamAlg, current_pose, circle_state, odomSub);
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
                    current_pose,use_slam,mcl_history);
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
        if use_slam
            % MoveRobot has updated slamAlg — get fresh map from it.
            [scans_local, poses_local] = scansAndPoses(slamAlg);
            map = buildMap(scans_local, poses_local, mapResolution, maxLidarRange);
            plot_all(figure1, map, poses_local);
        else
            % Map stays at the loaded base map; only the trajectory updates.
            if ~isempty(mcl_history.poses)
                plot_all(figure1, map, [base_poses; mcl_history.poses]);
            else
                plot_all(figure1, map, current_pose);
            end
        end
        drawnow;
    end
end


function idx = nearest_path_idx(path, pose)
    d = vecnorm(path - pose(1:2), 2, 2);
    [~, idx] = min(d);
end