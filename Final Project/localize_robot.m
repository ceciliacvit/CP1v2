function pose = localize_robot(map, scanSub, odomSub, cmdPub)
% Global relocalization using monteCarloLocalization.
% Spins the robot until the particle filter converges, then returns
% the estimated pose [x, y, theta] in the map (SLAM world) frame.

mcl = monteCarloLocalization;
mcl.UseLidarScan        = true;
mcl.GlobalLocalization  = true;
mcl.ParticleLimits      = [500, 5000];
mcl.UpdateThresholds    = [0.05, 0.05, 0.02];
mcl.ResamplingInterval  = 1;
mcl.SensorModel.Map          = map;
mcl.SensorModel.SensorLimits = [0.1, 8];
mcl.SensorModel.NumBeams     = 60;
mcl.MotionModel.Noise        = [0.1 0.1 0.05 0.05];

converge_threshold = 0.08;  % max xy std dev [m] to declare convergence

odom_msg = odomSub.LatestMessage;
while isempty(odom_msg)
    pause(0.05);
    odom_msg = odomSub.LatestMessage;
end

cmdMsg = ros2message('geometry_msgs/Twist');
cmdMsg.linear.x  = 0;
cmdMsg.angular.z = 0.4;
send(cmdPub, cmdMsg);

pose = [0, 0, 0];

%% Visualization setup
fig = figure('Name', 'MCL Localization');
ax  = axes(fig);
show(map, 'Parent', ax);
title(ax, 'Localizing...');
drawnow;

while true
    scan = receive(scanSub);
    try
        lidarScan = rosReadLidarScan(scan);
    catch
        continue;
    end

    odom_msg = odomSub.LatestMessage;
    if isempty(odom_msg)
        continue;
    end
    curr_odom_pose = odom_to_pose(odom_msg);

    [isUpdated, estimatedPose, covariance] = step(mcl, curr_odom_pose, lidarScan);

    if isUpdated
        xy_std = sqrt(trace(covariance(1:2, 1:2)) / 2);

        %% Update plot
        cla(ax);
        show(map, 'Parent', ax);
        hold(ax, 'on');

        % Particle cloud — colour-coded by weight
        try
            pts = mcl.Particles;
            scatter(ax, pts.Poses(:,1), pts.Poses(:,2), 4, ...
                    pts.Weights, 'filled', 'MarkerFaceAlpha', 0.5);
            colormap(ax, 'hot');
        catch
        end

        % Estimated pose: dot + heading arrow
        plot(ax, estimatedPose(1), estimatedPose(2), ...
             'g*', 'MarkerSize', 12, 'LineWidth', 2);
        quiver(ax, estimatedPose(1), estimatedPose(2), ...
               0.3*cos(estimatedPose(3)), 0.3*sin(estimatedPose(3)), ...
               'g', 'LineWidth', 2, 'MaxHeadSize', 1);

        hold(ax, 'off');
        title(ax, sprintf('Localizing — std = %.3f m  (converges at %.3f m)', ...
              xy_std, converge_threshold));
        drawnow limitrate;

        if xy_std < converge_threshold
            pose = estimatedPose;
            title(ax, sprintf('Converged at [%.2f, %.2f, %.2f rad]', ...
                  pose(1), pose(2), pose(3)));
            drawnow;
            break;
        end
    end
end

cmdMsg.angular.z = 0;
send(cmdPub, cmdMsg);
end


function p = odom_to_pose(odom_msg)
    x   = odom_msg.pose.pose.position.x;
    y   = odom_msg.pose.pose.position.y;
    eul = quat2eul([odom_msg.pose.pose.orientation.w, ...
                    odom_msg.pose.pose.orientation.x, ...
                    odom_msg.pose.pose.orientation.y, ...
                    odom_msg.pose.pose.orientation.z]);
    p = [x, y, eul(1)];
end
