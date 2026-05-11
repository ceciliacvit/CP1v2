function pose = localize_robot(map, scanSub, odomSub, cmdPub)
% Global relocalization using monteCarloLocalization.
% Spins the robot until the particle filter converges, then returns
% the estimated pose [x, y, theta] in the map (SLAM world) frame.

mcl = monteCarloLocalization;
mcl.UseLidarScan        = true;

% Ask user for initial guess
fig = figure('Name', 'MCL Initialization');
show(map);
title('Click 1) Robot position, 2) A point in front of the robot (Direction)');
[x_clicks, y_clicks] = ginput(2);
close(fig);

x_guess = x_clicks(1);
y_guess = y_clicks(1);
theta_guess = atan2(y_clicks(2) - y_clicks(1), x_clicks(2) - x_clicks(1));

mcl.GlobalLocalization  = false;
mcl.InitialPose         = [x_guess, y_guess, theta_guess];
mcl.InitialCovariance   = diag([0.25, 0.25, 0.25]);

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

% Anchor MCL with an initial scan before moving so the initial pose aligns with the start
scan = receive(scanSub);
try
    lidarScan = rosReadLidarScan(scan);
    lidarScan = removeInvalidData(lidarScan, 'RangeLimits', [0.1, 8]);
    step(mcl, odom_to_pose(odom_msg), lidarScan);
catch
end

cmdMsg = ros2message('geometry_msgs/Twist');

pose = [0, 0, 0];

%% Visualization setup
fig = figure('Name', 'MCL Localization');
ax  = axes(fig);
show(map, 'Parent', ax);
title(ax, 'Localizing...');
drawnow;

t_start = tic;

while true
    % Continuously publish motion commands to prevent robot timeout
    % Move forward/backward slightly in a sine wave while turning
    t = toc(t_start);
    cmdMsg.linear.x = 0.05 * sin(t);
    cmdMsg.angular.z = 0.3;
    send(cmdPub, cmdMsg);

    scan = receive(scanSub);
    try
        lidarScan = rosReadLidarScan(scan);
        lidarScan = removeInvalidData(lidarScan, 'RangeLimits', [0.1, 8]);
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
            title(ax, 'Stopping to finalize pose...');
            drawnow;

            % Stop the robot
            cmdMsg.linear.x = 0;
            cmdMsg.angular.z = 0;
            send(cmdPub, cmdMsg);

            % Wait for physical stop
            pause(1.0);

            % Process a few more scans to get the exact motionless pose
            for i = 1:10
                scan = receive(scanSub);
                odom_msg = odomSub.LatestMessage;
                if isempty(odom_msg)
                    continue;
                end
                curr_odom_pose = odom_to_pose(odom_msg);
                try
                    lidarScan = rosReadLidarScan(scan);
                    lidarScan = removeInvalidData(lidarScan, 'RangeLimits', [0.1, 8]);
                    [~, estimatedPose, ~] = step(mcl, curr_odom_pose, lidarScan);
                catch
                end
            end

            pose = estimatedPose;

            title(ax, sprintf('Converged at [%.2f, %.2f, %.2f rad]', ...
                  pose(1), pose(2), pose(3)));
            drawnow;
            break;
        end
    end
end

close(fig);
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
