function updated_slam = MoveRobot(path,slamAlg,scanSub,cmdPub,figure1,percentage_threshold,tolerance)
    arguments
        path;
        slamAlg;
        scanSub;
        cmdPub;
        figure1;
        percentage_threshold = 1;
        tolerance = 0.20;
    end

%% for finding position
maxLidarRange = 8;
mapResolution = 20;

%% for navigation

time_previous = tic;

path

path_index = 1;
if(isempty(path))
    return
end
path_size = size(path,1);
desired_position = path(path_index,:);

%% For plots
t0 = tic;

global timeData headingData desiredHeadingData distanceData ax2 ax3

timeData = [];
headingData = [];
desiredHeadingData = [];
distanceData = [];

global ax2 ax3

figure; % or use figure1 if you want everything in same window

ax2 = subplot(2,1,1); % top plot: heading
ax3 = subplot(2,1,2); % bottom plot: distance

hold(ax2, 'on');
hold(ax3, 'on');

%% PID initialization

headingErrorInt = 0;
headingErrorPrev = 0;

distanceErrorInt = 0;
distanceErrorPrev = 0;

% Heading PID gains
Kp_h = 0.8;
Ki_h = 0.2;
Kd_h = 0.2;

% Distance PID gains
Kp_d = 0.2;
Ki_d = 0.0;
Kd_d = 0.0;

% for stopping
drive = true;

while drive

    %% find position and angle
    
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
    
    [scans, optimizedPoses]  = scansAndPoses(slamAlg);
    map = buildMap(scans, optimizedPoses, mapResolution, maxLidarRange);

    position = optimizedPoses(end,1:2);
    desired_position;
    angle = optimizedPoses(end,3);

    plot_all(figure1,map,optimizedPoses,path);

    if (~validate_path([position; path],map))
        "invalid path :(((("
        updated_slam = slamAlg;
        cmdMsg = ros2message('geometry_msgs/Twist');
        cmdMsg.Linear.X = 0;
        cmdMsg.Angular.Z = 0;
        send(cmdPub, cmdMsg);
        return
    end
    
    
    %% setup PID
    position_delta = desired_position - position;
    desired_heading = atan2(position_delta(2),position_delta(1))
    distance_to_target = norm(position_delta)

    dt = toc(time_previous);
    time_previous = tic;

    if dt <= 0
        dt = 0.01;
    end

    headingError = atan2(sin(desired_heading-angle),cos(desired_heading-angle));
    headingErrorInt = headingErrorInt + headingError * dt;
    headingErrorDer = (headingError - headingErrorPrev) / dt;
    headingErrorPrev = headingError;

    distanceError = distance_to_target;
    distanceErrorInt = distanceErrorInt + distanceError * dt;
    distanceErrorDer = (distanceError - distanceErrorPrev) / dt;
    distanceErrorPrev = distanceError;

    plot_error(angle, desired_heading,distance_to_target,t0);

    %% PID

    angularVelocity = Kp_h * headingError ...
                    + Ki_h * headingErrorInt ...
                    + Kd_h * headingErrorDer;

    linearVelocity = Kp_d * distanceError ...
                   + Ki_d * distanceErrorInt ...
                   + Kd_d * distanceErrorDer;

    if abs(headingError) > pi/32
        linearVelocity = 0;
    end

    if distance_to_target < tolerance
        path_index = path_index + 1;

        percentage = path_index /path_size;
        if(~isempty(path) && percentage < percentage_threshold)
            path = path(2:end,:); % matlab plz no, whyyy :(
            desired_position = path(1,:);
            % desired_position = path(path_index,:);
        else
            "finished, no i'm danish"
            drive = false;
        end
    end

    cmdMsg = ros2message('geometry_msgs/Twist');
    cmdMsg.linear.x = clip(linearVelocity, 0.00, 0.4);
    cmdMsg.angular.z = clip(angularVelocity, -1.0, 1.0);

    % in case we really need to skiddadle
    if(abs(linearVelocity) <= 0.1 && abs(angularVelocity) >= 1)
        cmdMsg.linear.x = 0.00;
    end

    speed = cmdMsg.linear.x
    ang_speed = cmdMsg.angular.z

    send(cmdPub, cmdMsg);
end


updated_slam = slamAlg;
end

function plot_error(heading, desiredHeading, distanceToTarget, t0)
    global timeData headingData desiredHeadingData distanceData ax2 ax3

    t = toc(t0);

    % Append data
    timeData(end+1) = t;
    headingData(end+1) = heading;
    desiredHeadingData(end+1) = desiredHeading;
    distanceData(end+1) = distanceToTarget;

    % Plot heading
    plot(ax2, timeData, headingData, 'b', 'LineWidth', 1.5);
    plot(ax2, timeData, desiredHeadingData, 'r--', 'LineWidth', 1.5);

    xlabel(ax2, 'Time [s]');
    ylabel(ax2, 'Heading [rad]');
    legend(ax2, 'Actual', 'Desired');
    grid(ax2, 'on');

    % Plot distance
    plot(ax3, timeData, distanceData, 'k', 'LineWidth', 1.5);

    xlabel(ax3, 'Time [s]');
    ylabel(ax3, 'Distance [m]');
    grid(ax3, 'on');

    drawnow limitrate;
end