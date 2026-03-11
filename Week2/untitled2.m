%% Clear workspace, command window, and close all figures
clear all
clc
close all

%% Declare global variables for robot pose and laser scan data
global pose scan
pose = [];
scan = [];

%% Set the ROS domain ID for communication
setenv('ROS_DOMAIN_ID', '30');

%% Display available ROS2 topics (for debug)
ros2 topic list

%% Create a ROS2 node for communication
controlNode = ros2node('/base_station');

%% Subscribe to the odometry topic and set up a callback to process incoming messages
% (set best-effort to reduce dropped-message issues)
odomSub = ros2subscriber(controlNode, '/odom', @odomCallback, 'Reliability', 'besteffort');

% Callback function for processing odometry messages
function odomCallback(message)
    % Use global variable to store the robot's position and orientation
    global pose

    % Extract position and orientation data from the ROS message
    pose = message.pose.pose;
end

%% Subscribe to the laser scan topic and set up a callback with best-effort reliability
scanSub = ros2subscriber(controlNode, '/scan', @scanCallback, 'Reliability', 'besteffort');

% Callback function for processing laser scan messages
function scanCallback(message)
    % Use global variable to store laser scan data
    global scan

    % Save the laser scan message
    scan = message;
end

% Pause to allow ROS subscriptions to initialize
pause(0.1)

%% Set up a figure for visualization
f = figure('units', 'normalized');
hold on
grid on
axis equal
xlim([-5 5]);  % Define x-axis limits
ylim([-5 5]);  % Define y-axis limits

% Configure plot appearance
set(gca, 'fontsize', 10);
xlabel('$x$ [m]', 'interpreter', 'latex', 'fontsize', 10);
ylabel('$y$ [m]', 'interpreter', 'latex', 'fontsize', 10);
set(gca, 'TickLabelInterpreter', 'latex')

% Define scaling factor for visualization
scale = 2;

%% Initialize arrays for storing past and desired positions (if needed)
position_past = [];
position_desired = [];

%% Define robot dimensions for visualization
TB_width = 0.14;        % Robot body width
wheel_width = 0.02;     % Wheel width
wheel_radius = 0.03;    % Wheel radius

% Define the robot's body shape and orientation arrow
robotBody = scale * ...
    [TB_width/2,               TB_width/2,                TB_width/2 - 2*wheel_radius, TB_width/2 - 2*wheel_radius, -TB_width/2, -TB_width/2, TB_width/2 - 2*wheel_radius, TB_width/2 - 2*wheel_radius;
     TB_width/2 + wheel_width, -TB_width/2 - wheel_width, -TB_width/2 - wheel_width,   -TB_width/2,                 -TB_width/2, TB_width/2,  TB_width/2,                  TB_width/2 + wheel_width];
arrow = scale * ...
    [0,    0.08, 0.08, 0.1, 0.08,  0.08,  0;
     0.01, 0.01, 0.02, 0,   -0.02, -0.01, -0.01];

%% Infinite loop for real-time visualization, until the figure is closed
while true
    %% Wait until we have received data
    if isempty(pose) || isempty(scan)
        pause(0.05);
        continue;
    end

    %% Safely read pose + scan (skip frame if a message was dropped/invalid)
    try
        % (i) extract robot position and orientation from pose message
        posMsg = rosReadField(pose, "position");
        oriMsg = rosReadField(pose, "orientation");

        position = [rosReadField(posMsg,"x"); rosReadField(posMsg,"y")];

        qw = rosReadField(oriMsg,"w");
        qx = rosReadField(oriMsg,"x");
        qy = rosReadField(oriMsg,"y");
        qz = rosReadField(oriMsg,"z");
        quat = [qw qx qy qz];

        % (ii) convert orientation from quaternion to Euler angles
        orientation = quat2eul(quat);

        % (iii) extract heading from Euler angles (yaw)
        heading = orientation(1);

        % Read scan safely
        cart = rosReadCartesian(scan);

    catch
        pause(0.01);
        continue;
    end

    %% Transform the robot's body and axes
    % (iv) rotate and translate robot body and body axes using position + heading
    R = [cos(heading) -sin(heading); sin(heading) cos(heading)];

    robotBodyTransformed = R * robotBody + position;
    axisX = R * arrow + position;

    Ry = [cos(heading + pi/2) -sin(heading + pi/2); sin(heading + pi/2) cos(heading + pi/2)];
    axisY = Ry * arrow + position;

    %% Plot the robot's body and axes
    h_robotBody = patch(robotBodyTransformed(1, :), robotBodyTransformed(2, :), 'black', 'EdgeColor', 'none');
    h_axisX = patch(axisX(1, :), axisX(2, :), 'red', 'EdgeColor', 'none');
    h_axisY = patch(axisY(1, :), axisY(2, :), 'green', 'EdgeColor', 'none');
    h_axisZ = rectangle('Position', [position(1), position(2), 0, 0] + scale * [-0.01, -0.01, 0.02, 0.02], 'Curvature', [1 1], 'FaceColor', 'b', 'EdgeColor', 'none');

    %% Process and plot laser scan data
    % (v) transform Cartesian coordinates based on robot position and heading
    cartXY = cart(:,1:2);
    cart = (R * cartXY')' + position.';

    h_cart = scatter(cart(:,1), cart(:,2), 10, 'blue', 'filled');  % Plot laser points

    %% Pause to visualize and delete old plots
    pause(0.1)

    delete(h_robotBody);
    delete(h_axisX);
    delete(h_axisY);
    delete(h_axisZ);
    delete(h_cart);

    %% Exit the loop if the figure is closed
    if size(findobj(f)) == 0
        break;
    end
end

%% Clean up ROS subscriptions
clear odomSub scanSub
