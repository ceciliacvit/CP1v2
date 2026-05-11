%% Clear workspace, command window, and close all figures
clear all
clc
close all

%% Declare global variables for robot pose and laser scan data
global pose poseOffset scan image

%% Set the ROS domain ID for communication
setenv('ROS_DOMAIN_ID', '30');

%% Display available ROS2 topics (for debug)
ros2 topic list

%% Create a ROS2 node for communication
controlNode = ros2node('/base_station');

%% Define subscribers
odomSub = ros2subscriber(controlNode, '/odom', @odomCallback); % odometry topic
scanSub = ros2subscriber(controlNode, '/scan', @scanCallback, 'Reliability', 'besteffort'); % laser scan topic
imageSub = ros2subscriber(controlNode, '/camera/image_raw/compressed', @imageCallback); % image topic

% Pause to allow ROS subscriptions to initialize
pause(0.5);
    
try
    %% Define publishers
    cmdPub = ros2publisher(controlNode, '/cmd_vel', 'geometry_msgs/Twist');
    
    %% Create figure for TurtleBot's data
    visualise = TurtleBotVisualise();
    
    %% Initialize array for desired positions
    positionDesired = [1; 1];

    %% Calculate offset
    quatOffset = [poseOffset.orientation.x poseOffset.orientation.y poseOffset.orientation.z poseOffset.orientation.w];
    orientationOffset = quat2eul(quatOffset);  % Convert offset quaternion to Euler angles
    headingOffset = orientationOffset(3); % Extract offset heading (yaw)

    %% Calculate transformations for offset
    positionOffset = [poseOffset.position.x; poseOffset.position.y];
    R_W2R = [cos(-headingOffset), -sin(-headingOffset); sin(-headingOffset), cos(-headingOffset)];
    t_R2V = -R_W2R * positionOffset;
    R_R2V = [cos(headingOffset), -sin(headingOffset); sin(headingOffset), cos(headingOffset)]';
    
    %% Infinite loop for real-time visualization, until the figure is closed
    while true
        %% Visialise desired position
        % visualise = updatePositionDesired(visualise, positionDesired);

        %% Get the robot's current position and heading
        position = [pose.position.x; pose.position.y];
        quat = [pose.orientation.x pose.orientation.y pose.orientation.z pose.orientation.w];
        orientation = quat2eul(quat);  % Convert quaternion to Euler angles
        heading = orientation(3); % Extract heading (yaw)

        %% Apply offset
        position = R_R2V * position + t_R2V;
        heading = heading - headingOffset; % Offset heading

        %% Visualise the robot
        % visualise = updatePose(visualise, position, heading);
    
        %% Process and plot laser scan data
        cart = rosReadCartesian(scan);  % Convert scan to Cartesian coordinates
        cart = cart * [cos(heading), -sin(heading); sin(heading), cos(heading)]' + position'; % Transform based on robot position and heading
        % visualise = updateScan(visualise, cart);

        %% Step 4: Color segmentation using HSV
        % Convert the RGB image to HSV color space
        % HSV makes it easier to detect colors based on hue
        hsv_image = rgb2hsv(image);
        
        % Split into separate channels
        hue = hsv_image(:,:,1);        % color type (0-1)
        saturation = hsv_image(:,:,2); % how vivid the color is (0-1)
        brightness = hsv_image(:,:,3); % how bright the pixel is (0-1)
        
        % Define hue ranges for each color we want to detect
        % Red wraps around 0/1 in hue space, so we check both ends
        is_red   = hue < 0.05 | hue > 0.95;
        is_green = hue > 0.25 & hue < 0.45;
        is_blue  = hue > 0.55 & hue < 0.70;
        
        % Create a binary mask: pixel is 1 if it matches any color
        % Also filter out dull/dark pixels using saturation and brightness
        mask = (is_red | is_green | is_blue) & saturation > 0.3 & brightness > 0.3;
        
        %% Step 5: Clean up the mask using morphological operators
        mask = imopen(mask, strel('disk', 3));   % remove small noise spots
        mask = imclose(mask, strel('disk', 50)); % close small gaps
        mask = imfill(mask, 'holes');            % fill holes inside the shape
        
        %% Step 6: Detect circles in the mask, [20 600] is min and max size of circle
        [centers, radii] = imfindcircles(mask, [20 600], 'Sensitivity', 0.99, 'ObjectPolarity', 'bright');
        
        %% Step 7: Get the diameter and identify the color
        [rows, cols, channels] = size(image);
        
        if ~isempty(radii)
            % Calculate diameter from radius
            diameter = 2 * radii(1);
        
            % Figure out which color the circle is by checking
            % how many pixels of each color are inside the detected circle
            [xx, yy] = meshgrid(1:cols, 1:rows);
            inside_circle = (xx - centers(1,1)).^2 + (yy - centers(1,2)).^2 < radii(1)^2;
        
            red_count   = sum(is_red(:)   & inside_circle(:));
            green_count = sum(is_green(:) & inside_circle(:));
            blue_count  = sum(is_blue(:)  & inside_circle(:));
        
            % Pick whichever color has the most pixels
            [~, color_index] = max([red_count, green_count, blue_count]);
            color_names = {'Red', 'Green', 'Blue'};
            %fprintf('Detected: %s circle, diameter: %.1f pixels\n', color_names{color_index}, diameter);
        
            % Exercise 2
            f = 1266;
            H = 0.10;
            D_estimated = f * H / diameter;
            fprintf('Estimated distance: %.2f m\n', D_estimated);

            % Draw a green circle around the detected circle
            annotated = insertShape(image, 'Circle', [centers(1,1), centers(1,2), radii(1)], 'Color', 'green', 'LineWidth', 5);
        else
            % No circle found, just show the raw image
            annotated = image;
        end
        
        %% Visualise the results side by side
        % Create the filtered image by applying the mask to all 3 channels
        mask3 = repmat(mask, [1, 1, 3]);
        filtered_image = zeros(rows, cols, channels, 'uint8');
        filtered_image(mask3) = image(mask3);
        
        % Show annotated image on the left, filtered on the right
        visualise = updateImage(visualise, rot90([annotated, filtered_image], 2));


        %% PID controller for heading
        angularVelocity = 0.0;

        %% PID controller for position
        linearVelocity = 0.0;
    
        %% Publish velocity commands
        cmdMsg = ros2message('geometry_msgs/Twist');
        cmdMsg.linear.x = max(min(linearVelocity, 0.1), -0.1);
        cmdMsg.angular.z = max(min(angularVelocity, 1.0), -1.0);
        % send(cmdPub, cmdMsg);
    
        %% Pause to visualize and delete old plots
        pause(0.1)
    
        %% Exit the loop if the figure is closed
        if size(findobj(visualise.figAvatar)) == 0 | size(findobj(visualise.figImage)) == 0
            ME = MException('NonExeption:EndProgram', 'The program was closed.');
            throw(ME)
        end
    end
catch ME
    % Stop the robot
    cmdMsg = ros2message('geometry_msgs/Twist');
    cmdMsg.Linear.x = 0;
    cmdMsg.Angular.z = 0;
    send(cmdPub, cmdMsg);

    % Close all figures
    close all
    
    % Clean up ROS subscriptions
    clear odomSub scanSub imageSub

    % Show the error
    if ~strcmp(ME.identifier, 'NonExeption:EndProgram')
        rethrow(ME)
    end
end 

% %% Callback functions
function odomCallback(message)
    % Use global variable to store the robot's position and orientation
    global pose poseOffset

    % Extract position and orientation data from the ROS message
    pose = message.pose.pose;

    if isempty(poseOffset)
        poseOffset = message.pose.pose;
    end
end

function scanCallback(message)
    % Use global variable to store laser scan data
    global scan

    % Save the laser scan message
    scan = message;
end

function imageCallback(message)
    % Use global variable to store laser scan data
    global image

    % Save the laser scan message
    image = rosReadImage(message);
end