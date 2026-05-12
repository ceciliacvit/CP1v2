%% Standalone Camera Viewer
clear all
clc
close all

global camera_image new_image_available
camera_image = [];
new_image_available = false;

%% Set the ROS domain ID for communication
setenv('ROS_DOMAIN_ID', '30');

%% Create a ROS2 node
viewerNode = ros2node('/camera_viewer_node');

%% Subscribe to the camera feed robustly
disp('Connecting to /camera/image_raw/compressed ...');
while true
    try
        camSub = ros2subscriber(viewerNode, '/camera/image_raw/compressed', @cameraCallback, ...
            'Reliability', 'besteffort', 'History', 'keeplast', 'Depth', 1);
        break;
    catch
        pause(1.0);
    end
end
disp('Connected! Close the figure window to stop.');

%% Create a figure and display the loop
fig = figure('Name', 'TurtleBot Camera Feed', 'NumberTitle', 'off');
hImg = [];

while ishandle(fig)
    if new_image_available && ~isempty(camera_image)
        % Grab the latest image and clear the flag
        img = camera_image;
        new_image_available = false;
        
        rotated_image = rot90(img, 2);
        if isempty(hImg) || ~isvalid(hImg)
            hImg = imshow(rotated_image);
        else
            set(hImg, 'CData', rotated_image); % Updating CData prevents flickering
        end
        drawnow limitrate;
    else
        pause(0.01); % Sleep briefly to prevent high CPU usage when waiting
    end
end

%% Callback function
function cameraCallback(message)
    global camera_image new_image_available
    camera_image = rosReadImage(message);
    new_image_available = true;
end