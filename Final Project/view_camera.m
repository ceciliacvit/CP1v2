%% Standalone Camera Viewer
clear all
clc
close all

global camera_image
camera_image = [];

%% Set the ROS domain ID for communication
setenv('ROS_DOMAIN_ID', '30');

%% Create a ROS2 node
viewerNode = ros2node('/camera_viewer_node');

%% Subscribe to the camera feed robustly
disp('Connecting to /camera/image_raw/compressed ...');
while true
    try
        camSub = ros2subscriber(viewerNode, '/camera/image_raw/compressed', @cameraCallback, ...
            'Reliability', 'besteffort', 'History', 'keeplast');
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
    if ~isempty(camera_image)
        rotated_image = rot90(camera_image, 2);
        if isempty(hImg) || ~isvalid(hImg)
            hImg = imshow(rotated_image);
        else
            set(hImg, 'CData', rotated_image); % Updating CData prevents flickering
        end
        drawnow;
    end
    pause(0.03); % Cap the display refresh rate
end

%% Callback function
function cameraCallback(message)
    global camera_image
    camera_image = rosReadImage(message);
end