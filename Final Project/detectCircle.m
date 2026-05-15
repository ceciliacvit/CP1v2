function circle = detectCircle(sensitivity, ignore_orange, ignore_blue)
arguments
    sensitivity = 0.8;
    ignore_orange = false;
    ignore_blue = false;
end
% circle.found (bool), circle.distance (m), circle.angle_error (rad)

    global img new_image_available
    persistent controlNode sub figHandle hImg

    %controlNode = ros2node('/base_station');

    if isempty(controlNode)
        img = [];
        new_image_available = false;
        controlNode = ros2node('/base_station');
        while true
            try
                sub = ros2subscriber(controlNode, '/camera/image_raw/compressed', @imageCallback, ...
                    'Reliability', 'besteffort', 'History', 'keeplast', 'Depth', 1);
                break;
            catch
                disp('Waiting for /camera/image_raw/compressed topic...');
                pause(1.0);
            end
        end
        figHandle = figure;
        hImg = [];
    end

    new_image_available = false;  % wait for fresh frame
    while ~new_image_available || isempty(img)
        pause(0.01);
    end

    current_img = img;

    cols = size(current_img, 2);
    hsv = rgb2hsv(current_img);
    hue = hsv(:,:,1);
    sat = hsv(:,:,2);
    val = hsv(:,:,3);

    is_orange = (hue > 0.00 & hue < 0.15) & sat > 0.5 & val > 0.1;
    is_blue   = (hue > 0.55 & hue < 0.70) & sat > 0.5 & val > 0.1;

    mask = imopen(is_orange | is_blue, strel('disk', 3));
    mask = imclose(mask, strel('disk', 20));
    mask = imfill(mask, 'holes');

    [centers, radii] = imfindcircles(mask, [25 600], 'Sensitivity',sensitivity, 'ObjectPolarity', 'bright');

    circle.found = false;
    circle.color = "";
    annotated = current_img;

    if ~isempty(radii)
        rows = size(current_img, 1);
        [xx, yy] = meshgrid(1:cols, 1:rows);
        
        for i = 1:length(radii)
            inside_circle = (xx - centers(i,1)).^2 + (yy - centers(i,2)).^2 < radii(i)^2;
            
            orange_count = sum(is_orange(:) & inside_circle(:));
            blue_count   = sum(is_blue(:)   & inside_circle(:));
            
            detected_color = "";
            if orange_count > blue_count && orange_count > 0
                detected_color = "orange";
            elseif blue_count > orange_count && blue_count > 0
                detected_color = "blue";
            end
            
            % If we found a color, and it's not one we're ignoring, lock onto it
            if (detected_color == "orange" && ~ignore_orange) || (detected_color == "blue" && ~ignore_blue)
                circle.color = detected_color;
                circle.found = true;
                circle.angle_error = -atan2(centers(i,1) - cols/2, 1266);
                circle.distance    = 1266 * 0.1 / (2 * radii(i));
                annotated = insertShape(current_img, 'Circle', [centers(i,1), centers(i,2), radii(i)], 'Color', 'green', 'LineWidth', 5);
                break; % Stop checking circles once we found a valid one in this frame
            end
        end
    end

    if isempty(figHandle) || ~isvalid(figHandle)
        figHandle = figure;
        hImg = [];
    end
    
    rotated_image = rot90(annotated, 2);
    if isempty(hImg) || ~isvalid(hImg)
        figure(figHandle);
        hImg = imshow(rotated_image);
    else
        set(hImg, 'CData', rotated_image);
    end
    drawnow limitrate;
end


% after detecting the circle, we can use the following given the bot's current position (px, py, theta)
% bearing  = theta - circle.angle_error;
% target_x = px + (circle.distance - 1.0) * cos(bearing);
% target_y = py + (circle.distance - 1.0) * sin(bearing);
% which should take us to a point 1m in front of the circle

function imageCallback(message)
    global img new_image_available
    img = rosReadImage(message);
    new_image_available = true;
end
