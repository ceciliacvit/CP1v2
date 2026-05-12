function circle = detectCircle(sensitivity)
arguments
    sensitivity = 0.8;
end
% circle.found (bool), circle.distance (m), circle.angle_error (rad)

    global img
    persistent controlNode sub figHandle

    %controlNode = ros2node('/base_station');

    if isempty(controlNode)
        img = [];
        controlNode = ros2node('/base_station');
        while true
            try
                sub = ros2subscriber(controlNode, '/camera/image_raw/compressed', @imageCallback, ...
                    'Reliability', 'besteffort');
                break;
            catch
                disp('Waiting for /camera/image_raw/compressed topic...');
                pause(1.0);
            end
        end
        figHandle = figure;
    end

    img = [];  % clear stale image, wait for fresh frame
    while isempty(img)
        pause(0.01);
    end

    cols = size(img, 2);
    hsv = rgb2hsv(img);
    hue = hsv(:,:,1);
    sat = hsv(:,:,2);
    val = hsv(:,:,3);

    is_orange = (hue > 0.02 & hue < 0.13) & sat > 0.5 & val > 0.1;
    is_blue   = (hue > 0.55 & hue < 0.70) & sat > 0.5 & val > 0.1;

    mask = imopen(is_orange | is_blue, strel('disk', 3));
    mask = imclose(mask, strel('disk', 20));
    mask = imfill(mask, 'holes');

    [centers, radii] = imfindcircles(mask, [25 600], 'Sensitivity',sensitivity, 'ObjectPolarity', 'bright');

    circle.found = false;
    circle.color = "";
    if ~isempty(radii)
        cx = round(centers(1,1));
        cy = round(centers(1,2));
        if (is_orange(cy,cx))
            circle.color = "orange";
        elseif (is_blue(cy,cx))
            circle.color = "blue";
        end
        circle.found = true;
        circle.angle_error = -atan2(centers(1,1) - cols/2, 1266);
        circle.distance    = 1266 * 0.08 / (2 * radii(1));
        annotated = insertShape(img, 'Circle', [centers(1,1), centers(1,2), radii(1)], 'Color', 'green', 'LineWidth', 5);
    else
        annotated = img;
    end

    if isempty(figHandle) || ~isvalid(figHandle)
        figHandle = figure;
    end
    figure(figHandle);
    imshow(rot90(annotated, 2));
    drawnow;
end


% after detecting the circle, we can use the following given the bot's current position (px, py, theta)
% bearing  = theta - circle.angle_error;
% target_x = px + (circle.distance - 1.0) * cos(bearing);
% target_y = py + (circle.distance - 1.0) * sin(bearing);
% which should take us to a point 1m in front of the circle

function imageCallback(message)
    global img
    img = rosReadImage(message);
end
