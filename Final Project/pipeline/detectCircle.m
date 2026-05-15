function circle = detectCircle(sensitivity, ignore_orange, ignore_blue)
% Detect an orange/blue circle in the camera frame.
% Returns circle.found, circle.color, circle.distance (m), circle.angle_error (rad).
arguments
    sensitivity = 0.8;
    ignore_orange = false;
    ignore_blue = false;
end

    global img new_image_available
    persistent controlNode sub figHandle hImg

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

    new_image_available = false;
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

            if (detected_color == "orange" && ~ignore_orange) || (detected_color == "blue" && ~ignore_blue)
                circle.color = detected_color;
                circle.found = true;
                % 1266 = camera focal length in px, 0.1 = circle diameter in m
                circle.angle_error = -atan2(centers(i,1) - cols/2, 1266);
                circle.distance    = 1266 * 0.1 / (2 * radii(i));
                annotated = insertShape(current_img, 'Circle', [centers(i,1), centers(i,2), radii(i)], 'Color', 'green', 'LineWidth', 5);
                break;
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


function imageCallback(message)
    global img new_image_available
    img = rosReadImage(message);
    new_image_available = true;
end
