function circle = detectCircle()
% access to circle.found (bool), circle.distance (m), circle.angle_error (rad)

    node = ros2node('/camera_detect');
    sub  = ros2subscriber(node, '/camera/image_raw/compressed');
    img  = rosReadImage(receive(sub, 5));

    cols = size(img, 2); % width of the image in pixels
    R = img(:,:,1);
    G = img(:,:,2);
    B = img(:,:,3);

    is_orange = (R > 200) .* (G > 80)  .* (B < 80);
    is_blue = (R < 100) .* (G < 150) .* (B > 150);

    mask = imopen(is_orange | is_blue, strel('disk', 3)); % remove small noise spots
    mask = imclose(mask, strel('disk', 20)); % close small gaps
    mask = imfill(mask, 'holes'); % fill holes inside the shape

    % try imfindellipses if this doesnt work off-angle
    [centers, radii] = imfindcircles(mask, [20 600], 'Sensitivity', 0.99, 'ObjectPolarity', 'bright');

    circle.found = false;
    if isempty(radii)
        return; % no circle found, return
    end

    cx = centers(1,1);
    r  = radii(1);

    circle.found = true;
    circle.angle_error = -atan2(cx - cols/2, 1266); % negative because of flipped input image
    circle.distance = 1266 * 0.10 / (2 * r);
end

% after detecting the circle, we can use the following given the bot's current position (px, py, theta)
% bearing  = theta - circle.angle_error;
% target_x = px + (circle.distance - 1.0) * cos(bearing);
% target_y = py + (circle.distance - 1.0) * sin(bearing);
% which should take us to a point 1m in front of the circle