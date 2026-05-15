function captured = captureImage()
% Save the most recent camera frame. Reuses the persistent subscriber that
% detectCircle already maintains (global img) instead of creating a new
% ROS2 node/subscriber per call — a fresh subscriber frequently misses the
% DDS discovery window and times out in receive().
    global img new_image_available %#ok<GVMIS> shared with detectCircle's imageCallback

    try
        % Wait briefly for a fresh frame from detectCircle's callback.
        % captureImage is always called right after a successful detection,
        % so img is normally already populated; this just avoids a stale or
        % empty frame on the very first use.
        wait_timer = tic;
        while (isempty(img) || ~new_image_available) && toc(wait_timer) < 5
            pause(0.05);
        end

        if isempty(img)
            error('captureImage:NoFrame', ...
                'No camera frame available from detectCircle subscriber.');
        end

        captured = rot90(img, 2); % rotate 180 degrees to correct orientation
        filename = ['picture_', char(datetime('now', 'Format', 'yyyyMMdd_HHmmss')), '.png'];
        imwrite(captured, filename);
        fprintf('Picture saved: %s\n', filename);
    catch ME
        warning('Failed to capture image: %s', ME.message);
        captured = [];
    end
end
