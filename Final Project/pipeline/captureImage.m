function captured = captureImage()
% Save the latest camera frame, reusing detectCircle's subscriber (global img).
    global img new_image_available %#ok<GVMIS>

    wait_timer = tic;
    while (isempty(img) || ~new_image_available) && toc(wait_timer) < 5
        pause(0.05);
    end
    if isempty(img)
        error('no camera frame received');
    end

    captured = rot90(img, 2);
    filename = ['picture_', char(datetime('now', 'Format', 'yyyyMMdd_HHmmss')), '.png'];
    imwrite(captured, filename);
    fprintf('Picture saved: %s\n', filename);
end
