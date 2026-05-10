function img = captureImage()
    node = ros2node('/camera_capture');
    sub = ros2subscriber(node, '/camera/image_raw/compressed');
    img = rot90(rosReadImage(receive(sub, 5)), 2); % rotate 180 degrees to correct orientation
    filename = ['picture_', char(datetime('now', 'Format', 'yyyyMMdd_HHmmss')), '.png'];
    imwrite(img, filename);
    fprintf('Picture saved: %s\n', filename);
end
