function captureImage(frame)
    frame = rot90(frame, 2);
    filename = ['picture_', char(datetime('now', 'Format', 'yyyyMMdd_HHmmss')), '.png'];
    imwrite(frame, filename);
    fprintf('Picture saved: %s\n', filename);
end
