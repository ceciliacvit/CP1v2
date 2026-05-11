scan = lidarScan(1:10, linspace(0, pi, 10));
scan = removeInvalidData(scan, 'RangeLimits', [0.1, 8]);
disp(scan.Count)
