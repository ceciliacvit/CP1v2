%exampleHelperROSLoadMessages Load specialized message data 
%   This script will load messages that were recorded from various robotics
%   sensors.
%
%   See also ROSSpecializedMessagesExample.

%   Copyright 2014-2015 The MathWorks, Inc.

% Load laser scan data
scancell = load('scandata.mat');
scan = scancell.scan;

% Load uncompressed image data
imgcell = load('imgdata.mat');
img = imgcell.img;

% Load compressed image data
imgcompcell = load('imgcompressed.mat');
imgcomp = imgcompcell.imgcomp;

% Load point cloud data
pointcloudcell = load('pointcloud2.mat');
ptcloud = pointcloudcell.ptcloud;

% Clear variables from the workspace that are not needed anymore
clear scancell
clear imgcell
clear imgcompcell
clear pointcloudcell