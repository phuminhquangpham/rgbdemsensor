% The code below contains 3 functions: sensorCalibration, tracking, and ptcreconstruct

%% SENSORCALIBRATION
% This function is designed to perform sensor calibration by utilizing the given 
% translation and rotation from the tsv file with associated errors, with the 
% checkerboard frames given in the ros bag
% To use this function, simply call: 
% c = sensorCalibration();

% TRACKING: This function takes a matrix of Translations and Rotations
% as an input and uses the first one of each as the origin for the
% camera's movements through the frames of the rosbag.

% This function calculates the relative pose between video frames for 3D
% reconstruction
% To use this function, you need to call the sensorCalibration function first
% Then apply these lines to extract the transformation:
% c.extractRGBD();
% c.computeEMToCameraTransform();
% transform = c.emToCameraTransform
% Now call the function: 
% [camPose, frameSkip, extrinsic] = tracking(transform);

%% PTCRECONSTRUCT
% This function is used for 3D reconstruction from generating 3D pointClouds
% The function inheritS the camera intrinsics with focalLength,
% principalPoint and imageSize (from calib function) and camPose to apply
% the correct transformation when reconstruct 3D pointCloud (from tracking
% function)
% To use this function, you need to call both sensorCalibration function and 
% tracking function first
% c = sensorCalibration();
% c.extractRGBD();
% c.computeEMToCameraTransform();
% transform = c.emToCameraTransform
% [camPose, frameSkip, extrinsic] = tracking(transform);
% focalLength = c.fLength;
% principalPoint = c.pPoint;
% imageSize = size(c.imSize);
% [densePtCloud, sparsePtCloud] = ptcreconstruct(focalLength,principalPoint,imageSize,camPose)
