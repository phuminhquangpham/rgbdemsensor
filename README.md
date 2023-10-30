## The code below contains 3 functions: sensorCalibration, tracking, and ptcreconstruct

### SENSORCALIBRATION
% This function is designed to perform sensor calibration by utilizing the given \
% translation and rotation from the TSV file with associated errors, with the \
% checkerboard frames given in the ROS bag. \
% To use this function, simply call: \
% c = sensorCalibration(); \

### TRACKING
% This function takes a matrix of Translations and Rotations. \
% It uses the first one of each as the origin for the \
% camera's movements through the frames of the ROS bag. \

% This function calculates the relative pose between video frames for 3D \
% reconstruction. \
% To use this function, you need to call the sensorCalibration function first. \
% Then apply these lines to extract the transformation: \
% c.extractRGBD(); \
% c.computeEMToCameraTransform(); \
% transform = c.emToCameraTransform. \
% Now call the function: \
% [camPose, frameSkip, extrinsic] = tracking(transform). \

### PTCRECONSTRUCT
% This function is used for 3D reconstruction from generating 3D pointClouds. \ 
% The function inherits the camera intrinsics with focalLength, \
% principalPoint, and imageSize (from calib function) and camPose to apply. \ 
% the correct transformation when reconstructing 3D pointCloud (from the tracking). \ 
% function. \
% To use this function, you need to call both the sensorCalibration function and \
% the tracking function first. \ 
% c = sensorCalibration(); \ 
% c.extractRGBD(); \ 
% c.computeEMToCameraTransform(); \ 
% transform = c.emToCameraTransform. \ 
% [camPose, frameSkip, extrinsic] = tracking(transform). \ 
% focalLength = c.fLength; \ 
% principalPoint = c.pPoint; \ 
% imageSize = size(c.imSize). \
% [densePtCloud, sparsePtCloud] = ptcreconstruct(focalLength, principalPoint, imageSize, camPose
