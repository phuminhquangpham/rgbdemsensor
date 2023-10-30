%%runner script
clear;
clc;
clf;
RGBD = sensorCalibration(); 
RGBD.calibrationRGBD(); 
RGBD.computeEMTransform();
transform = RGBD.emTransform; 
[camPose, frameSkip, extrinsic] = tracking(transform);
focalLength = RGBD.fLength;
principalPoint = RGBD.pPoint; 
imageSize = size(RGBD.imSize); 
[densePtCloud, sparsePtCloud] = ptcreconstruct(focalLength, principalPoint, imageSize, camPose);
