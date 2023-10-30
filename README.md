## 3D reconstruction using RGB-D camera and EM sensor
This project was carried out by 3 members, with their individual contributions 

+ **Phu Minh Quang Pham**  - 14231688   **(33%)**
+ **Rhys Darcy**           - 24447183   **(34%**) 
+ **Alex Cacciola**        - 24569826   **(33%)**

**1 CLASS**: sensorCalibration (carried out by Alex) \
**2 FUNCTIONS**: tracking (carried out by Rhys) and ptcreconstruct (carried out by Phu)

<hr>

### [SENSORCALIBRATION CLASS](https://github.com/phuminhquangpham/rgbdemsensor/blob/main/sensorCalibration.m) 
+ [**sensorCalibration**](https://github.com/phuminhquangpham/rgbdemsensor/blob/main/sensorCalibration.m#L27) \
This function is designed for sensor calibration, specifically for calibrating a camera sensor \
This function takes the translation and Rotation from the tsv datafile, extract the camera intrinsics parameters from the rosbag 

+ **Code execution**: \
  RGBD = sensorCalibration(); 

+ [**calibrationRGBD**](https://github.com/phuminhquangpham/rgbdemsensor/blob/main/sensorCalibration.m#L59) \
This function is used for camera calibration, processing a sequence of frames (images), detecting checkerboard patterns, \
estimate the camera's extrinsic parameters (rotation and translation) based on the detected points \
The function stores the obtained parameters for further calibration analysis 

+ **Code execution**: \
  RGBD.calibrationRGBD(); 

+ [**computeEMTransform**](https://github.com/phuminhquangpham/rgbdemsensor/blob/main/sensorCalibration.m#L59) 

This function computes the transformation from EM sensor's coordinate system to the camera coordinate's systen. \
This can be achieved by chaining together the transformation involving the checkerboard as an intermediate reference 

+ **Code execution**: \
  RGBD.computeEMTransform();

<hr> 

### [TRACKING FUNCTION](https://github.com/phuminhquangpham/rgbdemsensor/blob/main/tracking.m)

This function takes a matrix of Translations and Rotations. \
It uses the first one of each as the origin for the \
camera's movements through the frames of the ROS bag. 

This function calculates the relative pose between video frames for 3D \
reconstruction. 

+ **Code execution**: \
  transform = RGBD.emTransform \
  [camPose, frameSkip, extrinsic] = tracking(transform)

<hr>

### [PTCRECONSTRUCT FUNCTION](https://github.com/phuminhquangpham/rgbdemsensor/blob/main/ptcreconstruct.m)
This function is used for 3D reconstruction from generating 3D pointClouds. \
The function inherits the camera intrinsics with focalLength, principalPoint, and imageSize (from calib function) and camPose to apply. \
the correct transformation when reconstructing 3D pointCloud (from the tracking function). 

+ **Code execution**: \
  focalLength = RGBD.fLength;\
  principalPoint = RGBD.pPoint; \
  imageSize = size(RGBD.imSize); \
  [densePtCloud, sparsePtCloud] = ptcreconstruct(focalLength, principalPoint, imageSize, camPose)
