%% This function is used for 3D reconstruction from generating 3D pointClouds
% The function inherit the camera intrinsics with focalLength,
% principalPoint and imageSize (from calib function) and camPose to apply
% the correct transformation when reconstruct 3D pointCloud (from tracking
% function)
%% To use this function, have these lines before calling this function
% c = sensorCalibration();
% c.extractRGBD();
% c.computeEMToCameraTransform();
% transform = c.emToCameraTransform;
% [camPose, frameSkip, extrinsic] = tracking(transform);
% focalLength = c.fLength;
% principalPoint = c.pPoint;
% imageSize = size(c.imSize);
% [densePtCloud, sparsePtCloud] =
% ptcreconstruct(focalLength,principalPoint,imageSize,camPose);
%%
function [densePtCloud, sparsePtCloud] = ptcreconstruct(focalLength,principalPoint,imageSize,camPose)

% Load the ROS bag files (replace file paths with your actual bag files)
bag2 = rosbag('Modelnew1_360.bag');

% Filter messages for color images and depth images for both bags
color_msgs2 = select(bag2, 'Topic', '/camera/color/image_raw');
color_data2 = readMessages(color_msgs2);
depth_msgs2 = select(bag2, 'Topic', '/camera/depth/image_rect_raw');
depth_data2 = readMessages(depth_msgs2);

% Determine the maximum number of frames in both bags
num_framescolor2 = numel(color_data2);
num_framesdepth2 = numel(depth_data2);

intrinsics = cameraIntrinsics(focalLength, principalPoint, imageSize);
depthScaleFactor = 5;
maxCameraDepth = 5e3;

totalTime = 0;
for i = 1:(num_framescolor2)
    color_msg2 = color_data2{i};
    % Start the timer
    tStart = tic();
    color_image2 = readImage(color_msg2);
    % subplot(2, 3, 1);
    % imshow(color_image2,[0, maxCameraDepth])
    % title(['Color Image (Bag 2), Frame ', num2str(i)]);
    % Calculate and print the elapsed time
    tElapsed = toc(tStart);
    fprintf('Frame %d processing time: %.2f seconds\n', i, tElapsed);

    % Update the cumulative time
    totalTime = totalTime + tElapsed;

    % Print the cumulative time
    fprintf('Cumulative processing time: %.2f seconds\n', totalTime);
end

% Create a cell array to store point clouds in each iteration
ptClouds = cell(0,1);

for i = 1:(num_framesdepth2)
    depth_msg2 = depth_data2{i};
    color_msg2 = color_data2{i};

    % Start the timer
    tStart = tic();

    color_image2 = readImage(color_msg2);
    depth_image2 = readImage(depth_msg2);
    % Remove 1 values from depth_image2 by setting them to NaN
    depth_image2(depth_image2 == 1) = NaN;

    % Check minimum and maximum depth values
    minDepth = min(depth_image2(~isnan(depth_image2)));
    maxDepth = max(depth_image2(~isnan(depth_image2)));

    fprintf('Color Frame %d: Min Depth = %.2f, Max Depth = %.2f\n', i, minDepth, maxDepth);

    % Resize the depth image to match the resolution of the color image
    depth_image2 = imresize(depth_image2, [size(color_image2, 1), size(color_image2, 2)]);

    % Convert depth maps into world points
    % ptCloud = pcfromdepth(depth_image2, depthScaleFactor, intrinsics, 'ColorImage', color_image2,'ImagePoints',imagePoints,'DepthRange', [0, maxCameraDepth]);
    ptCloud = pcfromdepth(depth_image2, depthScaleFactor, intrinsics, 'ColorImage', color_image2,'DepthRange', [0, maxCameraDepth]);

    % Store this frame's point cloud in the cell array
    ptClouds{i} = ptCloud;

    % % Visualize the depth image
    % subplot(2, 3, 4);
    % imshow(depth_image2, [0, maxCameraDepth]);
    % title(['Depth Image (Bag 2), Frame ', num2str(i)]);
    % drawnow;

    % Calculate and print the elapsed time
    tElapsed = toc(tStart);
    fprintf('Depth Frame %d processing time: %.2f seconds\n', i, tElapsed);

    % Update the cumulative time
    totalTime = totalTime + tElapsed;

    % Print the cumulative time
    fprintf('Cumulative processing time: %.2f seconds\n', totalTime);

    pause(0.1);  % Adjust the pause time as needed
    drawnow;  % Force immediate display update
end

% Merge all individual point clouds into a single point cloud
densePtCloud = ptClouds{1};
sparsePtCloud = ptClouds{1};

for i = 2:(num_framesdepth2)
    % Start the timer
    tStart = tic();
    transformedPtCloud = pctransform(ptClouds{i}, affinetform3d(camPose(i).A));
    densePtCloud = pccat(transformedPtCloud);
    sparsePtCloud = pcmerge(sparsePtCloud,transformedPtCloud,0.05);
    % Calculate and print the elapsed time
    tElapsed = toc(tStart);
    fprintf('Merge Frame %d processing time: %.2f seconds\n', i, tElapsed);
    % Update the cumulative time
    totalTime = totalTime + tElapsed;
    % Print the cumulative time
    fprintf('Cumulative processing time: %.2f seconds\n', totalTime);
end

%% Plot figure for Dense Point Cloud
figure;
pcshow(densePtCloud, 'VerticalAxis', 'Y', 'VerticalAxisDir', 'Down', 'ViewPlane', 'YX');
xlim([150 500]);
% ylim([-50 150]);
zlim([-300 150]);
xlabel('X')
ylabel('Y')
zlabel('Z')
title('Dense Point Cloud');

% Plot figure for Sparse Point Cloud
figure;
pcshow(sparsePtCloud, 'VerticalAxis', 'Y', 'VerticalAxisDir', 'Down', 'ViewPlane', 'YX' );
xlabel('X')
ylabel('Y')
zlabel('Z')
title('Sparse Point Cloud');

end
