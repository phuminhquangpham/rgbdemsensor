% TRACKING: This function takes a matrix of Translations and Rotations
% as an input and uses the first one of each as the origin for the
% camera's movements through the frames of the rosbag.

% This function calculates the relative pose between video frames for 3D
% reconstruction

function [camPose, frameSkip, extrinsic] = tracking(transforms)

    
    
    %% Setup 
    % Opens the rosbag and stores the raw images and camera info messages
    modelBag = rosbag('Modelnew1_360.bag');
    images = select(modelBag, 'Topic','/camera/color/image_raw');
    intrinsic = select(modelBag, 'Topic', '/camera/color/camera_info');
    
    % Reads intrinsics and arranges intrinsic parameter matrix
    intrinsicMsg = readMessages(intrinsic, 1);
    intrinsicParams = reshape(intrinsicMsg{1}.K, [3,3]);
    
    % Stores intrinsic information: Principal point, focal length and image
    % size (Height, Width)
    pPoint = [intrinsicParams(3,1), intrinsicParams(3,2)];
    fLength = [intrinsicParams(1,1), intrinsicParams(2,2)];
    imSize = [intrinsicMsg{1}.Height, intrinsicMsg{1}.Width];
    
    % Creates Camera Intrinsic object based on gathered parameters
    CamIntrinsics = cameraIntrinsics(fLength, pPoint, double(imSize));
    
    imgMessage = readMessages(images); % Reads all image frames at once
    imgMessageSize = size(imgMessage); % Gets vector size of the ROS message (should be 1513 x 1)

    % Preallocate memory for output vectors
    frameSkip = zeros(1, imgMessageSize(1));

    % Changes origin to pose in first frame of calibration
    camPose(1) = rigidtform3d(transforms.Rotation(:,:,1),transforms.Translation(1,:));
    
    %% Loop to calculate relative poses between all frames
    for i = 2:imgMessageSize(1) % Goes from 2 to 1513

        % Reads 2 ROS images from message and converts them to grayscale
        imgOrig1 = rgb2gray(readImage(imgMessage{i-1,1}));
        imgOrig2 = rgb2gray(readImage(imgMessage{i,1}));
        
        % Removes any distortion from the images based on calculated
        % intrinsics
        img1 = undistortImage(imgOrig1,CamIntrinsics);
        img2 = undistortImage(imgOrig2,CamIntrinsics);
        
        % Detects any features in the images based on the ORB algorithm and
        % extracts them
        points = detectORBFeatures(img1);
        [features1, validPoints1] = extractFeatures(img1, points);
        points = detectORBFeatures(img2);
        [features2, validPoints2] = extractFeatures(img2, points);

        % Matches same features between frames
        indexPairs = matchFeatures(features1, features2);
        matchedPoints1 = validPoints1(indexPairs(:, 1));
        matchedPoints2 = validPoints2(indexPairs(:, 2));
        
        % Displays initial point matches before removing outliers
        figure(1);
        
        showMatchedFeatures(img1, img2, matchedPoints1, matchedPoints2, 'montage')
        title('Matched Points - All Points');
        % Tries to estimate the fundamental matrix with current matched
        % points while removing outliers, fails when there are < 8 points. 
        % Outputs warning message and exception error/message for debugging purposes.
        
        try
            [fundMat, inliersIndex] = estimateFundamentalMatrix(matchedPoints1, matchedPoints2, Method="RANSAC",...
                                   NumTrials=2000,DistanceThreshold=1e-4);
            
            % Displays inliers of matched points (should be the same
            % features now)
            figure(2);
            
            showMatchedFeatures(img1, img2, matchedPoints1(inliersIndex), matchedPoints2(inliersIndex), 'montage')
            title('Matched Points - Inliers Only');
            % Estimates the relative pose between frames 
            % (how to get from frame 1 to frame 2)
            [relativePose, ~] = estrelpose(fundMat,CamIntrinsics,matchedPoints1(inliersIndex),matchedPoints2(inliersIndex));      
            
            % Splits relative pose information
            relativeOrientation = relativePose.R;
            relativeLocation = relativePose.Translation;

            frameSkip(i) = 0; % Tells point cloud creation function to not skip this frame

        catch ME % Catches any exception that may be thrown by the estimateFundamentalMatrix function
            
            % Displays warning and exception errors 
            warning('Fundamental Matrix not able to be estimated, assigning default values');
            disp(["Image ", i, " doesn't have a fundamental matrix that can be estimated"])
            disp(ME.identifier)
            disp(ME.message)
            
            % Splits default relative pose information 
            % (no rotation and no translation)
            relativeOrientation = eye(3);
            relativeLocation = [0,0,0];

            frameSkip(i) = 1; % Tells point cloud creation function to skip this frame
        end
        
        %% Stores estimated camera pose for each frame
        relPose = rigidtform3d(relativeOrientation,relativeLocation);

        camPose(i) = rigidtform3d(eye(3),[0,0,0]);
        camPose(i).Translation = camPose(i-1).Translation + relPose.Translation;
        % Adds rotations 
        camPose(i).R = quat2rotm(quatmultiply(rotm2quat(camPose(i-1).R), rotm2quat(relPose.R)));

        extrinsic(i) = pose2extr(camPose(i));

        camPose(i).Translation
    end

    %% Plots the relative positions to the camera origin
    figure(3);
    
    
    plotCameraPoses(camPose, imgMessageSize);
    title('Camera Poses');
    
end


% plotCameraPoses: This function plots every camera pose calculated from
% the tracking function
function plotCameraPoses(camPose, messageSize)

    plotCamera(AbsolutePose=camPose(1),Size=1,Color="magenta");

    for i = 2:20:messageSize(1)
        hold on
        plotCamera(AbsolutePose=camPose(i),Size=1,Color="green")
    end

    hold on
    plotCamera(AbsolutePose=camPose(messageSize(1)),Size=1,Color="blue")

    % Sets viewing parameters
    set(gca,CameraUpVector=[0 0 -1]); % inverts z axis (initial camera axis is reversed)
    camorbit(gca,-110,60,data=[0 0 1]);
    axis equal
    grid on

    cameratoolbar('SetMode','orbit');
end
