%% This class calibrates the rgbd camera and creates a transform from the 
% EM Sensor to the Camera. It reads the data from the calibration ROS and a 
% .tsv file to create an object that will be used to perform the
% aforementioned calibration. 
classdef sensorCalibration < handle
    %initilise properties for the class
    properties(SetObservable = true)
        tsvData;
        bag;
        bagSel;
        intrinsicParams;
        imSize;
        intrinsic;
        pPoint;
        fLength;
        translation;
        rotation;
        numImage;
        image;
        iPointsCell; 
        wPointsCell; 
        camParam;
        idx;
        emTransform;
    end

    methods
        %% Initilise the object: sensor
        function sensor = sensorCalibration()
            clc;
            clf;
            
            sensor.tsvData = readtable('forCali_GroundTruth_new360.tsv', 'FileType', 'text', 'Delimiter', '\t', 'ReadVariableNames', true); %Rz	Ry	Rx	Tx	Ty	Tz
            sensor.bag = rosbag('D:\Sensors\CalibNew_360.bag'); 
            intrinsic = select(sensor.bag, 'Topic', '/camera/color/camera_info');
            intrinsicMsg = readMessages(intrinsic, 1); %reads the intrinsic of the camera from ros
            sensor.intrinsicParams = reshape(intrinsicMsg{1}.K, [3,3]); %adjusts the intrinsics into the correct format
            sensor.pPoint = [sensor.intrinsicParams(3,1), sensor.intrinsicParams(3,2)]; %Principle point
            sensor.fLength = [sensor.intrinsicParams(1,1), sensor.intrinsicParams(2,2)]; %Focal Length
            sensor.imSize = [intrinsicMsg{1}.Width, intrinsicMsg{1}.Height]; %Size of the image
            sensor.intrinsic = cameraIntrinsics(sensor.fLength, sensor.pPoint, double(sensor.imSize)); % calulates the intrinsics of the camera
            sensor.bagSel = select(sensor.bag, 'Topic', '/camera/color/image_raw');
            sensor.image = readMessages(sensor.bagSel);
            sensor.numImage = length(sensor.image); 
            sensor.translation = zeros(sensor.numImage, 3); %matrix for translation s
            sensor.rotation = cell(sensor.numImage, 1); %cell matrices for rotations
            sensor.idx = 1; 
            for k = 1:sensor.numImage %ensure rotation is of correct format
                sensor.rotation{k} = NaN(3,3);
            end
            sensor.camParam; 
            sensor.iPointsCell = cell(sensor.numImage, 1); %cell for image points
            sensor.wPointsCell = cell(sensor.numImage, 1); %cell for world points
        
        end
        %% This function calibrates the RGBD camera by reading each image
        % and generating the checkerboard points for each. It then compares
        % these points to the world points and generates the camera
        % extrinsics. Finally, the code will calculate the camera parameters
        % for later use. 
        function calibrationRGBD(sensor)
        
            sensor.iPointsCell = cell(sensor.numImage, 1);%cell for image points
            sensor.wPointsCell = cell(sensor.numImage, 1);%cell for world points
            
            % Assuming a checkerboard pattern of size [8, 11] and square size of 0.015 meters
            checkerboardSize = [8, 11];
            squareSize = 0.015;
            % Begin calibration
            for i = 1:sensor.numImage
                I1 = readImage(sensor.image{i});
                I1_G = rgb2gray(I1);
                [iPoints, ~] = detectCheckerboardPoints(I1_G);
            
                if isempty(iPoints) || ~all(isfinite(iPoints(:)))
                    % Handle cases where the checkerboard is invalid
                    disp('invalid');
                    continue;
                end
                         
                imshow(I1);
                hold on;
                plot(iPoints(:, 1), iPoints(:, 2), 'ro', 'MarkerSize', 10);
                hold off;
               
                % Generate checkerboard world points
                wPoints = generateCheckerboardPoints(checkerboardSize, squareSize);
                %generate extrinsic data for camera
                [r, t] = extrinsics(iPoints, wPoints, sensor.intrinsic);
                %store extrinsic data
                sensor.translation(i,:) = t;
                sensor.rotation{i} = r;
                
              
                % Store iPoints and wPoints in the object
                sensor.iPointsCell{i} = iPoints;
                sensor.wPointsCell{i} = wPoints;
               
                sensor.idx = sensor.idx + 1;
                disp(sensor.idx);
                
                drawnow;
                pause(0);
                   
                  
            end
            %Indicates which stored data contrains NaN or otherwise
            %incorrectly formated data to ignore for later use
            validIndices = ~cellfun(@(x) any(isnan(x(:))), sensor.rotation) & ~all(sensor.translation == 0, 2);
            %store valid data
            sensor.rotation = sensor.rotation(validIndices);
            sensor.translation = sensor.translation(validIndices, :);
            allIPoints = cat(3, sensor.iPointsCell{validIndices});
            allWPoints = sensor.wPointsCell{1}; 
            sensor.camParam = estimateCameraParameters(allIPoints, allWPoints, 'ImageSize', double(sensor.imSize));
            
            %Displays calibration data
            figure;
            showExtrinsics(sensor.camParam);
            drawnow();
            figure; 
            imshow(readImage(sensor.image{1})); 
            hold on;
            plot(allIPoints(:,1,1), allIPoints(:,2,1),'go');
            plot(sensor.camParam.ReprojectedPoints(:,1,1),sensor.camParam.ReprojectedPoints(:,2,1),'r+');
            legend('Detected Points','ReprojectedPoints');
            hold off;
            showReprojectionErrors(sensor.camParam);
        
        end
        %% This function computes the transfrom from the EM to the RGBD camera
        % using the extrinsics from the calibration using the chain rule
        % and outputs them as a structure within the class. 
        function computeEMTransform(sensor)
    
            % Transformation from checkerboard to camera
            T_checkerboard_camera = sensor.translation(:); % Ensure column vector
        
            % Extract rotation and translation from sensor.tsvData
            all_rotations = eul2rotm([sensor.tsvData.Rz, sensor.tsvData.Ry, sensor.tsvData.Rx], 'ZYX');
            all_translations = [sensor.tsvData.Tx, sensor.tsvData.Ty, sensor.tsvData.Tz];
        
            % Initialize storage for computed transformations
            R_em_camera_all = zeros(3, 3, 70);
            T_em_camera_all = zeros(70, 3);
        
           for i = 1:70
                R_checkerboard_camera = sensor.rotation{i};
                R_em_checkerboard = squeeze(all_rotations(:,:,i));
                
                T_checkerboard_camera = sensor.translation(i, :).'; % Fetch the ith translation
                T_em_checkerboard = all_translations(i, :).'; % Ensure column vector
            
                % Compute the transformation from EM to Camera using the chain rule
                R_em_camera = R_checkerboard_camera * R_em_checkerboard;
                T_em_temp = R_checkerboard_camera * T_em_checkerboard;
                T_em_camera = T_checkerboard_camera + T_em_temp;
            
                R_em_camera_all(:,:,i) = R_em_camera;
                T_em_camera_all(i,:) = T_em_camera.';
            end
        
            % Store the computed transformations
            sensor.emTransform = struct('Rotation', R_em_camera_all, 'Translation', T_em_camera_all);
             
        end
    end
end
%% End Code
            
        

 
        
