classdef sensorCalibration < handle
    %initilise objects
    properties(SetObservable = true)
        tsvData;
        bag;
        positionEM = [0 0 0];
        orientationEM = [0 0 0];
        bagSel;
        intrinsicParams;
        imSize;
        intrinsicM;
        pPoint;
        fLength;
        translation;
        rotation;
        numM;
        M;
        iPointsCell; 
        wPointsCell; 
        camParam;
        idx;
        emToCameraTransform;
    end

    methods
        
        function sensor = sensorCalibration()
            clc;
            clf;
            

            sensor.tsvData = readtable('forCali_GroundTruth_new360.tsv', 'FileType', 'text', 'Delimiter', '\t', 'ReadVariableNames', true); %Rz	Ry	Rx	Tx	Ty	Tz
            % sensor.tsvData = readtable('forCali_GroundTruth.tsv', 'FileType', 'text', 'Delimiter', '\t', 'ReadVariableNames', true);
            sensor.bag = rosbag('D:\Sensors\CalibNew_360.bag');
            % sensor.bag = rosbag('D:\Sensors\forCali.bag');
            intrinsic = select(sensor.bag, 'Topic', '/camera/color/camera_info');
            intrinsicMsg = readMessages(intrinsic, 1);
            sensor.intrinsicParams = reshape(intrinsicMsg{1}.K, [3,3]);
            sensor.pPoint = [sensor.intrinsicParams(3,1), sensor.intrinsicParams(3,2)];
            sensor.fLength = [sensor.intrinsicParams(1,1), sensor.intrinsicParams(2,2)];
            sensor.imSize = [intrinsicMsg{1}.Width, intrinsicMsg{1}.Height];
            sensor.intrinsicM = cameraIntrinsics(sensor.fLength, sensor.pPoint, double(sensor.imSize));
            sensor.bagSel = select(sensor.bag, 'Topic', '/camera/color/image_raw');
            sensor.M = readMessages(sensor.bagSel);
            sensor.numM = length(sensor.M);
            sensor.translation = zeros(sensor.numM, 3);
            sensor.rotation = cell(sensor.numM, 1);
            sensor.idx = 1;
            for k = 1:sensor.numM
                sensor.rotation{k} = NaN(3,3);
            end
          
            sensor.camParam;
       
            sensor.iPointsCell = cell(sensor.numM, 1);
            sensor.wPointsCell = cell(sensor.numM, 1);
        end

        function extractRGBD(sensor)
           
            sensor.iPointsCell = cell(sensor.numM, 1);
            sensor.wPointsCell = cell(sensor.numM, 1);

            % Assuming a checkerboard pattern of size [8, 11] and square size of 0.015 meters
            checkerboardSize = [8, 11];
            squareSize = 0.015;

            for i = 1:sensor.numM
                I1 = readImage(sensor.M{i});
                I1_G = rgb2gray(I1);
                [iPoints, ~] = detectCheckerboardPoints(I1_G);

                if isempty(iPoints) || ~all(isfinite(iPoints(:)))
                    % Handle cases where the checkerboard is invalid
                    disp('invalid');
                    continue;
                end

                % Extract timestamp information from ROS message
                sec = sensor.M{i}.Header.Stamp.Sec;
                nsec = sensor.M{i}.Header.Stamp.Nsec;
                time = sec + nsec * 1e-9;

             
                imshow(I1);
                hold on;
                plot(iPoints(:, 1), iPoints(:, 2), 'ro', 'MarkerSize', 10);
                hold off;
                % fprintf('Specific Timestamp: %.4f\n', tsvTime(sensor.idx));
                fprintf('ROS Timestamp: %.4f\n', time);

                % Generate checkerboard world points
                wPoints = generateCheckerboardPoints(checkerboardSize, squareSize);

                [r, t] = extrinsics(iPoints, wPoints, sensor.intrinsicM);
                
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
           validIndices = ~cellfun(@(x) any(isnan(x(:))), sensor.rotation) & ~all(sensor.translation == 0, 2);
          
           sensor.rotation = sensor.rotation(validIndices);
           sensor.translation = sensor.translation(validIndices, :);
           allIPoints = cat(3, sensor.iPointsCell{validIndices});
           allWPoints = sensor.wPointsCell{1}; 
           sensor.camParam = estimateCameraParameters(allIPoints, allWPoints, 'ImageSize', double(sensor.imSize));
            
           % disp(sensor.camParam);
           figure;
           showExtrinsics(sensor.camParam);
           drawnow();
           figure; 
           imshow(readImage(sensor.M{1})); 
           hold on;
           plot(allIPoints(:,1,1), allIPoints(:,2,1),'go');
           plot(sensor.camParam.ReprojectedPoints(:,1,1),sensor.camParam.ReprojectedPoints(:,2,1),'r+');
           legend('Detected Points','ReprojectedPoints');
           hold off;
           showReprojectionErrors(sensor.camParam);
           
        end
        function computeEMToCameraTransform(sensor)

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
            sensor.emToCameraTransform = struct('Rotation', R_em_camera_all, 'Translation', T_em_camera_all);
            
        end
    end
end
            
        

 
        
