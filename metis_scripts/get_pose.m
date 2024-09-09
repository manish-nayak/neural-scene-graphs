function [vehBottomCenter,vehBottomCenterEulVkitti] = get_pose(fileNum, pointCloudCam, labels, vehBottom2rearWheelTranslation)

    %% Loading the pixel and point cloud data
    
    pixelCoordsRearWheel = labels.gTruth.LabelData.RearWheel{fileNum,1}{1,1};
    pixelCoordsFrontWheel = labels.gTruth.LabelData.FrontWheel{fileNum,1}{1,1};
    pixelCoordsVehiclePlane = labels.gTruth.LabelData.VehiclePlane{fileNum,1}{1,1};
    % pixelCoordsVehicleXDir = labels.gTruth.LabelData.VehicleXDir{1,1}{1,1};
    pixelCoordsVehicleTopPlane = labels.gTruth.LabelData.VehicleTopPlane{fileNum,1}{1,1};
    pixelCoordsGround = labels.gTruth.LabelData.Ground{fileNum,1}{1,1};
    
    %% Transform from Pixel to Point Cloud Coordinates
    
    imgSize = [1920,1080];
    dispSize = [1920,1058];
    
    % Mask for the point cloud
    ptCloudFiltRearWheel = pixel2pointCloud(pixelCoordsRearWheel,pointCloudCam,dispSize);
    ptCloudFiltFrontWheel = pixel2pointCloud(pixelCoordsFrontWheel,pointCloudCam,dispSize);
    ptCloudFiltVehiclePlane = pixel2pointCloud(pixelCoordsVehiclePlane,pointCloudCam,dispSize);
    % ptCloudFiltVehicleXDir = pixel2pointCloud(pixelCoordsVehicleXDir,pointCloudCam,dispSize);
    ptCloudFiltVehicleTopPlane = pixel2pointCloud(pixelCoordsVehicleTopPlane,pointCloudCam,dispSize);
    ptCloudFiltGround = pixel2pointCloud(pixelCoordsGround,pointCloudCam,dispSize);
    
    
    %% Visualize the filtered point cloud
    
    % plotPointCloud(pointCloudCam,ptCloudFiltRearWheel)
    % plotPointCloud(pointCloudCam,ptCloudFiltFrontWheel)
    % plotPointCloud(pointCloudCam,ptCloudFiltVehiclePlane)
    % plotPointCloud(pointCloudCam,ptCloudFiltVehicleXDir)
    % plotPointCloud(pointCloudCam,ptCloudGround)
    
    %% Find Centre and Normal
    
    [rearWheelCenter,rearWheelNormal] = getSurfaceCenterAndNormal(ptCloudFiltRearWheel);
    [frontWheelCenter,frontWheelNormal] = getSurfaceCenterAndNormal(ptCloudFiltFrontWheel);
    [vehPlaneCenter,vehPlaneNormal] = getSurfaceCenterAndNormal(ptCloudFiltVehiclePlane);
    [vehTopPlaneCenter,vehTopPlaneNormal] = getSurfaceCenterAndNormal(ptCloudFiltVehicleTopPlane);
    [groundCenter, groundNormal] = getSurfaceCenterAndNormal(ptCloudFiltGround);
    
    
    %% Visualize the filtered point cloud and the normal
    
    
    ptCloudFiltCat = [ptCloudFiltRearWheel;ptCloudFiltFrontWheel;...
                 ptCloudFiltVehiclePlane; ptCloudFiltVehicleTopPlane; ptCloudFiltGround];
    
    centerVec = [rearWheelCenter; frontWheelCenter; vehPlaneCenter; vehTopPlaneCenter; groundCenter];
    normalVec = [rearWheelNormal; frontWheelNormal; vehPlaneNormal; vehTopPlaneNormal; groundNormal];
    
    
    % Figure 1 - plot the filtered point cloud and normals
    % plotPointCloud(pointCloudCam,ptCloudFiltCat,centerVec,normalVec)
    
    %% Get Wheel-to-Wheel Vector
    
    rear2frontWheelVec = frontWheelCenter - rearWheelCenter;
    
    % Figure 2 - Plot wheel to wheel vector
    % plotPointCloud(pointCloudCam,ptCloudFiltCat,rearWheelCenter,rear2frontWheelVec)
    % hold on
    % plot3(centerVec(1:2,1), centerVec(1:2,2), centerVec(1:2,3), 'r-')
    
    v1 = frontWheelNormal;
    v2 = rear2frontWheelVec;
%     ThetaInDegrees = getAngle(v1,v2);
    
    %% Get the wheel centred coordinate system
    
    v1_norm = v1/norm(v1);
    refXCam = [1 0 0]; refYCam = [0 1 0]; refZCam = [0 0 1];
    
    if dot(refZCam, v1_norm)<0
        v1_norm = -v1_norm;
    end
    
    % v2 = vehTopPlaneNormal;
    v2 = groundNormal;
    v2_norm = v2/norm(v2);
    
    if dot(refYCam, v2_norm)<0
        v2_norm = -v2_norm;
    end
    
    v3 = cross(v2_norm,v1_norm);
    v3_norm = v3/norm(v3);
    
    csys_center = repmat(rearWheelCenter,3,1);
    csys_vector = [v3_norm; v2_norm; v1_norm];
    
    % Figure 3 - Plot the wheel centred vehicle coordinate system
    % plotPointCloud(pointCloudCam,ptCloudFiltCat,csys_center,csys_vector)
    
    %% Get the vehicle centre coordinate system
    
    R_wheel = [v3_norm; v2_norm; v1_norm]'; % Rotation Matrix in Source Frame
    
    T_wheel = [R_wheel rearWheelCenter'; [0 0 0 1]];
%     rotm2eul(R_wheel)*180/pi
    
    %% Pose of Vehicle Bottom Center
     
    dx = vehBottom2rearWheelTranslation(1); % 1.05 - Along the vehicle length
    dy = vehBottom2rearWheelTranslation(2); % 0.15 - Along the vehicle height
    dz = vehBottom2rearWheelTranslation(3); % 0.3 - Along the vehicle width
    
    wheel_test_cen = rearWheelCenter;
    % R_wheel = [v3_norm; v2_norm; v1_norm];
    
    T_wheel = [R_wheel wheel_test_cen'; [0 0 0 1]];
    csys_center = repmat(wheel_test_cen,3,1);
    csys_vector = T_wheel(1:3,1:3);
    
    T_vehBottom2WheelCenter = [eye(3) [dx dy dz]'; [0 0 0 1]];
    T_vehBottom2Cam = T_wheel*T_vehBottom2WheelCenter;
    
    vehBottomCenter = T_vehBottom2Cam(1:3,4);
    csys_vehBottomCenterVec = T_vehBottom2Cam(1:3,1:3);
    csys_vehBottomCenter = repmat(vehBottomCenter',3,1);
    
    vehBottomCenterEul = rotm2eul(csys_vehBottomCenterVec);
    vehBottomCenterEulVkitti = [vehBottomCenterEul(2) vehBottomCenterEul(3) vehBottomCenterEul(1)];
    
    % Figure 4 - Plot the vehicle bottom coordinate system
%     plotPointCloud(pointCloudCam,ptCloudFiltCat,[csys_center;csys_vehBottomCenter],...
%         [csys_vector';csys_vehBottomCenterVec'])

end