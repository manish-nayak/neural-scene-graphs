function pose_arr = get_pose_arr(fileNumBegin, fileNumEnd)

    pose_arr = zeros((fileNumEnd-fileNumBegin+1)*2,19);
    labels = load('label_0_4421_4432_4.mat');    
    dx = 1.05; dy = 0.15; dz = 0.3;
    L_obj = 2.8; H_obj = 2.7; W_obj = 1;
    fileNumStart = 4421;


    for fileNum=fileNumBegin:fileNumEnd
        %% Loading the pixel and camera point cloud data
        currentFileNum = fileNumStart + fileNum - 1;
        fprintf('Current File Number is - %d\n',currentFileNum)
        
        pointCloudCam = pcread(sprintf('camera_point_cloud\\pointCloud_0_%i.ply',currentFileNum));
        
        vehBottom2rearWheelTranslation = [dx dy dz];
        
        [vehBottomCenterCam,vehBottomCenterEulVkittiCam] = get_pose(fileNum, pointCloudCam, labels, vehBottom2rearWheelTranslation);
%         vehBottomCenterEulVkittiCam_deg = vehBottomCenterEulVkittiCam*180/pi;
        
        
        %% Loading the world point cloud data
        
        pointCloudVeh = pcread(sprintf('vehicle_point_cloud\\pointCloud_0_%i.ply',currentFileNum));
        R_veh2vkitti = [0 -1 0; 0 0 -1; 1 0 0];
        pointCloudVkitti = (R_veh2vkitti*pointCloudVeh.Location')';
        pointCloudVehVkitti = pointCloud(pointCloudVkitti, Color=pointCloudVeh.Color);
        
        [vehBottomCenterVeh,vehBottomCenterEulVkittiVeh] = get_pose(fileNum, pointCloudVehVkitti, labels, vehBottom2rearWheelTranslation);
%         vehBottomCenterEulVkittiVeh_deg = vehBottomCenterEulVkittiVeh*180/pi; % Ry, Rx, Rz
        
        %% Get the point cloud for the right camera
        
        B = 0.2; % Camera Baseline = 20cm
        T_camLeft2camRight = [eye(3) [-B 0 0]'; [0 0 0 1]];
        nPoints = length(pointCloudCam.Location(:,1));
        homPointsLeft = [pointCloudCam.Location ones(nPoints,1)];
        homPointsRight = T_camLeft2camRight*homPointsLeft';
        pointCloudCamRight = pointCloud(homPointsRight(1:3,:)', "Color",pointCloudCam.Color);
        
        [vehBottomCenterCamRight,vehBottomCenterEulVkittiCamRight] = get_pose(fileNum, pointCloudCamRight, labels, vehBottom2rearWheelTranslation);
%         vehBottomCenterEulVkittiCamRight_deg = vehBottomCenterEulVkittiCamRight*180/pi;
        
        %% Get the Pose Array
        
        alphaLeft = atan(vehBottomCenterCam(3)/vehBottomCenterCam(1));
        alphaRight = atan(vehBottomCenterCamRight(3)/vehBottomCenterCamRight(1));
        trackID = 0;
        
        LeftCamPose = [fileNum-fileNumBegin 0 trackID alphaLeft W_obj H_obj L_obj vehBottomCenterVeh' vehBottomCenterEulVkittiVeh vehBottomCenterCam' vehBottomCenterEulVkittiCam];
        RightCamPose = [fileNum-fileNumBegin 1  trackID alphaRight W_obj H_obj L_obj vehBottomCenterVeh' vehBottomCenterEulVkittiVeh vehBottomCenterCamRight' vehBottomCenterEulVkittiCamRight];

        pose_arr(2*(fileNum-fileNumBegin)+1,:) = LeftCamPose;
        pose_arr(2*(fileNum-fileNumBegin+1),:) = RightCamPose;
    end
    
    pose_arr_w_dummy = [pose_arr([1 1 2 2 3:end],:)];
    pose_arr_w_dummy([2 4],3) = 1; %Dummy Tracking ID
    pose_arr_w_dummy([2 4],[5 6 7]) = repmat([W_obj H_obj L_obj]*0.05,2,1);
    pose_arr_w_dummy([2 4],[8 14]) = pose_arr_w_dummy([2 4],[8 14]) + 4;

    writematrix(pose_arr_w_dummy, 'pose_arr.txt');
end