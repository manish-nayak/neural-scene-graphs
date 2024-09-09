function ext_arr = get_extrinsics(fileNumBegin, fileNumEnd)

    
%% Loading the pixel and point cloud data

    fileNumStart = 4421;
    ext_arr = zeros((fileNumEnd-fileNumBegin+1)*2,18);

    for fileNum=fileNumBegin:fileNumEnd

        currentFileNum = fileNumStart + fileNum - 1;
        fprintf('Current File Number is - %d\n',currentFileNum)

        % Load camera point cloud
        pointCloudCam = pcread(sprintf('camera_point_cloud\\pointCloud_0_%i.ply',currentFileNum));
        R_cam2camMech = [0 0 1; -1 0 0; 0 -1 0];
        T_cam2camMech = [R_cam2camMech [0 0 0]'; [0 0 0 1]];
        pointCloudcamTemp = (R_cam2camMech*pointCloudCam.Location')';
        pointCloudCamMech = pointCloud(pointCloudcamTemp, Color=pointCloudCam.Color);
        % figure()
        % pcshow(pointCloudCamMech)
        
        % Load vehicle point cloud
        pointCloudVehTemp = pcread(sprintf('vehicle_point_cloud\\pointCloud_0_%i.ply',currentFileNum));
        % figure()
        % pcshow(pointCloudVehTemp)
        R_veh2vkitti = [0 -1 0; 0 0 -1; 1 0 0];
        T_veh2vkitti = [R_veh2vkitti [0 0 0]'; [0 0 0 1]];
        pointCloudVkitti = (R_veh2vkitti*pointCloudVehTemp.Location')';
        pointCloudVeh = pointCloud(pointCloudVkitti, Color=pointCloudVehTemp.Color);
        
        %% Check the extrinsic matrix
        
        T_creo = [0.8261491704	0.0000000000	0.5634514604	655.07608921
                    0.0000000000	1.0000000000	0.0000000000	122.52379655
                    -0.5634514604	0.0000000000	0.8261491704	1685.0635965];
        T_creo = [T_creo; [ 0 0 0 1]];
        R_creo = T_creo(1:3,1:3);
        eul_creo_deg = rotm2eul(R_creo)*180/pi;
        
        %% Intermediate check
        
        % Parameters from Robotics Workbench
        tx = 1.016; ty = 1.149; tz = 2.010;
        pitch_mech = 35; roll_mech = 1.94; yaw_mech = -5.02;
        
        R_mech = eul2rotm([yaw_mech pitch_mech roll_mech]*pi/180);
        T_camMech2veh = [R_mech [tx ty tz]'; [0 0 0 1]]; % Cam in veh frame
        
        id = 1244361;
        cam_vec = pointCloudCamMech.Location(id,:)';
        cam_vec = [cam_vec;1];
        veh_vec = pointCloudVehTemp.Location(id,:)';
        veh_vec = [veh_vec;1];
        
        % Verification of Camera Mechanical to Vehicle Mechanical
        veh_vec_calc = T_camMech2veh*cam_vec;
        
        %% Camera CVML Frame to Vehicle VKitti Frame
        
        T_cam2vkitti = T_veh2vkitti*T_camMech2veh*T_cam2camMech;
        T_vkitti2cam = inv(T_cam2vkitti);
        
        id = 1244361;
        cam_vec = pointCloudCam.Location(id,:)';
        cam_vec = [cam_vec;1];
        veh_vec = pointCloudVeh.Location(id,:)';
        veh_vec = [veh_vec;1];
        
        % Verification of Camera CVML to Vehicle Kitti
        veh_vec_calc = T_cam2vkitti*cam_vec;
        
        %% Vehicle Vkitti to Camera CVML Frame
        
        % cam_vec
        cam_vec_calc = T_vkitti2cam*veh_vec;
        
        B = 0.2; % Camera Baseline = 20cm
        T_camLeft2camRight = [eye(3) [-B 0 0]'; [0 0 0 1]];
        
        T_vkitti2camRight = T_camLeft2camRight*T_vkitti2cam;
        cam_vec_right = T_vkitti2camRight*veh_vec;
        
        LeftCamExtrinsic = [fileNum-fileNumBegin 0 reshape(T_vkitti2cam',1,16)];
        RightCamExtrinsic = [fileNum-fileNumBegin 1 reshape(T_vkitti2camRight',1,16)]; 
        
        ext_arr(2*(fileNum-fileNumBegin)+1,:) = LeftCamExtrinsic;
        ext_arr(2*(fileNum-fileNumBegin+1),:) = RightCamExtrinsic;
        
    end 

    writematrix(ext_arr, 'ext_arr.txt');

end