%% Add to Path

addpath(genpath('C:\Users\QYGXAMU\OneDrive - Deere & Co\Documents\NeRF\neural-scene-graphs\data\metis_re_extracted'))

%% Loading the pixel and camera point cloud data

fileNum = 3;
fileNumStart = 4421;
currentFileNum = fileNumStart + fileNum - 1;

pointCloudCam = pcread(sprintf('camera_point_cloud\\pointCloud_0_%i.ply',currentFileNum));
labels = load('label_0_4421_4432_4.mat');    

dx = 1.05; dy = 0.15; dz = 0.3;
vehBottom2rearWheelTranslation = [dx dy dz];

L_obj = 2.8; H_obj = 2.7; W_obj = 1;

[vehBottomCenterCam,vehBottomCenterEulVkittiCam] = get_pose(fileNum, pointCloudCam, labels, vehBottom2rearWheelTranslation);
vehBottomCenterEulVkittiCam_deg = vehBottomCenterEulVkittiCam*180/pi;


%% Loading the world point cloud data

pointCloudVeh = pcread(sprintf('vehicle_point_cloud\\pointCloud_0_%i.ply',currentFileNum));
R_veh2vkitti = [0 -1 0; 0 0 -1; 1 0 0];
pointCloudVkitti = (R_veh2vkitti*pointCloudVeh.Location')';
pointCloudVehVkitti = pointCloud(pointCloudVkitti, Color=pointCloudVeh.Color);

[vehBottomCenterVeh,vehBottomCenterEulVkittiVeh] = get_pose(fileNum, pointCloudVehVkitti, labels, vehBottom2rearWheelTranslation);
vehBottomCenterEulVkittiVeh_deg = vehBottomCenterEulVkittiVeh*180/pi; % Ry, Rx, Rz

%% Get the point cloud for the right camera

B = 0.2; % Camera Baseline = 20cm
T_camLeft2camRight = [eye(3) [-B 0 0]'; [0 0 0 1]];
nPoints = length(pointCloudCam.Location(:,1));
homPointsLeft = [pointCloudCam.Location ones(nPoints,1)];
homPointsRight = T_camLeft2camRight*homPointsLeft';
pointCloudCamRight = pointCloud(homPointsRight(1:3,:)', "Color",pointCloudCam.Color);

[vehBottomCenterCamRight,vehBottomCenterEulVkittiCamRight] = get_pose(fileNum, pointCloudCamRight, labels, vehBottom2rearWheelTranslation);
vehBottomCenterEulVkittiCamRight_deg = vehBottomCenterEulVkittiCamRight*180/pi;

%% Get the Pose Array

alphaLeft = atan(vehBottomCenterCam(3)/vehBottomCenterCam(1));
alphaRight = atan(vehBottomCenterCamRight(3)/vehBottomCenterCamRight(1));

LeftCamPose = [currentFileNum 0 1 alphaLeft W_obj H_obj L_obj vehBottomCenterVeh' vehBottomCenterEulVkittiVeh vehBottomCenterCam' vehBottomCenterEulVkittiCam];
RightCamPose = [currentFileNum 1  1 alphaRight W_obj H_obj L_obj vehBottomCenterVeh' vehBottomCenterEulVkittiVeh vehBottomCenterCamRight' vehBottomCenterEulVkittiCamRight];
