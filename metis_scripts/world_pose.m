%% Add to Path

addpath(genpath('C:\Users\QYGXAMU\OneDrive - Deere & Co\Documents\NeRF\neural-scene-graphs\data\metis_re_extracted'))

%% Loading the pixel and point cloud data

% Assuming pointCloud is a (N, 3) matrix and pixelCoords is a (M, 2) matrix

fileNum = 4;
fileNumStart = 4421;
currentFileNum = fileNumStart + fileNum - 1;

% pointCloudCam = pcread(sprintf('camera_point_cloud\\pointCloud_0_%i.ply',currentFileNum));
pointCloudCam2 = pcread(sprintf('vehicle_point_cloud\\pointCloud_0_%i.ply',currentFileNum));
R_veh2vkitti = [0 -1 0; 0 0 -1; 1 0 0];
pointCloudVkitti = (R_veh2vkitti*pointCloudCam2.Location')';
pointCloudCam = pointCloud(pointCloudVkitti, Color=pointCloudCam2.Color);


% vehiclePtCloud = pcread('vehicle_point_cloud\');
% labels = load('label_0_4421.mat');
load('label_0_4421_4432_4.mat');

pixelCoordsRearWheel = gTruth.LabelData.RearWheel{fileNum,1}{1,1};
pixelCoordsFrontWheel = gTruth.LabelData.FrontWheel{fileNum,1}{1,1};
pixelCoordsVehiclePlane = gTruth.LabelData.VehiclePlane{fileNum,1}{1,1};
% pixelCoordsVehicleXDir = gTruth.LabelData.VehicleXDir{1,1}{1,1};
pixelCoordsVehicleTopPlane = gTruth.LabelData.VehicleTopPlane{fileNum,1}{1,1};
pixelCoordsGround = gTruth.LabelData.Ground{fileNum,1}{1,1};

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
ThetaInDegrees = getAngle(v1,v2);

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
% csys_vector = [v1_norm; v2_norm; v3_norm];
csys_vector = [v3_norm; v2_norm; v1_norm];

% Figure 3 - Plot the wheel centred vehicle coordinate system
% plotPointCloud(pointCloudCam,ptCloudFiltCat,csys_center,csys_vector)

%% Get the vehicle centre coordinate system

R_wheel = [v3_norm; v2_norm; v1_norm]'; % Rotation Matrix in Source Frame

T_wheel = [R_wheel rearWheelCenter'; [0 0 0 1]];
rotm2eul(R_wheel)*180/pi

%% Pose of Vehicle Bottom Center

dy = 0.15; % Along the vehicle height
dx = 1.05; % Along the vehicle length
dz = 0.3; % Along the vehicle width

wheel_test_cen = rearWheelCenter;
eul_seq_c = [pi/4, 0, 0];
% R_wheel = eul2rotm(eul_seq_c);
% R_wheel = [v3_norm; v2_norm; v1_norm];

T_wheel = [R_wheel wheel_test_cen'; [0 0 0 1]];
csys_center = repmat(wheel_test_cen,3,1);
csys_vector = T_wheel(1:3,1:3);

T_vehBottom2WheelCenter = [eye(3) [dx dy dz]'; [0 0 0 1]];
T_vehBottom2Cam = T_wheel*T_vehBottom2WheelCenter;

vehBottomCenter = T_vehBottom2Cam(1:3,4);
csys_vehBottomCenterVec = T_vehBottom2Cam(1:3,1:3);
csys_vehBottomCenter = repmat(vehBottomCenter',3,1);


% Figure 4 - Plot the vehicle bottom coordinate system
plotPointCloud(pointCloudCam,ptCloudFiltCat,[csys_center;csys_vehBottomCenter],...
    [csys_vector';csys_vehBottomCenterVec'])
