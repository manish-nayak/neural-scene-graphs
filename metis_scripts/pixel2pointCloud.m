function ptCloudFilt = pixel2pointCloud(pixelCoords,ptCloud,dispSize)

% Get the coordinates corresponding to the labels
nPixels = ptCloud.Count;
ptCloudMask = zeros(nPixels,1);
pixels = length(pixelCoords);

% dispSize = [1920,1058];
nCols = dispSize(1);

for i=1:pixels
    imgRow = round(pixelCoords(i,2));
    imgCol = round(pixelCoords(i,1));
    ptCloudId = (imgRow-1)*nCols + imgCol;
    ptCloudMask(ptCloudId) = 1;
end

ptCloudFilt = ptCloud.Location;
ptCloudFilt = ptCloudFilt(ptCloudMask==1,:);
ptCloudFilt(isnan(ptCloudFilt)) = Inf;
[r,~]=find(ismember(ptCloudFilt,Inf));
ptCloudFilt(r,:)=[];



%% Verification

% To find the index corresponding to a point in the point cloud image
% ptCloudData = pointCloudCam.Location;
% X = 7.0258; Y = 5.7624; Z = 0.30375; C = [33 29 28];
% ptCloudIndex = [abs(ptCloudData(:,1)-X)<1e-4 abs(ptCloudData(:,2)-Y)<1e-4 abs(ptCloudData(:,3)-Z)<1e-5];
% idx=find(all(ptCloudIndex,2));
