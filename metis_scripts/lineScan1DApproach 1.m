clc
clear all
close all

ptCloud = pcread('milledEdgePointCloud_Frame_3876_Source_Stereo.ply');
% [ptCloudOut, inliers, outliers] = pcdenoise(ptCloud);

imgSz = [1920, 1080];
numScanLines = 100; 
step = floor(imgSz(2)/numScanLines);

heights = zeros(numScanLines, imgSz(1));
envelopes = zeros(numScanLines, imgSz(1));

for j = 1:1:numScanLines
    offset = imgSz(1) * (j-1) * step;
    for i = 1:1:imgSz(1)
        % check if point is outlier
%        outlierPos = find(outliers == offset+i);
%        if isempty(outlierPos)
%             
            if(isinf(ptCloud.Location(offset+i , 3)))
                heights(j,i) = NaN;
            else
                heights(j,i) = ptCloud.Location(offset+i , 3);
            end  
%        else
%              heights(j,i) = NaN;
%        end 
    end
end


peakPosArray = zeros(numScanLines, 1);
for i = 1:1:numScanLines
     scanLine = heights(i,:);
% 
     scanLine = outlierRejection(scanLine, 50, 0.1);
     scanLineSlope = diff(scanLine);
%      [yupper,ylower] = envelope(scanLine);
%      envelopes(i,:) = yupper; 
%      scanLineSlope = diff(scanLineSlope);
     max(scanLineSlope)
     peakPos = find(scanLineSlope == max(scanLineSlope)); 
     if ~isempty(peakPos)
         peakPosArray(i) = peakPos(1);
     else
         peakPosArray(i) = NaN;
     end
end

stepsArr = zeros(numScanLines, 1);
for j = 1:numScanLines 
    stepsArr(j) = (j-1)*step;
end

% outlier rejection 
figure; plot(peakPosArray); hold on;

peakPosArray = outlierRejection(peakPosArray, 5, 0.2);

plot(peakPosArray);

p = polyfit(stepsArr(1:100), peakPosArray(1:100), 1); 
% 
edgeLoc = [];
for i = 1:1:numScanLines
    edgeLoc(i) = polyval(p, stepsArr(i));
end

plot(edgeLoc, 'm');

edgePoints_3d = [];

for i=1:1:numScanLines
    offset = imgSz(1) * (i-1) * step;
    ptLocation = offset + round(edgeLoc(i));
    edgePoints_3d = [edgePoints_3d; ptCloud.Location(ptLocation , :)];
end

figure;
pcshow(ptCloud); hold on;
scatter3(edgePoints_3d(:,1), edgePoints_3d(:,2), edgePoints_3d(:,3),'LineWidth',0.5);