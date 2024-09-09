function [surfaceCenter,surfaceNormal] = getSurfaceCenterAndNormal(ptCloudFilt)
% Get surface center and Normal Vector
% Normal output is unconstrained and may be in the opposite direction

surfaceCenter = mean(ptCloudFilt);
% Xc = surfaceCenter(1); Yc = surfaceCenter(2); Zc = surfaceCenter(3);

ptCloudFiltNew = pointCloud(ptCloudFilt);
maxDistance = 0.02;
modelPlane = pcfitplane(ptCloudFiltNew,maxDistance);
% u = modelPlane.Normal(1);
% v = modelPlane.Normal(2);
% w = modelPlane.Normal(3);
surfaceNormal = modelPlane.Normal;

end