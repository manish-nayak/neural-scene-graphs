%% Define function to calculate angle between 2 vectors
function ThetaInDegrees = getAngle(v1,v2)
    CosTheta = max(min(dot(v1,v2)/(norm(v1)*norm(v2)),1),-1);
    ThetaInDegrees = real(acosd(CosTheta));
end