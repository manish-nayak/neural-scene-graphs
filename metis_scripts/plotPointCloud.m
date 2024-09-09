function plotPointCloud(ptCloud,ptCloudFilt,centreVec,normalVec)
% Plot the Point Cloud and Filtered Points
    switch nargin
        case 2
            figure;
            pcshow(ptCloud);
            hold on;
            plot3(ptCloudFilt(:,1),ptCloudFilt(:,2),ptCloudFilt(:,3),'o')
            xlabel('X');
            ylabel('Y');
            zlabel('Z');
            title('Original and Masked Point Clouds');
            hold off
    
        case 4
            figure;
            pcshow(ptCloud);
            hold on;
            plot3(ptCloudFilt(:,1),ptCloudFilt(:,2),ptCloudFilt(:,3),'o')
            % Plot Vectors 
            n_csys = length(centreVec(:,1));
            for i=1:3:n_csys
                centreVec_csys = centreVec(i:i+2,:);
                normalVec_csys = normalVec(i:i+2,:);
                quiver3(centreVec_csys(1,1), centreVec_csys(1,2), centreVec_csys(1,3), normalVec_csys(1,1),...
                    normalVec_csys(1,2), normalVec_csys(1,3),'Color','r'); 
                quiver3(centreVec_csys(2,1), centreVec_csys(2,2), centreVec_csys(2,3), normalVec_csys(2,1),...
                    normalVec_csys(2,2), normalVec_csys(2,3),'Color','g');
                quiver3(centreVec_csys(3,1), centreVec_csys(3,2), centreVec_csys(3,3), normalVec_csys(3,1),...
                    normalVec_csys(3,2), normalVec_csys(3,3),'Color','y');
            end
            xlabel('X');
            ylabel('Y');
            zlabel('Z');
            title('Original and Masked Point Clouds with Quivers');
            hold off


    end
end