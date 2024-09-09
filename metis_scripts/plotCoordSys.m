function plotCoordSys(centreVec,normalVec)
           figure;
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


function [centreVec,normalVec] = get_vectors(transforms)
    transform_arr = ext_arr([1,2],3:end);
    for i=1:size(transform_arr,1)
        transform_vec = transform_arr(i,:);
        T = reshape(transform_vec,[],4)';
        R = T(1:3,1:3);
        eul = rotm2eul(R)*180/pi
        translation = T(1:3,4)
    end

end