function bbox_arr = get_bbox_arr(fileNumBegin, fileNumEnd)

labels_left = load('label_0_4421_4432_5_Left.mat');  
labels_right = load('label_0_4421_4432_5_Right.mat');  

fileNumStart = 4421;
% fileNumBegin = 1;
% fileNumEnd = 14;

bbox_arr = zeros((fileNumEnd - fileNumBegin + 1)*2,11);

%% Loading the pixel and camera point cloud data

    for fileNum=fileNumBegin:fileNumEnd

        currentFileNum = fileNumStart + fileNum - 1;
%         fprintf('Current File Number is - %d\n',currentFileNum)
        
        pix_left = labels_left.gTruth.LabelData.ObjectBBox{fileNum,1};
        pix_right = labels_right.gTruth.LabelData.ObjectBBox{fileNum,1};
    
        pixelBBox_left = pix_left([1 3 2 4]);
        pixelBBox_right = pix_right([1 3 2 4]);
    
        nPixels_left = (pix_left(3)-pix_left(1))*(pix_left(4)-pix_left(2));
        nPixels_right = (pix_right(3)-pix_right(1))*(pix_right(4)-pix_right(2));
    
        truncation_ratio = 0;
        occupancy_ratio = 0.9;
        isMoving = 1;
        trackID = 0;
        
        bbox_left = [fileNum-fileNumBegin 0 trackID pixelBBox_left nPixels_left ...
            truncation_ratio occupancy_ratio isMoving];
        bbox_right = [fileNum-fileNumBegin 1 trackID pixelBBox_right nPixels_right ...
            truncation_ratio occupancy_ratio isMoving];
    
        bbox_arr(2*(fileNum-fileNumBegin)+1,:) = bbox_left;
        bbox_arr(2*(fileNum-fileNumBegin+1),:) = bbox_right;
    end

    bbox_arr_w_dummy = [bbox_arr([1 1 2 2 3:end],:)];
    bbox_arr_w_dummy([2 4],3) = 1; %Dummy Tracking ID
    bbox_arr_w_dummy(2,4:8) = [595 605 165 175 100];
    bbox_arr_w_dummy(4,4:8) = [590 600 165 175 100];

    writematrix(bbox_arr_w_dummy, 'bbox_arr.txt');
end