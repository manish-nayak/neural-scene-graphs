function int_arr = get_intrinsics(fileNumBegin, fileNumEnd)

    int_arr = zeros((fileNumEnd-fileNumBegin+1)*2,6);

    fx=875.0654; fy=875.0654;
    cx = 959.3386; cy = 536.8488;

    intrinsics = [fx fy cx cy];

    for fileNum=fileNumBegin:fileNumEnd
   
        LeftCamIntrinsic = [fileNum-fileNumBegin 0 intrinsics];
        RightCamIntrinsic = [fileNum-fileNumBegin 1 intrinsics]; 
        
        int_arr(2*(fileNum-fileNumBegin)+1,:) = LeftCamIntrinsic;
        int_arr(2*(fileNum-fileNumBegin+1),:) = RightCamIntrinsic;
        
    end 

    writematrix(int_arr, 'int_arr.txt');

end