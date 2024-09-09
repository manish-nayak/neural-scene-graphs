import os
import numpy as np
from scipy.spatial.transform import Rotation

files = ['intrinsic.txt', 'extrinsic.txt', 'pose.txt']

N = 7

for si, file_name in enumerate(files):
    # Define the file name
    #file_name = 'intrinsic.txt'
    
    # Check if the file exists
    if os.path.exists(file_name):
        print("Data for {} exists, skipping.".format(file_name))
    else:
        # Create the file if it doesn't exist
        with open(file_name, 'w') as file:
            if (si==0): ## Write the header for intrinsic
                file.write("frame cameraID K[0,0] K[1,1] K[0,2] K[1,2]\n")
                for i in range(N):
                    file.write("{} 0 {} {} {} {}\n".format(i, 702.403, 702.403, 632.471, 353.683))
                    file.write("{} 1 {} {} {} {}\n".format(i, 639.099, 639.099, 586.787, 344.058))

            elif (si==1): ### Extrinsic parameters add here. Simpler because
                file.write("frame,cameraID,r1_1,r1_2,r1_3,t1,r2_1,r2_2,r2_3,t2,r3_1,r3_2,r3_3,t3,0,0,0,1\n")
                for i in range(N):
                    file.write("{} 0 1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1\n".format(i))
                    file.write("{} 0 1 0 0 -0.12 0 1 0 0 0 0 1 0 0 0 0 1\n".format(i))
                    #file.write("{} 0 1.0 0 0 0 0 1.0 0 1.75 0 0 1.0 0 0 0 0 1\n".format(i))
                    #file.write("{} 1 1.0 0 0 -0.175 0 1.0 0.0 1.75 0 0 1.0 0 0 0 0 1\n".format(i))
            
 
        print("Created file: {} with {} entries".format(file_name, N))
