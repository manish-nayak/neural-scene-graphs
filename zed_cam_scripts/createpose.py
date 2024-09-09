import os
import numpy as np
from scipy.spatial.transform import Rotation

N = 7
file_name = 'pose.txt'

objs = {'0': {'x':[0.34, 0.32, 0.295, 0.26, 0.225, 0.20, 0.17], 'y':[0.15 for i in range(7)], 'z':[0.66, 0.73, 0.81, 0.895, 0.995, 1.075, 1.165]},
        '1': {'x':[0.47, 0.45, 0.425, 0.40, 0.375, 0.35, 0.32], 'y':[0.15 for i in range(7)], 'z':[0.5, 0.56, 0.645, 0.745, 0.825, 0.9, 1.00]},
        '2': {'x':[0.305, 0.23, 0.13, 0.055, -0.04, -0.12, -0.21], 'y':[0.15 for i in range(7)], 'z':[1.305, 1.31, 1.31, 1.315, 1.315, 1.32, 1.325]}}
yaw_all = [-107.35, -108.48, -180]
cam = [0, 1]
obj = ['0', '1', '2']
Width = [0.09, 0.085, 0.085] # 0.09 0.115 0.12
Height = [0.115, 0.105, 0.110]
Length = [0.12, 0.12, 0.20]
with open(file_name, 'w') as file:
    file.write("frame,cameraID,trackID,alpha,width,height,length,world_space_X,world_space_Y,world_space_Z,rotation_world_space_y,rotation_world_space_x,rotation_world_space_z,camera_space_X,camera_space_Y,camera_space_Z,rotation_camera_space_y,rotation_camera_space_x,rotation_camera_space_z\n")
    for i in range(N):
        for c in cam:
            for o in obj:
                if c == 1:
                    x = objs[o]['x'][i] - 0.12
                else:
                    x = objs[o]['x'][i]
                y = objs[o]['y'][i]
                z = objs[o]['z'][i]
                alpha = np.arctan2(x, z) # in radians
                yaw = yaw_all[int(o)]*np.pi/180 # in radians
                w = Width[int(o)]
                h = Height[int(o)]
                l = Length[int(o)]
                file.write("{} {} {} {:0.3f} {} {} {} {:0.3f} {} {} {:0.3f} 0 0 {:0.3f} {} {} {:0.3f} 0 0\n".format(i, c, int(o), alpha, w, h, l, x, y, z, yaw, x, y, z, yaw))
                