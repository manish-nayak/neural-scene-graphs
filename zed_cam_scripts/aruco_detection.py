import cv2
import numpy as np
import os

# Task 1
def get_split_images(image):
    height, width = image.shape[:2]
    half_width = width // 2
    left_img = image[:, :half_width, :]
    right_img = image[:, half_width:, :]
    return left_img, right_img

# Get the relative path to the raw_images folder
relative_path_to_raw_images = 'data/conf_room_boxes/multiple_boxes/raw_images'

# Construct the full path to the image file
image_filename = 'p{}.png'  # Replace with your actual image filename
image_path = os.path.join(relative_path_to_raw_images, image_filename)

left_images = []
right_images = []
for i in range(1,7):
    image = cv2.imread(image_path.format(i), cv2.IMREAD_COLOR)
    
    left_img, right_img = get_split_images(image)
    left_images.append(left_img)
    right_images.append(right_img)

# Task 3
# marker_id = 7 # ID of the marker to detect
marker_size = 80  # in mm
# Apply aruco marker detection using cv2.aruco module
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

print("Aruco detection running...")
for i in range(len(left_images)):
    left_img = left_images[i]
    right_img = right_images[i]

    # Detect markers in the left image
    corners_left, ids_left, rejected = detector.detectMarkers(left_img)
    corners_right, ids_right, rejected = detector.detectMarkers(right_img)

    # Print marker information and visualize corners
    # Compute the distance and orientation of each marker
    
    if ids_left is not None:
        for j in range(len(ids_left)):
            cv2.aruco.drawDetectedMarkers(left_img, corners_left, ids_left)
            cv2.aruco.drawDetectedMarkers(right_img, corners_right, ids_right)

            # Define the 3D points in the object's coordinate space (e.g., corners of a marker)
            object_points = np.array([
                [0, 0, 0],
                [marker_size, 0, 0],
                [marker_size, marker_size, 0],
                [0, marker_size, 0]
            ], dtype=np.float32)

            # Define the corresponding 2D points in the image plane (e.g., detected corners)
            image_points_left = corners_left[0].reshape(-1, 2)
            image_points_right = corners_right[0].reshape(-1, 2)

            # Camera matrix and distortion coefficients
            camera_matrix_left = np.array([[699.75, 0, 640.5],
                                        [0, 699.75, 349.28],
                                        [0, 0, 1]], dtype=np.float32)
            dist_coeffs_left = np.array([-0.1739, 0.0272, 0.0002, -0.0009, 0.0000], dtype=np.float32)

            camera_matrix_right = np.array([[699.475, 0, 601.94],
                                            [0, 699.475, 342.1295],
                                            [0, 0, 1]], dtype=np.float32)
            dist_coeffs_right = np.array([-0.1755, 0.0295, 0.0002, -0.0009, 0.0000], dtype=np.float32)

            # Estimate pose for the left image
            success_left, rvec_left, tvec_left = cv2.solvePnP(object_points, image_points_left, camera_matrix_left, dist_coeffs_left)

            # Estimate pose for the right image
            success_right, rvec_right, tvec_right = cv2.solvePnP(object_points, image_points_right, camera_matrix_right, dist_coeffs_right)

            # Draw frame axes
            cv2.drawFrameAxes(left_img, camera_matrix_left, dist_coeffs_left, rvec_left, tvec_left, 100)
            cv2.drawFrameAxes(right_img, camera_matrix_right, dist_coeffs_right, rvec_right, tvec_right, 100)

            # Display the images (for testing purposes)
            cv2.imshow("Left Image with Markers", left_img)
            cv2.imshow("Right Image with Markers", right_img)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
            
            # Compute distance and orientation

            distance_left = cv2.norm(tvec_left)
            #orientation_left = cv2.Rodrigues(rvec_left[j][0])[0]

            distance_right = cv2.norm(tvec_right)


            # Lets get the location of the bottom center [0, 55, 42.5] in marker frame, to camera frame
            # The location of bottom center is [0, 55, 42.5]
            # The homogeneous matrix relating the marker frame to camera frame is 
            H_left = np.eye(4)
            H_right = np.eye(4)
            H_left[:3, :3] = cv2.Rodrigues(rvec_left)[0]
            H_right[:3, :3] = cv2.Rodrigues(rvec_right)[0]
            H_left[:3, 3] = tvec_left.flatten()
            H_right[:3, 3] = tvec_right.flatten()  

            bottom_center = np.array([0, -55, -42.5, 1]) #np.array([0, 0, 0, 1])#np.array([0, 55, 42.5, 1])
            bottom_center_left = np.dot(H_left, bottom_center)/1000
            bottom_center_right = np.dot(H_right, bottom_center)/1000

            Ex_mat = np.eye(4) ## Extrinsic matrix is Identity matrix
            E = Ex_mat

            tw = bottom_center_left[:3]
            # print("{} 0 0 0 0.085 0.105 0.120 {:0.2f} {:0.2f} {:0.2f} {:0.2f} {:0.2f} {:0.2f} {:0.2f} {:0.2f} {:0.2f} {:0.2f} {:0.2f} {:0.2f}".format(i, tw[0], tw[1], tw[2], rvec_left[0], rvec_left[1], rvec_left[2], bottom_center_left[0], bottom_center_left[1], bottom_center_left[2], rvec_left[0], rvec_left[1], rvec_left[2]))
            # print("{} 1 0 0 0.085 0.105 0.120 {:0.2f} {:0.2f} {:0.2f} {:0.2f} {:0.2f} {:0.2f} {:0.2f} {:0.2f} {:0.2f} {:0.2f} {:0.2f} {:0.2f}".format(i, tw[0], tw[1], tw[2], rvec_left[0], rvec_left[1], rvec_left[2], bottom_center_right[0]-0.120, bottom_center_right[1], bottom_center_right[2], rvec_right[1], rvec_right[1], rvec_right[2]))

            rvec_left_modified = np.zeros_like(rvec_left)#[0,0,0]#rvec_left - [0, 0, 3.141592653589793]
            rvec_right_modified = np.zeros_like(rvec_right)#[0,0,0]#rvec_right - [0, 0, 3.141592653589793]
            cv2.drawFrameAxes(left_img, camera_matrix_left, dist_coeffs_left, rvec_left, bottom_center_left[:3], 100)
            cv2.drawFrameAxes(right_img, camera_matrix_right, dist_coeffs_right, rvec_right, bottom_center_right[:3], 100)


            # Annotate distance and orientation on the image
            cv2.putText(left_img, f"Distance: {distance_left:.2f} mm", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv2.putText(left_img, f"Orientation: {rvec_left[j][0]}", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

            save_directory_rel_path = 'data/conf_room_boxes/multiple_boxes/frames/rgb'

            left_img_save_dir = os.path.join(save_directory_rel_path, 'Camera_0')
            right_img_save_dir = os.path.join(save_directory_rel_path, 'Camera_1')

            #cv2.imshow("Left Image with Markers", left_img)
            filename = "rgb_000{:02d}.png".format(i)
            left_save_path = os.path.join(left_img_save_dir, filename)
            right_save_path = os.path.join(right_img_save_dir, filename)

            cv2.imwrite(left_save_path, left_img)
            # Annotate distance and orientation on the image
            cv2.putText(right_img, f"Distance: {distance_right:.2f} mm", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv2.putText(right_img, f"Orientation: {rvec_right[j][0]}", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            
            #cv2.imshow("Right Image with Markers", right_img)
            cv2.imwrite(right_save_path, right_img)

            cv2.waitKey(0)
            cv2.destroyAllWindows()
