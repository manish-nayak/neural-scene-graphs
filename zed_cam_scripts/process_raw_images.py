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
for i in range(1,8):
    image = cv2.imread(image_path.format(i), cv2.IMREAD_COLOR)
    print('Currently loaded image no. -', i)
    
    left_img, right_img = get_split_images(image)
    left_images.append(left_img)
    right_images.append(right_img)

for i in range(len(left_images)):
    left_img = left_images[i]
    right_img = right_images[i]

    save_directory_rel_path = 'data/conf_room_boxes/multiple_boxes/frames/rgb'

    left_img_save_dir = os.path.join(save_directory_rel_path, 'Camera_0')
    right_img_save_dir = os.path.join(save_directory_rel_path, 'Camera_1')

    #cv2.imshow("Left Image with Markers", left_img)
    filename = "rgb_000{:02d}.png".format(i)
    left_save_path = os.path.join(left_img_save_dir, filename)
    right_save_path = os.path.join(right_img_save_dir, filename)

    cv2.imwrite(left_save_path, left_img)
    
    #cv2.imshow("Right Image with Markers", right_img)
    cv2.imwrite(right_save_path, right_img)


