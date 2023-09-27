import cv2
import matplotlib.pyplot as plt
import numpy as np


def compute_disparity_and_depth(left_rectified, right_rectified):
    # Set up parameters for StereoSGBM algorithm
    window_size = 5
    min_disp = 8
    num_disp = 16 - min_disp
    stereo = cv2.StereoSGBM_create(minDisparity=min_disp,
                                   numDisparities=num_disp,
                                   blockSize=16,
                                   P1=8*3*window_size**2,
                                   P2=32*3*window_size**2,
                                   disp12MaxDiff=1,
                                   uniquenessRatio=10,
                                   speckleWindowSize=100,
                                   speckleRange=32
                                  )

    # Compute disparity image
    disparity = stereo.compute(left_rectified, right_rectified).astype(np.float32) / 16.0

    f = 900  # extract & average from stereoParameters.mat
    T = 0.123  # extract from stereoParameters.mat

    # Compute depth map
    depth = f * T / (disparity + 0.00001)  # adding a small value to avoid division by zero

    # Return disparity and depth maps
    return disparity, depth

# Using the adjusted rectified images to compute disparity and depth
left_rectified_path = "../img_file/left_rectified.png"
right_rectified_path = "../img_file/right_rectified.png"
left_image_rectified = cv2.imread(left_rectified_path, cv2.IMREAD_GRAYSCALE)
right_image_rectified = cv2.imread(right_rectified_path, cv2.IMREAD_GRAYSCALE)

disparity_map, depth_map = compute_disparity_and_depth(left_image_rectified, right_image_rectified)

# Plotting the results
fig, axes = plt.subplots(1, 3, figsize=(12, 4))

# Displaying disparity map
axes[0].imshow(disparity_map, cmap='jet')
axes[0].set_title("Disparity Map")

# Displaying depth map
axes[1].imshow(depth_map, cmap='jet')
axes[1].set_title("Depth Map")

# Displaying depth map
axes[2].imshow(right_image_rectified)
axes[2].set_title("orginal")

plt.tight_layout()
plt.show()
