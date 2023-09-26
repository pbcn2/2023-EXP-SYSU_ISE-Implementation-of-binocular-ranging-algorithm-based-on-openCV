import cv2
import matplotlib.pyplot as plt
import numpy as np

def compute_disparity_and_depth(left_rectified, right_rectified):
    # Set up parameters for StereoSGBM algorithm
    window_size = 5
    min_disp = 0
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

    # Assuming you have the focal length (f) in pixel and baseline (T) in meters
    # This is just an example value, you need to use your actual values
    f = 900  # example value in pixels
    T = 0.123  # example value in meters

    # Compute depth map
    depth = f * T / (disparity + 0.00001)  # adding a small value to avoid division by zero

    # Return disparity and depth maps
    return disparity, depth


def compute_disparity_with_wls_filter(left_rectified, right_rectified):
    # Set up parameters for StereoSGBM algorithm
    window_size = 5
    min_disp = 4
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
    
    # Compute disparity for left and right images
    disparity_left = stereo.compute(left_rectified, right_rectified).astype(np.float32) / 16.0
    stereo_right = cv2.ximgproc.createRightMatcher(stereo)
    disparity_right = stereo_right.compute(right_rectified, left_rectified).astype(np.float32) / 16.0

    # WLS filter parameters
    lmbda = 8000
    sigma = 1.5
    wls_filter = cv2.ximgproc.createDisparityWLSFilter(stereo)
    wls_filter.setLambda(lmbda)
    wls_filter.setSigmaColor(sigma)
    
    # Apply WLS filter
    disparity_wls = wls_filter.filter(disparity_left, left_rectified, None, disparity_right)
    
    return disparity_wls


left_rectified_path = "../img_file/left_rectified.png"
right_rectified_path = "../img_file/right_rectified.png"
left_image_rectified = cv2.imread(left_rectified_path, cv2.IMREAD_GRAYSCALE)
right_image_rectified = cv2.imread(right_rectified_path, cv2.IMREAD_GRAYSCALE)

# Compute disparity map
disparity_map, depth_map = compute_disparity_and_depth(left_image_rectified, right_image_rectified)

# Compute disparity map with WLS filter
disparity_wls_map = compute_disparity_with_wls_filter(left_image_rectified, right_image_rectified)

# Plotting the results
fig, axes = plt.subplots(1, 3, figsize=(15, 7))

# Displaying original disparity map
axes[0].imshow(disparity_map, cmap='jet')
axes[0].set_title("Original Disparity Map")

# Displaying WLS filtered disparity map
axes[1].imshow(disparity_wls_map, cmap='jet')
axes[1].set_title("WLS Filtered Disparity Map")

# Displaying original image
axes[2].imshow(left_image_rectified, cmap='gray')
axes[2].set_title("Original Image")

plt.tight_layout()
plt.show()