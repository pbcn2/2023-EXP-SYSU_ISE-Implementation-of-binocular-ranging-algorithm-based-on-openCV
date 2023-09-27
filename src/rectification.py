import cv2
import matplotlib.pyplot as plt


def stereo_image_rectification(left_image, right_image, stereo_params):
    # Extract the stereo parameters
    M1 = stereo_params['M1']
    D1 = stereo_params['D1']
    M2 = stereo_params['M2']
    D2 = stereo_params['D2']
    R = stereo_params['R']
    T = stereo_params['T']
    # size = tuple(map(int, stereo_params['size'][0]))
    size = [640,480]
    print("1")
    # Calculate rectification transform for both cameras
    R1, R2, P1, P2, Q, validPixROI1, validPixROI2 = cv2.stereoRectify(
        M1, D1, M2, D2, size, R, T, alpha=0
    )

    print("2")
    # Compute the mapping for rectification
    map1x, map1y = cv2.initUndistortRectifyMap(M1, D1, R1, P1, size, cv2.CV_32FC1)
    map2x, map2y = cv2.initUndistortRectifyMap(M2, D2, R2, P2, size, cv2.CV_32FC1)
    print("3")
    # Rectify the images
    left_rectified = cv2.remap(left_image, map1x, map1y, cv2.INTER_LINEAR)
    right_rectified = cv2.remap(right_image, map2x, map2y, cv2.INTER_LINEAR)

    return left_rectified, right_rectified


def test_rectification(left_image_path, right_image_path, stereo_params):
    # Load the images
    print("0")
    left_image = cv2.imread(left_image_path, cv2.IMREAD_COLOR)
    right_image = cv2.imread(right_image_path, cv2.IMREAD_COLOR)
    print("00")
    # Convert from BGR to RGB
    left_image_rgb = cv2.cvtColor(left_image, cv2.COLOR_BGR2RGB)
    right_image_rgb = cv2.cvtColor(right_image, cv2.COLOR_BGR2RGB)
    print("000")
    # Rectify the images
    left_rectified, right_rectified = stereo_image_rectification(left_image_rgb, right_image_rgb, stereo_params)
    
    # Plot the results
    fig, axes = plt.subplots(2, 2, figsize=(10, 8))

    # Original Images
    axes[0, 0].imshow(left_image_rgb)
    axes[0, 0].set_title("Original Left Image")
    axes[0, 1].imshow(right_image_rgb)
    axes[0, 1].set_title("Original Right Image")

    # Rectified Images
    axes[1, 0].imshow(left_rectified)
    axes[1, 0].set_title("Rectified Left Image")
    axes[1, 1].imshow(right_rectified)
    axes[1, 1].set_title("Rectified Right Image")

    # Save the rectified images
    cv2.imwrite("../img_file/left_rectified.png", cv2.cvtColor(left_rectified, cv2.COLOR_RGB2BGR))
    cv2.imwrite("../img_file/right_rectified.png", cv2.cvtColor(right_rectified, cv2.COLOR_RGB2BGR))
    
    # Draw horizontal lines to demonstrate the effect of rectification
    for ax in axes.ravel():
        for i in range(0, int(stereo_params['size'][0][1]), 50):
            ax.axhline(i, color='red', linestyle='--')
    
    plt.tight_layout()
    plt.show()

def main():
    # Load the stereo parameters from the provided YAML file
    fs = cv2.FileStorage('matlabStereoParam.yml', cv2.FILE_STORAGE_READ)
    print("found yml")
    stereo_params = {}
    for key in fs.root().keys():
        stereo_params[key] = fs.getNode(key).mat()
    print("found img")
    # Get the file paths from the user
    left_image_path = "../img_file/left_017.png"
    right_image_path = "../img_file/right_017.png"

    # Perform rectification and display results
    test_rectification(left_image_path, right_image_path, stereo_params)


main()
