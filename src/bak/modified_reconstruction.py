import cv2, glob, os
import numpy as np
# Load the stereo parameters from the yml file
fs = cv2.FileStorage("matlabStereoParam.yml", cv2.FILE_STORAGE_READ)
M1 = fs.getNode('M1').mat()
D1 = fs.getNode('D1').mat()
M2 = fs.getNode('M2').mat()
D2 = fs.getNode('D2').mat()
R = fs.getNode('R').mat()
T = fs.getNode('T').mat()
fs.release()



mtx_l = [[ 9.01311245e+02 -3.24745533e-01  2.41807382e+02]
         [ 0.00000000e+00  8.98439112e+02  2.69165091e+02]
        [ 0.00000000e+00  0.00000000e+00  1.00000000e+00]]

dist_l = [[-6.73954420e-01  2.86799170e+00 -3.30750875e-03  1.16447156e-02
  -1.20341486e+01]]
mtx_r = [[8.49684111e+02 5.72069099e-02 1.50517456e+02]
 [0.00000000e+00 8.48799359e+02 3.10959870e+02]
 [0.00000000e+00 0.00000000e+00 1.00000000e+00]]
dist_r = np.array([[-0.38851184 -0.43056324 -0.0100523   0.03272735  1.09716727]])
R = np.array([[ 0.99828523  0.01048072  0.05759131]
 [-0.0072567   0.99840957 -0.05590765]
 [-0.05808567  0.05539386  0.99677358]])
T = np.array([[119.36750852]
 [  1.32562922]
 [-32.04515122]
 [  0.27233791]
 [  0.49530174]])
rect_left = np.array([[ 0.99998384, -0.005285,    0.00209416],
 [ 0.00526285,  0.99993159,  0.01044553],
 [-0.00214922, -0.01043434,  0.99994325]])
rect_right = np.array([[ 0.99995854, -0.00438734, -0.00797926],
 [ 0.00430379,  0.99993606, -0.01045726],
 [ 0.00802463,  0.01042249,  0.99991348]])
proj_left = np.array([[480.3073899,    0.,         322.84606934,   0.,        ],
 [  0.,         480.3073899,  235.60386848,   0.,        ],
 [  0.,           0.,           1.,           0.,        ]])
proj_right = np.array([[ 4.80307390e+02,  0.00000000e+00,  3.22846069e+02, -2.98144281e+04],
 [ 0.00000000e+00,  4.80307390e+02,  2.35603868e+02,  0.00000000e+00],
 [ 0.00000000e+00,  0.00000000e+00,  1.00000000e+00,  0.00000000e+00]])
dispartity = np.array([[ 1.00000000e+00,  0.00000000e+00,  0.00000000e+00, -3.22846069e+02],
 [ 0.00000000e+00,  1.00000000e+00,  0.00000000e+00, -2.35603868e+02],
 [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  4.80307390e+02],
 [ 0.00000000e+00,  0.00000000e+00,  1.61098978e-02, -0.00000000e+00]])
ROI_left = np.array((5, 10, 612, 456))
ROI_right = np.array((14, 5, 626, 475))
img_left = []
img_right = []
corners_left = []
corners_right = []
img_file = glob.glob('./calibration/*.jpg')
imgsize = (640, 480)
 
for img in img_file:
    frame = cv2.imread(img)
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    l = frame[:,0:640]
    r = frame[:,640:]
    img_left.append(l)
    img_right.append(r)
print("获取角点", "left")
get_corners(img_left, corners_left)
print("获取角点", "right")
get_corners(img_right, corners_right)
for i in range(len(img_left)):
    l = img_left[i]
    r = img_right[i]
    # 计算双目校正的矩阵
    R1, R2, P1, P2, Q, validPixROI1, validPixROI2 = cv2.stereoRectify(mtx_l, dist_l, mtx_r, dist_r, imgsize, R, T)
    # 计算校正后的映射关系
    maplx , maply = cv2.initUndistortRectifyMap(mtx_l, dist_l, R1, P1, imgsize, cv2.CV_16SC2)
    maprx , mapry = cv2.initUndistortRectifyMap(mtx_r, dist_r, R2, P2, imgsize, cv2.CV_16SC2)
    # 映射新图像
    lr = cv2.remap(l, maplx, maply, cv2.INTER_LINEAR)
    rr = cv2.remap(r, maprx, mapry, cv2.INTER_LINEAR)
    all = np.hstack((lr,rr))
    # 变换之后和变换之前的角点坐标不一致，所以线不是正好经过角点，只是粗略估计，但偶尔能碰到离角点比较近的线，观察会比较明显
    cv2.line(all, (-1, int(corners_left[i][0][0][1])), (all.shape[1], int(corners_left[i][0][0][1])), (255), 1)
    # 可以看出左右图像y坐标对齐还是比较完美的，可以尝试着打印双目校正前的图片，很明显，左右y坐标是不对齐的
    cv2.imshow('a', all)
    c = cv2.waitKey()
    cv2.destroyAllWindows()
    if c == 27:
        break
 
 
print("end")
 