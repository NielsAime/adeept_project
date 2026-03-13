import cv2
import numpy as np
import glob

# define checkerboard inner intersections width
board_width = 7
# define checkerboard inner intersections height
board_height = 5

board_size = (board_width, board_height)

# prepare real world physical points
obj_points = np.zeros((board_width * board_height, 3), np.float32)
obj_points[:, :2] = np.mgrid[0:board_width, 0:board_height].T.reshape(-1, 2)

# arrays to store points from all images
real_world_points = []
image_points = []

# load all captured images
images = glob.glob('calibration_images/*.png')

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # find checkerboard corners
    ret, corners = cv2.findChessboardCorners(gray, board_size, None)

    if ret:
        real_world_points.append(obj_points)
        image_points.append(corners)

        # show the detected corners
        cv2.drawChessboardCorners(img, board_size, corners, ret)
        cv2.imshow('finding corners', img)
        cv2.waitKey(500)

cv2.destroyAllWindows()

# compute calibration matrix and distortion coefficients
ret, matrix, distortion, rvecs, tvecs = cv2.calibrateCamera(
    real_world_points, image_points, gray.shape[::-1], None, None
)

# save data for the gps system
np.savez('camera_calibration_data.npz', matrix=matrix, distortion=distortion)
print("calibration successful and saved")

