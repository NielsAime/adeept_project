import cv2
import numpy as np

# load calibration data
calib_data = np.load('camera_calibration_data.npz')
matrix = calib_data['matrix']
distortion = calib_data['distortion']

# open usb webcam
cap = cv2.VideoCapture(1)

while True:
    ret, frame = cap.read()
    
    if not ret:
        break

    # apply calibration to remove distortion
    undistorted_frame = cv2.undistort(frame, matrix, distortion, None, matrix)

    # display both streams
    cv2.imshow('original frame', frame)
    cv2.imshow('calibrated frame', undistorted_frame)

    # press escape key to close
    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()