import cv2
import numpy as np

# load calibration data
calib_data = np.load('camera_calibration_data.npz')
camera_matrix = calib_data['matrix']
dist_coeffs = calib_data['distortion']

# set marker size in meters
marker_size = 0.05

# setup aruco dictionary
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

# open usb webcam
cap = cv2.VideoCapture(1)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # detect markers
    corners, ids, rejected = detector.detectMarkers(gray)

    if ids is not None:
        # draw square around markers
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        for i in range(len(ids)):
            # get 3d pose
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners[i], marker_size, camera_matrix, dist_coeffs
            )
            
            # draw axis lines
            cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, marker_size / 2)
            
            # print coordinates
            x_pos = tvec[0][0][0]
            y_pos = tvec[0][0][1]
            print(f"id {ids[i][0]} x: {x_pos:.3f} y: {y_pos:.3f}")

    # show video
    cv2.imshow('gps tracking', frame)

    # press escape to exit
    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()