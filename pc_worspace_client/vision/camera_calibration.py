import cv2
import os

# create directory for images
save_folder = "calibration_images"
if not os.path.exists(save_folder):
    os.makedirs(save_folder)

# open usb webcam
cap = cv2.VideoCapture(1)

img_count = 0

# loop to capture frames continuously
while True:
    ret, frame = cap.read()

    if not ret:
        break

    # show video stream
    cv2.imshow("Calibration Capture", frame)

    key = cv2.waitKey(1) & 0xFF

    # press s key to save image
    if key == ord('s'):
        img_name = f"{save_folder}/calib_{img_count}.png"
        cv2.imwrite(img_name, frame)
        print(f"saved {img_name}")
        img_count += 1

    # press escape key to close
    elif key == 27:
        break

cap.release()
cv2.destroyAllWindows()