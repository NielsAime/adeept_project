import cv2

# open default usb webcam
cap = cv2.VideoCapture(1)

while True:
    ret, frame = cap.read()

    if not ret:
        break

    # display video stream
    cv2.imshow("External Camera", frame)

    # press q key to close window
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()