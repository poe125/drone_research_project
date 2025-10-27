import cv2

cap0 = cv2.VideoCapture(0)  # 内蔵
cap1 = cv2.VideoCapture(1)  # 外付け

while True:
    ret0, frame0 = cap0.read()
    ret1, frame1 = cap1.read()

    if ret0:
        cv2.imshow("Camera 0", frame0)
    if ret1:
        cv2.imshow("Camera 1", frame1)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap0.release()
cap1.release()
cv2.destroyAllWindows()
