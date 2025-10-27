import cv2

# 0 はカメラデバイス番号（内蔵カメラは通常0、外付けなら1,2…）
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("カメラが開けませんでした")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("フレームが取得できませんでした")
        break

    # 画面に表示
    cv2.imshow('Camera', frame)

    # 'q' を押したら終了
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
