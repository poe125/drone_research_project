import cv2
import os

# ==== 保存フォルダの設定 ====
os.makedirs("images/left", exist_ok=True)
os.makedirs("images/right", exist_ok=True)

# ==== カメラ設定 ====
capL = cv2.VideoCapture(0)  # 左カメラ
capR = cv2.VideoCapture(1)  # 右カメラ

# 画像番号カウンタ
idx = 0

print("スペースキーでキャプチャ、qで終了")

while True:
    retL, frameL = capL.read()
    retR, frameR = capR.read()
    if not (retL and retR):
        print("カメラが認識されていません。接続を確認してください。")
        break

    # 並べて表示
    both = cv2.hconcat([frameL, frameR])
    cv2.imshow("Stereo Cameras (Left | Right)", both)

    key = cv2.waitKey(1) & 0xFF

    if key == ord(' '):  # スペースキーで保存
        filenameL = f"images/left/{idx:02d}.png"
        filenameR = f"images/right/{idx:02d}.png"
        cv2.imwrite(filenameL, frameL)
        cv2.imwrite(filenameR, frameR)
        print(f"Saved pair {idx:02d}")
        idx += 1

    elif key == ord('q'):  # qで終了
        break

capL.release()
capR.release()
cv2.destroyAllWindows()
