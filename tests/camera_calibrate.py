import cv2
import numpy as np
import glob

# ===== 基本設定 =====
CHECKERBOARD = (9, 6)
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# ===== 単体カメラのキャリブレーション関数 =====
def calibrate_single_camera(image_dir, CHECKERBOARD=(10,7)):
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    objp = np.zeros((CHECKERBOARD[0]*CHECKERBOARD[1],3), np.float32)
    objp[:,:2] = np.mgrid[0:CHECKERBOARD[0],0:CHECKERBOARD[1]].T.reshape(-1,2)

    objpoints = []
    imgpoints = []
    gray_shape = None  # ← 安全用

    images = sorted(glob.glob(f'{image_dir}/*.png'))
    print(f"📸 {image_dir} にある画像枚数: {len(images)}")
    if len(images) == 0:
        raise RuntimeError(f"❌ {image_dir} に画像が見つかりませんでした。")

    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

        if ret:
            gray_shape = gray.shape[::-1]  # 成功したときだけ更新
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
            imgpoints.append(corners2)
        else:
            print(f"⚠️ コーナー検出失敗: {fname}")

    if len(objpoints) == 0:
        raise RuntimeError(f"❌ {image_dir} の画像からチェッカーボードが検出できませんでした。パターン数（{CHECKERBOARD}）を確認してください。")

    ret, cameraMatrix, distCoeffs, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, gray_shape, None, None
    )

    print(f"{image_dir} calibration done.")
    return ret, cameraMatrix, distCoeffs, objpoints, imgpoints


# ===== 左右カメラのキャリブレーション =====
_, cameraMatrix_left, distCoeffs_left, objpoints_left, imgpoints_left = calibrate_single_camera("images/left", CHECKERBOARD)
_, cameraMatrix_right, distCoeffs_right, objpoints_right, imgpoints_right = calibrate_single_camera("images/right", CHECKERBOARD)

# ===== ステレオキャリブレーション =====
# 同時に撮影した左右ペア画像をロード
images_left = sorted(glob.glob("images/left/*.png"))
images_right = sorted(glob.glob("images/right/*.png"))

objpoints = []
imgpointsL = []
imgpointsR = []

for left_img, right_img in zip(images_left, images_right):
    imgL = cv2.imread(left_img)
    imgR = cv2.imread(right_img)
    grayL = cv2.cvtColor(imgL, cv2.COLOR_BGR2GRAY)
    grayR = cv2.cvtColor(imgR, cv2.COLOR_BGR2GRAY)

    retL, cornersL = cv2.findChessboardCorners(grayL, CHECKERBOARD, None)
    retR, cornersR = cv2.findChessboardCorners(grayR, CHECKERBOARD, None)

    if retL and retR:
        objp = np.zeros((CHECKERBOARD[0]*CHECKERBOARD[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
        objpoints.append(objp)

        cornersL2 = cv2.cornerSubPix(grayL, cornersL, (11,11), (-1,-1), criteria)
        cornersR2 = cv2.cornerSubPix(grayR, cornersR, (11,11), (-1,-1), criteria)
        imgpointsL.append(cornersL2)
        imgpointsR.append(cornersR2)

ret, _, _, _, _, R, T, E, F = cv2.stereoCalibrate(
    objpoints, imgpointsL, imgpointsR,
    cameraMatrix_left, distCoeffs_left,
    cameraMatrix_right, distCoeffs_right,
    grayL.shape[::-1],
    criteria=criteria,
    flags=cv2.CALIB_FIX_INTRINSIC
)

print("✅ Stereo calibration finished!")
print("Rotation matrix (R):\n", R)
print("Translation vector (T):\n", T)
print("Essential matrix (E):\n", E)
print("Fundamental matrix (F):\n", F)
