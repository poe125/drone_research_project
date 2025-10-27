# stereo_aruco_live.py
import cv2
import numpy as np
import time
import os

CALIB_FILE = "calib_data/calibration.npz"

def live_stereo_calibrate(CHECKERBOARD=(9,6), num_pairs=12, wait_sec=2.0):
    capL = cv2.VideoCapture(2) # change depending on your camera inputs
    capR = cv2.VideoCapture(1) # change depending on your camera inputs
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    objp = np.zeros((CHECKERBOARD[0]*CHECKERBOARD[1],3), np.float32)
    objp[:,:2] = np.mgrid[0:CHECKERBOARD[0],0:CHECKERBOARD[1]].T.reshape(-1,2)

    objpoints = []
    imgpointsL = []
    imgpointsR = []

    last = 0
    print(">> show checkerboard (quit with 'q')")

    while len(objpoints) < num_pairs:
        retL, fL = capL.read()
        retR, fR = capR.read()
        if not (retL and retR):
            raise RuntimeError("No camera found")
        gL = cv2.cvtColor(fL, cv2.COLOR_BGR2GRAY)
        gR = cv2.cvtColor(fR, cv2.COLOR_BGR2GRAY)

        rL, cL = cv2.findChessboardCorners(gL, CHECKERBOARD, None)
        rR, cR = cv2.findChessboardCorners(gR, CHECKERBOARD, None)
        both = cv2.hconcat([fL, fR])
        cv2.imshow("Live Stereo Calib (L | R)", both)

        if rL and rR and (time.time() - last > wait_sec):
            cL2 = cv2.cornerSubPix(gL, cL, (11,11), (-1,-1), criteria)
            cR2 = cv2.cornerSubPix(gR, cR, (11,11), (-1,-1), criteria)
            objpoints.append(objp.copy())
            imgpointsL.append(cL2)
            imgpointsR.append(cR2)
            last = time.time()
            print(f"  captured {len(objpoints)}/{num_pairs}")

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    capL.release()
    capR.release()
    cv2.destroyAllWindows()

    if len(objpoints) < 3:
        raise RuntimeError("lacking available frames")

    print(">> self calibration start...")
    _, K1, dist1, _, _ = cv2.calibrateCamera(objpoints, imgpointsL, gL.shape[::-1], None, None)
    _, K2, dist2, _, _ = cv2.calibrateCamera(objpoints, imgpointsR, gR.shape[::-1], None, None)

    print(">> stereo calibration start...")
    flags = cv2.CALIB_FIX_INTRINSIC
    retval, _, _, _, _, R, T, E, F = cv2.stereoCalibrate(
        objpoints, imgpointsL, imgpointsR,
        K1, dist1, K2, dist2, gL.shape[::-1],
        criteria=criteria, flags=flags
    )
    print(">> calibration finished")
    return K1, dist1, K2, dist2, R, T

def save_calibration(K1, dist1, K2, dist2, R, T):
    np.savez(CALIB_FILE, K1=K1, dist1=dist1, K2=K2, dist2=dist2, R=R, T=T)
    
def load_calibration():
    if os.path.exists(CALIB_FILE):
        data = np.load(CALIB_FILE)
        print("📂 既存のキャリブデータをロードしました。")
        return data["K1"], data["dist1"], data["K2"], data["dist2"], data["R"], data["T"]
    else:
        print("⚠️ キャリブデータが存在しません。新規作成します。")
        K1, dist1, K2, dist2, R, T = live_stereo_calibrate(CHECKERBOARD=(9,6), num_pairs=12)
        calib_dir = os.path.dirname(CALIB_FILE)
        if not os.path.exists(calib_dir):
            os.makedirs(calib_dir)
        np.savez(CALIB_FILE, K1=K1, dist1=dist1, K2=K2, dist2=dist2, R=R, T=T)
        print("💾 キャリブデータを保存しました。")
        return K1, dist1, K2, dist2, R, T

def live_stereo_aruco_height(K1, dist1, K2, dist2, R, T, marker_length=0.05, camera_height=1.2):
    # Projection matrices
    P1 = np.hstack((np.eye(3), np.zeros((3,1))))         
    P1 = K1 @ P1
    RT = np.hstack((R, T.reshape(3,1)))                 
    P2 = K2 @ RT

    capL = cv2.VideoCapture(2)
    capR = cv2.VideoCapture(1)

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, params)

    print(">> Start AruCo tracking (quit with 'q')")

    marker_initial_height = {}  # {id: initial height}
    marker_previous_height = {} # {id: prior height}

    while True:
        retL, fL = capL.read()
        retR, fR = capR.read()
        if not (retL and retR):
            print("Failed to get Camera")
            break

        cornersL, idsL, _ = detector.detectMarkers(fL)
        cornersR, idsR, _ = detector.detectMarkers(fR)

        if idsL is not None:
            cv2.aruco.drawDetectedMarkers(fL, cornersL, idsL)
        if idsR is not None:
            cv2.aruco.drawDetectedMarkers(fR, cornersR, idsR)
            
        # Find matching ID and use triangulation
        if idsL is not None and idsR is not None:
            idsL_list = idsL.flatten().tolist()
            idsR_list = idsR.flatten().tolist()
            common_ids = set(idsL_list).intersection(idsR_list)
            for marker_id in common_ids:
                idxL = idsL_list.index(marker_id)
                idxR = idsR_list.index(marker_id)
                cL = cornersL[idxL][0]
                cR = cornersR[idxR][0]

                uL = cL.mean(axis=0)
                uR = cR.mean(axis=0)

                pts1 = np.array([[uL[0]],[uL[1]]], dtype=float)
                pts2 = np.array([[uR[0]],[uR[1]]], dtype=float)

                X_hom = cv2.triangulatePoints(P1, P2, pts1, pts2)
                X = (X_hom[:3] / X_hom[3]).reshape(3)
                X_m, Y_m, Z_m = X[0], X[1], X[2]

                # Text to show on the console
                text_on_screen = f"ID {marker_id} | X={X_m:.2f} Y={Y_m:.2f} Z={Z_m:.2f} m"
                # if camera height is known, calculate
                if camera_height is not None:
                    height_from_ground = camera_height - Y_m

                    if marker_id not in marker_initial_height:
                        marker_initial_height[marker_id] = height_from_ground
                    if marker_id in marker_previous_height:
                        delta_prev = (height_from_ground - marker_previous_height[marker_id]) * 100
                        delta_prev_text = f"({delta_prev:+.1f}cm)"
                    else:
                        delta_prev_text = "(Init)"
                        delta_prev = 0

                    delta_start = (height_from_ground - marker_initial_height[marker_id]) * 100
                    print(f"ID {marker_id} | X={X_m:.3f} Y={Y_m:.3f} Z={Z_m:.3f} | Δ_start={delta_start:+.1f}cm {delta_prev_text}")
                    text_on_screen += f" Δ={delta_start:+.1f}cm"
                    marker_previous_height[marker_id] = height_from_ground

                # Show on screen
                cx, cy = int(uL[0]), int(uL[1])
                cv2.putText(fL, text_on_screen, (cx-50, cy-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

        both = cv2.hconcat([fL, fR])
        cv2.imshow("Stereo ArUco (L | R)", both)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    capL.release()
    capR.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    K1, dist1, K2, dist2, R, T = load_calibration()
    live_stereo_aruco_height(K1, dist1, K2, dist2, R, T, marker_length=0.05, camera_height=1.2)
