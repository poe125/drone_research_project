# stereo_aruco_live.py
import cv2 # type: ignore
import numpy as np # type: ignore
import calibration # type: ignore
import socket
import time

HOST = '127.0.0.1'
PORT = 5005

def live_stereo_aruco_height(K1, dist1, K2, dist2, R, T, marker_length=0.05, camera_height=1.2):
    # Projection matrices
    P1 = np.hstack((np.eye(3), np.zeros((3,1))))         
    P1 = K1 @ P1
    RT = np.hstack((R, T.reshape(3,1)))                 
    P2 = K2 @ RT

    capL = cv2.VideoCapture(0) #change this if "camera not found"
    capR = cv2.VideoCapture(1) #change this if "camera not found"

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
                    # print(f"ID {marker_id} | X={X_m:.3f} Y={Y_m:.3f} Z={Z_m:.3f} | Δ_start={delta_start:+.1f}cm {delta_prev_text}")
                    # text_on_screen += f" Δ={delta_start:+.1f}cm"
                    msg = f"{delta_start:+.1f}"
                    sock.sendall(msg.encode('utf-8'))
                    
                    marker_previous_height[marker_id] = height_from_ground

                # # Show on screen
                # cx, cy = int(uL[0]), int(uL[1])
                # cv2.putText(fL, text_on_screen, (cx-50, cy-10),
                #             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

        both = cv2.hconcat([fL, fR])
        cv2.imshow("Stereo ArUco (L | R)", both)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    capL.release()
    capR.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    # calibrate cameras
    # can take time here
    K1, dist1, K2, dist2, R, T = calibration.load_calibration()
    
    # start socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((HOST, PORT))

    # start sending data to c program. 
    live_stereo_aruco_height(K1, dist1, K2, dist2, R, T, marker_length=0.05, camera_height=1.2)
    
    #close socket
    sock.close()
