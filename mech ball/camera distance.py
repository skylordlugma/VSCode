import cv2
import cv2.aruco as aruco
import numpy as np
import time
from collections import deque

# ----------------------------
# CONFIG
# ----------------------------
ROBOT1_ID = 8              # <-- set Robot 1's ArUco ID
MAX_HISTORY = 30           # how many past points to keep for velocity estimate
MIN_STEP_PX = 2            # ignore jitter smaller than this
ALPHA_POS = 0.35           # smoothing factor for position (0..1); higher = less smoothing

PREDICT_HORIZON_S = 2.0    # predict this many seconds into the future
PREDICT_DT_S = 0.1         # spacing between predicted points (polyline resolution)

# ----------------------------
# Camera calibration (not required for pixel prediction, but kept for consistency)
# ----------------------------
camera_calibration = np.load('Calibration.npz')
CM = camera_calibration['CM']
dist_coef = camera_calibration['dist_coef']

# ----------------------------
# ArUco setup
# ----------------------------
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters()

cap = cv2.VideoCapture(1)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

cv2.namedWindow("frame", cv2.WINDOW_AUTOSIZE)

# ----------------------------
# State for Robot 1
# ----------------------------
history = deque(maxlen=MAX_HISTORY)  # elements: (t, (x,y))
trail = deque(maxlen=300)            # for drawing past path

pos_filt = None
last_added_pt = None

def centroid_from_corners(corners_4x2: np.ndarray) -> np.ndarray:
    """Return centroid as float array [x,y] from marker corners (4,2)."""
    return np.mean(corners_4x2, axis=0)

def maybe_append_trail(pt_int):
    global last_added_pt
    if last_added_pt is None:
        trail.append(pt_int)
        last_added_pt = pt_int
        return
    dx = pt_int[0] - last_added_pt[0]
    dy = pt_int[1] - last_added_pt[1]
    if dx*dx + dy*dy >= MIN_STEP_PX*MIN_STEP_PX:
        trail.append(pt_int)
        last_added_pt = pt_int

def estimate_velocity(history_deque):
    """
    Estimate pixel velocity (vx, vy) using a simple least-squares fit
    of x(t), y(t) over recent history.
    Returns (vx, vy) in px/s, or None if insufficient data.
    """
    if len(history_deque) < 5:
        return None

    t = np.array([h[0] for h in history_deque], dtype=np.float64)
    p = np.array([h[1] for h in history_deque], dtype=np.float64)  # shape (N,2)

    # Improve conditioning: shift time so t0=0
    t0 = t[0]
    tt = t - t0

    # Fit x = a*tt + b; y = c*tt + d
    A = np.vstack([tt, np.ones_like(tt)]).T  # (N,2)
    vx, _bx = np.linalg.lstsq(A, p[:, 0], rcond=None)[0]
    vy, _by = np.linalg.lstsq(A, p[:, 1], rcond=None)[0]
    return float(vx), float(vy)

while True:
    loop_start = time.perf_counter()

    ret, frame = cap.read()
    if not ret or frame is None:
        print("Failed to grab frame.")
        break

    now = time.perf_counter()

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    out = frame.copy()

    robot1_seen = False
    current_pt = None

    if ids is not None and len(ids) > 0:
        ids_flat = ids.flatten()
        aruco.drawDetectedMarkers(out, corners, ids)

        for i, mid in enumerate(ids_flat):
            if int(mid) != ROBOT1_ID:
                continue

            robot1_seen = True
            c = corners[i][0]  # (4,2)
            pt = centroid_from_corners(c)  # float [x,y] pixels

            # Position smoothing (EMA)
            if pos_filt is None:
                pos_filt = pt.copy()
            else:
                pos_filt = (1.0 - ALPHA_POS) * pos_filt + ALPHA_POS * pt

            current_pt = pos_filt

            # Save for velocity estimation
            history.append((now, current_pt.copy()))

            # Add to trail (as ints)
            pt_int = (int(current_pt[0]), int(current_pt[1]))
            maybe_append_trail(pt_int)

            # Draw current point + label
            cv2.circle(out, pt_int, 6, (0, 255, 0), -1)
            cv2.putText(out, f"Robot1 ID:{ROBOT1_ID}", (pt_int[0] + 10, pt_int[1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)

            break  # assume only one instance of Robot1 marker

    # Draw past trail
    if len(trail) >= 2:
        pts = np.array(trail, dtype=np.int32).reshape((-1, 1, 2))
        cv2.polylines(out, [pts], False, (0, 255, 255), 2)  # yellow-ish

    # Predict and draw future path
    v = estimate_velocity(history)
    if robot1_seen and current_pt is not None and v is not None:
        vx, vy = v  # px/s

        pred_pts = []
        steps = int(PREDICT_HORIZON_S / PREDICT_DT_S)
        for k in range(1, steps + 1):
            tau = k * PREDICT_DT_S
            px = current_pt[0] + vx * tau
            py = current_pt[1] + vy * tau
            pred_pts.append((int(px), int(py)))

        # Draw predicted polyline
        if len(pred_pts) >= 2:
            pred_arr = np.array(pred_pts, dtype=np.int32).reshape((-1, 1, 2))
            cv2.polylines(out, [pred_arr], False, (255, 0, 255), 3)  # magenta prediction line

        # Draw intercept point marker (end of horizon)
        ip = pred_pts[-1]
        cv2.circle(out, ip, 8, (255, 0, 255), -1)
        cv2.putText(out, "Pred end", (ip[0] + 10, ip[1]),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2, cv2.LINE_AA)

        # Display velocity
        cv2.putText(out, f"v(px/s)=({vx:.1f},{vy:.1f})",
                    (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2, cv2.LINE_AA)
    else:
        cv2.putText(out, "Prediction: waiting for enough data / marker",
                    (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2, cv2.LINE_AA)

    # HUD
    loop_end = time.perf_counter()
    fps = 1.0 / (loop_end - loop_start + 1e-9)
    cv2.putText(out, f"FPS: {fps:.1f} | press 'c' clear, 'q' quit",
                (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2, cv2.LINE_AA)

    cv2.imshow("frame", out)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    if key == ord('c'):
        history.clear()
        trail.clear()
        pos_filt = None
        last_added_pt = None

cap.release()
cv2.destroyAllWindows()
exit(0)
