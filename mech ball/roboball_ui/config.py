# config.py
import cv2
import cv2.aruco as aruco

# ----------------------------
# Camera
# ----------------------------
CAM_INDEX = 1
FRAME_W = 1280
FRAME_H = 720

TRY_DISABLE_AUTO_EXPOSURE = True
MANUAL_EXPOSURE = -6
MANUAL_GAIN = 0

# ----------------------------
# ArUco
# ----------------------------
ARUCO_DICT = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
ARUCO_PARAMS = aruco.DetectorParameters()

# IDs
CRITICAL_ID = 1
ROBOT1_ID = 1
ROBOT2_ID = 145

# ----------------------------
# UDP
# ----------------------------
UDP_IP = "138.38.229.153"

UDP_PORT_CRITICAL = 671
TRIGGER_BYTE = b"\x01"
PULSE_DURATION_S = 0.25
REARM_TIMEOUT_S = 5.0
CRITICAL_SEND_RATE_HZ = 30.0

UDP_PORT_ANGLE = 672
SEND_RATE_HZ_ANGLE = 50.0
FLIP_ANGLE_SIGN = False
HEADING_OFFSET_RAD = 0.0

UDP_PORT_CRASH = 673
MIN_PULSE_PERIOD_S = 0.25

# Crash overlay
CRASH_FLASH_S = 1.2
CRASH_FLASH_HZ = 6.0

# ----------------------------
# Tracking / stability defaults
# ----------------------------
USE_CLAHE = True
USE_BILATERAL = True

MAX_HISTORY = 30
ALPHA_POS = 0.35
MIN_STEP_PX = 2

ID_VOTE_WINDOW = 7
ID_MIN_VOTES = 4
GATE_DIST_PX = 25
LOST_TIMEOUT_S = 0.35

PATH_LEN_PX = 800

# Robot speed model
SPHERE_SPEED_M_S = 1.66

# IP stability
IP_STABLE_PX = 6.0
IP_STABLE_FRAMES = 10
IP_LOST_RESET_S = 0.30

ANGLE_SENT_SHOW_S = 2.0
