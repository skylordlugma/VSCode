# vision_helpers.py
import math
import numpy as np
import cv2
from collections import Counter

def centroid_from_corners(corners_4x2: np.ndarray) -> np.ndarray:
    return np.mean(corners_4x2, axis=0)

def dist2(a, b):
    dx = float(a[0] - b[0])
    dy = float(a[1] - b[1])
    return dx * dx + dy * dy

def majority_vote(id_deque):
    if not id_deque:
        return None, 0
    c = Counter(id_deque)
    winner, votes = c.most_common(1)[0]
    return int(winner), int(votes)

def estimate_velocity(history_deque):
    if len(history_deque) < 6:
        return None
    t = np.array([h[0] for h in history_deque], dtype=np.float64)
    p = np.array([h[1] for h in history_deque], dtype=np.float64)
    t0 = t[0]
    tt = t - t0
    A = np.vstack([tt, np.ones_like(tt)]).T
    vx, _bx = np.linalg.lstsq(A, p[:, 0], rcond=None)[0]
    vy, _by = np.linalg.lstsq(A, p[:, 1], rcond=None)[0]
    return np.array([float(vx), float(vy)], dtype=np.float64)

def wrap_to_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi

def angle_from_vec(vx: float, vy: float) -> float:
    return math.atan2(vy, vx)

def draw_x(img, pt, size=12, color=(255, 0, 255), thickness=3):
    x, y = pt
    cv2.line(img, (x - size, y - size), (x + size, y + size), color, thickness, cv2.LINE_AA)
    cv2.line(img, (x - size, y + size), (x + size, y - size), color, thickness, cv2.LINE_AA)

def draw_banner(out_img, text, top_left=(20, 20), pad=10, bg_color=(0, 0, 200), text_color=(255, 255, 255)):
    x, y = top_left
    (tw, th), baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.85, 2)
    w = tw + 2 * pad
    h = th + baseline + 2 * pad
    cv2.rectangle(out_img, (x, y), (x + w, y + h), bg_color, -1)
    cv2.putText(out_img, text, (x + pad, y + pad + th),
                cv2.FONT_HERSHEY_SIMPLEX, 0.85, text_color, 2, cv2.LINE_AA)

def compute_intercept_point(p1, v1, p2, chaser_speed, t_max=5.0):
    r = (p1 - p2).astype(np.float64)
    v = v1.astype(np.float64)
    s = float(chaser_speed)

    a = float(np.dot(v, v) - s * s)
    b = float(2.0 * np.dot(r, v))
    c = float(np.dot(r, r))

    if abs(a) < 1e-9:
        if abs(b) < 1e-9:
            return None, None
        t = -c / b
        if t > 0.0:
            t = min(t, t_max)
            return (p1 + v1 * t).astype(np.float64), float(t)
        return None, None

    disc = b * b - 4.0 * a * c
    if disc < 0.0:
        return None, None

    sqrt_disc = math.sqrt(disc)
    t1 = (-b - sqrt_disc) / (2.0 * a)
    t2 = (-b + sqrt_disc) / (2.0 * a)

    candidates = [t for t in (t1, t2) if t > 0.0]
    if not candidates:
        return None, None

    t = min(candidates)
    if t > t_max:
        return None, None

    ip = (p1 + v1 * t).astype(np.float64)
    return ip, float(t)
