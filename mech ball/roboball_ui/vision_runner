# vision_runner.py
import time, math, socket, struct, threading
from collections import deque

import cv2
import numpy as np
import cv2.aruco as aruco

from config import (
    CAM_INDEX, FRAME_W, FRAME_H, TRY_DISABLE_AUTO_EXPOSURE,
    ARUCO_DICT, ARUCO_PARAMS,
    MANUAL_EXPOSURE, MANUAL_GAIN,
    UDP_IP, UDP_PORT_CRITICAL, UDP_PORT_ANGLE, UDP_PORT_CRASH,
    TRIGGER_BYTE, PULSE_DURATION_S, REARM_TIMEOUT_S,
    CRITICAL_SEND_RATE_HZ, SEND_RATE_HZ_ANGLE,
    FLIP_ANGLE_SIGN, HEADING_OFFSET_RAD,
    MIN_PULSE_PERIOD_S,
    CRASH_FLASH_S, CRASH_FLASH_HZ,
    MAX_HISTORY, ALPHA_POS, MIN_STEP_PX,
    ID_MIN_VOTES, GATE_DIST_PX, LOST_TIMEOUT_S,
    PATH_LEN_PX,
    IP_STABLE_PX, IP_STABLE_FRAMES, IP_LOST_RESET_S,
    ANGLE_SENT_SHOW_S,
    CRITICAL_ID, ROBOT1_ID, ROBOT2_ID,
    USE_CLAHE, USE_BILATERAL,
    SPHERE_SPEED_M_S,
)

from shared_params import SharedParams
from vision_helpers import (
    centroid_from_corners, dist2, majority_vote, estimate_velocity,
    wrap_to_pi, angle_from_vec, draw_x, draw_banner, compute_intercept_point
)

class VisionRunner:
    def __init__(self, params: SharedParams, on_frame=None, on_status=None):
        self.params = params
        self.on_frame = on_frame
        self.on_status = on_status

        self.stop_evt = threading.Event()
        self.started = False

        # CLAHE instance
        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)) if USE_CLAHE else None

        # UDP sockets
        self.sock_critical = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_angle = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_crash = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.critical_send_interval = 1.0 / max(1.0, CRITICAL_SEND_RATE_HZ)
        self.angle_send_min_dt = 1.0 / max(1.0, SEND_RATE_HZ_ANGLE)

        # camera
        self.cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_DSHOW)
        if not self.cap.isOpened():
            raise RuntimeError(f"Camera not opened. CAM_INDEX={CAM_INDEX}")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)

        if TRY_DISABLE_AUTO_EXPOSURE:
            self._apply_camera_settings(force=True)

        self.full_reset()

    # ---------------- controls ----------------
    def set_started(self, v: bool):
        self.started = bool(v)

    def request_stop(self):
        self.stop_evt.set()

    def clear_tracking_only(self):
        self.hist1.clear()
        self.hist2.clear()
        self.trail1.clear()
        self.pos1_filt = None
        self.pos2_filt = None
        self.last_trail1_pt = None
        self.id_votes_r1.clear()
        self.id_votes_r2.clear()
        self.last_seen_r1_t = None
        self.last_seen_r2_t = None

        self.ip_last = None
        self.ip_stable_count = 0
        self.ip_last_seen_t = None
        self.tracking_armed = False

        self.last_sent_angle = None
        self.last_sent_time = None
        self.last_udp_sent_time = None

        self.angle_zero_count = 0
        self.angle_zero_latched = False

        self.last_crash_pulse_t = 0.0
        self.crash_latched = False
        self.last_crash_event_t = None

    def full_reset(self):
        self.started = False

        self.armed = True
        self.pulse_active = False
        self.pulse_start_time = None
        self.last_seen_critical_t = None
        self.last_critical_send_t = 0.0
        self.unlocked = False

        self.hist1 = deque(maxlen=MAX_HISTORY)
        self.hist2 = deque(maxlen=MAX_HISTORY)

        self.trail1 = deque(maxlen=400)
        self.pos1_filt = None
        self.pos2_filt = None
        self.last_trail1_pt = None

        self.id_votes_r1 = deque(maxlen=max(1, self.params.get().id_vote_window))
        self.id_votes_r2 = deque(maxlen=max(1, self.params.get().id_vote_window))

        self.last_seen_r1_t = None
        self.last_seen_r2_t = None

        self.ip_last = None
        self.ip_stable_count = 0
        self.ip_last_seen_t = None
        self.tracking_armed = False

        self.last_sent_angle = None
        self.last_sent_time = None
        self.last_udp_sent_time = None

        self.angle_zero_count = 0
        self.angle_zero_latched = False

        self.last_crash_pulse_t = 0.0
        self.crash_latched = False
        self.last_crash_event_t = None

    # ---------------- internals ----------------
    def _apply_camera_settings(self, force=False):
        p = self.params.get()
        # Only set if force or values differ significantly
        try:
            self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
            self.cap.set(cv2.CAP_PROP_EXPOSURE, float(p.exposure))
            self.cap.set(cv2.CAP_PROP_GAIN, float(p.gain))
        except Exception:
            pass

    def _preprocess_gray(self, frame_bgr: np.ndarray) -> np.ndarray:
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        if USE_CLAHE and self.clahe is not None:
            gray = self.clahe.apply(gray)
        if USE_BILATERAL:
            gray = cv2.bilateralFilter(gray, 5, 50, 50)
        return gray

    def _send_crash_pulse(self, now_s: float):
        self.sock_crash.sendto(struct.pack("B", 1), (UDP_IP, UDP_PORT_CRASH))
        self.last_crash_pulse_t = now_s
        self.last_crash_event_t = now_s

    def _status(self, **overrides):
        # Build a small status dict (QoL: no wall of text)
        base = dict(
            started=self.started,
            unlocked=self.unlocked,
            pulse_active=self.pulse_active,
            tracking_armed=self.tracking_armed,
            zero_latched=self.angle_zero_latched,
            last_angle=self.last_sent_angle,
            crash_flash=(self.last_crash_event_t is not None) and ((time.time() - self.last_crash_event_t) <= CRASH_FLASH_S),
        )
        base.update(overrides)
        if self.on_status:
            self.on_status(base)

    def shutdown(self):
        try:
            self.cap.release()
        except Exception:
            pass
        for s in (self.sock_critical, self.sock_angle, self.sock_crash):
            try:
                s.close()
            except Exception:
                pass

    # ---------------- main loop ----------------
    def run(self):
        last_status_t = 0.0
        status_hz = 10.0
        status_dt = 1.0 / status_hz

        while not self.stop_evt.is_set():
            loop_start = time.perf_counter()

            # Apply live camera settings occasionally (cheap)
            self._apply_camera_settings()

            ret, frame = self.cap.read()
            if not ret or frame is None:
                self._status(error="Camera frame grab failed")
                time.sleep(0.05)
                continue

            now = time.perf_counter()
            now_s = time.time()

            p = self.params.get()

            # update vote window live
            if self.id_votes_r1.maxlen != max(1, p.id_vote_window):
                self.id_votes_r1 = deque(self.id_votes_r1, maxlen=max(1, p.id_vote_window))
                self.id_votes_r2 = deque(self.id_votes_r2, maxlen=max(1, p.id_vote_window))

            gray = self._preprocess_gray(frame)
            corners, ids, rejected = aruco.detectMarkers(gray, ARUCO_DICT, parameters=ARUCO_PARAMS)
            out = frame.copy()

            ids_flat = None
            if ids is not None and len(ids) > 0:
                ids_flat = ids.flatten().astype(int)
                aruco.drawDetectedMarkers(out, corners, ids)

            # ========== UI gate ==========
            if not self.started:
                draw_banner(out, "SYSTEM ARMED — PRESS START", top_left=(20, 20), bg_color=(0, 0, 200))
                self._emit_frame(out)
                self._throttled_status(loop_start, rejected, last_status_t, status_dt)
                last_status_t = time.perf_counter()
                continue

            # ========== critical watch ==========
            critical_seen = (ids_flat is not None) and (CRITICAL_ID in ids_flat)
            if critical_seen:
                self.last_seen_critical_t = now
                if self.armed and not self.pulse_active:
                    self.pulse_active = True
                    self.pulse_start_time = now
                    self.last_critical_send_t = 0.0
                    self.armed = False
                    self.unlocked = True

            if self.pulse_active:
                if (now - self.pulse_start_time) <= PULSE_DURATION_S:
                    if (now - self.last_critical_send_t) >= self.critical_send_interval:
                        self.sock_critical.sendto(TRIGGER_BYTE, (UDP_IP, UDP_PORT_CRITICAL))
                        self.last_critical_send_t = now
                else:
                    self.pulse_active = False

            if (not critical_seen) and (not self.armed):
                if self.last_seen_critical_t is not None and (now - self.last_seen_critical_t) >= REARM_TIMEOUT_S:
                    self.armed = True

            cv2.putText(out, f"STATE: {'UNLOCKED' if self.unlocked else 'LOCKED'}",
                        (20, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255,255,255), 2, cv2.LINE_AA)

            if not self.unlocked:
                cv2.putText(out, "Awaiting Criti-Bot (critical marker) to unlock...",
                            (20, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0,0,255), 2, cv2.LINE_AA)
                self._emit_frame(out)
                self._throttled_status(loop_start, rejected, last_status_t, status_dt)
                last_status_t = time.perf_counter()
                continue

            # ========== unlocked: tracking ==========
            seen1 = seen2 = False
            p1 = p2 = None
            r2_corners = None

            detections = []
            if ids_flat is not None:
                for i, mid in enumerate(ids_flat):
                    pt = centroid_from_corners(corners[i][0])
                    detections.append((int(mid), pt, i))
                for mid, _, _ in detections:
                    if mid == ROBOT1_ID: self.id_votes_r1.append(mid)
                    if mid == ROBOT2_ID: self.id_votes_r2.append(mid)

                r1_id, r1_votes = majority_vote(self.id_votes_r1)
                r2_id, r2_votes = majority_vote(self.id_votes_r2)
                r1_allowed = (r1_id == ROBOT1_ID and r1_votes >= ID_MIN_VOTES)
                r2_allowed = (r2_id == ROBOT2_ID and r2_votes >= ID_MIN_VOTES)

                def pick(expected_id, last_pos, allowed):
                    if not detections:
                        return None
                    candidates = [(mid, pt, idx) for (mid, pt, idx) in detections if mid == expected_id]
                    if allowed and candidates:
                        if last_pos is None:
                            return candidates[0]
                        candidates.sort(key=lambda x: dist2(x[1], last_pos))
                        return candidates[0]
                    if last_pos is not None:
                        close = [(mid, pt, idx) for (mid, pt, idx) in detections
                                 if dist2(pt, last_pos) <= GATE_DIST_PX * GATE_DIST_PX]
                        if close:
                            close.sort(key=lambda x: dist2(x[1], last_pos))
                            return close[0]
                    return None

                pick1 = pick(ROBOT1_ID, self.pos1_filt, r1_allowed)
                pick2 = pick(ROBOT2_ID, self.pos2_filt, r2_allowed)

                if pick1 is not None:
                    _, pt, _ = pick1
                    seen1 = True
                    self.last_seen_r1_t = now
                    self.pos1_filt = pt.copy() if self.pos1_filt is None else (1-ALPHA_POS)*self.pos1_filt + ALPHA_POS*pt
                    p1 = self.pos1_filt
                    self.hist1.append((now, p1.copy()))
                    p1i = (int(p1[0]), int(p1[1]))
                    cv2.circle(out, p1i, 6, (0,255,0), -1)

                    if self.last_trail1_pt is None:
                        self.trail1.append(p1i); self.last_trail1_pt = p1i
                    else:
                        ddx = p1i[0]-self.last_trail1_pt[0]
                        ddy = p1i[1]-self.last_trail1_pt[1]
                        if ddx*ddx+ddy*ddy >= MIN_STEP_PX*MIN_STEP_PX:
                            self.trail1.append(p1i); self.last_trail1_pt = p1i

                if pick2 is not None:
                    _, pt, idx = pick2
                    seen2 = True
                    self.last_seen_r2_t = now
                    r2_corners = corners[idx][0]
                    self.pos2_filt = pt.copy() if self.pos2_filt is None else (1-ALPHA_POS)*self.pos2_filt + ALPHA_POS*pt
                    p2 = self.pos2_filt
                    self.hist2.append((now, p2.copy()))
                    p2i = (int(p2[0]), int(p2[1]))
                    cv2.circle(out, p2i, 6, (255,255,0), -1)

            if (not seen1) and self.last_seen_r1_t is not None and (now - self.last_seen_r1_t) <= LOST_TIMEOUT_S:
                p1 = self.pos1_filt; seen1 = p1 is not None
            if (not seen2) and self.last_seen_r2_t is not None and (now - self.last_seen_r2_t) <= LOST_TIMEOUT_S:
                p2 = self.pos2_filt; seen2 = p2 is not None

            if len(self.trail1) >= 2:
                pts = np.array(self.trail1, dtype=np.int32).reshape((-1,1,2))
                cv2.polylines(out, [pts], False, (0,255,255), 2)

            # ========== crash ==========
            if (p1 is not None) and (p2 is not None):
                d = math.hypot(float(p2[0]-p1[0]), float(p2[1]-p1[1]))
                cv2.circle(out, (int(p1[0]), int(p1[1])), int(p.crash_radius_px), (0,200,200), 2)

                if (not self.crash_latched) and (d <= p.crash_radius_px):
                    if (now_s - self.last_crash_pulse_t) >= MIN_PULSE_PERIOD_S:
                        self._send_crash_pulse(now_s)
                    self.crash_latched = True
                elif self.crash_latched and (d >= (p.crash_radius_px + 8)):
                    self.crash_latched = False

            crash_flash_active = (self.last_crash_event_t is not None) and ((now_s - self.last_crash_event_t) <= CRASH_FLASH_S)
            if crash_flash_active:
                phase = math.sin(2.0 * math.pi * CRASH_FLASH_HZ * (now_s - self.last_crash_event_t))
                if phase > 0.0:
                    overlay = out.copy()
                    cv2.rectangle(overlay, (0,0), (FRAME_W, FRAME_H), (0,0,255), -1)
                    out = cv2.addWeighted(overlay, 0.45, out, 0.55, 0)
                    cv2.putText(out, "IMPACT!", (40, 140), cv2.FONT_HERSHEY_SIMPLEX, 3.0, (255,255,255), 10, cv2.LINE_AA)

            # ========== intercept + angle ==========
            if self.ip_last_seen_t is not None and (now - self.ip_last_seen_t) > IP_LOST_RESET_S:
                self.ip_stable_count = 0
                self.ip_last = None
                self.ip_last_seen_t = None
                self.tracking_armed = False
                self.angle_zero_count = 0
                self.angle_zero_latched = False

            v1 = estimate_velocity(self.hist1)
            if seen1 and p1 is not None and v1 is not None:
                speed1 = float(np.linalg.norm(v1))
                if speed1 >= p.min_speed_px_s:
                    A = p1.astype(np.float64)
                    u = (v1 / (speed1 + 1e-12)).astype(np.float64)
                    B = (A + u * float(PATH_LEN_PX)).astype(np.float64)
                    cv2.line(out, (int(A[0]), int(A[1])), (int(B[0]), int(B[1])), (255,0,255), 3, cv2.LINE_AA)

                    if seen2 and p2 is not None:
                        P = p2.astype(np.float64)
                        chaser_speed_px_s = SPHERE_SPEED_M_S * float(p.px_per_m)

                        IP, t_hit = compute_intercept_point(A, v1, P, chaser_speed_px_s, t_max=20.0)
                        if IP is not None:
                            IPi = (int(IP[0]), int(IP[1]))
                            Pi = (int(P[0]), int(P[1]))
                            cv2.line(out, Pi, IPi, (255,0,255), 2, cv2.LINE_AA)
                            draw_x(out, IPi, size=14, color=(255,0,255), thickness=3)

                            self.ip_last_seen_t = now
                            if self.ip_last is None:
                                self.ip_last = IP.copy(); self.ip_stable_count = 0
                            else:
                                step = float(np.linalg.norm(IP - self.ip_last))
                                self.ip_stable_count = (self.ip_stable_count + 1) if (step <= IP_STABLE_PX) else 0
                                self.ip_last = IP.copy()

                            ip_is_stable = (self.ip_stable_count >= IP_STABLE_FRAMES)
                            if (not self.tracking_armed) and ip_is_stable:
                                self.tracking_armed = True

                            if self.tracking_armed and (r2_corners is not None):
                                can_send = (self.last_udp_sent_time is None) or ((now - self.last_udp_sent_time) >= self.angle_send_min_dt)
                                if can_send:
                                    c0 = r2_corners[0]
                                    c1 = r2_corners[1]
                                    hx = float(c1[0]-c0[0]); hy = float(c1[1]-c0[1])
                                    heading = wrap_to_pi(angle_from_vec(hx, hy) + HEADING_OFFSET_RAD)

                                    ddx = float(IP[0]-P[0]); ddy = float(IP[1]-P[1])
                                    desired = angle_from_vec(ddx, ddy)
                                    ang_err = wrap_to_pi(desired - heading)
                                    if FLIP_ANGLE_SIGN:
                                        ang_err = -ang_err

                                    # symmetric threshold (abs)
                                    if self.angle_zero_latched:
                                        ang_to_send = 0.0
                                    else:
                                        if abs(ang_err) <= float(p.angle_zero_thresh_rad):
                                            self.angle_zero_count += 1
                                            if self.angle_zero_count >= int(p.angle_zero_frames):
                                                self.angle_zero_latched = True
                                            ang_to_send = 0.0
                                        else:
                                            self.angle_zero_count = 0
                                            ang_to_send = float(ang_err)

                                    self.sock_angle.sendto(struct.pack("<f", float(ang_to_send)), (UDP_IP, UDP_PORT_ANGLE))
                                    self.last_udp_sent_time = now
                                    self.last_sent_angle = float(ang_to_send)
                                    self.last_sent_time = now

            if self.last_sent_time is not None and (now - self.last_sent_time) <= ANGLE_SENT_SHOW_S:
                draw_banner(out, f"AIM: {self.last_sent_angle:+.3f} rad", top_left=(20, 20), bg_color=(0, 160, 0))

            # emit
            self._emit_frame(out)
            self._throttled_status(loop_start, rejected, last_status_t, status_dt)
            last_status_t = time.perf_counter()

        self.shutdown()

    def _emit_frame(self, out_bgr: np.ndarray):
        if not self.on_frame:
            return
        rgb = cv2.cvtColor(out_bgr, cv2.COLOR_BGR2RGB)
        self.on_frame(rgb)

    def _throttled_status(self, loop_start, rejected, last_status_t, status_dt):
        nowt = time.perf_counter()
        if (nowt - last_status_t) < status_dt:
            return
        fps = 1.0 / (nowt - loop_start + 1e-9)
        rej_n = 0 if rejected is None else len(rejected)
        self._status(error=None, fps=float(fps), rejected_n=int(rej_n))
