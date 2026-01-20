# ui_app.py
import threading
import customtkinter as ctk
from PIL import Image, ImageTk

from shared_params import SharedParams, Params
from vision_runner import VisionRunner

class App(ctk.CTk):
    def __init__(self):
        super().__init__()
        ctk.set_appearance_mode("dark")
        ctk.set_default_color_theme("dark-blue")

        self.title("Robo-Ball vs Criti-Bot — Control Panel")
        self.geometry("1400x860")
        self.protocol("WM_DELETE_WINDOW", self.on_quit)

        # Shared params
        self.params = SharedParams(Params())

        # UI state
        self._tk_img = None
        self._latest_frame = None
        self._latest_status = None
        self._lock = threading.Lock()

        # Runner with callbacks
        self.runner = VisionRunner(
            params=self.params,
            on_frame=self._on_frame,
            on_status=self._on_status
        )
        self.worker = threading.Thread(target=self.runner.run, daemon=True)
        self.worker.start()

        self._build_layout()
        self.after(15, self._ui_tick)

    # ---------- callbacks from runner (thread) ----------
    def _on_frame(self, rgb_np):
        with self._lock:
            self._latest_frame = rgb_np

    def _on_status(self, status: dict):
        with self._lock:
            self._latest_status = status

    # ---------- UI ----------
    def _build_layout(self):
        self.grid_columnconfigure(0, weight=3)
        self.grid_columnconfigure(1, weight=1)
        self.grid_rowconfigure(0, weight=1)

        # video
        self.video_label = ctk.CTkLabel(self, text="")
        self.video_label.grid(row=0, column=0, padx=12, pady=12, sticky="nsew")

        # panel
        panel = ctk.CTkFrame(self)
        panel.grid(row=0, column=1, padx=12, pady=12, sticky="nsew")
        panel.grid_columnconfigure(0, weight=1)

        self.btn_start = ctk.CTkButton(panel, text="START ATTACK", height=44, command=self.on_start)
        self.btn_start.grid(row=0, column=0, sticky="ew", padx=12, pady=(14, 8))

        row = 1
        self.btn_clear = ctk.CTkButton(panel, text="CLEAR TRACK", command=self.on_clear)
        self.btn_clear.grid(row=row, column=0, sticky="ew", padx=12, pady=6); row += 1

        self.btn_reset = ctk.CTkButton(panel, text="RESET SYSTEM", command=self.on_reset)
        self.btn_reset.grid(row=row, column=0, sticky="ew", padx=12, pady=6); row += 1

        self.btn_quit = ctk.CTkButton(panel, text="QUIT", fg_color="#8b0000", command=self.on_quit)
        self.btn_quit.grid(row=row, column=0, sticky="ew", padx=12, pady=(6, 14)); row += 1

        self.crash_label = ctk.CTkLabel(panel, text="IMPACT: NO", font=ctk.CTkFont(size=22, weight="bold"))
        self.crash_label.grid(row=row, column=0, sticky="ew", padx=12, pady=(6, 12)); row += 1

        # ---- sliders / controls ----
        self._section(panel, "Tuning", row); row += 1

        # Crash radius
        self.crash_slider = ctk.CTkSlider(panel, from_=40, to=300, number_of_steps=260, command=self._set_crash)
        self.crash_slider.set(self.params.get().crash_radius_px)
        self.crash_slider.grid(row=row, column=0, sticky="ew", padx=12); row += 1
        self.crash_txt = ctk.CTkLabel(panel, text=f"Crash Radius: {self.params.get().crash_radius_px}px")
        self.crash_txt.grid(row=row, column=0, sticky="w", padx=12, pady=(0, 10)); row += 1

        # PX_PER_M
        self.pxpm_slider = ctk.CTkSlider(panel, from_=5, to=80, number_of_steps=150, command=self._set_pxpm)
        self.pxpm_slider.set(self.params.get().px_per_m)
        self.pxpm_slider.grid(row=row, column=0, sticky="ew", padx=12); row += 1
        self.pxpm_txt = ctk.CTkLabel(panel, text=f"PX_PER_M: {self.params.get().px_per_m:.1f}")
        self.pxpm_txt.grid(row=row, column=0, sticky="w", padx=12, pady=(0, 10)); row += 1

        # Min speed
        self.mins_slider = ctk.CTkSlider(panel, from_=0, to=80, number_of_steps=160, command=self._set_minspeed)
        self.mins_slider.set(self.params.get().min_speed_px_s)
        self.mins_slider.grid(row=row, column=0, sticky="ew", padx=12); row += 1
        self.mins_txt = ctk.CTkLabel(panel, text=f"Min Target Speed: {self.params.get().min_speed_px_s:.1f}px/s")
        self.mins_txt.grid(row=row, column=0, sticky="w", padx=12, pady=(0, 10)); row += 1

        # Vote window
        self.vote_slider = ctk.CTkSlider(panel, from_=1, to=15, number_of_steps=14, command=self._set_vote)
        self.vote_slider.set(self.params.get().id_vote_window)
        self.vote_slider.grid(row=row, column=0, sticky="ew", padx=12); row += 1
        self.vote_txt = ctk.CTkLabel(panel, text=f"ID Vote Window: {self.params.get().id_vote_window} frames")
        self.vote_txt.grid(row=row, column=0, sticky="w", padx=12, pady=(0, 10)); row += 1

        # Angle threshold
        self.ath_slider = ctk.CTkSlider(panel, from_=0.01, to=1.20, number_of_steps=119, command=self._set_ang_thresh)
        self.ath_slider.set(self.params.get().angle_zero_thresh_rad)
        self.ath_slider.grid(row=row, column=0, sticky="ew", padx=12); row += 1
        self.ath_txt = ctk.CTkLabel(panel, text=f"Aim-Lock Threshold: {self.params.get().angle_zero_thresh_rad:.2f} rad")
        self.ath_txt.grid(row=row, column=0, sticky="w", padx=12, pady=(0, 10)); row += 1

        # Exposure / gain
        self._section(panel, "Camera", row); row += 1

        self.exp_slider = ctk.CTkSlider(panel, from_=-12, to=0, number_of_steps=120, command=self._set_exposure)
        self.exp_slider.set(self.params.get().exposure)
        self.exp_slider.grid(row=row, column=0, sticky="ew", padx=12); row += 1
        self.exp_txt = ctk.CTkLabel(panel, text=f"Exposure: {self.params.get().exposure:.1f}")
        self.exp_txt.grid(row=row, column=0, sticky="w", padx=12, pady=(0, 10)); row += 1

        self.gain_slider = ctk.CTkSlider(panel, from_=0, to=30, number_of_steps=300, command=self._set_gain)
        self.gain_slider.set(self.params.get().gain)
        self.gain_slider.grid(row=row, column=0, sticky="ew", padx=12); row += 1
        self.gain_txt = ctk.CTkLabel(panel, text=f"Gain: {self.params.get().gain:.1f}")
        self.gain_txt.grid(row=row, column=0, sticky="w", padx=12, pady=(0, 10)); row += 1

        # Status compact
        self.status_label = ctk.CTkLabel(panel, text="Status: —", justify="left")
        self.status_label.grid(row=row, column=0, sticky="ew", padx=12, pady=(8, 12))

    def _section(self, panel, title, row):
        lbl = ctk.CTkLabel(panel, text=title, font=ctk.CTkFont(size=16, weight="bold"))
        lbl.grid(row=row, column=0, sticky="w", padx=12, pady=(10, 6))

    # ---------- controls ----------
    def on_start(self):
        self.runner.set_started(True)

    def on_clear(self):
        self.runner.clear_tracking_only()

    def on_reset(self):
        self.runner.full_reset()

    def on_quit(self):
        try:
            self.runner.request_stop()
        except Exception:
            pass
        self.after(150, self.destroy)

    # ---------- slider handlers ----------
    def _set_crash(self, v):
        v = int(round(float(v)))
        self.params.update(crash_radius_px=v)
        self.crash_txt.configure(text=f"Crash Radius: {v}px")

    def _set_pxpm(self, v):
        v = float(v)
        self.params.update(px_per_m=v)
        self.pxpm_txt.configure(text=f"PX_PER_M: {v:.1f}")

    def _set_minspeed(self, v):
        v = float(v)
        self.params.update(min_speed_px_s=v)
        self.mins_txt.configure(text=f"Min Target Speed: {v:.1f}px/s")

    def _set_vote(self, v):
        v = int(round(float(v)))
        self.params.update(id_vote_window=v)
        self.vote_txt.configure(text=f"ID Vote Window: {v} frames")

    def _set_ang_thresh(self, v):
        v = float(v)
        self.params.update(angle_zero_thresh_rad=v)
        self.ath_txt.configure(text=f"Aim-Lock Threshold: {v:.2f} rad")

    def _set_exposure(self, v):
        v = float(v)
        self.params.update(exposure=v)
        self.exp_txt.configure(text=f"Exposure: {v:.1f}")

    def _set_gain(self, v):
        v = float(v)
        self.params.update(gain=float(v))
        self.gain_txt.configure(text=f"Gain: {float(v):.1f}")

    # ---------- UI tick ----------
    def _ui_tick(self):
        with self._lock:
            frame = self._latest_frame
            status = self._latest_status
            self._latest_frame = None

        if frame is not None:
            self._update_frame(frame)

        if status is not None:
            self._update_status(status)

        self.after(15, self._ui_tick)

    def _update_frame(self, rgb_np):
        h, w = rgb_np.shape[:2]
        target_w = 1020
        scale = target_w / float(w)
        target_h = int(h * scale)

        img = Image.fromarray(rgb_np).resize((target_w, target_h))
        self._tk_img = ImageTk.PhotoImage(img)
        self.video_label.configure(image=self._tk_img)
        self.video_label.image = self._tk_img

    def _update_status(self, s: dict):
        if s.get("crash_flash", False):
            self.crash_label.configure(text="IMPACT: YES")
        else:
            self.crash_label.configure(text="IMPACT: NO")

        ang = s.get("last_angle", None)
        ang_txt = "—" if ang is None else f"{ang:+.3f} rad"

        txt = (
            f"Mode: {'ATTACK' if s.get('started') else 'STANDBY'}   "
            f"Unlocked: {s.get('unlocked')}   "
            f"Aim-Lock: {s.get('zero_latched')}   "
            f"Aim: {ang_txt}   "
            f"FPS: {s.get('fps', 0):.1f}"
        )
        self.status_label.configure(text=txt)
