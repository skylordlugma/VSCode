# shared_params.py
from dataclasses import dataclass
import threading

@dataclass
class Params:
    crash_radius_px: int = 120
    px_per_m: float = 20.0
    min_speed_px_s: float = 5.0
    id_vote_window: int = 7
    exposure: float = -6.0
    gain: float = 0.0

    # angle zero-latch
    angle_zero_thresh_rad: float = 0.3
    angle_zero_frames: int = 1

class SharedParams:
    def __init__(self, initial: Params):
        self._lock = threading.Lock()
        self._p = initial

    def get(self) -> Params:
        with self._lock:
            return Params(**self._p.__dict__)  # copy

    def update(self, **kwargs):
        with self._lock:
            for k, v in kwargs.items():
                if hasattr(self._p, k):
                    setattr(self._p, k, v)
