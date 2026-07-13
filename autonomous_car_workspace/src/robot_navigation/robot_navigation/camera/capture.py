"""Threaded frame producers.

`FrameSource` is an abstract base that runs a background thread, pumping
frames from a hardware source (Pi camera or USB webcam) into a thread-safe
slot that the rest of the pipeline reads via `get_frame()`. Both concrete
backends emit BGR numpy arrays shaped (height, width, 3) so the downstream
vision / streamer code doesn't need to know which one is active."""

from __future__ import annotations

import threading
import logging
from abc import ABC, abstractmethod
from typing import Optional

import numpy as np

log = logging.getLogger(__name__)


class FrameSource(ABC):
    """Abstract base: threaded BGR-frame producer with black-frame fallback."""

    #: subclass override: set True if the backend's capture call already
    #: rate-limits to the camera's native fps (e.g. blocking reads from
    #: cv2.VideoCapture). Prevents us from double-throttling.
    _internal_rate_limits: bool = False

    def __init__(self, width=640, height=480, framerate=30, rotate_180=False, **kwargs):
        self.width = width
        self.height = height
        self.framerate = framerate
        self.rotate_180 = rotate_180
        self._latest: Optional[np.ndarray] = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._opened = False

    def start(self) -> None:
        self._opened = self._open()
        self._thread = threading.Thread(
            target=self._run, name=type(self).__name__, daemon=True
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)
        self._close()

    def get_frame(self) -> Optional[np.ndarray]:
        with self._lock:
            return None if self._latest is None else self._latest.copy()

    # --- subclass API ------------------------------------------------------

    @abstractmethod
    def _open(self) -> bool:
        """Open the underlying hardware. Return True on success."""

    @abstractmethod
    def _close(self) -> None:
        """Release the underlying hardware."""

    @abstractmethod
    def _capture_one(self, black: np.ndarray) -> np.ndarray:
        """Return the next BGR frame (or `black` on failure)."""

    # --- main loop --------------------------------------------------------

    def _run(self) -> None:
        period = 1.0 / max(1, self.framerate)
        black = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        while not self._stop.is_set():
            frame = self._capture_one(black)
            with self._lock:
                self._latest = frame
            if not self._internal_rate_limits:
                self._stop.wait(period)
