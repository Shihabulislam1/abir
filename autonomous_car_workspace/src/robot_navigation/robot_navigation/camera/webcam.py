"""USB webcam frame source (V4L2 / cv2.VideoCapture).

Matches the `FrameSource` interface so the rest of the pipeline is unchanged.
Best for USB wide-angle cameras connected to the Pi — the picamera2 backend
is not used in this case."""

from __future__ import annotations

import logging
from typing import Optional

import cv2
import numpy as np

from .capture import FrameSource

log = logging.getLogger(__name__)


class WebcamFrameSource(FrameSource):
    """Opens /dev/videoN via OpenCV's V4L2 backend. Applies `rotate_180` in
    software via `cv2.rotate` (the Pi camera does it in hardware)."""

    #: `cap.read()` blocks at the camera's native fps, so don't also sleep.
    _internal_rate_limits = True

    def __init__(self, webcam_index=1, width=640, height=480, framerate=30, rotate_180=False, **kwargs):
        super().__init__(width=width, height=height, framerate=framerate, rotate_180=rotate_180, **kwargs)
        self.webcam_index = webcam_index
        self._cap: Optional[cv2.VideoCapture] = None

    def _open(self) -> bool:
        index = int(self.webcam_index)
        cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
        if not cap.isOpened():
            # Fall back to auto-detect backend (useful on non-Linux dev boxes).
            cap = cv2.VideoCapture(index)
        if not cap.isOpened():
            log.warning("webcam index %d could not be opened; falling back to black frames", index)
            return False

        # Prefer MJPG for higher resolutions on USB bus; many wide-angle webcams
        # only hit their advertised fps under MJPG.
        try:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        except Exception:
            pass
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.framerate)
        # Keep the driver buffer short so we always render the freshest frame.
        try:
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass

        # Explicitly enable auto-exposure.
        try:
            # 3 = auto (Linux UVC convention); 1 = manual.
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)
        except Exception:
            pass

        aw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        ah = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        afps = cap.get(cv2.CAP_PROP_FPS)
        log.info(
            "webcam[%d] opened: requested %dx%d@%d, got %dx%d@%.1f, rotate_180=%s",
            index, self.width, self.height, self.framerate,
            aw, ah, afps, self.rotate_180,
        )
        self._cap = cap
        return True

    def _close(self) -> None:
        if self._cap is not None:
            try:
                self._cap.release()
            except Exception:
                pass
            self._cap = None

    def _capture_one(self, black: np.ndarray) -> np.ndarray:
        if self._cap is None:
            return black
        ok, frame = self._cap.read()
        if not ok or frame is None:
            return black
        if self.rotate_180:
            frame = cv2.rotate(frame, cv2.ROTATE_180)
        # If the webcam delivered a different size than requested, resize so
        # the downstream pipeline (perspective transform etc.) sees the
        # dimensions it was calibrated for.
        h, w = frame.shape[:2]
        if w != self.width or h != self.height:
            frame = cv2.resize(frame, (self.width, self.height))
        return frame
