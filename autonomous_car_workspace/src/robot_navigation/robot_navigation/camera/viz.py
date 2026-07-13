"""Wraps a FrameSource to hold an annotated copy of the latest frame.

The vision pipeline pulls raw frames from `source.get_frame()`, does its
detection, draws the HUD overlay, and pushes the result back here via
`set_annotated()`.

The MJPEG streamer reads `wait_next()` to get a notification the instant
a fresh frame finishes rendering, so it never encodes the same frame twice
and never polls."""

from __future__ import annotations

import threading
from typing import Optional

import numpy as np


class AnnotatedFrameProvider:
    """Duck-types as a FrameSource for the MJPEG streamer."""

    def __init__(self, source):
        self.source = source
        self._cv = threading.Condition()
        self._frame_id = 0
        self._annotated: Optional[np.ndarray] = None

    def set_annotated(self, frame: Optional[np.ndarray]) -> None:
        """Push a newly drawn frame. Wakes up the streamer."""
        with self._cv:
            self._annotated = frame
            self._frame_id += 1
            self._cv.notify_all()

    def wait_next(self, after_id: int, timeout: float = 1.0):
        """Block until a strictly-newer annotated frame arrives.
        Returns `(frame_id, frame_copy)`.
        If timeout expires, returns `(after_id, None)`."""
        with self._cv:
            self._cv.wait_for(lambda: self._frame_id > after_id, timeout=timeout)
            if self._frame_id <= after_id or self._annotated is None:
                return after_id, None
            return self._frame_id, self._annotated.copy()

    def get_frame(self) -> Optional[np.ndarray]:
        """Returns the annotated frame if available, else raw."""
        with self._cv:
            if self._annotated is not None:
                return self._annotated.copy()
        return self.source.get_frame()

    def get_raw_frame(self) -> Optional[np.ndarray]:
        """Always returns the un-annotated frame directly from the source."""
        return self.source.get_frame()
