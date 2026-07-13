from __future__ import annotations

import logging
from typing import Optional

import numpy as np

from .capture import FrameSource

log = logging.getLogger(__name__)


class PiCameraFrameSource(FrameSource):
    """Reads from the Raspberry Pi Camera via picamera2 / libcamera. Applies
    hardware 180° rotation and tuning controls (EV bias, gain, brightness,
    contrast, saturation) on open."""

    _internal_rate_limits = False      # capture_array() is non-blocking

    def __init__(self, width=640, height=480, framerate=30, rotate_180=False,
                 exposure_value=None, analogue_gain=None, brightness=None,
                 contrast=None, saturation=None, **kwargs):
        super().__init__(width=width, height=height, framerate=framerate, rotate_180=rotate_180, **kwargs)
        self.exposure_value = exposure_value
        self.analogue_gain = analogue_gain
        self.brightness = brightness
        self.contrast = contrast
        self.saturation = saturation
        self._picam = None

    def _open(self) -> bool:
        try:
            from picamera2 import Picamera2
            from libcamera import Transform

            cam = Picamera2()
            transform = Transform(hflip=1, vflip=1) if self.rotate_180 else Transform()
            frame_us = int(1_000_000 / max(1, self.framerate))
            config = cam.create_video_configuration(
                # BGR888 matches OpenCV's native channel order.
                main={"size": (self.width, self.height), "format": "BGR888"},
                transform=transform,
                controls={"FrameDurationLimits": (frame_us, frame_us)},
            )
            cam.configure(config)
            cam.start()

            tuning = self._build_tuning_controls()
            if tuning:
                try:
                    cam.set_controls(tuning)
                except Exception as e:
                    log.warning("set_controls failed (%s); continuing with defaults", e)

            log.info(
                "picamera2 started at %dx%d fps=%d rotate_180=%s tuning=%s",
                self.width, self.height, self.framerate,
                self.rotate_180, tuning,
            )
            self._picam = cam
            return True
        except Exception as e:
            log.warning("picamera2 unavailable (%s); falling back to black frames", e)
            return False

    def _close(self) -> None:
        if self._picam is not None:
            try:
                self._picam.stop()
            except Exception:
                pass
            self._picam = None

    def _capture_one(self, black: np.ndarray) -> np.ndarray:
        if self._picam is None:
            return black
        try:
            return self._picam.capture_array()
        except Exception as e:
            log.warning("picamera capture failed: %s", e)
            return black

    def _build_tuning_controls(self) -> dict:
        ctrls: dict = {}
        if self.exposure_value is not None:
            ctrls["ExposureValue"] = float(self.exposure_value)
        if self.analogue_gain is not None:
            ctrls["AnalogueGain"] = float(self.analogue_gain)
        if self.brightness is not None:
            ctrls["Brightness"] = float(self.brightness)
        if self.contrast is not None:
            ctrls["Contrast"] = float(self.contrast)
        if self.saturation is not None:
            ctrls["Saturation"] = float(self.saturation)
        return ctrls
