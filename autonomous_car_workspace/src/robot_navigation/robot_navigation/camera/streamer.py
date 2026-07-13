from __future__ import annotations

import json
import socketserver
import threading
from http import server
from typing import Callable, Optional
import rclpy

import cv2
import logging

log = logging.getLogger(__name__)

_BOUNDARY = "frame"


class _JpegBroker:
    """Single-slot pub/sub for the latest encoded MJPEG frame. The streamer's
    encoder thread is the sole publisher; every connected `/stream.mjpg`
    viewer blocks in `wait_next` until a strictly-newer frame is available.
    Decoupling encode from each handler thread (a) caps stream FPS at the
    producer rate, (b) collapses duplicate sends, and (c) amortizes JPEG
    encode cost across viewers (one encode for N watchers)."""

    def __init__(self):
        self._cv = threading.Condition()
        self._jpeg: Optional[bytes] = None
        self._frame_id: int = 0
        self._closed = False

    def publish(self, frame_id: int, jpeg: bytes) -> None:
        with self._cv:
            self._jpeg = jpeg
            self._frame_id = frame_id
            self._cv.notify_all()

    def wait_next(self, after_id: int, timeout: float = 2.0):
        with self._cv:
            self._cv.wait_for(
                lambda: self._closed or self._frame_id > after_id,
                timeout=timeout,
            )
            if self._closed or self._frame_id <= after_id or self._jpeg is None:
                return after_id, None
            return self._frame_id, self._jpeg

    def close(self) -> None:
        with self._cv:
            self._closed = True
            self._cv.notify_all()


def _make_handler(source, quality: int, tuner_hooks: "TunerHooks",
                  broker: "_JpegBroker"):
    class MJPEGHandler(server.BaseHTTPRequestHandler):
        def log_message(self, format, *args):  # noqa: A002 - stdlib signature
            log.debug("http: " + format, *args)

        def do_GET(self):  # noqa: N802 - stdlib signature
            if self.path in ("/stream.mjpg",):
                self._serve_mjpeg()
                return
            if self.path in ("/raw.jpg", "/raw"):
                self._serve_snapshot(raw=True)
                return
            if self.path in ("/annotated.jpg", "/annotated"):
                self._serve_snapshot(raw=False)
                return
            if self.path in ("/", "/tune", "/tune/"):
                self._serve_text(_TUNER_HTML, "text/html; charset=utf-8")
                return
            if self.path.startswith("/config.json"):
                self._serve_json(tuner_hooks.snapshot())
                return
            if self.path.startswith("/knobs.json"):
                self._serve_json(tuner_hooks.get_knobs_spec())
                return
            if self.path.startswith("/run.json"):
                self._serve_json({"running": tuner_hooks.running()})
                return
            self.send_error(404)

        def do_POST(self):  # noqa: N802
            length = int(self.headers.get("Content-Length") or 0)
            body = self.rfile.read(length) if length > 0 else b""
            if self.path.startswith("/config"):
                try:
                    payload = json.loads(body.decode("utf-8") or "{}")
                except Exception as e:
                    self.send_error(400, f"bad json: {e}")
                    return
                applied = tuner_hooks.apply(payload)
                self._serve_json({"applied": applied})
                return
            if self.path.startswith("/save"):
                self._serve_json({"saved": "Saving not supported via HTTP in ROS2 (use config file)"})
                return
            if self.path.startswith("/run"):
                try:
                    payload = json.loads(body.decode("utf-8") or "{}")
                except Exception as e:
                    self.send_error(400, f"bad json: {e}")
                    return
                running = bool(payload.get("running", True))
                actual = tuner_hooks.set_running(running)
                self._serve_json({"running": actual})
                return
            self.send_error(404)

        # --- helpers ---

        def _serve_mjpeg(self) -> None:
            self.send_response(200)
            self.send_header(
                "Content-Type", f"multipart/x-mixed-replace; boundary={_BOUNDARY}"
            )
            self.end_headers()
            last = -1
            try:
                while True:
                    fid, jpg = broker.wait_next(last)
                    if jpg is None:
                        continue       # timeout — loop, no spin
                    last = fid
                    self.wfile.write(f"--{_BOUNDARY}\r\n".encode())
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Content-Length", str(len(jpg)))
                    self.end_headers()
                    self.wfile.write(jpg)
                    self.wfile.write(b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                pass

        def _serve_snapshot(self, raw: bool) -> None:
            getter = getattr(source, "get_raw_frame", None) if raw else None
            frame = getter() if getter else source.get_frame()
            if frame is None:
                self.send_error(503, "no frame")
                return
            ok, jpg = cv2.imencode(
                ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), quality]
            )
            if not ok:
                self.send_error(500, "encode failed")
                return
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(jpg)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(jpg.tobytes())

        def _serve_text(self, text: str, ctype: str) -> None:
            data = text.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)

        def _serve_json(self, obj) -> None:
            data = json.dumps(obj).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)

    return MJPEGHandler


class TunerHooks:
    """Bridge to ROS2 parameter system."""

    def __init__(self, node, tunable_params: list[str]):
        self._node = node
        self._tunable_params = tunable_params
        self._lock = threading.Lock()

    def snapshot(self) -> dict:
        with self._lock:
            snap = {}
            for name in self._tunable_params:
                if self._node.has_parameter(name):
                    snap[name] = self._node.get_parameter(name).value
            return snap
            
    def get_knobs_spec(self) -> list:
        # Construct a basic spec from the current parameters.
        # Ideally this would use ParameterDescriptor ranges, but we can fake it
        # based on parameter names.
        specs = []
        snap = self.snapshot()
        for name, value in snap.items():
            if isinstance(value, float):
                # Guess a range based on the value
                if 'frac' in name:
                    min_v, max_v, step = -0.2, 1.2, 0.01
                else:
                    min_v, max_v, step = 0.0, max(1.0, value * 3.0), 0.05
                specs.append({
                    "label": name, "path": name, "min": min_v, "max": max_v, "step": step, "kind": "float"
                })
            elif isinstance(value, int):
                if 'threshold' in name or 'c' in name or 'area' in name or 'margin' in name:
                    min_v, max_v, step = 0, 255, 1
                elif 'kernel' in name or 'block' in name:
                    min_v, max_v, step = 1, 31, 2
                else:
                    min_v, max_v, step = 0, max(10, value * 3), 1
                specs.append({
                    "label": name, "path": name, "min": min_v, "max": max_v, "step": step, "kind": "int"
                })
        return specs

    def apply(self, patch: dict) -> dict:
        applied: dict = {}
        with self._lock:
            params = []
            for key, value in patch.items():
                if self._node.has_parameter(key):
                    # Try to match the type of the existing parameter
                    current_val = self._node.get_parameter(key).value
                    if isinstance(current_val, int):
                        val = int(float(value))
                    elif isinstance(current_val, float):
                        val = float(value)
                    elif isinstance(current_val, bool):
                        val = bool(value)
                    else:
                        val = value
                    params.append(rclpy.parameter.Parameter(key, value=val))
                    applied[key] = val
            
            if params:
                self._node.set_parameters(params)
        return applied

    def running(self) -> bool:
        return True # Not implemented in ROS2 adapter yet

    def set_running(self, running: bool) -> bool:
        return True # Not implemented in ROS2 adapter yet


class _ThreadingServer(socketserver.ThreadingMixIn, server.HTTPServer):
    allow_reuse_address = True
    daemon_threads = True


class MJPEGStreamer:
    """Serves:
        /stream.mjpg   — continuous MJPEG feed
        /raw.jpg       — one-shot un-annotated frame
        /annotated.jpg — one-shot HUD-annotated frame
        /              — HTML live-tuner page with sliders
        /config.json   — GET current tunable values (JSON)
        /knobs.json    — GET spec of all sliders (JSON)
        /config        — POST {path: value, ...} to mutate config live
        /save          — POST to write current values back (no-op in ROS2)
        /run.json      — GET {"running": bool}
        /run           — POST {"running": bool} to e-stop / resume"""

    def __init__(self, source, port=8080, jpeg_quality=80,
                 tuner_hooks: Optional[TunerHooks] = None):
        self.source = source
        self.port = port
        self.jpeg_quality = jpeg_quality
        self._server: Optional[_ThreadingServer] = None
        self._thread: Optional[threading.Thread] = None
        self._tuner_hooks = tuner_hooks
        self._broker = _JpegBroker()
        self._encoder_thread: Optional[threading.Thread] = None
        self._encoder_stop = threading.Event()

    def start(self) -> None:
        hooks = self._tuner_hooks
        handler = _make_handler(self.source, self.jpeg_quality, hooks,
                                self._broker)
        self._server = _ThreadingServer(
            ("0.0.0.0", self.port), handler
        )
        self._thread = threading.Thread(
            target=self._server.serve_forever, name="MJPEGStreamer", daemon=True
        )
        self._thread.start()
        self._encoder_stop.clear()
        self._encoder_thread = threading.Thread(
            target=self._encoder_loop, name="MJPEGEncoder", daemon=True,
        )
        self._encoder_thread.start()

    def stop(self) -> None:
        self._encoder_stop.set()
        self._broker.close()
        if self._server:
            self._server.shutdown()
            self._server.server_close()
        if self._thread:
            self._thread.join(timeout=2.0)
        if self._encoder_thread:
            self._encoder_thread.join(timeout=2.0)

    def _encoder_loop(self) -> None:
        """Wait on the AnnotatedFrameProvider for each fresh frame, encode
        once, publish to the broker."""
        last = -1
        quality = int(self.jpeg_quality)
        wait_next = getattr(self.source, "wait_next", None)
        while not self._encoder_stop.is_set():
            if wait_next is not None:
                fid, frame = wait_next(last, timeout=0.5)
            else:
                self._encoder_stop.wait(1.0 / 30)
                fid, frame = (last + 1, self.source.get_frame())
            if frame is None:
                continue
            try:
                ok, jpg = cv2.imencode(
                    ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), quality]
                )
            except Exception as e:
                log.debug("jpeg encode failed: %s", e)
                continue
            if not ok:
                continue
            last = fid
            self._broker.publish(fid, jpg.tobytes())


# ---------------------------------------------------------------------------
# Tuner HTML (served at `/`). Plain page, no external deps.
# ---------------------------------------------------------------------------

_TUNER_HTML = """<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <title>autocar tuner</title>
  <style>
    :root { color-scheme: dark; }
    body { margin: 0; background:#111; color:#ddd; font-family: ui-sans-serif, system-ui, sans-serif; }
    .wrap { display:flex; flex-direction:row; gap:12px; padding:12px; align-items:flex-start; }
    .stream { flex:0 0 auto; border:1px solid #333; }
    .stream img { display:block; max-width: 640px; max-height: 480px; }
    .controls { flex:1 1 auto; display:grid; grid-template-columns: 1fr; gap:8px; font-size: 13px; }
    .knob { display:grid; grid-template-columns: 170px 1fr 64px; gap:8px; align-items:center; }
    .knob label { color:#aaa; }
    .knob input[type=range] { width: 100%; accent-color:#8be9fd; }
    .knob .value { text-align:right; color:#f1fa8c; font-variant-numeric: tabular-nums; }
    .section { margin-top: 10px; color:#50fa7b; border-bottom: 1px solid #333; padding-bottom: 2px; }
    .actions { display:flex; gap:8px; margin-top: 12px; }
    button { background:#282a36; color:#f8f8f2; border:1px solid #444; border-radius:4px;
             padding:6px 12px; cursor:pointer; font-size: 13px; }
    button:hover { background:#3a3c52; }
    button:disabled { opacity:0.4; cursor: default; }
    #status { color:#6272a4; margin-left: 8px; }
    .runbar { display:flex; gap:8px; align-items:center; margin-bottom: 10px; }
    button.estop { background:#5b1620; border-color:#d9434b; color:#ffd5da;
                   font-weight:600; padding:10px 22px; font-size: 15px; }
    button.estop:hover:not(:disabled) { background:#7a1c2a; }
    button.go    { background:#143b1a; border-color:#3ec45a; color:#cdfbd6;
                   font-weight:600; padding:10px 22px; font-size: 15px; }
    button.go:hover:not(:disabled) { background:#1f5a27; }
    .runstate { font-weight:600; font-variant-numeric: tabular-nums; }
    .runstate.on  { color:#50fa7b; }
    .runstate.off { color:#ff5555; }
  </style>
</head>
<body>
  <div class="wrap">
    <div class="stream">
      <div class="runbar">
        <button id="stop"  class="estop">⏹ E-STOP</button>
        <button id="start" class="go">▶ START</button>
        <span class="runstate" id="runstate">…</span>
      </div>
      <img src="/stream.mjpg" alt="live" />
    </div>
    <div class="controls" id="controls">
      <div class="section">loading…</div>
    </div>
  </div>
  <script>
  (async () => {
    const ctrlEl = document.getElementById('controls');
    const [knobs, values] = await Promise.all([
      fetch('/knobs.json').then(r => r.json()),
      fetch('/config.json').then(r => r.json()),
    ]);

    // Group knobs by the leading cfg path segment (vision / control).
    const groups = {};
    for (const k of knobs) {
      const g = k.path.split('.')[0] || "vision";
      (groups[g] ||= []).push(k);
    }

    const fmt = (k, v) => k.kind === 'float' ? Number(v).toFixed(3) : String(Math.round(Number(v)));

    let html = '';
    for (const [g, items] of Object.entries(groups)) {
      html += `<div class="section">${g}</div>`;
      for (const k of items) {
        const v = values[k.path];
        if (v === undefined) continue;
        html += `
        <div class="knob" data-path="${k.path}" data-kind="${k.kind}">
          <label title="${k.path}">${k.label}</label>
          <input type="range" min="${k.min}" max="${k.max}" step="${k.step}" value="${v}">
          <span class="value">${fmt(k, v)}</span>
        </div>`;
      }
    }
    html += `
      <div class="actions">
        <button id="save">Save (N/A in ROS2)</button>
        <button id="revert">Revert (reload from Pi)</button>
        <span id="status"></span>
      </div>`;
    ctrlEl.innerHTML = html;

    const status = document.getElementById('status');
    const setStatus = (t) => { status.textContent = t; };

    // Debounced POST to /config
    const pending = {};
    let timer = null;
    function schedulePush() {
      if (timer) return;
      timer = setTimeout(async () => {
        timer = null;
        const body = JSON.stringify(pending);
        Object.keys(pending).forEach(k => delete pending[k]);
        try {
          const res = await fetch('/config', {method:'POST', body});
          const j = await res.json();
          // Sync UI with the actually-applied (clamped / rounded) values.
          for (const [p, v] of Object.entries(j.applied || {})) {
            const row = document.querySelector(`.knob[data-path="${CSS.escape(p)}"]`);
            if (!row) continue;
            const kind = row.dataset.kind;
            row.querySelector('input').value = v;
            row.querySelector('.value').textContent = kind === 'float'
              ? Number(v).toFixed(3) : String(Math.round(Number(v)));
          }
          setStatus('');
        } catch (e) {
          setStatus('push failed: ' + e);
        }
      }, 60);
    }

    ctrlEl.addEventListener('input', (ev) => {
      const row = ev.target.closest('.knob');
      if (!row) return;
      const path = row.dataset.path;
      const value = ev.target.value;
      row.querySelector('.value').textContent = row.dataset.kind === 'float'
        ? Number(value).toFixed(3) : String(Math.round(Number(value)));
      pending[path] = value;
      schedulePush();
    });

    document.getElementById('save').onclick = async () => {
      setStatus('saving not supported via UI in ROS2');
    };

    document.getElementById('revert').onclick = async () => {
      setStatus('reloading…');
      location.reload();
    };

    // --- E-STOP / START -----------------------------------------------------
    const stopBtn  = document.getElementById('stop');
    const startBtn = document.getElementById('start');
    const runEl    = document.getElementById('runstate');
    const reflectRun = (running) => {
      stopBtn.disabled  = !running;   
      startBtn.disabled = running;    
      runEl.textContent = running ? 'RUNNING' : 'STOPPED';
      runEl.className   = 'runstate ' + (running ? 'on' : 'off');
    };

    async function postRun(running) {
      try {
        const res = await fetch('/run', {
          method:'POST',
          body: JSON.stringify({running}),
        });
        const j = await res.json();
        reflectRun(j.running);
        setStatus(j.running ? 'running' : 'STOPPED');
      } catch (e) { setStatus('run toggle failed: ' + e); }
    }

    stopBtn.onclick  = () => postRun(false);
    startBtn.onclick = () => postRun(true);

    document.addEventListener('keydown', (e) => {
      if (e.code === 'Space' && !['INPUT','TEXTAREA'].includes(e.target.tagName)) {
        e.preventDefault();
        postRun(false);
      }
    });

    fetch('/run.json').then(r => r.json()).then(j => reflectRun(j.running))
      .catch(() => reflectRun(true));
  })();
  </script>
</body>
</html>
"""
