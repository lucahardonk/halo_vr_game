"""Kinect v1 capture layer for the dashboard (libfreenect_sync via ctypes).

The C "sync" wrapper of libfreenect is used for everything (video, depth, tilt,
LED, accelerometer) so that a single process-wide device handle is shared and
no second context fights for the USB device.

Hardware note: the Kinect v1 has ONE video endpoint, so RGB and IR cannot be
streamed at the same time. The video thread captures whichever mode is
selected (``KinectManager.set_video_mode``); the other stream shows a
placeholder. Depth always runs in parallel.
"""
import ctypes as C
import queue
import threading
import time
from ctypes.util import find_library

import cv2
import numpy as np

WIDTH, HEIGHT = 640, 480

# libfreenect enums
VIDEO_RGB = 0
VIDEO_IR_8BIT = 2
DEPTH_11BIT = 0
COUNTS_PER_G = 819.0
GRAVITY = 9.80665
TILT_STATUS = {0: "stopped", 1: "at_limit", 4: "moving"}
LED_OPTIONS = {
    "off": 0,
    "green": 1,
    "red": 2,
    "yellow": 3,
    "blink_green": 4,
    "blink_red_yellow": 6,
}
TILT_MIN, TILT_MAX = -27, 27
JPEG_PARAMS = [cv2.IMWRITE_JPEG_QUALITY, 85]


class RawTiltState(C.Structure):
    """Mirror of freenect_raw_tilt_state (libfreenect.h)."""

    _fields_ = [
        ("accelerometer_x", C.c_int16),
        ("accelerometer_y", C.c_int16),
        ("accelerometer_z", C.c_int16),
        ("tilt_angle", C.c_int8),
        ("tilt_status", C.c_int),
    ]


class FreenectSync:
    """Thin ctypes binding of libfreenect_sync."""

    def __init__(self, index=0):
        self.index = index
        name = find_library("freenect_sync")
        if not name:
            raise RuntimeError(
                "libfreenect_sync not found. Install it with: sudo apt install libfreenect-dev"
            )
        lib = C.CDLL(name)
        frame_args = [C.POINTER(C.c_void_p), C.POINTER(C.c_uint32), C.c_int, C.c_int]
        for fn in (lib.freenect_sync_get_video, lib.freenect_sync_get_depth):
            fn.argtypes = frame_args
            fn.restype = C.c_int
        lib.freenect_sync_stop.argtypes = []
        lib.freenect_sync_stop.restype = None
        self.has_motor = all(
            hasattr(lib, n)
            for n in ("freenect_sync_get_tilt_state", "freenect_sync_set_tilt_degs", "freenect_sync_set_led")
        )
        if self.has_motor:
            lib.freenect_sync_get_tilt_state.argtypes = [C.POINTER(C.POINTER(RawTiltState)), C.c_int]
            lib.freenect_sync_get_tilt_state.restype = C.c_int
            lib.freenect_sync_set_tilt_degs.argtypes = [C.c_int, C.c_int]
            lib.freenect_sync_set_tilt_degs.restype = C.c_int
            lib.freenect_sync_set_led.argtypes = [C.c_int, C.c_int]
            lib.freenect_sync_set_led.restype = C.c_int
        self.lib = lib
        self.ctrl_lock = threading.Lock()
        # libfreenect_sync is not thread-safe while it (re)opens the device: concurrent
        # calls from the video/depth/accel threads with no Kinect (or after an unplug)
        # end in "double free or corruption". Until a call has succeeded, every call is
        # serialised through init_lock; once the device is up they run in parallel.
        self.init_lock = threading.Lock()
        self.ready = False
        self._first_try = None

    def _call(self, fn, *args, blocking=False):
        """Run a libfreenect_sync call.

        ``blocking`` marks frame calls (get_video/get_depth): they wait inside the C
        library until a frame arrives, which may be forever if a stream is broken
        (e.g. RGB on some USB3 controllers). They must therefore never run while
        holding init_lock, otherwise one dead stream freezes every other thread.
        The device is opened by a non-blocking call (tilt state) first; frame calls
        only take the lock as a fallback when the motor API is unusable.
        """
        if self.ready:
            status = fn(*args)
            if status != 0:
                self.ready = False
            return status
        if blocking and self.has_motor:
            if self._first_try is None:
                self._first_try = time.monotonic()
            if time.monotonic() - self._first_try < 6.0:
                time.sleep(0.2)  # wait for the accelerometer thread to open the device
                return -1
        with self.init_lock:
            if self.ready:
                return fn(*args)
            status = fn(*args)
            if status == 0:
                self.ready = True
                self._first_try = None
            else:
                time.sleep(0.2)  # let the library settle before the next thread retries
            return status

    def _frame(self, fn, fmt, shape, ctype):
        ptr = C.c_void_p()
        ts = C.c_uint32()
        status = self._call(fn, C.byref(ptr), C.byref(ts), self.index, fmt, blocking=True)
        if status != 0 or not ptr.value:
            raise RuntimeError(f"{fn.__name__} failed (code {status})")
        size = int(np.prod(shape))
        data = C.cast(ptr, C.POINTER(ctype))
        return np.ctypeslib.as_array(data, shape=(size,)).reshape(shape).copy()

    def rgb(self):
        return self._frame(self.lib.freenect_sync_get_video, VIDEO_RGB, (HEIGHT, WIDTH, 3), C.c_uint8)

    def ir(self):
        return self._frame(self.lib.freenect_sync_get_video, VIDEO_IR_8BIT, (HEIGHT, WIDTH), C.c_uint8)

    def depth(self):
        return self._frame(self.lib.freenect_sync_get_depth, DEPTH_11BIT, (HEIGHT, WIDTH), C.c_uint16)

    def tilt_state(self):
        """Return (ax, ay, az [m/s^2], tilt_deg or None, status_str)."""
        if not self.has_motor:
            raise RuntimeError("this libfreenect_sync build has no motor API")
        with self.ctrl_lock:
            ptr = C.POINTER(RawTiltState)()
            status = self._call(self.lib.freenect_sync_get_tilt_state, C.byref(ptr), self.index)
            if status != 0 or not ptr:
                raise RuntimeError(f"freenect_sync_get_tilt_state failed (code {status})")
            s = ptr.contents
            raw = (s.accelerometer_x, s.accelerometer_y, s.accelerometer_z, s.tilt_angle, s.tilt_status)
        ax, ay, az = (v / COUNTS_PER_G * GRAVITY for v in raw[:3])
        # tilt_angle is in half degrees; -128 (0x80) means "unknown / moving"
        tilt = None if raw[3] == -128 else raw[3] / 2.0
        return ax, ay, az, tilt, TILT_STATUS.get(raw[4], f"unknown({raw[4]})")

    def set_tilt(self, degrees):
        with self.ctrl_lock:
            if self._call(self.lib.freenect_sync_set_tilt_degs, int(round(degrees)), self.index) != 0:
                raise RuntimeError("freenect_sync_set_tilt_degs failed")

    def set_led(self, value):
        with self.ctrl_lock:
            if self._call(self.lib.freenect_sync_set_led, int(value), self.index) != 0:
                raise RuntimeError("freenect_sync_set_led failed")

    def stop(self):
        self.lib.freenect_sync_stop()


class FpsMeter:
    def __init__(self):
        self._count = 0
        self._t0 = time.monotonic()
        self.fps = 0.0

    def tick(self):
        self._count += 1
        now = time.monotonic()
        dt = now - self._t0
        if dt >= 1.0:
            self.fps = self._count / dt
            self._count = 0
            self._t0 = now

    def reset(self):
        self._count = 0
        self._t0 = time.monotonic()
        self.fps = 0.0


def placeholder_jpeg(text, size=(WIDTH, HEIGHT)):
    img = np.full((size[1], size[0], 3), 30, np.uint8)
    for i, line in enumerate(text.split("\n")):
        (tw, th), _ = cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2)
        y = size[1] // 2 + i * (th + 14)
        cv2.putText(img, line, ((size[0] - tw) // 2, y), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (200, 200, 200), 2)
    return cv2.imencode(".jpg", img, JPEG_PARAMS)[1].tobytes()


class FrameStream:
    """Raw frames -> Queue(maxsize=1) -> encoder thread -> latest JPEG broadcast.

    The grabber never blocks on JPEG encoding, and any number of HTTP clients
    can read the latest JPEG through a Condition.
    """

    def __init__(self, name, render):
        self.name = name
        self.render = render  # numpy frame -> BGR image
        self.raw = queue.Queue(maxsize=1)
        self.cond = threading.Condition()
        self.jpeg = placeholder_jpeg(f"{name.upper()}: waiting for Kinect...")
        self.seq = 0
        self.fps = FpsMeter()
        self.active = False
        self._running = True
        threading.Thread(target=self._encode_loop, name=f"enc-{name}", daemon=True).start()

    def push(self, frame):
        """Non-blocking put that drops the stale frame if the queue is full."""
        try:
            self.raw.put_nowait(frame)
        except queue.Full:
            try:
                self.raw.get_nowait()
            except queue.Empty:
                pass
            try:
                self.raw.put_nowait(frame)
            except queue.Full:
                pass

    def set_placeholder(self, text):
        self.active = False
        self.fps.reset()
        self._publish(placeholder_jpeg(text))

    def _publish(self, jpeg):
        with self.cond:
            self.jpeg = jpeg
            self.seq += 1
            self.cond.notify_all()

    def _encode_loop(self):
        while self._running:
            try:
                frame = self.raw.get(timeout=0.5)
            except queue.Empty:
                continue
            ok, buf = cv2.imencode(".jpg", self.render(frame), JPEG_PARAMS)
            if ok:
                self.active = True
                self.fps.tick()
                self._publish(buf.tobytes())

    def mjpeg(self):
        """Generator for a multipart/x-mixed-replace HTTP response."""
        last = -1
        while True:
            with self.cond:
                if self.seq == last:
                    self.cond.wait(timeout=1.0)
                jpeg, last = self.jpeg, self.seq
            yield (b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                   + str(len(jpeg)).encode() + b"\r\n\r\n" + jpeg + b"\r\n")

    def stop(self):
        self._running = False


def render_rgb(frame):
    return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)


def render_ir(frame):
    return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)


def render_depth(depth):
    """11-bit raw depth -> JET colormap; invalid pixels (2047) are black."""
    invalid = depth >= 2047
    d = np.clip(depth, 400, 1050).astype(np.float32)
    norm = ((1050 - d) * (255.0 / 650.0)).astype(np.uint8)  # near = red, far = blue
    color = cv2.applyColorMap(norm, cv2.COLORMAP_JET)
    color[invalid] = 0
    cv2.drawMarker(color, (WIDTH // 2, HEIGHT // 2), (255, 255, 255), cv2.MARKER_CROSS, 16, 1)
    return color


def raw_depth_to_meters(raw):
    """Standard libfreenect approximation for the 11-bit depth value."""
    if raw >= 2047 or raw <= 0:
        return None
    m = 1.0 / (raw * -0.0030711016 + 3.3309495161)
    return m if 0.3 < m < 10 else None


class KinectManager:
    """Owns the device and all capture threads."""

    VIDEO_STALL_S = 4.0

    def __init__(self, index=0, accel_hz=100.0, accel_emit_hz=20.0, on_accel=None, video_mode="rgb"):
        self.index = index
        self.accel_period = 1.0 / accel_hz
        self.emit_period = 1.0 / accel_emit_hz
        self.on_accel = on_accel or (lambda data: None)
        self.dev = None
        self.error = None
        self.rgb = FrameStream("rgb", render_rgb)
        self.ir = FrameStream("ir", render_ir)
        self.depth = FrameStream("depth", render_depth)
        self.video_mode = video_mode if video_mode in ("rgb", "ir") else "rgb"
        self._video_wait_since = None  # monotonic time of the pending video call
        self._video_wait_mode = None
        self.video_stalled = None      # mode whose stream delivers no frames
        self.connected = False
        self.center_depth_m = None
        self.tilt_deg = None
        self.tilt_target = 0
        self.tilt_status = "unknown"
        self.led = "green"
        self.accel = (0.0, 0.0, 0.0)
        self.accel_fps = FpsMeter()
        self._running = False
        self._threads = []
        self._tilt_sent = False

    # ---------------------------------------------------------------- control
    def start(self):
        try:
            self.dev = FreenectSync(self.index)
        except Exception as exc:  # library missing
            self.error = str(exc)
            for s in (self.rgb, self.ir, self.depth):
                s.set_placeholder("libfreenect_sync not found\nsudo apt install libfreenect-dev")
            print(f"[kinect] {exc}")
            return
        self._running = True
        other = self.ir if self.video_mode == "rgb" else self.rgb
        other.set_placeholder(f"{other.name.upper()} inactive\nselect {other.name.upper()} mode (RGB and IR\nshare the same sensor)")
        for target, name in ((self._video_loop, "video"), (self._depth_loop, "depth"),
                             (self._accel_loop, "accel"), (self._watchdog_loop, "watchdog")):
            t = threading.Thread(target=target, name=name, daemon=True)
            t.start()
            self._threads.append(t)

    def stop(self):
        self._running = False
        for t in self._threads:
            t.join(timeout=2.0)
        for s in (self.rgb, self.ir, self.depth):
            s.stop()
        if self.dev is not None and self.dev.ready:  # never touch an unopened device on exit
            try:
                self.dev.set_led(LED_OPTIONS["green"])
            except Exception:
                pass
            if not self.video_stalled:  # a thread is still blocked inside the library: let exit clean up
                self.dev.stop()

    def set_video_mode(self, mode):
        if mode not in ("rgb", "ir"):
            raise ValueError("mode must be 'rgb' or 'ir'")
        if mode != self.video_mode:
            if self.video_stalled:
                raise RuntimeError(
                    f"{self.video_stalled.upper()} stream is stuck inside libfreenect (no frames): "
                    f"restart with: python app.py --video-mode {mode}")
            other = self.ir if mode == "rgb" else self.rgb
            self.video_mode = mode
            other.set_placeholder(f"{other.name.upper()} inactive\nselect {other.name.upper()} mode (RGB and IR\nshare the same sensor)")
        return self.video_mode

    def set_tilt(self, degrees):
        self._require_dev()
        degrees = int(max(TILT_MIN, min(TILT_MAX, round(float(degrees)))))
        if degrees == self.tilt_target and self._tilt_sent:
            return degrees  # avoid flooding the motor with identical commands (slider drags)
        self.dev.set_tilt(degrees)
        self._tilt_sent = True
        self.tilt_target = degrees
        return degrees

    def set_led(self, name):
        self._require_dev()
        if name not in LED_OPTIONS:
            raise ValueError(f"unknown LED state '{name}', valid: {list(LED_OPTIONS)}")
        self.dev.set_led(LED_OPTIONS[name])
        self.led = name
        return name

    def _require_dev(self):
        if self.dev is None:
            raise RuntimeError(self.error or "Kinect not initialised")
        if not self.dev.has_motor:
            raise RuntimeError("motor/LED API not available in this libfreenect_sync")

    def status(self):
        ax, ay, az = self.accel
        return {
            "connected": self.connected,
            "error": self.error,
            "video_mode": self.video_mode,
            "video_stalled": self.video_stalled,
            "fps": {
                "rgb": round(self.rgb.fps.fps, 1),
                "ir": round(self.ir.fps.fps, 1),
                "depth": round(self.depth.fps.fps, 1),
                "accel": round(self.accel_fps.fps, 1),
            },
            "tilt": self.tilt_deg,
            "tilt_target": self.tilt_target,
            "tilt_status": self.tilt_status,
            "led": self.led,
            "accel": {"x": round(ax, 3), "y": round(ay, 3), "z": round(az, 3)},
            "center_depth_m": self.center_depth_m,
        }

    # ---------------------------------------------------------------- threads
    def _video_loop(self):
        current = None
        while self._running:
            mode = self.video_mode
            stream = self.rgb if mode == "rgb" else self.ir
            try:
                self._video_wait_mode = mode
                self._video_wait_since = time.monotonic()
                try:
                    frame = self.dev.rgb() if mode == "rgb" else self.dev.ir()
                finally:
                    self._video_wait_since = None
                if self.video_stalled:
                    print(f"[kinect] {mode.upper()} frames arriving again")
                    self.video_stalled = None
                if mode != current:
                    print(f"[kinect] video mode: {mode}")
                    current = mode
                self.connected = True
                self.error = None
                if mode == self.video_mode:  # discard a frame grabbed across a switch
                    stream.push(frame)
            except RuntimeError as exc:
                self._on_capture_error(stream, exc)

    def _depth_loop(self):
        while self._running:
            try:
                depth = self.dev.depth()
                self.connected = True
                self.center_depth_m = self._center_depth(depth)
                self.depth.push(depth)
            except RuntimeError as exc:
                self._on_capture_error(self.depth, exc)

    def _watchdog_loop(self):
        """Detect a video stream that never delivers frames.

        freenect_sync_get_video() blocks until a frame arrives. If the device is
        open (depth/accelerometer work) but the video call hangs, the RGB/IR
        isochronous stream is broken - typically a USB3 (xHCI) controller or hub
        that mangles the 1908-byte video packets. The thread cannot be
        interrupted, so show a clear explanation instead of a frozen image.
        """
        while self._running:
            time.sleep(0.5)
            since, mode = self._video_wait_since, self._video_wait_mode
            if since is None or self.video_stalled or not self.dev.ready:
                continue
            if time.monotonic() - since < self.VIDEO_STALL_S:
                continue
            self.video_stalled = mode
            name = mode.upper()
            other = "ir" if mode == "rgb" else "rgb"
            msg = (f"{name} stream: no frames from the Kinect for {self.VIDEO_STALL_S:.0f}s while depth/accel "
                   f"work -> USB video packets lost. Try another USB port (USB 2.0 port or a POWERED USB 2.0 hub), "
                   f"no extension cable; test with 'freenect-glview'; or restart with --video-mode {other}")
            print(f"[kinect] {msg}")
            self.error = msg
            (self.rgb if mode == "rgb" else self.ir).set_placeholder(
                f"{name}: no frames from Kinect\n(depth works: USB video\npackets lost)\n"
                f"try another USB port /\npowered USB2 hub, or\n--video-mode {other}")

    def _on_capture_error(self, stream, exc):
        if self.connected or self.error is None:
            print(f"[kinect] capture error: {exc}")
        self.connected = False
        self.error = f"{exc} - check power supply, USB and that no other program uses the Kinect"
        stream.set_placeholder("Kinect not connected\ncheck power + USB\n(retrying...)")
        time.sleep(1.0)

    @staticmethod
    def _center_depth(depth):
        patch = depth[HEIGHT // 2 - 3:HEIGHT // 2 + 4, WIDTH // 2 - 3:WIDTH // 2 + 4]
        valid = patch[patch < 2047]
        if valid.size == 0:
            return None
        m = raw_depth_to_meters(int(np.median(valid)))
        return round(m, 3) if m else None

    def _accel_loop(self):
        if not self.dev.has_motor:
            print("[kinect] motor API missing: accelerometer disabled")
            return
        samples = []
        t0 = time.monotonic()
        next_tick = t0
        last_emit = t0
        error_reported = False
        backoff = 0.05
        ax = ay = az = 0.0
        while self._running:
            now = time.monotonic()
            try:
                ax, ay, az, tilt, status = self.dev.tilt_state()
                error_reported = False
                self.accel = (ax, ay, az)
                if tilt is not None:
                    self.tilt_deg = tilt
                self.tilt_status = status
                self.accel_fps.tick()
                samples.append([round(now - t0, 4), round(ax, 3), round(ay, 3), round(az, 3)])
                backoff = 0.05
            except RuntimeError as exc:
                if not error_reported:
                    print(f"[kinect] accelerometer error: {exc}")
                    error_reported = True
                time.sleep(backoff)
                backoff = min(2.0, backoff * 2)  # exponential backoff while the motor device is unavailable
                next_tick = time.monotonic()
                continue
            if samples and now - last_emit >= self.emit_period:
                g = max(1e-6, (ax * ax + ay * ay + az * az) ** 0.5)
                self.on_accel({
                    "t": round(now - t0, 4),
                    "samples": samples,
                    "x": round(ax, 3), "y": round(ay, 3), "z": round(az, 3),
                    "pitch": round(float(np.degrees(np.arcsin(np.clip(az / g, -1, 1)))), 1),
                    "roll": round(float(np.degrees(np.arctan2(ax, ay))), 1),
                    "tilt": self.tilt_deg,
                    "tilt_status": self.tilt_status,
                    "hz": round(self.accel_fps.fps, 1),
                })
                samples = []
                last_emit = now
            next_tick += self.accel_period
            delay = next_tick - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_tick = time.monotonic()
