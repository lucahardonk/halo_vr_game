"""Control the desktop mouse with an AprilTag seen by the Kinect v1 RGB camera.

Mapping: empirical 2D homography (no geometry assumptions).
  During calibration the user holds the tag frontal (perpendicular to the screen)
  while pointing at each target.  The tag direction (ax = X/Z, ay = Y/Z in camera
  frame) is captured for each screen position.  After N saved points a RANSAC
  homography is fitted:
      [px, py] = H * [ax, ay, 1]      (H is 3x3)
  This mapping is Z-independent: the cursor position depends only on where the tag
  points, not how far it is held from the camera.

Calibration:
  - 13 target points spread near the screen edges (top-right HUD and bottom-centre
    buttons avoided).
  - auto.samples (default 34) frames per point are auto-accumulated; the arc on the
    overlay fills automatically and the system advances to the next point.
  - h / SPACE = standby: buffer paused, cursor frozen at target; press again to resume
    (the partial buffer is discarded so the next run starts fresh).
  - ENTER forces a save when at least 5 samples are in the buffer.
  - Salta skips the current point.  Conferma fits early (needs >= 5 saved points).

Tracking (STATE_ACTIVE):
  - h / SPACE = clutch: cursor frozen, tag can be repositioned freely; press again to resume.
  - c = recalibrate.   q / ESC = quit.

Mouse/keys backend: on Wayland the kernel uinput/evdev devices are used
(input_backend.py), on X11 pynput. Keys are global; also on the HUD/camera window.
"""
import argparse
import atexit
import ctypes as C
import math
import os
import signal
import threading
import time
from ctypes.util import find_library

# OpenCV's Qt window on GNOME/Wayland prints a warning and runs through XWayland anyway
os.environ.setdefault("QT_QPA_PLATFORM", "xcb")

import cv2
import numpy as np

# ----------------------------------------------------------------- settings
TAG_SIZE_M = 0.065     # side length of the tag's black square, metres
AXIS_LEN_M = 0.04      # drawn axis length, metres
TAG_FAMILY = "tag36h11"

# Kinect v1 RGB intrinsics (640x480, standard approximate values)
FX, FY = 525.0, 525.0
CX, CY = 319.5, 239.5
CAMERA_MATRIX = np.array([[FX, 0, CX], [0, FY, CY], [0, 0, 1]], dtype=np.float64)
DIST_COEFFS = np.zeros(5)
WIDTH, HEIGHT = 640, 480

STATE_CALIBRATING = "CALIBRATING"
STATE_FROZEN = "FROZEN"
STATE_ACTIVE = "ACTIVE"
STATE_COLORS = {STATE_ACTIVE: "#2ecc71", STATE_FROZEN: "#f39c12", STATE_CALIBRATING: "#f1c40f"}

# 13 calibration targets: near the screen edges.
# Top-right corner is left free for the HUD; bottom-centre buttons are avoided.
CALIB_NORM = [
    (0.04, 0.05), (0.50, 0.04), (0.80, 0.05),   # top (HUD sits top-right, leave gap)
    (0.96, 0.50),                                  # mid-right
    (0.96, 0.92), (0.50, 0.92), (0.04, 0.92),    # bottom
    (0.04, 0.50),                                  # mid-left
    (0.20, 0.22), (0.80, 0.22),                   # inner top
    (0.20, 0.78), (0.80, 0.78),                   # inner bottom
    (0.50, 0.50),                                  # centre
]
MIN_CALIB_POINTS = 5     # minimum saved points required to fit the homography
CALIB_MIN_SAVE = 5       # minimum samples in buffer for a manual ENTER save
CALIB_FRESH_S = 0.5      # tag must have been seen this recently to accept a point
TAG_LOST_RELEASE_S = 0.5
PIN_EVERY_S = 0.05
PREVIEW_SIZE = (480, 360)
PREVIEW_WINDOW = "mouse_control - camera"
HUD_MIN_W = 240
OVERLAY_PREVIEW = (320, 240)
TICK_MS = 30

# One-Euro filter on the cursor (pixels): lower MIN_CUTOFF = less jitter, more lag
FILTER_MIN_CUTOFF = 1.5
FILTER_BETA = 0.01


# ----------------------------------------------------------------- cameras
class FreenectPython:
    """Official python bindings (``import freenect``)."""

    name = "freenect (python)"

    def __init__(self, index):
        import freenect
        self.fn = freenect
        self.index = index

    def read(self):
        res = self.fn.sync_get_video(self.index, self.fn.VIDEO_RGB)
        if res is None:
            raise RuntimeError("freenect.sync_get_video returned no frame")
        return cv2.cvtColor(res[0], cv2.COLOR_RGB2BGR)

    def close(self):
        self.fn.sync_stop()


class FreenectCtypes:
    """libfreenect_sync through ctypes (no python bindings needed)."""

    name = "libfreenect_sync (ctypes)"

    def __init__(self, index):
        lib_name = find_library("freenect_sync")
        if not lib_name:
            raise RuntimeError("libfreenect_sync not found: sudo apt install libfreenect-dev")
        self.lib = C.CDLL(lib_name)
        self.lib.freenect_sync_get_video.argtypes = [
            C.POINTER(C.c_void_p), C.POINTER(C.c_uint32), C.c_int, C.c_int]
        self.lib.freenect_sync_get_video.restype = C.c_int
        self.lib.freenect_sync_stop.argtypes = []
        self.lib.freenect_sync_stop.restype = None
        self.index = index

    def read(self):
        ptr, ts = C.c_void_p(), C.c_uint32()
        status = self.lib.freenect_sync_get_video(C.byref(ptr), C.byref(ts), self.index, 0)
        if status != 0 or not ptr.value:
            raise RuntimeError(
                f"freenect_sync_get_video failed (code {status}): check power supply, USB, "
                "and that no other program (freenect-glview, dashboard) is using the Kinect")
        n = HEIGHT * WIDTH * 3
        rgb = np.ctypeslib.as_array(C.cast(ptr, C.POINTER(C.c_uint8)), shape=(n,)).reshape(HEIGHT, WIDTH, 3)
        return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

    def close(self):
        self.lib.freenect_sync_stop()


class OpenCVCamera:
    name = "OpenCV VideoCapture"

    def __init__(self, index):
        self.cap = cv2.VideoCapture(index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
        if not self.cap.isOpened():
            raise RuntimeError(f"cannot open OpenCV camera {index}")

    def read(self):
        ok, frame = self.cap.read()
        if not ok:
            raise RuntimeError("OpenCV camera read failed")
        return frame

    def close(self):
        self.cap.release()


def open_camera(args):
    if args.opencv is not None:
        return OpenCVCamera(args.opencv)
    try:
        return FreenectPython(args.index)
    except ImportError:
        return FreenectCtypes(args.index)


# ----------------------------------------------------------------- geometry
def tag_object_points(size):
    """3D corners in the order returned by pupil_apriltags and required by IPPE_SQUARE."""
    h = size / 2.0
    return np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float64)


FLIP = np.diag([1.0, -1.0, -1.0])


def camera_matrix_for(w, h):
    """Intrinsics are given for 640x480: scale them to the actual frame size."""
    k = CAMERA_MATRIX.copy()
    k[0, :] *= w / float(WIDTH)
    k[1, :] *= h / float(HEIGHT)
    return k


def solve_pose(corners, obj_pts, K):
    img_pts = corners.astype(np.float64)
    ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, K, DIST_COEFFS, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        return None
    R, _ = cv2.Rodrigues(rvec)
    R = R @ FLIP
    rvec, _ = cv2.Rodrigues(R)
    return rvec, tvec, R


AXIS_PTS = np.float64([[0, 0, 0], [AXIS_LEN_M, 0, 0], [0, AXIS_LEN_M, 0], [0, 0, AXIS_LEN_M]])
AXIS_COLORS = ((0, 0, 255), (0, 255, 0), (255, 0, 0))  # BGR: X red, Y green, Z blue


def draw_tag(frame, det, rvec, tvec, K):
    pts = det.corners.astype(np.int32)
    cv2.polylines(frame, [pts], True, (0, 255, 0), 2, cv2.LINE_AA)
    proj, _ = cv2.projectPoints(AXIS_PTS, rvec, tvec, K, DIST_COEFFS)
    proj = proj.reshape(-1, 2)
    if not np.all(np.isfinite(proj)) or np.abs(proj).max() > 1e5:
        return
    proj = proj.astype(np.int32)
    o = tuple(int(v) for v in proj[0])
    for i, color in enumerate(AXIS_COLORS):
        cv2.arrowedLine(frame, o, tuple(int(v) for v in proj[i + 1]), color, 3, cv2.LINE_AA, tipLength=0.15)
    label = f"ID {det.tag_id}  {float(np.linalg.norm(tvec)):.2f} m"
    org = (int(pts[:, 0].min()), int(pts[:, 1].min()) - 8)
    cv2.putText(frame, label, org, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(frame, label, org, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2, cv2.LINE_AA)


def put_text(img, text, org, scale=0.6, color=(255, 255, 255)):
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)


# ----------------------------------------------------------------- filtering
class OneEuro:
    """One-Euro filter (Casiez et al. 2012) for a scalar signal."""

    def __init__(self, min_cutoff=FILTER_MIN_CUTOFF, beta=FILTER_BETA, d_cutoff=1.0):
        self.min_cutoff, self.beta, self.d_cutoff = min_cutoff, beta, d_cutoff
        self.reset()

    def reset(self):
        self.x = self.dx = self.t = None

    @staticmethod
    def _alpha(cutoff, dt):
        tau = 1.0 / (2 * math.pi * cutoff)
        return 1.0 / (1.0 + tau / dt)

    def __call__(self, x, t):
        if self.t is None or t <= self.t:
            self.x, self.dx, self.t = x, 0.0, t
            return x
        dt = t - self.t
        dx = (x - self.x) / dt
        a_d = self._alpha(self.d_cutoff, dt)
        self.dx = a_d * dx + (1 - a_d) * self.dx
        a = self._alpha(self.min_cutoff + self.beta * abs(self.dx), dt)
        self.x = a * x + (1 - a) * self.x
        self.t = t
        return self.x


# ----------------------------------------------------------------- screen
class Screen:
    def __init__(self, x, y, w, h):
        self.x, self.y, self.w, self.h = x, y, w, h

    def clamp(self, sx, sy):
        return (min(max(sx, self.x), self.x + self.w - 1),
                min(max(sy, self.y), self.y + self.h - 1))


def primary_screen(tk_root):
    try:
        from screeninfo import get_monitors
        mons = get_monitors()
        if mons:
            m = next((m for m in mons if getattr(m, "is_primary", False)), mons[0])
            return Screen(m.x, m.y, m.width, m.height)
    except Exception as exc:
        print(f"[screen] screeninfo failed ({exc}), using Tk screen size")
    return Screen(0, 0, tk_root.winfo_screenwidth(), tk_root.winfo_screenheight())


# ----------------------------------------------------------------- controller
class MouseTagController:
    """Thread-safe state machine shared by camera, keyboard and Tk threads.

    Calibration: empirical 2D homography.
      Feature vector: (ax, ay) = (X/Z, Y/Z) from solvePnP — Z-independent direction.
      For each of the 13 screen targets the camera auto-accumulates args.samples frames
      and saves the median.  cv2.findHomography(RANSAC) maps (ax,ay) -> screen pixel.

    Tracking: perspectiveTransform with optional offset rebasing after a clutch.
    """

    def __init__(self, args, screen, mouse):
        self.args = args
        self.screen = screen
        self.mouse = mouse
        self.lock = threading.RLock()
        self.quit_flag = False
        self.state = STATE_CALIBRATING
        self.H = None                # 3x3 homography: (ax, ay, 1) -> (px, py, w)
        self.offset = np.zeros(2)    # cursor offset after a clutch rebase
        self.rebase_pending = False
        self.pressed = False         # safety only: no click logic in this version
        self.filters = (OneEuro(), OneEuro())
        self.last_seen = 0.0
        self.last_R = None
        self.last_t = None
        self.z = None
        self.fps = 0.0
        self.error = None
        self.message = ""
        self.preview = None
        self.thumb = None            # small BGR frame for the Tk overlay camera thumbnail
        self.tag_norm = None         # tag centre in the image, normalised 0..1 (mirrored X)
        self.pin = None              # where the cursor must stay (re-asserted by the main thread)
        self.last_move = 0.0
        self.calib_points = [(screen.x + nx * screen.w, screen.y + ny * screen.h)
                             for nx, ny in CALIB_NORM]
        self._reset_calibration()

    # ------------------------------------------------------------ helpers
    def _reset_calibration(self):
        self.calib_buffers = {}   # idx -> list of (ax, ay);  ax = X/Z, ay = Y/Z
        self.calib_saved = {}     # idx -> (ax, ay) median (committed)
        self.calib_idx = 0
        self.standby = False
        self.message = ""

    def release(self):
        """Safety release (no click logic, but keeps the mouse from getting stuck)."""
        with self.lock:
            if self.pressed:
                try:
                    self.mouse.release()
                finally:
                    self.pressed = False

    # ------------------------------------------------------------ camera thread
    def on_detection(self, tvec, R, now):
        t = tvec.ravel()
        with self.lock:
            self.last_seen = now
            self.last_R = R
            self.last_t = t
            self.z = float(t[2])
            if abs(t[2]) < 1e-6:
                return
            # Direction in camera frame (Z-independent): the key calibration feature
            ax, ay = float(t[0] / t[2]), float(t[1] / t[2])
            if self.state == STATE_CALIBRATING:
                if not self.standby and self.calib_idx not in self.calib_saved:
                    buf = self.calib_buffers.setdefault(self.calib_idx, [])
                    buf.append((ax, ay))
                    if len(buf) >= self.args.samples:
                        self._auto_save(self.calib_idx)
            elif self.state == STATE_ACTIVE:
                self._active_step(ax, ay, now)

    def on_no_detection(self, now):
        with self.lock:
            if now - self.last_seen > TAG_LOST_RELEASE_S:
                self.release()
                self.z = None

    def _advance(self):
        """Move calib_idx to the next unsaved point.  Fits and starts if all done."""
        n = len(self.calib_points)
        for k in range(1, n + 1):
            idx = (self.calib_idx + k) % n
            if idx not in self.calib_saved:
                self.calib_idx = idx
                self.calib_buffers.pop(idx, None)   # clear any partial buffer for this slot
                self.standby = False
                return
        # Every point has been saved -> fit homography and start tracking
        self._fit_and_start()

    def _auto_save(self, idx):
        """Commit the median of the accumulated buffer as the calibration point."""
        buf = self.calib_buffers.get(idx, [])
        if not buf:
            return
        pts = np.array(buf)
        ax = float(np.median(pts[:, 0]))
        ay = float(np.median(pts[:, 1]))
        self.calib_saved[idx] = (ax, ay)
        sp = self.calib_points[idx]
        n = len(self.calib_points)
        print(f"[calib] point {idx + 1}/{n} saved: "
              f"dir ({ax:+.4f}, {ay:+.4f}) -> screen ({int(sp[0])}, {int(sp[1])})")
        self.message = f"punto {idx + 1} salvato"
        self._advance()

    def _fit_and_start(self):
        """Fit a 2D homography on all saved calibration points, then switch to tracking."""
        n_saved = len(self.calib_saved)
        if n_saved < MIN_CALIB_POINTS:
            self.message = (f"troppo pochi punti ({n_saved}/{MIN_CALIB_POINTS} minimi): "
                            "punta altri cerchi")
            return
        indices = sorted(self.calib_saved.keys())
        src = np.array([[self.calib_saved[i][0], self.calib_saved[i][1]]
                        for i in indices], dtype=np.float64)
        dst = np.array([[float(self.calib_points[i][0]), float(self.calib_points[i][1])]
                        for i in indices], dtype=np.float64)
        H, mask = cv2.findHomography(src, dst, cv2.RANSAC, 5.0)
        if H is None or not np.all(np.isfinite(H)):
            print("[calib] homography fit failed: points too collinear — recalibrate")
            self.message = "fit fallito: punti insufficienti o collineari, premi c per ricalibrare"
            return
        self.H = H
        # Reprojection error report
        n_pts = len(src)
        src_h = np.concatenate([src, np.ones((n_pts, 1))], axis=1)   # Nx3
        proj_h = (H @ src_h.T).T                                       # Nx3
        proj = proj_h[:, :2] / proj_h[:, 2:3]
        err = np.linalg.norm(proj - dst, axis=1)
        inliers = mask.ravel().astype(bool) if mask is not None else np.ones(n_pts, bool)
        print(f"[calib] homography: {int(inliers.sum())}/{n_pts} inliers, "
              f"mean err {err[inliers].mean():.0f} px, max {err[inliers].max():.0f} px")
        for k, i in enumerate(indices):
            status = "inlier" if inliers[k] else "OUTLIER"
            print(f"[calib]   point {i + 1}: {err[k]:.0f} px ({status})")
        self._start_tracking()

    def _start_tracking(self):
        self.offset = np.zeros(2)
        self.rebase_pending = False
        for f in self.filters:
            f.reset()
        self.message = ""
        self.state = STATE_ACTIVE
        print("[state] ACTIVE")

    def _active_step(self, ax, ay, now):
        """Apply the homography and move the mouse cursor."""
        if self.H is None:
            return
        pt = np.array([[[ax, ay]]], dtype=np.float64)   # shape 1x1x2
        result = cv2.perspectiveTransform(pt, self.H)
        if result is None or not np.all(np.isfinite(result)):
            return
        raw = result[0, 0]   # (px, py) in screen pixels
        if self.rebase_pending:
            # Clutch: cursor stays at pin, tag repositioned; absorb the new tag position
            cur = np.array(self.pin if self.pin is not None else self.mouse.position,
                           dtype=np.float64)
            self.offset = cur - raw
            self.rebase_pending = False
            for f in self.filters:
                f.reset()
        q = raw + self.offset
        sx = self.filters[0](float(q[0]), now)
        sy = self.filters[1](float(q[1]), now)
        sx, sy = self.screen.clamp(sx, sy)
        self.mouse.position = (int(sx), int(sy))
        self.pin = (int(sx), int(sy))
        self.last_move = now

    # ------------------------------------------------------------ commands (any thread)
    def toggle_standby_or_freeze(self):
        """h or SPACE: standby (buffer pause) during calibration, clutch during tracking."""
        with self.lock:
            if self.state == STATE_CALIBRATING:
                self.standby = not self.standby
                if not self.standby:
                    # On resume, discard the partial buffer so the next run starts clean
                    self.calib_buffers.pop(self.calib_idx, None)
                self.message = ("\u23f8 STANDBY \u2014 riposiziona il tag, h/SPAZIO per riprendere"
                                if self.standby else "")
                print(f"[calib] standby {'ON' if self.standby else 'OFF'} "
                      f"(point {self.calib_idx + 1})")
            elif self.state == STATE_ACTIVE:
                self._freeze()
            elif self.state == STATE_FROZEN:
                self._resume()

    def _freeze(self):
        """Clutch: hold the cursor where it is while the tag is repositioned."""
        self.release()
        if self.pin is None:
            self.pin = tuple(int(v) for v in self.mouse.position)
        self.state = STATE_FROZEN
        print(f"[state] FROZEN at {self.pin}")

    def _resume(self):
        if self.state != STATE_FROZEN:
            return
        self.release()
        if self.H is None:
            self.state = STATE_CALIBRATING
        else:
            self.rebase_pending = True   # next detection absorbs the new tag position
            self.state = STATE_ACTIVE
        print(f"[state] {self.state}")

    def recalibrate(self):
        with self.lock:
            self.release()
            self._reset_calibration()
            self.H = None
            self.state = STATE_CALIBRATING
            print("[state] CALIBRATING")

    def enter(self):
        """ENTER: force-save with the current buffer (>= CALIB_MIN_SAVE), or resume if frozen."""
        with self.lock:
            if self.state == STATE_CALIBRATING:
                if self.standby:
                    self.message = "standby attivo: premi h/SPAZIO per sbloccare"
                    return
                idx = self.calib_idx
                if idx in self.calib_saved:
                    self.message = "punto gi\u00e0 salvato"
                    return
                buf = self.calib_buffers.get(idx, [])
                now = time.monotonic()
                if now - self.last_seen > CALIB_FRESH_S:
                    self.message = "tag non visibile: punto NON salvato"
                    return
                if len(buf) < CALIB_MIN_SAVE:
                    self.message = (f"troppo pochi campioni ({len(buf)}/{CALIB_MIN_SAVE}): "
                                    "aspetta o tieni il tag fermo")
                    return
                self._auto_save(idx)
            elif self.state == STATE_FROZEN:
                self._resume()

    def skip_point(self):
        """Skip the current calibration point without saving it."""
        with self.lock:
            if self.state != STATE_CALIBRATING:
                return
            self.calib_buffers.pop(self.calib_idx, None)
            self._advance()

    def confirm(self):
        """Close calibration early and fit the homography (needs >= MIN_CALIB_POINTS saved)."""
        with self.lock:
            if (self.state == STATE_CALIBRATING
                    and len(self.calib_saved) >= MIN_CALIB_POINTS):
                self._fit_and_start()

    def hold_cursor(self):
        """Main thread, every tick: keep the cursor on its pin so physical mouse cannot steal it."""
        with self.lock:
            if self.state == STATE_CALIBRATING:
                self.pin = tuple(int(v) for v in self.calib_points[self.calib_idx])
            if self.pin is None:
                return
            if self.state == STATE_ACTIVE and time.monotonic() - self.last_move < PIN_EVERY_S:
                return
            try:
                self.mouse.hold(self.pin)
            except Exception as exc:
                if not getattr(self, "_hold_err", False):
                    print(f"[mouse] cannot move the cursor: {exc}")
                    self._hold_err = True

    def request_quit(self):
        self.quit_flag = True
        self.release()

    def snapshot(self):
        with self.lock:
            buf = self.calib_buffers.get(self.calib_idx, [])
            return {
                "state": self.state,
                "z": self.z,
                "fps": self.fps,
                "error": self.error,
                "message": self.message,
                "calib_idx": self.calib_idx,
                "calib_done": set(self.calib_saved.keys()),
                "calib_buf_len": len(buf),
                "calib_total": self.args.samples,
                "standby": self.standby,
                "tag_norm": self.tag_norm,
                "tag_fresh": time.monotonic() - self.last_seen <= CALIB_FRESH_S,
                "mouse": getattr(self.mouse, "name", "?"),
            }


# ----------------------------------------------------------------- camera thread
def camera_loop(ctl, cam, detector, obj_pts, show_preview):
    frames, t_fps = 0, time.monotonic()
    K, K_size = None, None
    error_logged = None
    while not ctl.quit_flag:
        try:
            frame = cam.read()
        except RuntimeError as exc:
            if error_logged != str(exc):
                print(f"[camera] {exc}")
                error_logged = str(exc)
            ctl.error = str(exc)
            ctl.on_no_detection(time.monotonic())
            time.sleep(0.5)
            continue
        ctl.error = error_logged = None
        h, w = frame.shape[:2]
        if K is None or K_size != (w, h):
            K, K_size = camera_matrix_for(w, h), (w, h)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detections = detector.detect(gray)
        now = time.monotonic()

        best = None
        if detections:
            # largest tag in the image = the one being used as a pointer
            det = max(detections, key=lambda d: cv2.contourArea(d.corners.astype(np.float32)))
            pose = solve_pose(det.corners, obj_pts, K)
            if pose is not None:
                best = (det, pose)
                ctl.tag_norm = (1.0 - det.center[0] / w, det.center[1] / h)
                ctl.on_detection(pose[1], pose[2], now)
        if best is None:
            ctl.tag_norm = None
            ctl.on_no_detection(now)

        frames += 1
        if now - t_fps >= 0.5:
            ctl.fps, frames, t_fps = frames / (now - t_fps), 0, now

        # mirrored thumbnail for the overlay preview
        tw, th = OVERLAY_PREVIEW
        thumb = cv2.flip(cv2.resize(frame, OVERLAY_PREVIEW, interpolation=cv2.INTER_AREA), 1)
        if best is not None:
            det, (rvec, tvec, _) = best
            pts = det.corners * np.array([tw / w, th / h])
            pts[:, 0] = tw - 1 - pts[:, 0]
            cv2.polylines(thumb, [pts.astype(np.int32)], True, (0, 255, 0), 2, cv2.LINE_AA)
            put_text(thumb, f"Z {tvec.ravel()[2]:.2f} m", (8, 22), 0.6, (0, 160, 0))
            draw_tag(frame, det, rvec, tvec, K)
        ctl.thumb = thumb
        if show_preview:
            snap = ctl.snapshot()
            z = "--" if snap["z"] is None else f"{snap['z']:.3f}"
            put_text(frame, f"{snap['state']}  Z {z} m  {snap['fps']:.0f} fps", (10, 28), 0.8)
            ctl.preview = cv2.resize(frame, PREVIEW_SIZE, interpolation=cv2.INTER_AREA)


# ----------------------------------------------------------------- Tk UI
class HUD:
    def __init__(self, root, screen):
        import tkinter as tk
        self.root = root
        root.overrideredirect(True)
        root.attributes("-topmost", True)
        root.configure(bg="#1b2028", padx=4, pady=4)
        self.screen = screen
        self.size = None
        mk = lambda font, fg="#e0e0e0": tk.Label(root, text="", font=font, fg=fg, bg="#1b2028", anchor="w")
        self.state_lbl = mk(("DejaVu Sans", 13, "bold"))
        self.info_lbl  = mk(("DejaVu Sans Mono", 11))
        self.z_lbl     = mk(("DejaVu Sans Mono", 11))
        self.fps_lbl   = mk(("DejaVu Sans Mono", 9), "#8a939e")
        self.backend   = mk(("DejaVu Sans Mono", 9), "#8a939e")
        self.msg       = tk.Label(root, text="", font=("DejaVu Sans", 9), fg="#f39c12", bg="#1b2028",
                                  anchor="w", justify="left", wraplength=HUD_MIN_W)
        for w in (self.state_lbl, self.info_lbl, self.z_lbl, self.fps_lbl, self.backend, self.msg):
            w.pack(fill="x", padx=6)

    def _place(self):
        self.root.update_idletasks()
        w = max(HUD_MIN_W, self.root.winfo_reqwidth())
        h = self.root.winfo_reqheight()
        if (w, h) != self.size:
            self.size = (w, h)
            sc = self.screen
            self.root.geometry(f"{w}x{h}+{sc.x + sc.w - w - 10}+{sc.y + 10}")

    def update(self, s):
        self.state_lbl.config(text=s["state"], fg=STATE_COLORS[s["state"]])
        n_total = len(CALIB_NORM)
        if s["state"] == STATE_CALIBRATING:
            done = len(s["calib_done"])
            buf = s["calib_buf_len"]
            tot = s["calib_total"]
            pt = s["calib_idx"] + 1
            if s["standby"]:
                self.info_lbl.config(text=f"punto {pt}/{n_total}  STANDBY")
            else:
                bar = int(10 * buf / max(tot, 1))
                self.info_lbl.config(text=f"punto {pt}/{n_total}  [{'#' * bar}{'.' * (10 - bar)}] {buf}/{tot}")
            self.z_lbl.config(text=f"salvati: {done}/{n_total}")
        elif s["state"] == STATE_FROZEN:
            self.info_lbl.config(text="FERMO \u2014 riposiziona il tag")
            self.z_lbl.config(text="Z: --" if s["z"] is None else f"Z: {s['z']:.3f} m")
        else:
            self.info_lbl.config(text="tracking attivo")
            self.z_lbl.config(text="Z: --" if s["z"] is None else f"Z: {s['z']:.3f} m")
        self.fps_lbl.config(text=f"camera {s['fps']:4.1f} fps")
        self.backend.config(text=f"mouse: {s['mouse'].split(' ')[0]}")
        if s["error"]:
            msg = f"Camera: {s['error'][:90]}"
        elif s["state"] == STATE_FROZEN:
            msg = "h / SPAZIO per riprendere"
        elif s["state"] == STATE_CALIBRATING and s["standby"]:
            msg = f"STANDBY punto {s['calib_idx'] + 1} \u2014 h/SPAZIO per riprendere"
        elif s["message"]:
            msg = s["message"]
        else:
            msg = ""
        self.msg.config(text=msg)
        self._place()


class CalibrationOverlay:
    def __init__(self, root, screen, ctl):
        import tkinter as tk
        self.ctl, self.screen = ctl, screen
        self.win = tk.Toplevel(root)
        self.win.overrideredirect(True)
        self.win.configure(bg="black")
        self.win.geometry(f"{screen.w}x{screen.h}+{screen.x}+{screen.y}")
        try:
            self.win.attributes("-alpha", 0.85)
        except tk.TclError:
            pass
        self.win.attributes("-topmost", True)
        self.canvas = tk.Canvas(self.win, bg="black", highlightthickness=0, width=screen.w, height=screen.h)
        self.canvas.pack(fill="both", expand=True)
        btn = dict(font=("DejaVu Sans", 12, "bold"), padx=14, pady=6, bd=0)
        self.skip_btn = tk.Button(self.win, text="Salta", command=ctl.skip_point,
                                  bg="#495057", fg="white", activebackground="#6c757d", **btn)
        self.save_btn = tk.Button(self.win, text="Salva punto (INVIO)", command=ctl.enter,
                                  bg="#0d6efd", fg="white", activebackground="#3d8bfd", **btn)
        self.confirm_btn = tk.Button(self.win, text="Conferma", command=ctl.confirm, state="disabled",
                                     bg="#198754", fg="white", activebackground="#20c997",
                                     disabledforeground="#9aa", **btn)
        self.preview_lbl = tk.Label(self.win, bg="black", bd=2, relief="solid")
        self.preview_lbl.place(relx=1.0, rely=1.0, anchor="se", x=-20, y=-20)
        self.photo = None
        self.save_btn.place(relx=0.5, rely=0.94, anchor="center")
        self.skip_btn.place(relx=0.5, rely=0.94, anchor="e", x=-120)
        self.confirm_btn.place(relx=0.5, rely=0.94, anchor="w", x=120)
        self.visible = True

    def show(self, on):
        if on and not self.visible:
            self.win.deiconify()
            self.win.attributes("-topmost", True)
            self.visible = True
        elif not on and self.visible:
            self.win.withdraw()
            self.visible = False

    def update(self, s):
        c, sc = self.canvas, self.screen
        c.delete("dyn")
        n = len(CALIB_NORM)
        done = s["calib_done"]
        idx = s["calib_idx"]

        # Draw all calibration targets
        for i, (px, py) in enumerate(self.ctl.calib_points):
            x, y = px - sc.x, py - sc.y
            if i in done:
                # Saved: green filled circle
                c.create_oval(x - 10, y - 10, x + 10, y + 10,
                              fill="#2ecc71", outline="", tags="dyn")
            elif i != idx:
                # Pending: grey outline circle
                c.create_oval(x - 7, y - 7, x + 7, y + 7,
                              outline="#6c757d", width=2, tags="dyn")

        # Current target: arc progress or standby indicator
        px, py = self.ctl.calib_points[idx]
        x, y = px - sc.x, py - sc.y
        r = 34
        buf_len = s["calib_buf_len"]
        calib_total = max(s["calib_total"], 1)

        if s["standby"]:
            # Red filled circle = paused
            c.create_oval(x - r, y - r, x + r, y + r,
                          fill="#e74c3c", outline="", tags="dyn")
            c.create_oval(x - 5, y - 5, x + 5, y + 5,
                          fill="black", outline="", tags="dyn")
        else:
            # Background ring
            c.create_oval(x - r, y - r, x + r, y + r,
                          outline="#f39c12", width=3, tags="dyn")
            # Filled arc: progress
            if buf_len > 0:
                extent = -360.0 * buf_len / calib_total
                c.create_arc(x - r, y - r, x + r, y + r,
                             start=90, extent=extent,
                             fill="#f39c12", outline="", style="pieslice", tags="dyn")
            c.create_oval(x - 5, y - 5, x + 5, y + 5,
                          fill="white", outline="", tags="dyn")

        # Instruction text — placed in the vertical band between inner and edge rows
        ty_text = int(sc.h * 0.40)
        fs = max(12, min(18, sc.w // 70))
        k_done = len(done)
        if s["standby"]:
            text = (f"punto {k_done + 1}/{n} \u2014 STANDBY: riposiziona il tag, "
                    "poi premi h o SPAZIO per riprendere")
        else:
            text = (f"punto {k_done + 1}/{n}: tieni il tag fermo, puntato al cerchio "
                    "\u2014 raccolta automatica in corso")
        c.create_text(sc.w // 2, ty_text, text=text,
                      fill="white", font=("DejaVu Sans", fs, "bold"),
                      width=int(sc.w * 0.8), justify="center", tags="dyn")

        # Sub-line: Z, buffer count, message
        sub = []
        if s["z"] is None:
            sub.append("tag non visibile")
        else:
            sub.append(f"Z = {s['z']:.3f} m")
        if not s["standby"]:
            sub.append(f"campioni: {buf_len}/{calib_total}")
        if s["message"]:
            sub.append(s["message"])
        if k_done >= MIN_CALIB_POINTS:
            sub.append("Conferma chiude la calibrazione")
        c.create_text(sc.w // 2, ty_text + 2 * fs + 10,
                      text="   \u00b7   ".join(sub),
                      fill="#adb5bd", font=("DejaVu Sans", 13), tags="dyn")

        # Horizontal progress bar: saved points / total
        bw = int(sc.w * 0.5)
        bh = 14
        by = ty_text + 3 * fs + 24
        bx = (sc.w - bw) // 2
        c.create_rectangle(bx, by, bx + bw, by + bh,
                           outline="#6c757d", width=1, tags="dyn")
        if k_done > 0:
            c.create_rectangle(bx, by, bx + int(bw * k_done / n), by + bh,
                               fill="#2ecc71", outline="", tags="dyn")

        self.confirm_btn.config(
            state="normal" if k_done >= MIN_CALIB_POINTS else "disabled",
            text=f"Conferma ({k_done})")

        # Live tag crosshair: where in the camera view the tag currently is
        if s["tag_norm"] is not None:
            tx2 = s["tag_norm"][0] * sc.w
            ty2 = s["tag_norm"][1] * sc.h
            col = "#2ecc71" if s["tag_fresh"] else "#e67e22"
            c.create_line(tx2 - 18, ty2, tx2 + 18, ty2, fill=col, width=3, tags="dyn")
            c.create_line(tx2, ty2 - 18, tx2, ty2 + 18, fill=col, width=3, tags="dyn")
            c.create_oval(tx2 - 10, ty2 - 10, tx2 + 10, ty2 + 10,
                          outline=col, width=2, tags="dyn")

        self._update_preview()

    def _update_preview(self):
        import base64
        import tkinter as tk
        img = self.ctl.thumb
        if img is None:
            return
        ok, buf = cv2.imencode(".png", img, [cv2.IMWRITE_PNG_COMPRESSION, 1])
        if not ok:
            return
        try:
            self.photo = tk.PhotoImage(data=base64.b64encode(buf.tobytes()), format="png")
            self.preview_lbl.config(image=self.photo)
        except tk.TclError:
            pass


# ----------------------------------------------------------------- keyboard
def key_dispatcher(ctl):
    """Global key listener actions (evdev / pynput backend)."""
    actions = {
        "space": ctl.toggle_standby_or_freeze,
        "enter": ctl.enter,
        "esc":   ctl.request_quit,
        "q":     ctl.request_quit,
        "c":     ctl.recalibrate,
        "h":     ctl.toggle_standby_or_freeze,
    }
    return lambda name: actions.get(name, lambda: None)()


def handle_local_key(ctl, k):
    """Same bindings for keys received by the OpenCV camera window."""
    if k == 32:                              # SPACE
        ctl.toggle_standby_or_freeze()
    elif k in (13, 10):                      # ENTER
        ctl.enter()
    elif k in (27, ord("q"), ord("Q")):      # ESC / q
        ctl.request_quit()
    elif k in (ord("c"), ord("C")):          # c
        ctl.recalibrate()
    elif k in (ord("h"), ord("H")):          # h
        ctl.toggle_standby_or_freeze()


# ----------------------------------------------------------------- main
def parse_args():
    p = argparse.ArgumentParser(description="Mouse control with an AprilTag and the Kinect v1 camera")
    p.add_argument("--tag-size", type=float, default=TAG_SIZE_M,
                   help="tag black-square side in metres")
    p.add_argument("--index", type=int, default=0,
                   help="Kinect device index")
    p.add_argument("--opencv", type=int, default=None, metavar="N",
                   help="use OpenCV camera N instead of the Kinect")
    p.add_argument("--samples", type=int, default=34,
                   help="frames to auto-collect per calibration point (default 34)")
    p.add_argument("--decimate", type=float, default=1.0,
                   help="detector quad_decimate (2.0 = faster, less accurate)")
    p.add_argument("--threads", type=int, default=4,
                   help="detector threads")
    p.add_argument("--no-display", action="store_true",
                   help="no camera window, only the Tk HUD")
    p.add_argument("--mouse", choices=("auto", "uinput", "pynput"), default="auto",
                   help="mouse backend: auto = uinput on Wayland, pynput on X11")
    p.add_argument("--keys", choices=("auto", "evdev", "pynput"), default="auto",
                   help="global keyboard backend: auto = evdev if readable, else pynput")
    return p.parse_args()


def main():
    args = parse_args()

    import tkinter as tk
    from pupil_apriltags import Detector
    from input_backend import make_keyboard, make_mouse

    detector = Detector(families=TAG_FAMILY, nthreads=args.threads, quad_decimate=args.decimate,
                        quad_sigma=0.0, refine_edges=1, decode_sharpening=0.25)
    cam = open_camera(args)
    print(f"Camera: {cam.name} | tag {TAG_FAMILY} {args.tag_size * 100:.1f} cm | "
          f"{args.samples} samples/point, {len(CALIB_NORM)} calibration points")

    root = tk.Tk()
    root.title("mouse_control HUD")
    screen = primary_screen(root)
    print(f"Screen: {screen.w}x{screen.h} at ({screen.x},{screen.y})")
    try:
        mouse = make_mouse(args.mouse, screen)
    except Exception as exc:
        cam.close()
        raise SystemExit(f"[mouse] {exc}")
    print(f"Mouse: {mouse.name}")
    ctl = MouseTagController(args, screen, mouse)

    closed = {"done": False}

    def shutdown():
        if closed["done"]:
            return
        closed["done"] = True
        ctl.quit_flag = True
        ctl.release()
        mouse.close()
        try:
            cam.close()
        except Exception:
            pass
        if not args.no_display:
            cv2.destroyAllWindows()
        try:
            root.destroy()
        except tk.TclError:
            pass
        print("bye")

    atexit.register(ctl.release)
    signal.signal(signal.SIGINT, lambda *_: ctl.request_quit())

    hud = HUD(root, screen)
    overlay = CalibrationOverlay(root, screen, ctl)
    # Both h and SPACE trigger the unified standby/freeze toggle
    for w in (root, overlay.win):
        w.bind_all("<space>",  lambda e: ctl.toggle_standby_or_freeze())
        w.bind_all("<Return>", lambda e: ctl.enter())
        w.bind_all("<Escape>", lambda e: ctl.request_quit())
        w.bind_all("q",        lambda e: ctl.request_quit())
        w.bind_all("c",        lambda e: ctl.recalibrate())
        w.bind_all("h",        lambda e: ctl.toggle_standby_or_freeze())

    try:
        kb = make_keyboard(args.keys, key_dispatcher(ctl))
        print(f"Keys: {kb.name}")
    except Exception as exc:
        print(f"[keys] global listener unavailable ({exc}); keys work only on the HUD/camera window")

    if not args.no_display:
        try:
            cv2.namedWindow(PREVIEW_WINDOW, cv2.WINDOW_AUTOSIZE)
        except cv2.error as exc:
            print(f"[display] OpenCV has no GUI support ({str(exc).strip()[:80]}...): camera window disabled")
            args.no_display = True
    cam_thread = threading.Thread(target=camera_loop, name="camera", daemon=True,
                                  args=(ctl, cam, detector, tag_object_points(args.tag_size),
                                        not args.no_display))
    cam_thread.start()

    def tick():
        if ctl.quit_flag:
            shutdown()
            return
        s = ctl.snapshot()
        hud.update(s)
        calibrating = s["state"] == STATE_CALIBRATING
        ctl.hold_cursor()
        overlay.show(calibrating)
        if calibrating:
            overlay.update(s)
        root.lift()
        if not args.no_display:
            img = ctl.preview
            if img is not None:
                cv2.imshow(PREVIEW_WINDOW, img)
            k = cv2.waitKey(1)
            if k != -1:
                handle_local_key(ctl, k & 0xFF)
        root.after(TICK_MS, tick)

    root.after(TICK_MS, tick)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        shutdown()


if __name__ == "__main__":
    main()
