"""Control the desktop mouse with an AprilTag seen by the Kinect v1 RGB camera.

- Pointing = ray cast: the line through the tag along its normal is intersected
  with the screen plane (assumed to be the camera plane Z=0, the Kinect sits on
  the monitor; --screen-offset moves it). Tilting the tag therefore moves the
  cursor more when the tag is far, like a laser pointer.
- Small tags give a noisy orientation (several degrees when frontal), so the
  first --tilt-deadzone degrees of tilt are ignored.
- Calibration = two sets of 5-8 points saved with ENTER, tag held straight:
  first a FAR set (normal pointing distance), then a NEAR set (tag pushed
  towards the camera, as when clicking). Each set gives a homography
  ray-hit XY -> screen; while tracking the two are blended according to Z.
- Click thresholds come from the two sets: the left button is held when the tag
  gets close to the NEAR distance and released near the FAR one.
- The HUD shows the tag rotation about its own Z axis, relative to the
  orientation it had when tracking (re)started.

Mouse/keys backend: on Wayland the kernel uinput/evdev devices are used
(input_backend.py), on X11 pynput. Keys are global; also on the HUD/camera window:
    ENTER  calibration: save the point / frozen: resume
    SPACE  clutch: cursor stays still, reposition the tag; SPACE/ENTER again resumes
    c      recalibrate                           q/ESC  quit
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

# OpenCV's Qt window on GNOME/Wayland prints a warning and runs through XWayland anyway: say so explicitly
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

CALIB_NORM = [
    (0.2, 0.2), (0.8, 0.2), (0.5, 0.5),
    (0.2, 0.8), (0.8, 0.8),
    (0.5, 0.15), (0.5, 0.85),
    (0.15, 0.5),
]
MIN_CALIB_POINTS = 5
TAG_LOST_RELEASE_S = 0.5
PIN_EVERY_S = 0.05      # re-assert the cursor position this often when the tag is not moving it
PREVIEW_SIZE = (480, 360)
PREVIEW_WINDOW = "mouse_control - camera"
HUD_MIN_W = 220
OVERLAY_PREVIEW = (320, 240)   # camera thumbnail shown inside the calibration overlay
CALIB_HISTORY_S = 0.4          # tag samples (median) used when a calibration point is saved
CALIB_FRESH_S = 0.3            # the tag must have been seen this recently to save a point
MIN_Z_GAP = 0.04               # minimum FAR-NEAR distance difference (m)
PRESS_AT = 0.4                 # press when Z is 40% of the way from NEAR to FAR ...
RELEASE_AT = 0.7               # ... release when it is back at 70% (hysteresis)
MAX_TILT_DEG = 65.0            # beyond this the ray hits the screen too obliquely: ignored
PHASE_LABEL = {"far": "LONTANO", "near": "VICINO"}
PHASE_COLOR = {"far": "#f1c40f", "near": "#3498db"}
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
        status = self.lib.freenect_sync_get_video(C.byref(ptr), C.byref(ts), self.index, 0)  # 0 = RGB
        if status != 0 or not ptr.value:
            raise RuntimeError(
                f"freenect_sync_get_video failed (code {status}): check power supply, USB, "
                "and that no other program (freenect-glview, dashboard) is using the Kinect")
        n = HEIGHT * WIDTH * 3
        rgb = np.ctypeslib.as_array(C.cast(ptr, C.POINTER(C.c_uint8)), shape=(n,)).reshape(HEIGHT, WIDTH, 3)
        return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)  # cvtColor copies out of the driver buffer

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


# 180 deg about X: tag frame with x right, y up, z out of the tag towards the camera
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
    except Exception as exc:  # screeninfo missing or no backend
        print(f"[screen] screeninfo failed ({exc}), using Tk screen size")
    return Screen(0, 0, tk_root.winfo_screenwidth(), tk_root.winfo_screenheight())


# ----------------------------------------------------------------- controller
class MouseTagController:
    """Thread-safe state machine shared by camera, keyboard and Tk threads."""

    def __init__(self, args, screen, mouse):
        self.args = args
        self.screen = screen
        self.mouse = mouse
        self.lock = threading.RLock()
        self.quit_flag = False
        self.state = STATE_CALIBRATING
        self.H_far = self.H_near = None
        self.z_far = self.z_near = None
        self.click_near = args.click_near     # None = computed from the calibration
        self.click_far = args.click_far
        self.normal_filters = (OneEuro(1.0, 0.3), OneEuro(1.0, 0.3), OneEuro(1.0, 0.3))
        self.tilt = None                      # current tag tilt (deg) w.r.t. the camera axis
        self.R0 = None
        self.offset = np.zeros(2)
        self.rebase_pending = False
        self.pressed = False
        self.click_armed = False
        self.filters = (OneEuro(), OneEuro())
        self.last_seen = 0.0
        self.last_R = None
        self.last_t = None
        self.z = None
        self.yaw = None
        self.fps = 0.0
        self.error = None
        self.message = ""
        self.preview = None
        self.thumb = None             # small BGR frame for the Tk overlay
        self.tag_norm = None          # tag centre in the image, normalised 0..1 (mirrored X)
        self.pin = None               # where the cursor must stay (re-asserted by the main thread)
        self.last_move = 0.0
        self.calib_points = [(screen.x + nx * screen.w, screen.y + ny * screen.h) for nx, ny in CALIB_NORM]
        self._reset_calibration()

    # ------------------------------------------------------------ helpers
    def _reset_calibration(self):
        self.phase = "far"
        self.sets = {"far": {}, "near": {}}   # idx -> (hit_x, hit_y, tag_z)
        self.calib_idx = 0
        self.calib_hist = []                  # (time, hit_x, hit_y, tag_z)
        self.message = ""

    def _press(self):
        if not self.pressed:
            self.mouse.press()
            self.pressed = True

    def release(self):
        with self.lock:
            if self.pressed:
                try:
                    self.mouse.release()
                finally:
                    self.pressed = False

    def _yaw(self, R):
        if self.R0 is None or R is None:
            return None
        R_rel = self.R0.T @ R
        return math.degrees(math.atan2(R_rel[1, 0], R_rel[0, 0]))

    def _aim(self, t, R, now):
        """Intersection of the tag's pointing ray (its normal) with the screen plane Z = offset.

        hit = t_xy + (offset - t_z) * tan(tilt)   (camera frame, metres).
        A straight tag gives hit = t_xy; tilting it shifts the hit by ~Z * tan(tilt).
        """
        n = R[:, 2]
        n = n / np.linalg.norm(n)
        if n[2] < 0:
            n = -n                                       # normal sign is irrelevant for a line
        n = np.array([f(float(v), now) for f, v in zip(self.normal_filters, n)])
        n = n / np.linalg.norm(n)
        self.tilt = math.degrees(math.acos(min(1.0, abs(n[2]))))
        if self.tilt > MAX_TILT_DEG:
            return None
        if self.state == STATE_CALIBRATING:
            return np.array([t[0], t[1]])               # calibration: tag held straight, position only
        # soft dead zone: the pose of a small frontal tag wobbles by several degrees, so the
        # first --tilt-deadzone degrees are ignored and only the excess tilts the ray (no jump)
        ax, ay = math.atan2(n[0], n[2]), math.atan2(n[1], n[2])
        mag = math.hypot(ax, ay)
        dz = math.radians(self.args.tilt_deadzone)
        if mag <= dz:
            return np.array([t[0], t[1]])
        scale = (mag - dz) / mag
        k = (self.args.screen_offset - t[2]) * self.args.tilt_gain
        return np.array([t[0] + k * math.tan(ax * scale), t[1] + k * math.tan(ay * scale)])

    def _map(self, hit, z):
        """Ray hit -> screen pixel: FAR and NEAR homographies blended by the tag Z."""
        xy = np.array([[hit]], dtype=np.float64)
        pf = cv2.perspectiveTransform(xy, self.H_far)[0, 0]
        pn = cv2.perspectiveTransform(xy, self.H_near)[0, 0]
        w = min(max((self.z_far - z) / (self.z_far - self.z_near), 0.0), 1.0)   # 0 FAR .. 1 NEAR
        p = (1.0 - w) * pf + w * pn
        return p if np.all(np.isfinite(p)) else None

    # ------------------------------------------------------------ camera thread
    def on_detection(self, tvec, R, now):
        t = tvec.ravel()
        with self.lock:
            self.last_seen, self.last_R, self.last_t, self.z = now, R, t, float(t[2])
            hit = self._aim(t, R, now)
            if hit is not None:
                if self.state == STATE_CALIBRATING:
                    self.calib_hist.append((now, float(hit[0]), float(hit[1]), float(t[2])))
                    self.calib_hist = [h for h in self.calib_hist if now - h[0] <= CALIB_HISTORY_S]
                elif self.state == STATE_ACTIVE:
                    self._active_step(hit, t, R, now)
            self.yaw = self._yaw(R)

    def on_no_detection(self, now):
        with self.lock:
            if now - self.last_seen > TAG_LOST_RELEASE_S:
                self.release()       # safety: never keep the button down without a tag
                self.z = None
                self.tilt = None
                for f in self.normal_filters:
                    f.reset()

    def _advance(self):
        reg = self.sets[self.phase]
        n = len(self.calib_points)
        for k in range(1, n + 1):
            idx = (self.calib_idx + k) % n
            if idx not in reg:
                self.calib_idx = idx
                return

    def _save_point(self):
        """ENTER during calibration: store the ray hit for the highlighted point."""
        now = time.monotonic()
        win = [h for h in self.calib_hist if now - h[0] <= CALIB_FRESH_S]
        if now - self.last_seen > CALIB_FRESH_S or not win:
            self.message = "tag non visibile (o troppo inclinato): punto NON salvato"
            print("[calib] ENTER ignored: tag not visible")
            return
        x, y, z = (float(np.median([h[k] for h in win])) for k in (1, 2, 3))
        if self.phase == "near" and z > self.z_far - MIN_Z_GAP:
            self.message = (f"troppo lontano (Z {z:.2f} m): nel set VICINO tieni il tag sotto "
                            f"{self.z_far - MIN_Z_GAP:.2f} m")
            print(f"[calib] ENTER ignored: Z {z:.3f} m not closer than the FAR set")
            return
        self.sets[self.phase][self.calib_idx] = (x, y, z)
        sp = tuple(int(v) for v in self.calib_points[self.calib_idx])
        print(f"[calib] {self.phase} point {self.calib_idx + 1}: tag ({x:+.3f}, {y:+.3f}) m, "
              f"Z {z:.3f} m -> screen {sp}")
        self.message = ""
        if len(self.sets[self.phase]) == len(self.calib_points):
            self._end_phase()
        else:
            self._advance()

    def _fit(self, reg):
        idx = sorted(reg)
        src = np.array([reg[i][:2] for i in idx], dtype=np.float64)
        dst = np.array([self.calib_points[i] for i in idx], dtype=np.float64)
        thr = 0.08 * max(self.screen.w, self.screen.h)  # hand pointing is noisy: tolerant RANSAC
        H, mask = cv2.findHomography(src, dst, cv2.RANSAC, thr)
        if H is None or mask is None or int(mask.sum()) < 4:
            H, _ = cv2.findHomography(src, dst, 0)      # least squares on all points
        if H is None:
            return None, None
        err = np.linalg.norm(cv2.perspectiveTransform(src[None], H)[0] - dst, axis=1)
        return H, float(err.mean())

    def _end_phase(self):
        reg = self.sets[self.phase]
        H, err = self._fit(reg)
        if H is None:
            print(f"[calib] {self.phase} set: homography failed, repeat the set")
            self.sets[self.phase] = {}
            self.calib_idx = 0
            self.message = f"set {PHASE_LABEL[self.phase]} non valido: ripetilo con punti piu' distinti"
            return
        z = float(np.median([v[2] for v in reg.values()]))
        print(f"[calib] {self.phase} set: {len(reg)} points, Z median {z:.3f} m, error {err:.0f} px")
        if self.phase == "far":
            self.H_far, self.z_far = H, z
            self.phase = "near"
            self.calib_idx = -1
            self._advance()
            self.message = f"set LONTANO salvato (Z {z:.2f} m): ora avvicina il tag e ripeti i punti"
            return
        if self.z_far - z < MIN_Z_GAP:
            self.sets["near"] = {}
            self.calib_idx = 0
            self.message = "set VICINO troppo simile al LONTANO: avvicina di piu' il tag"
            return
        self.H_near, self.z_near = H, z
        gap = self.z_far - self.z_near
        if self.args.click_near is None:
            self.click_near = self.z_near + PRESS_AT * gap
        if self.args.click_far is None:
            self.click_far = self.z_near + RELEASE_AT * gap
        print(f"[calib] click: press Z < {self.click_near:.3f} m, release Z > {self.click_far:.3f} m")
        self._start_tracking()

    def _start_tracking(self):
        self.R0 = self.last_R
        self.offset = np.zeros(2)
        self.rebase_pending = False
        self.click_armed = False
        for f in self.filters:
            f.reset()
        self.message = ""
        self.state = STATE_ACTIVE
        print("[state] ACTIVE")

    def _active_step(self, hit, t, R, now):
        if self.H_near is None:
            return
        raw = self._map(hit, t[2])
        if raw is None:
            return
        if self.rebase_pending:
            # clutch: keep the cursor where it was frozen, continue from the tag's new position
            cur = np.array(self.pin if self.pin is not None else self.mouse.position, dtype=np.float64)
            self.offset = cur - raw
            self.R0 = R
            self.rebase_pending = False
            for f in self.filters:
                f.reset()
        p = raw + self.offset
        sx = self.filters[0](float(p[0]), now)
        sy = self.filters[1](float(p[1]), now)
        sx, sy = self.screen.clamp(sx, sy)
        self.mouse.position = (int(sx), int(sy))
        self.pin = (int(sx), int(sy))
        self.last_move = now

        z = t[2]
        if not self.click_armed:
            if z > self.click_far:
                self.click_armed = True
        elif z < self.click_near and not self.pressed:
            self._press()
        elif z > self.click_far and self.pressed:
            self.release()

    # ------------------------------------------------------------ commands (any thread)
    def freeze(self):
        """Clutch: the cursor stays exactly where it is while the tag is repositioned."""
        with self.lock:
            if self.state == STATE_ACTIVE:
                self.release()
                if self.pin is None:
                    self.pin = tuple(int(v) for v in self.mouse.position)
                self.state = STATE_FROZEN
                print(f"[state] FROZEN, cursor held at {self.pin}")

    def toggle_freeze(self):
        with self.lock:
            if self.state == STATE_ACTIVE:
                self.freeze()
            elif self.state == STATE_FROZEN:
                self.resume()
            # during calibration SPACE does nothing (use Salta / Conferma)

    def resume(self):
        with self.lock:
            if self.state != STATE_FROZEN:
                return
            self.release()
            if self.H_near is None:
                self.state = STATE_CALIBRATING
            else:
                self.rebase_pending = True    # R0 + offset taken from the next detection
                self.click_armed = False
                self.state = STATE_ACTIVE
            print(f"[state] {self.state}")

    def recalibrate(self):
        with self.lock:
            self.release()
            self._reset_calibration()
            self.state = STATE_CALIBRATING
            print("[state] CALIBRATING")

    def enter(self):
        """ENTER: save the calibration point, or resume after a freeze."""
        with self.lock:
            if self.state == STATE_CALIBRATING:
                self._save_point()
            elif self.state == STATE_FROZEN:
                self.resume()

    def skip_point(self):
        with self.lock:
            if self.state == STATE_CALIBRATING:
                self._advance()

    def confirm(self):
        """Close the current set early (needs MIN_CALIB_POINTS points)."""
        with self.lock:
            if self.state == STATE_CALIBRATING and len(self.sets[self.phase]) >= MIN_CALIB_POINTS:
                self._end_phase()

    def hold_cursor(self):
        """Main thread, every tick: keep the cursor on its pin (calibration target, frozen spot,
        or last tag position) so that moving the physical mouse cannot steal it."""
        with self.lock:
            if self.state == STATE_CALIBRATING:
                self.pin = tuple(int(v) for v in self.calib_points[self.calib_idx])
            if self.pin is None or time.monotonic() - self.last_move < PIN_EVERY_S:
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
            return {
                "state": self.state, "z": self.z, "yaw": self.yaw, "tilt": self.tilt,
                "pressed": self.pressed, "fps": self.fps, "error": self.error, "message": self.message,
                "phase": self.phase, "calib_idx": self.calib_idx,
                "calib_done": set(self.sets[self.phase]), "z_far": self.z_far,
                "click_near": self.click_near, "click_far": self.click_far,
                "click_armed": self.click_armed, "tag_norm": self.tag_norm,
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
                ctl.tag_norm = (1.0 - det.center[0] / w, det.center[1] / h)  # mirrored: like a mirror
                ctl.on_detection(pose[1], pose[2], now)
        if best is None:
            ctl.tag_norm = None
            ctl.on_no_detection(now)

        frames += 1
        if now - t_fps >= 0.5:
            ctl.fps, frames, t_fps = frames / (now - t_fps), 0, now

        # mirrored thumbnail (natural when facing the camera); tag outline drawn after the flip
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
            if snap["pressed"]:
                put_text(frame, "CLICK", (10, 60), 0.8, (0, 0, 255))
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
        self.state = mk(("DejaVu Sans", 13, "bold"))
        self.angle = mk(("DejaVu Sans Mono", 12))
        self.z = mk(("DejaVu Sans Mono", 11))
        self.click = mk(("DejaVu Sans", 11))
        self.fps = mk(("DejaVu Sans Mono", 9), "#8a939e")
        self.backend = mk(("DejaVu Sans Mono", 9), "#8a939e")
        self.msg = tk.Label(root, text="", font=("DejaVu Sans", 9), fg="#f39c12", bg="#1b2028",
                            anchor="w", justify="left", wraplength=HUD_MIN_W)
        for w in (self.state, self.angle, self.z, self.click, self.fps, self.backend, self.msg):
            w.pack(fill="x", padx=6)

    def _place(self):
        """Let Tk compute the size (fonts scale with DPI) and keep the HUD in the top-right corner."""
        self.root.update_idletasks()
        w = max(HUD_MIN_W, self.root.winfo_reqwidth())
        h = self.root.winfo_reqheight()
        if (w, h) != self.size:
            self.size = (w, h)
            sc = self.screen
            self.root.geometry(f"{w}x{h}+{sc.x + sc.w - w - 10}+{sc.y + 10}")

    def update(self, s):
        self.state.config(text=s["state"], fg=STATE_COLORS[s["state"]])
        yaw = "--.-" if s["yaw"] is None else f"{s['yaw']:+.1f}"
        tilt = "--" if s["tilt"] is None else f"{s['tilt']:.0f}"
        self.angle.config(text=f"\u21ba {yaw}\u00b0  incl {tilt}\u00b0")
        self.z.config(text="Z: --" if s["z"] is None else f"Z: {s['z']:.3f} m")
        self.click.config(text="\u25cf PREMUTO" if s["pressed"] else "\u25cb rilasciato",
                          fg="#e74c3c" if s["pressed"] else "#adb5bd")
        self.fps.config(text=f"camera {s['fps']:4.1f} fps")
        self.backend.config(text=f"mouse: {s['mouse'].split(' ')[0]}")
        if s["error"]:
            msg = f"Camera: {s['error'][:90]}"
        elif s["state"] == STATE_FROZEN:
            msg = "FERMO \u2014 riposiziona il tag, SPAZIO o INVIO per riprendere"
        elif s["state"] == STATE_ACTIVE and not s["click_armed"]:
            msg = f"allontana il tag (Z > {s['click_far']:.2f} m) per abilitare il click"
        elif s["state"] == STATE_ACTIVE:
            msg = f"click Z < {s['click_near']:.2f} m, rilascio Z > {s['click_far']:.2f} m"
        else:
            msg = s["message"]
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
        for i, (px, py) in enumerate(self.ctl.calib_points):
            x, y = px - sc.x, py - sc.y
            if i in done:
                c.create_oval(x - 8, y - 8, x + 8, y + 8, fill="#2ecc71", outline="", tags="dyn")
            elif i != idx:
                c.create_oval(x - 6, y - 6, x + 6, y + 6, outline="#6c757d", width=2, tags="dyn")
        # current target
        col_phase = PHASE_COLOR[s["phase"]]
        px, py = self.ctl.calib_points[idx]
        x, y = px - sc.x, py - sc.y
        r = 30
        c.create_oval(x - r, y - r, x + r, y + r, fill=col_phase, outline="", tags="dyn")
        c.create_oval(x - 4, y - 4, x + 4, y + 4, fill="black", outline="", tags="dyn")

        k = len(done) + 1 if len(done) < n else n
        if s["phase"] == "far":
            text = (f"Set 1/2  LONTANO \u2014 punto {k}/{n}: tag dritto alla distanza normale, "
                    f"spostalo verso il cerchio (il cursore e' li') e premi INVIO")
        else:
            text = (f"Set 2/2  VICINO \u2014 punto {k}/{n}: tag dritto e avvicinato come per cliccare "
                    f"(sotto {s['z_far'] - MIN_Z_GAP:.2f} m), spostalo verso il cerchio e premi INVIO")
        # header lives in the free band between the 0.2 and 0.5 rows of calibration points
        ty = int(sc.h * 0.30)
        fs = max(12, min(18, sc.w // 70))
        c.create_text(sc.w // 2, ty, text=text, fill="white", font=("DejaVu Sans", fs, "bold"),
                      width=int(sc.w * 0.8), justify="center", tags="dyn")
        sub = []
        if s["z"] is None:
            sub.append("tag non visibile")
        else:
            sub.append(f"Z = {s['z']:.3f} m")
            if s["tilt"] is not None:
                sub.append(f"inclinazione {s['tilt']:.0f}\u00b0")
        if s["z_far"] is not None:
            sub.append(f"set LONTANO: Z {s['z_far']:.2f} m")
        if s["message"]:
            sub.append(s["message"])
        if len(done) >= MIN_CALIB_POINTS:
            sub.append("Conferma chiude il set")
        c.create_text(sc.w // 2, ty + 2 * fs + 8, text="   \u00b7   ".join(sub), fill="#adb5bd",
                      font=("DejaVu Sans", 13), tags="dyn")
        # progress bar
        bw, bh, by = int(sc.w * 0.5), 14, ty + 3 * fs + 20
        bx = (sc.w - bw) // 2
        c.create_rectangle(bx, by, bx + bw, by + bh, outline="#6c757d", width=1, tags="dyn")
        if done:
            c.create_rectangle(bx, by, bx + int(bw * len(done) / n), by + bh,
                               fill=col_phase, outline="", tags="dyn")
        self.confirm_btn.config(state="normal" if len(done) >= MIN_CALIB_POINTS else "disabled",
                                text=f"Conferma set ({len(done)})")
        # live tag marker: where the tag is in the (mirrored) camera image, spread over the screen
        if s["tag_norm"] is not None:
            tx, ty2 = s["tag_norm"][0] * sc.w, s["tag_norm"][1] * sc.h
            col = "#2ecc71" if s["tag_fresh"] else "#e67e22"
            c.create_line(tx - 18, ty2, tx + 18, ty2, fill=col, width=3, tags="dyn")
            c.create_line(tx, ty2 - 18, tx, ty2 + 18, fill=col, width=3, tags="dyn")
            c.create_oval(tx - 10, ty2 - 10, tx + 10, ty2 + 10, outline=col, width=2, tags="dyn")
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
    actions = {"space": ctl.toggle_freeze, "enter": ctl.enter, "esc": ctl.request_quit,
               "q": ctl.request_quit, "c": ctl.recalibrate}
    return lambda name: actions.get(name, lambda: None)()


def handle_local_key(ctl, k):
    """Same bindings for keys received by the camera window (all commands are idempotent)."""
    if k == 32:
        ctl.toggle_freeze()
    elif k in (13, 10):
        ctl.enter()
    elif k in (27, ord("q"), ord("Q")):
        ctl.request_quit()
    elif k in (ord("c"), ord("C")):
        ctl.recalibrate()


# ----------------------------------------------------------------- main
def parse_args():
    p = argparse.ArgumentParser(description="Mouse control with an AprilTag and the Kinect v1 camera")
    p.add_argument("--tag-size", type=float, default=TAG_SIZE_M, help="tag black-square side in metres")
    p.add_argument("--index", type=int, default=0, help="Kinect device index")
    p.add_argument("--opencv", type=int, default=None, metavar="N", help="use OpenCV camera N instead of the Kinect")
    p.add_argument("--click-near", type=float, default=None,
                   help="override: Z below this (m) = press left button (default: from calibration)")
    p.add_argument("--click-far", type=float, default=None,
                   help="override: Z above this (m) = release (default: from calibration)")
    p.add_argument("--screen-offset", type=float, default=0.0,
                   help="screen plane Z in the camera frame (m): 0 = Kinect on the monitor, "
                        ">0 = screen behind the Kinect")
    p.add_argument("--tilt-gain", type=float, default=1.0,
                   help="how much tilting moves the cursor (0 = position only, 1 = true ray cast)")
    p.add_argument("--tilt-deadzone", type=float, default=10.0,
                   help="degrees of tilt ignored (pose noise of a small tag); only the excess counts")
    p.add_argument("--decimate", type=float, default=1.0, help="detector quad_decimate (2.0 = faster)")
    p.add_argument("--threads", type=int, default=4, help="detector threads")
    p.add_argument("--no-display", action="store_true", help="no camera window, only the Tk HUD")
    p.add_argument("--mouse", choices=("auto", "uinput", "pynput"), default="auto",
                   help="mouse backend: auto = uinput on Wayland, pynput on X11")
    p.add_argument("--keys", choices=("auto", "evdev", "pynput"), default="auto",
                   help="global keyboard backend: auto = evdev if readable, else pynput")
    args = p.parse_args()
    if args.click_near is not None and args.click_far is not None and args.click_far <= args.click_near:
        p.error("--click-far must be greater than --click-near")
    return args


def main():
    args = parse_args()

    # heavy / display-dependent imports after argparse so that --help always works
    import tkinter as tk
    from pupil_apriltags import Detector
    from input_backend import make_keyboard, make_mouse

    detector = Detector(families=TAG_FAMILY, nthreads=args.threads, quad_decimate=args.decimate,
                        quad_sigma=0.0, refine_edges=1, decode_sharpening=0.25)
    cam = open_camera(args)
    print(f"Camera: {cam.name} | tag {TAG_FAMILY} {args.tag_size * 100:.1f} cm | "
          f"screen plane Z={args.screen_offset} m, tilt gain {args.tilt_gain}, dead zone {args.tilt_deadzone} deg")

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
    for w in (root, overlay.win):
        w.bind_all("<space>", lambda e: ctl.toggle_freeze())
        w.bind_all("<Return>", lambda e: ctl.enter())
        w.bind_all("<Escape>", lambda e: ctl.request_quit())
        w.bind_all("q", lambda e: ctl.request_quit())
        w.bind_all("c", lambda e: ctl.recalibrate())

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
                                  args=(ctl, cam, detector, tag_object_points(args.tag_size), not args.no_display))
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
        root.lift()  # HUD above the overlay
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
