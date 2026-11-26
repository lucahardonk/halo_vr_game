"""Control the desktop mouse with an AprilTag seen by the Kinect v1 RGB camera.

- The tag position (tvec X/Y, camera frame) is mapped to screen pixels with a
  homography computed from 5-8 calibration points.
- Bringing the tag close to the camera (Z < --click-near) holds the left button,
  moving it away (Z > --click-far) releases it (hysteresis).
- The HUD shows the tag rotation about its own Z axis, relative to the
  orientation it had when tracking (re)started.

Keys (global, via pynput; also work when the camera window is focused):
    SPACE  freeze (mouse released and free)      ENTER  resume
    c      recalibrate                           q/ESC  quit
"""
import argparse
import atexit
import ctypes as C
import math
import signal
import threading
import time
from ctypes.util import find_library

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
PREVIEW_SIZE = (480, 360)
PREVIEW_WINDOW = "mouse_control - camera"
HUD_W, HUD_H = 200, 170
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

    def __init__(self, args, screen, mouse, button_left):
        self.args = args
        self.screen = screen
        self.mouse = mouse
        self.button_left = button_left
        self.lock = threading.RLock()
        self.quit_flag = False
        self.state = STATE_CALIBRATING
        self.H = None
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
        self.calib_points = [(screen.x + nx * screen.w, screen.y + ny * screen.h) for nx, ny in CALIB_NORM]
        self._reset_calibration()

    # ------------------------------------------------------------ helpers
    def _reset_calibration(self):
        self.calib_registered = {}        # idx -> (tag_x, tag_y)
        self.calib_idx = 0
        self.calib_hold_start = None
        self.calib_armed = False          # tag must be pulled back (Z > far) before a point registers
        self.message = ""

    def _press(self):
        if not self.pressed:
            self.mouse.press(self.button_left)
            self.pressed = True

    def release(self):
        with self.lock:
            if self.pressed:
                try:
                    self.mouse.release(self.button_left)
                finally:
                    self.pressed = False

    def _yaw(self, R):
        if self.R0 is None or R is None:
            return None
        R_rel = self.R0.T @ R
        return math.degrees(math.atan2(R_rel[1, 0], R_rel[0, 0]))

    def _homography_point(self, t):
        p = cv2.perspectiveTransform(np.array([[[t[0], t[1]]]], dtype=np.float64), self.H)[0, 0]
        return p if np.all(np.isfinite(p)) else None

    # ------------------------------------------------------------ camera thread
    def on_detection(self, tvec, R, now):
        t = tvec.ravel()
        with self.lock:
            self.last_seen, self.last_R, self.last_t, self.z = now, R, t, float(t[2])
            if self.state == STATE_CALIBRATING:
                self._calibration_step(t, now)
            elif self.state == STATE_ACTIVE:
                self._active_step(t, R, now)
            self.yaw = self._yaw(R)

    def on_no_detection(self, now):
        with self.lock:
            self.calib_hold_start = None
            if now - self.last_seen > TAG_LOST_RELEASE_S:
                self.release()       # safety: never keep the button down without a tag
                self.calib_armed = True
                self.z = None

    def _calibration_step(self, t, now):
        a = self.args
        z = t[2]
        if not self.calib_armed:
            if z > a.click_far:
                self.calib_armed = True
            self.calib_hold_start = None
            return
        if z < a.click_near:
            if self.calib_hold_start is None:
                self.calib_hold_start = now
            elif now - self.calib_hold_start >= a.click_hold:
                self.calib_registered[self.calib_idx] = (float(t[0]), float(t[1]))
                print(f"[calib] point {self.calib_idx + 1}: tag ({t[0]:+.3f}, {t[1]:+.3f}) m -> "
                      f"screen {tuple(int(v) for v in self.calib_points[self.calib_idx])}")
                self.calib_hold_start = None
                self.calib_armed = False
                if len(self.calib_registered) == len(self.calib_points):
                    self._finish_calibration()
                else:
                    self._advance()
        else:
            self.calib_hold_start = None

    def _advance(self):
        n = len(self.calib_points)
        for k in range(1, n + 1):
            idx = (self.calib_idx + k) % n
            if idx not in self.calib_registered:
                self.calib_idx = idx
                return

    def _finish_calibration(self):
        idx = sorted(self.calib_registered)
        src = np.array([self.calib_registered[i] for i in idx], dtype=np.float64)
        dst = np.array([self.calib_points[i] for i in idx], dtype=np.float64)
        thr = 0.08 * max(self.screen.w, self.screen.h)  # hand pointing is noisy: tolerant RANSAC
        H, mask = cv2.findHomography(src, dst, cv2.RANSAC, thr)
        if H is None or mask is None or int(mask.sum()) < 4:
            H, _ = cv2.findHomography(src, dst, 0)
        if H is None:
            print("[calib] homography failed (points degenerate?), restarting calibration")
            self._reset_calibration()
            self.message = "Calibrazione fallita: ripeti con punti piu' distinti"
            return
        inl = int(mask.sum()) if mask is not None else len(idx)
        err = np.linalg.norm(cv2.perspectiveTransform(src[None], H)[0] - dst, axis=1)
        print(f"[calib] homography from {len(idx)} points ({inl} inliers), "
              f"mean reprojection error {err.mean():.0f} px")
        self.H = H
        self.R0 = self.last_R
        self.offset = np.zeros(2)
        self.rebase_pending = False
        self.click_armed = False
        for f in self.filters:
            f.reset()
        self.message = ""
        self.state = STATE_ACTIVE

    def _active_step(self, t, R, now):
        if self.H is None:
            return
        raw = self._homography_point(t)
        if raw is None:
            return
        if self.rebase_pending:
            # clutch: keep the cursor where it is now, continue from the tag's new position
            cur = np.array(self.mouse.position, dtype=np.float64)
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

        z, a = t[2], self.args
        if not self.click_armed:
            if z > a.click_far:
                self.click_armed = True
        elif z < a.click_near and not self.pressed:
            self._press()
        elif z > a.click_far and self.pressed:
            self.release()

    # ------------------------------------------------------------ commands (any thread)
    def freeze(self):
        with self.lock:
            if self.state in (STATE_ACTIVE, STATE_CALIBRATING):
                self.release()
                self.state = STATE_FROZEN
                print("[state] FROZEN")

    def resume(self):
        with self.lock:
            if self.state != STATE_FROZEN:
                return
            self.release()
            if self.H is None:
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

    def skip_point(self):
        with self.lock:
            if self.state == STATE_CALIBRATING:
                self.calib_hold_start = None
                self._advance()

    def confirm(self):
        with self.lock:
            if self.state == STATE_CALIBRATING and len(self.calib_registered) >= MIN_CALIB_POINTS:
                self._finish_calibration()

    def request_quit(self):
        self.quit_flag = True
        self.release()

    def snapshot(self):
        with self.lock:
            hold = 0.0
            if self.calib_hold_start is not None:
                hold = min(1.0, (time.monotonic() - self.calib_hold_start) / max(1e-3, self.args.click_hold))
            return {
                "state": self.state, "z": self.z, "yaw": self.yaw, "pressed": self.pressed,
                "fps": self.fps, "error": self.error, "message": self.message,
                "calib_idx": self.calib_idx, "calib_done": set(self.calib_registered),
                "calib_armed": self.calib_armed, "hold": hold,
                "click_armed": self.click_armed,
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
                ctl.on_detection(pose[1], pose[2], now)
        if best is None:
            ctl.on_no_detection(now)

        frames += 1
        if now - t_fps >= 0.5:
            ctl.fps, frames, t_fps = frames / (now - t_fps), 0, now

        if show_preview:
            if best is not None:
                det, (rvec, tvec, _) = best
                draw_tag(frame, det, rvec, tvec, K)
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
        root.configure(bg="#1b2028")
        x = screen.x + screen.w - HUD_W - 10
        root.geometry(f"{HUD_W}x{HUD_H}+{x}+{screen.y + 10}")
        mk = lambda font, fg="#e0e0e0": tk.Label(root, text="", font=font, fg=fg, bg="#1b2028", anchor="w")
        self.state = mk(("DejaVu Sans", 13, "bold"))
        self.angle = mk(("DejaVu Sans Mono", 12))
        self.z = mk(("DejaVu Sans Mono", 11))
        self.click = mk(("DejaVu Sans", 11))
        self.fps = mk(("DejaVu Sans Mono", 9), "#8a939e")
        self.msg = tk.Label(root, text="", font=("DejaVu Sans", 8), fg="#f39c12", bg="#1b2028",
                            anchor="w", justify="left", wraplength=HUD_W - 12)
        for w in (self.state, self.angle, self.z, self.click, self.fps, self.msg):
            w.pack(fill="x", padx=6)

    def update(self, s):
        self.state.config(text=s["state"], fg=STATE_COLORS[s["state"]])
        self.angle.config(text="\u21ba --.-\u00b0" if s["yaw"] is None else f"\u21ba {s['yaw']:+6.1f}\u00b0")
        self.z.config(text="Z: --" if s["z"] is None else f"Z: {s['z']:.3f} m")
        self.click.config(text="\u25cf PREMUTO" if s["pressed"] else "\u25cb rilasciato",
                          fg="#e74c3c" if s["pressed"] else "#adb5bd")
        self.fps.config(text=f"camera {s['fps']:4.1f} fps")
        if s["error"]:
            msg = f"Camera: {s['error'][:90]}"
        elif s["state"] == STATE_FROZEN:
            msg = "FROZEN \u2014 sposta il tag, premi INVIO per riprendere"
        elif s["state"] == STATE_ACTIVE and not s["click_armed"]:
            msg = "allontana il tag per abilitare il click"
        else:
            msg = s["message"]
        self.msg.config(text=msg)


class CalibrationOverlay:
    def __init__(self, root, screen, ctl, click_hold):
        import tkinter as tk
        self.ctl, self.screen, self.click_hold = ctl, screen, click_hold
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
        self.confirm_btn = tk.Button(self.win, text="Conferma", command=ctl.confirm, state="disabled",
                                     bg="#198754", fg="white", activebackground="#20c997",
                                     disabledforeground="#9aa", **btn)
        self.skip_btn.place(relx=0.5, rely=0.94, anchor="e", x=-10)
        self.confirm_btn.place(relx=0.5, rely=0.94, anchor="w", x=10)
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
        px, py = self.ctl.calib_points[idx]
        x, y = px - sc.x, py - sc.y
        r = 30
        c.create_oval(x - r, y - r, x + r, y + r, fill="#f1c40f", outline="", tags="dyn")
        c.create_oval(x - 4, y - 4, x + 4, y + 4, fill="black", outline="", tags="dyn")
        if s["hold"] > 0:
            c.create_arc(x - r - 8, y - r - 8, x + r + 8, y + r + 8, start=90, extent=-359.9 * s["hold"],
                         style="arc", outline="#2ecc71", width=6, tags="dyn")

        text = (f"Calibrazione: punto {len(done) + 1 if len(done) < n else n}/{n} \u2014 avvicina il tag "
                f"al punto evidenziato e tienici per {self.click_hold:g}s")
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
            if not s["calib_armed"]:
                sub.append(f"allontana il tag (Z > {self.ctl.args.click_far:.2f} m) per il prossimo punto")
        if s["message"]:
            sub.append(s["message"])
        if len(done) >= MIN_CALIB_POINTS:
            sub.append("puoi premere Conferma")
        c.create_text(sc.w // 2, ty + 2 * fs + 8, text="   \u00b7   ".join(sub), fill="#adb5bd",
                      font=("DejaVu Sans", 13), tags="dyn")
        # progress bar
        bw, bh, by = int(sc.w * 0.5), 14, ty + 3 * fs + 20
        bx = (sc.w - bw) // 2
        c.create_rectangle(bx, by, bx + bw, by + bh, outline="#6c757d", width=1, tags="dyn")
        if done:
            c.create_rectangle(bx, by, bx + int(bw * len(done) / n), by + bh,
                               fill="#2ecc71", outline="", tags="dyn")
        self.confirm_btn.config(state="normal" if len(done) >= MIN_CALIB_POINTS else "disabled",
                                text=f"Conferma ({len(done)})")


# ----------------------------------------------------------------- keyboard
def start_keyboard_listener(ctl):
    from pynput import keyboard

    def on_press(key):
        if key == keyboard.Key.space:
            ctl.freeze()
        elif key == keyboard.Key.enter:
            ctl.resume()
        elif key == keyboard.Key.esc:
            ctl.request_quit()
        else:
            ch = getattr(key, "char", None)
            if ch in ("c", "C"):
                ctl.recalibrate()
            elif ch in ("q", "Q"):
                ctl.request_quit()

    listener = keyboard.Listener(on_press=on_press)
    listener.daemon = True
    listener.start()
    return listener


def handle_local_key(ctl, k):
    """Same bindings for keys received by the camera window (all commands are idempotent)."""
    if k == 32:
        ctl.freeze()
    elif k in (13, 10):
        ctl.resume()
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
    p.add_argument("--click-near", type=float, default=0.40, help="Z below this (m) = press left button")
    p.add_argument("--click-far", type=float, default=0.50, help="Z above this (m) = release (hysteresis)")
    p.add_argument("--click-hold", type=float, default=0.15,
                   help="seconds Z must stay below --click-near to register a calibration point")
    p.add_argument("--decimate", type=float, default=1.0, help="detector quad_decimate (2.0 = faster)")
    p.add_argument("--threads", type=int, default=4, help="detector threads")
    p.add_argument("--no-display", action="store_true", help="no camera window, only the Tk HUD")
    args = p.parse_args()
    if args.click_far <= args.click_near:
        p.error("--click-far must be greater than --click-near")
    return args


def main():
    args = parse_args()

    # heavy / display-dependent imports after argparse so that --help always works
    import tkinter as tk
    from pupil_apriltags import Detector
    from pynput.mouse import Button, Controller as MouseController

    detector = Detector(families=TAG_FAMILY, nthreads=args.threads, quad_decimate=args.decimate,
                        quad_sigma=0.0, refine_edges=1, decode_sharpening=0.25)
    cam = open_camera(args)
    print(f"Camera: {cam.name} | tag {TAG_FAMILY} {args.tag_size * 100:.1f} cm | "
          f"click Z<{args.click_near} m, release Z>{args.click_far} m")

    root = tk.Tk()
    root.title("mouse_control HUD")
    screen = primary_screen(root)
    print(f"Screen: {screen.w}x{screen.h} at ({screen.x},{screen.y})")
    ctl = MouseTagController(args, screen, MouseController(), Button.left)

    closed = {"done": False}

    def shutdown():
        if closed["done"]:
            return
        closed["done"] = True
        ctl.quit_flag = True
        ctl.release()
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
    overlay = CalibrationOverlay(root, screen, ctl, args.click_hold)
    for w in (root, overlay.win):
        w.bind_all("<space>", lambda e: ctl.freeze())
        w.bind_all("<Return>", lambda e: ctl.resume())
        w.bind_all("<Escape>", lambda e: ctl.request_quit())
        w.bind_all("q", lambda e: ctl.request_quit())
        w.bind_all("c", lambda e: ctl.recalibrate())

    try:
        start_keyboard_listener(ctl)
    except Exception as exc:
        print(f"[keyboard] global listener unavailable ({exc}); keys work only on the camera window")

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
