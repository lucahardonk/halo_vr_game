"""Mouse control with an AprilTag seen by the Kinect v1 RGB camera (T1/T2/T3 method).

Math (all 4x4 homogeneous transforms, camera frame: x right, y down, z forward):
  T1      camera -> tag pose, estimated every frame (cv2.aruco + IPPE_SQUARE).
          Tag frame is expressed like the camera frame (x right, y down, z into the
          tag), so a tag facing the camera has rotation ~= identity.
  T2      fixed reference pose: translation (0, 0, D), rotation identity, D = --calib-dist
          (default 0.5 m) = "tag in front of the screen centre, at D, pointing at it".
  ENTER   captures T1_ref and computes  T3 = T2 * inv(T1_ref)  (virtual camera->monitor).
  Every frame (laser-pointer ray cast):
          T_rel  = inv(T1_ref) * T1_current          (tag motion relative to reference)
          T_virt = T3 * T1_current = T2 * T_rel      (tag in the virtual monitor frame,
                                                       monitor = plane z = 0)
          p = T_virt translation, d = T_virt R * (0,0,-1)   (tag normal = pointing ray)
          hit    = p + s d  with  s = -p_z / d_z     (intersection with the screen plane)
          pixel  = screen centre + (hit_x, hit_y) * px_per_m  -> clamped to 3840 x 2160
          Tilting by theta from the centre moves the hit by D * tan(theta): a straight line.

Keys: ENTER = calibrate / recalibrate, SPACE = pause / resume mouse control,
      Ctrl+C = quit (also 'q' / ESC in the preview window).
Keys are read from: the terminal (ENTER), the preview window (ENTER/SPACE/q) and,
if pynput is installed and works (X11), globally from any window.

    python3 mouse_control.py --tag-size 0.05
    python3 mouse_control.py --tag-size 0.065 --calib-dist 0.5 --reach 0.30 --no-preview
"""
import argparse
import ctypes as C
import os
import queue
import re
import select
import subprocess
import sys
import threading
import time
from ctypes.util import find_library

import cv2
import numpy as np

# ----------------------------------------------------------------- settings
SCREEN_W, SCREEN_H = 3840, 2160          # size of the TARGET monitor (overwritten at startup by --monitor)
DEFAULT_CALIB_DIST_M = 0.5               # T2: tag at 50 cm in front of the screen centre
DEFAULT_TAG_SIZE_M = 0.05
POINTING_AXIS = np.array([0.0, 0.0, -1.0])  # tag normal towards camera/screen (tag frame, see FLIP)

# Kinect v1 RGB intrinsics (640x480, datasheet / commonly used values)
CAM_W, CAM_H = 640, 480
FX, FY = 525.0, 525.0
CX, CY = 319.5, 239.5
CAMERA_MATRIX = np.array([[FX, 0.0, CX], [0.0, FY, CY], [0.0, 0.0, 1.0]], dtype=np.float64)
DIST_COEFFS = np.zeros(5, dtype=np.float64)

# OpenCV/IPPE marker frame is x right, y up, z out of the tag (towards the camera).
# Rotating 180 deg about X gives x right, y down, z into the tag: same axes as the
# camera, so a tag facing the camera has R ~= I, consistent with T2 = identity rotation.
FLIP = np.diag([1.0, -1.0, -1.0])

PREVIEW_WINDOW = "mouse_control - Kinect (ENTER calibra, SPACE pausa, q esci)"
TAG_LOST_HOLD_S = 0.3
KEY_DEBOUNCE_S = 0.25


# ----------------------------------------------------------------- Kinect
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
            raise RuntimeError("freenect.sync_get_video non ha restituito frame")
        return cv2.cvtColor(res[0], cv2.COLOR_RGB2BGR)

    def close(self):
        self.fn.sync_stop()


class FreenectCtypes:
    """libfreenect_sync through ctypes (no python bindings needed)."""

    name = "libfreenect_sync (ctypes)"

    def __init__(self, index):
        lib_name = find_library("freenect_sync")
        if not lib_name:
            raise RuntimeError("libfreenect_sync non trovata: sudo apt install libfreenect-dev")
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
            raise RuntimeError(f"freenect_sync_get_video fallita (codice {status})")
        n = CAM_H * CAM_W * 3
        rgb = np.ctypeslib.as_array(C.cast(ptr, C.POINTER(C.c_uint8)), shape=(n,)).reshape(CAM_H, CAM_W, 3)
        return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)  # copies out of the driver buffer

    def close(self):
        self.lib.freenect_sync_stop()


def open_kinect(index):
    """Open the Kinect and grab one frame to make sure a device is really there."""
    errors = []
    for cls in (FreenectPython, FreenectCtypes):
        cam = None
        try:
            cam = cls(index)
            cam.read()
            return cam
        except Exception as exc:  # ImportError, missing lib, no device...
            errors.append(f"{cls.name}: {exc}")
            if cam is not None:
                try:
                    cam.close()
                except Exception:
                    pass
    raise RuntimeError("Kinect v1 non disponibile.\n  " + "\n  ".join(errors) +
                       "\nControlla alimentazione/USB e che nessun altro programma stia usando la Kinect.")


# ----------------------------------------------------------------- AprilTag pose
class TagPoseEstimator:
    def __init__(self, tag_size):
        self.tag_size = tag_size
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        if hasattr(cv2.aruco, "ArucoDetector"):  # OpenCV >= 4.7
            params = cv2.aruco.DetectorParameters()
            params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
            self.detector = cv2.aruco.ArucoDetector(self.dictionary, params)
        else:
            self.detector = None
            self.params = cv2.aruco.DetectorParameters_create()
            self.params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
        h = tag_size / 2.0
        # corner order used by aruco and required by SOLVEPNP_IPPE_SQUARE
        self.obj_pts = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float64)

    def detect(self, gray):
        if self.detector is not None:
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.params)
        return corners, ids

    def pose(self, corner):
        """corner: (1, 4, 2) array. Returns T1 (4x4) camera -> tag, or None.
        IPPE_SQUARE, solution with the lowest reprojection error."""
        ok, rvec, tvec = cv2.solvePnP(self.obj_pts, corner.reshape(4, 2).astype(np.float64),
                                      CAMERA_MATRIX, DIST_COEFFS, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            return None
        R, _ = cv2.Rodrigues(rvec)
        T = np.eye(4)
        T[:3, :3] = R @ FLIP
        T[:3, 3] = np.ravel(tvec)
        return T


def inv_T(T):
    """Inverse of a rigid transform (R^T, -R^T t)."""
    Ti = np.eye(4)
    R = T[:3, :3]
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ T[:3, 3]
    return Ti


# ----------------------------------------------------------------- screen mapping
class ScreenMapper:
    def __init__(self, calib_dist, reach, mirror, invert_y, smooth):
        self.T2 = np.eye(4)
        self.T2[2, 3] = calib_dist
        # px_per_m: a hit 'reach' metres from the centre on the screen plane = horizontal screen edge
        self.px_per_m = (SCREEN_W / 2.0) / reach
        self.sx = -1.0 if mirror else 1.0   # camera looks at the user: image right = user's left
        self.sy = -1.0 if invert_y else 1.0
        self.smooth = smooth
        self.T1_ref = None
        self.T3 = None
        self.filtered = None

    def calibrate(self, T1):
        self.T1_ref = T1.copy()
        self.T3 = self.T2 @ inv_T(self.T1_ref)
        # at calibration T_rel = I -> the ray hits (0, 0) = screen centre: start the filter there
        self.filtered = np.array([SCREEN_W / 2.0, SCREEN_H / 2.0])
        return SCREEN_W // 2, SCREEN_H // 2

    def map(self, T1):
        """Ray cast of the tag normal onto the screen plane (z = 0 of the monitor frame).
        Returns (px, py, T_rel, hit) or None if the tag does not point towards the screen."""
        T_rel = inv_T(self.T1_ref) @ T1
        T_virt = self.T3 @ T1            # == T2 @ T_rel
        p = T_virt[:3, 3]
        d = T_virt[:3, :3] @ POINTING_AXIS
        if p[2] <= 0.0 or d[2] > -0.1:   # behind the screen, or ray (almost) parallel / away (> ~84 deg)
            return None
        s = -p[2] / d[2]
        hit = p + s * d                  # metres on the screen plane, origin = screen centre
        px = SCREEN_W / 2.0 + self.sx * self.px_per_m * hit[0]
        py = SCREEN_H / 2.0 + self.sy * self.px_per_m * hit[1]
        if self.smooth <= 0:
            self.filtered = np.array([px, py])
        else:
            self.filtered = self.smooth * self.filtered + (1.0 - self.smooth) * np.array([px, py])
        fx = int(round(min(max(self.filtered[0], 0), SCREEN_W - 1)))
        fy = int(round(min(max(self.filtered[1], 0), SCREEN_H - 1)))
        return fx, fy, T_rel, hit


# ----------------------------------------------------------------- mouse output
# On Wayland pyautogui/pynput only reach XWayland windows. A kernel virtual
# absolute pointer (/dev/uinput, like the previous working version) moves the
# real cursor everywhere: the compositor maps ABS 0..32767 to the whole screen.
ABS_MAX = 32767
UINPUT_NAME = "halo-apriltag-pointer"

# Multi-monitor: the compositor spreads ABS 0..32767 over the WHOLE desktop
# (bounding box of all monitors). The cursor must live only on one monitor, so
# monitor-local pixels are shifted by the monitor offset and divided by the
# desktop size. Geometry comes from `xrandr --listmonitors` (works under
# XWayland too) or from the --monitor / --desktop flags.
MONITOR_RECT = (0, 0, SCREEN_W, SCREEN_H)  # x, y, w, h of the target monitor inside the desktop
DESKTOP_RECT = (0, 0, SCREEN_W, SCREEN_H)  # x, y, w, h of the whole desktop

_XRANDR_RE = re.compile(r"^\s*(\d+):\s+\+?(\*?)(\S+)\s+(\d+)/\d+x(\d+)/\d+\+(-?\d+)\+(-?\d+)")


def list_monitors():
    """[(index, name, primary, x, y, w, h)] from xrandr, or [] if unavailable."""
    try:
        out = subprocess.run(["xrandr", "--listmonitors"], capture_output=True, text=True, timeout=3).stdout
    except (OSError, subprocess.SubprocessError):
        return []
    mons = []
    for line in out.splitlines():
        m = _XRANDR_RE.match(line)
        if m:
            idx, star, name, w, h, x, y = m.groups()
            mons.append((int(idx), name, star == "*", int(x), int(y), int(w), int(h)))
    return mons


def _parse_rect(text, n, what):
    try:
        vals = [int(v) for v in re.split(r"[,x+]", text.strip().lower()) if v != ""]
    except ValueError:
        vals = []
    if len(vals) != n or any(v <= 0 for v in vals[-2:]):
        raise ValueError(f"{what} non valido: '{text}'")
    return tuple(vals)


def resolve_geometry(monitor_arg, desktop_arg):
    """Returns (monitor_rect, desktop_rect, description_lines)."""
    mons = list_monitors()
    info = [f"  xrandr {i}: {name}{' (primario)' if pr else ''} {w}x{h}+{x}+{y}"
            for i, name, pr, x, y, w, h in mons] or ["  xrandr non disponibile: uso i valori dei flag/default"]

    # target monitor
    if "," in monitor_arg:
        mon = _parse_rect(monitor_arg, 4, "--monitor")
    elif mons:
        if monitor_arg == "primary":
            pick = next((m for m in mons if m[2]), mons[0])
        else:
            pick = next((m for m in mons if str(m[0]) == monitor_arg or m[1] == monitor_arg), None)
            if pick is None:
                raise ValueError(f"--monitor '{monitor_arg}' non trovato in xrandr")
        mon = pick[3:7]
    else:
        mon = (0, 0, 3840, 2160)

    # whole desktop
    if desktop_arg != "auto":
        dw, dh = _parse_rect(desktop_arg, 2, "--desktop")
        desk = (0, 0, dw, dh)
    elif mons:
        x0 = min(m[3] for m in mons); y0 = min(m[4] for m in mons)
        x1 = max(m[3] + m[5] for m in mons); y1 = max(m[4] + m[6] for m in mons)
        desk = (x0, y0, x1 - x0, y1 - y0)
    else:
        desk = (0, 0, mon[0] + mon[2], mon[1] + mon[3])
    if (mon[0] < desk[0] or mon[1] < desk[1] or
            mon[0] + mon[2] > desk[0] + desk[2] or mon[1] + mon[3] > desk[1] + desk[3]):
        raise ValueError(f"il monitor {mon} esce dal desktop {desk}: controlla --monitor/--desktop")
    return mon, desk, info


def monitor_to_desktop(x, y):
    """Monitor-local pixel -> global desktop pixel."""
    return MONITOR_RECT[0] + x, MONITOR_RECT[1] + y


def is_wayland():
    return os.environ.get("XDG_SESSION_TYPE", "").lower() == "wayland" or bool(os.environ.get("WAYLAND_DISPLAY"))


def uinput_problem():
    """None if /dev/uinput is usable, else a readable reason."""
    try:
        import evdev  # noqa: F401
    except ImportError:
        return "modulo python 'evdev' mancante (pip install evdev)"
    if not os.path.exists("/dev/uinput"):
        return "/dev/uinput mancante (sudo modprobe uinput)"
    if not os.access("/dev/uinput", os.W_OK):
        return "nessun permesso di scrittura su /dev/uinput (regola udev + gruppo 'input', poi logout/login)"
    return None


class UInputMouse:
    name = "uinput (puntatore virtuale kernel, funziona su Wayland)"

    def __init__(self):
        from evdev import AbsInfo, UInput, ecodes as e
        self.e = e
        caps = {
            e.EV_KEY: [e.BTN_LEFT, e.BTN_RIGHT, e.BTN_MIDDLE],
            e.EV_ABS: [(e.ABS_X, AbsInfo(0, 0, ABS_MAX, 0, 0, 0)),
                       (e.ABS_Y, AbsInfo(0, 0, ABS_MAX, 0, 0, 0))],
        }
        self.ui = UInput(caps, name=UINPUT_NAME)

    def move(self, x, y):
        gx, gy = monitor_to_desktop(x, y)
        ax = int(round((gx - DESKTOP_RECT[0]) / (DESKTOP_RECT[2] - 1) * ABS_MAX))
        ay = int(round((gy - DESKTOP_RECT[1]) / (DESKTOP_RECT[3] - 1) * ABS_MAX))
        self.ui.write(self.e.EV_ABS, self.e.ABS_X, min(max(ax, 0), ABS_MAX))
        self.ui.write(self.e.EV_ABS, self.e.ABS_Y, min(max(ay, 0), ABS_MAX))
        self.ui.syn()

    def close(self):
        try:
            self.ui.close()
        except Exception:
            pass


class PyAutoGuiMouse:
    name = "pyautogui (X11; su Wayland solo finestre XWayland)"

    def __init__(self):
        import pyautogui
        pyautogui.FAILSAFE = False
        pyautogui.PAUSE = 0
        self.pg = pyautogui

    def move(self, x, y):
        gx, gy = monitor_to_desktop(x, y)
        self.pg.moveTo(gx, gy, _pause=False)

    def close(self):
        pass


def make_mouse(backend):
    """backend: auto | uinput | pyautogui."""
    if backend in ("auto", "uinput"):
        problem = uinput_problem()
        if problem is None:
            return UInputMouse()
        if backend == "uinput" or is_wayland():
            print(f"[mouse] uinput non utilizzabile: {problem}")
        if backend == "uinput":
            raise RuntimeError(problem)
        if is_wayland():
            print("[mouse] fallback pyautogui: su Wayland il cursore si muove SOLO sopra finestre XWayland")
    return PyAutoGuiMouse()


# ----------------------------------------------------------------- keyboard
class EvdevKeyboard:
    """Global ENTER/SPACE from the real keyboards in /dev/input (no grab, works on Wayland)."""

    def __init__(self, keys):
        from evdev import InputDevice, ecodes as e, list_devices
        self.e = e
        self.keys = keys
        self.devices = []
        for path in list_devices():
            try:
                dev = InputDevice(path)
            except OSError:
                continue
            caps = dev.capabilities().get(e.EV_KEY, [])
            if dev.name != UINPUT_NAME and e.KEY_SPACE in caps and e.KEY_ENTER in caps:
                self.devices.append(dev)
            else:
                dev.close()
        if not self.devices:
            raise RuntimeError("nessuna tastiera leggibile in /dev/input (utente nel gruppo 'input'?)")
        self.map = {e.KEY_SPACE: "space", e.KEY_ENTER: "enter", e.KEY_KPENTER: "enter"}
        self.running = True
        threading.Thread(target=self._loop, name="evdev-keys", daemon=True).start()

    def _loop(self):
        fds = {dev.fd: dev for dev in self.devices}
        while self.running and fds:
            r, _, _ = select.select(list(fds), [], [], 0.5)
            for fd in r:
                try:
                    for ev in fds[fd].read():
                        if ev.type == self.e.EV_KEY and ev.value == 1 and ev.code in self.map:
                            self.keys.put(self.map[ev.code])
                except OSError:
                    fds.pop(fd, None)

    def stop(self):
        self.running = False


def start_terminal_reader(keys):
    """Every line typed in the terminal (ENTER) becomes an 'enter' event."""
    def loop():
        for _ in sys.stdin:
            keys.put("enter")
    if sys.stdin is not None and sys.stdin.isatty():
        threading.Thread(target=loop, name="stdin-enter", daemon=True).start()


class _PynputListener:
    name = "pynput (X11)"


def start_global_listener(keys):
    """Global ENTER/SPACE: evdev (works on Wayland) first, then pynput (X11). None if unavailable."""
    try:
        kb = EvdevKeyboard(keys)
        kb.name = "evdev /dev/input (globale, anche Wayland)"
        return kb
    except Exception as exc:
        print(f"[tasti] evdev non disponibile ({exc})")
    try:
        from pynput import keyboard
    except Exception:
        return None

    def on_press(key):
        if key == keyboard.Key.space:
            keys.put("space")
        elif key == keyboard.Key.enter:
            keys.put("enter")
    try:
        listener = keyboard.Listener(on_press=on_press)
        listener.daemon = True
        listener.start()
        listener.name = _PynputListener.name
        return listener
    except Exception:
        return None


# ----------------------------------------------------------------- drawing
def draw_preview(frame, corners, ids, status, info):
    if ids is not None and len(ids):
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
    lines = [status] + info
    for i, text in enumerate(lines):
        y = 22 + i * 22
        cv2.putText(frame, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(frame, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)


# ----------------------------------------------------------------- main
def parse_args():
    p = argparse.ArgumentParser(description="Controllo mouse con AprilTag + Kinect v1 (metodo T1/T2/T3)")
    p.add_argument("--tag-size", type=float, default=DEFAULT_TAG_SIZE_M,
                   help="lato del quadrato nero del tag in metri (default 0.05)")
    p.add_argument("--tag-id", type=int, default=None, help="usa solo questo ID (default: primo tag visto)")
    p.add_argument("--index", type=int, default=0, help="indice device Kinect")
    p.add_argument("--calib-dist", type=float, default=DEFAULT_CALIB_DIST_M,
                   help="distanza tag-schermo (m) al momento della calibrazione con INVIO (default 0.5)")
    p.add_argument("--reach", type=float, default=0.30,
                   help="metri sul piano dello schermo dal centro al bordo orizzontale "
                        "(= mezza larghezza fisica del monitor per un puntatore 'laser' esatto; default 0.30)")
    p.add_argument("--no-mirror", action="store_true",
                   help="non specchiare l'asse X (usa se il cursore va al contrario in orizzontale)")
    p.add_argument("--invert-y", action="store_true", help="inverti l'asse Y")
    p.add_argument("--smooth", type=float, default=0.5, help="filtro esponenziale 0..0.95 (0 = off)")
    p.add_argument("--no-preview", action="store_true", help="non mostrare la finestra della camera")
    p.add_argument("--monitor", default="primary",
                   help="monitor su cui resta il cursore: 'primary' (default), indice/nome di "
                        "`xrandr --listmonitors`, oppure X,Y,W,H in pixel del desktop")
    p.add_argument("--desktop", default="auto",
                   help="dimensione totale del desktop WxH (tutti i monitor); default auto da xrandr")
    p.add_argument("--mouse", choices=("auto", "uinput", "pyautogui"), default="auto",
                   help="backend mouse: auto = uinput se disponibile (Wayland ok), altrimenti pyautogui")
    args = p.parse_args()
    if args.tag_size <= 0:
        p.error("--tag-size deve essere > 0")
    if args.reach <= 0:
        p.error("--reach deve essere > 0")
    if args.calib_dist <= 0:
        p.error("--calib-dist deve essere > 0")
    args.smooth = min(max(args.smooth, 0.0), 0.95)
    return args


def main():
    global SCREEN_W, SCREEN_H, MONITOR_RECT, DESKTOP_RECT
    args = parse_args()

    try:
        MONITOR_RECT, DESKTOP_RECT, geo_info = resolve_geometry(args.monitor, args.desktop)
    except ValueError as exc:
        print(f"ERRORE geometria monitor: {exc}", file=sys.stderr)
        return 1
    SCREEN_W, SCREEN_H = MONITOR_RECT[2], MONITOR_RECT[3]
    print("Monitor:")
    for line in geo_info:
        print(line)
    mx, my, mw, mh = MONITOR_RECT
    print(f"  -> cursore limitato a {mw}x{mh}+{mx}+{my} dentro desktop "
          f"{DESKTOP_RECT[2]}x{DESKTOP_RECT[3]}+{DESKTOP_RECT[0]}+{DESKTOP_RECT[1]}")

    try:
        mouse = make_mouse(args.mouse)
    except Exception as exc:
        print(f"ERRORE mouse: {exc}", file=sys.stderr)
        return 1

    try:
        cam = open_kinect(args.index)
    except RuntimeError as exc:
        print(f"ERRORE: {exc}", file=sys.stderr)
        mouse.close()
        return 1

    estimator = TagPoseEstimator(args.tag_size)
    mapper = ScreenMapper(args.calib_dist, args.reach, not args.no_mirror, args.invert_y, args.smooth)
    keys = queue.Queue()
    start_terminal_reader(keys)
    listener = start_global_listener(keys)

    print(f"Kinect: {cam.name} | tag36h11 {args.tag_size * 100:.1f} cm | schermo {SCREEN_W}x{SCREEN_H}")
    print(f"Mouse: {mouse.name}")
    print("Tastiera globale: " + (listener.name if listener else "non disponibile (usa terminale / finestra preview)"))
    print(f"Posiziona il tag al centro dello schermo a ~{args.calib_dist * 100:.0f} cm, rivolto verso lo schermo. "
          "Premi INVIO per calibrare.")

    preview = not args.no_preview
    if preview:
        cv2.namedWindow(PREVIEW_WINDOW, cv2.WINDOW_AUTOSIZE)

    active = False          # mouse control on/off (SPACE)
    last_T1, last_seen = None, 0.0
    last_print = 0.0
    last_key_t = {}
    try:
        while True:
            frame = cam.read()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids = estimator.detect(gray)
            now = time.monotonic()

            T1 = None
            if ids is not None:
                for corner, tag_id in zip(corners, ids.ravel()):
                    if args.tag_id is None or int(tag_id) == args.tag_id:
                        T1 = estimator.pose(corner)
                        if T1 is not None:
                            break
            if T1 is not None:
                last_T1, last_seen = T1, now

            # ---- keys (preview window first, then queued terminal/global events)
            if preview:
                k = cv2.waitKey(1) & 0xFF
                if k in (13, 10):
                    keys.put("enter")
                elif k == 32:
                    keys.put("space")
                elif k in (ord("q"), 27):
                    break
            while not keys.empty():
                key = keys.get_nowait()
                # the same physical press can arrive from several sources
                # (evdev + terminal + preview window): keep only the first one
                if now - last_key_t.get(key, -1.0) < KEY_DEBOUNCE_S:
                    continue
                last_key_t[key] = now
                if key == "enter":
                    if last_T1 is not None and now - last_seen < TAG_LOST_HOLD_S:
                        cx_px, cy_px = mapper.calibrate(last_T1)
                        active = True
                        mouse.move(cx_px, cy_px)          # cursor starts from the screen centre
                        t = last_T1[:3, 3]
                        print(f"T1_ref: t = [{t[0]:+.3f} {t[1]:+.3f} {t[2]:+.3f}] m  "
                              f"(T2 = 0, 0, {args.calib_dist:.2f}) -> cursore al centro ({cx_px}, {cy_px})")
                        print("Calibrazione OK — controllo mouse attivo. [SPAZIO] per pausa, Ctrl+C per uscire.")
                    else:
                        print("Tag non visibile: impossibile calibrare. Inquadra il tag e premi di nuovo INVIO.")
                elif key == "space" and mapper.T3 is not None:
                    active = not active
                    print("Controllo mouse " + ("RIPRESO" if active else "IN PAUSA (cursore libero)"))

            # ---- mouse
            info = []
            if mapper.T3 is None:
                status = f"NON CALIBRATO - tag al centro a ~{args.calib_dist * 100:.0f} cm, premi INVIO"
            else:
                status = "ATTIVO" if active else "PAUSA (SPACE per riprendere)"
                if T1 is not None:
                    res = mapper.map(T1)
                    if res is not None:
                        px, py, T_rel, hit = res
                        if active:
                            mouse.move(px, py)
                        d = T_rel[:3, 3]
                        info = [f"dT = [{d[0]:+.3f} {d[1]:+.3f} {d[2]:+.3f}] m",
                                f"hit schermo = [{hit[0]:+.3f} {hit[1]:+.3f}] m",
                                f"cursore = ({px}, {py})"]
                        if now - last_print > 0.5:
                            last_print = now
                            print(f"dT=[{d[0]:+.3f} {d[1]:+.3f} {d[2]:+.3f}] m  hit=[{hit[0]:+.3f} {hit[1]:+.3f}] m  "
                                  f"cursore=({px:4d},{py:4d}){'' if active else '  [pausa]'}")
                    else:
                        info = ["tag non punta verso lo schermo"]
                else:
                    info = ["tag non visibile"]

            if preview:
                draw_preview(frame, corners, ids, status, info)
                cv2.imshow(PREVIEW_WINDOW, frame)
                if cv2.getWindowProperty(PREVIEW_WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                    break
    except KeyboardInterrupt:
        print("\nCtrl+C: uscita.")
    finally:
        if listener is not None:
            listener.stop()
        mouse.close()
        try:
            cam.close()
        except Exception:
            pass
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
