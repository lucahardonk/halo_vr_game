"""Mouse control with an AprilTag seen by the Kinect v1 RGB camera (T1/T2/T3 method).

Math (all 4x4 homogeneous transforms, camera frame: x right, y down, z forward):
  T1      camera -> tag pose, estimated every frame (cv2.aruco + IPPE_SQUARE).
          Tag frame is expressed like the camera frame (x right, y down, z into the
          tag), so a tag facing the camera has rotation ~= identity.
  T2      fixed reference pose: translation (0, 0, REF_DIST=0.6 m), rotation identity
          = "centre of the screen at 60 cm from the camera".
  ENTER   captures T1_ref and computes  T3 = T2 * inv(T1_ref)  (virtual camera->monitor).
  Every frame:
          T_rel  = inv(T1_ref) * T1_current          (tag motion relative to reference)
          T_virt = T3 * T1_current = T2 * T_rel      (tag in the virtual monitor frame)
          u, v   = x / z, y / z of T_virt            (projection, normalised by depth)
          pixel  = screen centre + gain * (u, v)      -> clamped to 3840 x 2160

Keys: ENTER = calibrate / recalibrate, SPACE = pause / resume mouse control,
      Ctrl+C = quit (also 'q' / ESC in the preview window).
Keys are read from: the terminal (ENTER), the preview window (ENTER/SPACE/q) and,
if pynput is installed and works (X11), globally from any window.

    python3 mouse_control.py --tag-size 0.05
    python3 mouse_control.py --tag-size 0.065 --reach 0.15 --no-preview
"""
import argparse
import ctypes as C
import queue
import sys
import threading
import time
from ctypes.util import find_library

import cv2
import numpy as np

# ----------------------------------------------------------------- settings
SCREEN_W, SCREEN_H = 3840, 2160          # 4K 16:9, hardcoded for now
REF_DIST_M = 0.6                         # T2: screen centre at 60 cm from the camera
DEFAULT_TAG_SIZE_M = 0.05

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

T2 = np.eye(4)
T2[2, 3] = REF_DIST_M

PREVIEW_WINDOW = "mouse_control - Kinect (ENTER calibra, SPACE pausa, q esci)"
TAG_LOST_HOLD_S = 0.3


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
        """corner: (1, 4, 2) array. Returns T1 (4x4) camera -> tag, or None."""
        rvec = tvec = None
        if hasattr(cv2.aruco, "estimatePoseSingleMarkers"):  # removed in OpenCV >= 4.7
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                [corner], self.tag_size, CAMERA_MATRIX, DIST_COEFFS)
            rvec, tvec = rvecs[0].reshape(3, 1), tvecs[0].reshape(3, 1)
        else:
            ok, rvec, tvec = cv2.solvePnP(self.obj_pts, corner.reshape(4, 2).astype(np.float64),
                                          CAMERA_MATRIX, DIST_COEFFS, flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not ok:
                return None
        R, _ = cv2.Rodrigues(rvec)
        T = np.eye(4)
        T[:3, :3] = R @ FLIP
        T[:3, 3] = tvec.ravel()
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
    def __init__(self, reach, mirror, invert_y, smooth):
        # gain: lateral displacement 'reach' (metres) at REF_DIST moves from centre to screen edge
        self.gain = (SCREEN_W / 2.0) / (reach / REF_DIST_M)
        self.sx = -1.0 if mirror else 1.0   # camera looks at the user: image right = user's left
        self.sy = -1.0 if invert_y else 1.0
        self.smooth = smooth
        self.T1_ref = None
        self.T3 = None
        self.filtered = None

    def calibrate(self, T1):
        self.T1_ref = T1.copy()
        self.T3 = T2 @ inv_T(self.T1_ref)
        self.filtered = None

    def map(self, T1):
        """Returns (px, py, T_rel, T_virt) or None if behind the virtual camera."""
        T_rel = inv_T(self.T1_ref) @ T1
        T_virt = self.T3 @ T1            # == T2 @ T_rel
        x, y, z = T_virt[:3, 3]
        if z < 0.05:
            return None
        u, v = x / z, y / z
        px = SCREEN_W / 2.0 + self.sx * self.gain * u
        py = SCREEN_H / 2.0 + self.sy * self.gain * v
        if self.filtered is None or self.smooth <= 0:
            self.filtered = np.array([px, py])
        else:
            self.filtered = self.smooth * self.filtered + (1.0 - self.smooth) * np.array([px, py])
        fx = int(round(min(max(self.filtered[0], 0), SCREEN_W - 1)))
        fy = int(round(min(max(self.filtered[1], 0), SCREEN_H - 1)))
        return fx, fy, T_rel, T_virt


# ----------------------------------------------------------------- keyboard
def start_terminal_reader(keys):
    """Every line typed in the terminal (ENTER) becomes an 'enter' event."""
    def loop():
        for _ in sys.stdin:
            keys.put("enter")
    if sys.stdin is not None and sys.stdin.isatty():
        threading.Thread(target=loop, name="stdin-enter", daemon=True).start()


def start_global_listener(keys):
    """Global ENTER/SPACE via pynput (X11). Optional: returns None if unavailable."""
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
    p.add_argument("--reach", type=float, default=0.20,
                   help="spostamento laterale (m, a 60 cm) per portare il cursore dal centro al bordo")
    p.add_argument("--no-mirror", action="store_true",
                   help="non specchiare l'asse X (usa se il cursore va al contrario in orizzontale)")
    p.add_argument("--invert-y", action="store_true", help="inverti l'asse Y")
    p.add_argument("--smooth", type=float, default=0.5, help="filtro esponenziale 0..0.95 (0 = off)")
    p.add_argument("--no-preview", action="store_true", help="non mostrare la finestra della camera")
    args = p.parse_args()
    if args.tag_size <= 0:
        p.error("--tag-size deve essere > 0")
    if args.reach <= 0:
        p.error("--reach deve essere > 0")
    args.smooth = min(max(args.smooth, 0.0), 0.95)
    return args


def main():
    args = parse_args()

    try:
        import pyautogui
    except Exception as exc:
        print(f"ERRORE: pyautogui non disponibile ({exc}). pip install pyautogui", file=sys.stderr)
        return 1
    pyautogui.FAILSAFE = False
    pyautogui.PAUSE = 0

    try:
        cam = open_kinect(args.index)
    except RuntimeError as exc:
        print(f"ERRORE: {exc}", file=sys.stderr)
        return 1

    estimator = TagPoseEstimator(args.tag_size)
    mapper = ScreenMapper(args.reach, not args.no_mirror, args.invert_y, args.smooth)
    keys = queue.Queue()
    start_terminal_reader(keys)
    listener = start_global_listener(keys)

    print(f"Kinect: {cam.name} | tag36h11 {args.tag_size * 100:.1f} cm | schermo {SCREEN_W}x{SCREEN_H}")
    print("Tastiera globale: " + ("pynput" if listener else "non disponibile (usa terminale / finestra preview)"))
    print("Posiziona il tag al centro dello schermo a ~60 cm. Premi INVIO per calibrare.")

    preview = not args.no_preview
    if preview:
        cv2.namedWindow(PREVIEW_WINDOW, cv2.WINDOW_AUTOSIZE)

    active = False          # mouse control on/off (SPACE)
    last_T1, last_seen = None, 0.0
    last_print = 0.0
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
                if key == "enter":
                    if last_T1 is not None and now - last_seen < TAG_LOST_HOLD_S:
                        mapper.calibrate(last_T1)
                        active = True
                        t = last_T1[:3, 3]
                        print(f"T1_ref: t = [{t[0]:+.3f} {t[1]:+.3f} {t[2]:+.3f}] m")
                        print("Calibrazione OK — controllo mouse attivo. [SPAZIO] per pausa, Ctrl+C per uscire.")
                    else:
                        print("Tag non visibile: impossibile calibrare. Inquadra il tag e premi di nuovo INVIO.")
                elif key == "space" and mapper.T3 is not None:
                    active = not active
                    print("Controllo mouse " + ("RIPRESO" if active else "IN PAUSA (cursore libero)"))

            # ---- mouse
            info = []
            if mapper.T3 is None:
                status = "NON CALIBRATO - tag al centro a ~60 cm, premi INVIO"
            else:
                status = "ATTIVO" if active else "PAUSA (SPACE per riprendere)"
                if T1 is not None:
                    res = mapper.map(T1)
                    if res is not None:
                        px, py, T_rel, T_virt = res
                        if active:
                            pyautogui.moveTo(px, py, _pause=False)
                        d = T_rel[:3, 3]
                        info = [f"dT = [{d[0]:+.3f} {d[1]:+.3f} {d[2]:+.3f}] m",
                                f"cursore = ({px}, {py})"]
                        if now - last_print > 0.5:
                            last_print = now
                            print(f"dT=[{d[0]:+.3f} {d[1]:+.3f} {d[2]:+.3f}] m  z_virt={T_virt[2, 3]:.3f} m  "
                                  f"cursore=({px:4d},{py:4d}){'' if active else '  [pausa]'}")
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
        try:
            cam.close()
        except Exception:
            pass
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
