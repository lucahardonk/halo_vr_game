"""Sorgente posa AprilTag dalla Kinect v1 (RGB), adattata da mouse_control/mouse_control.py.

Fornisce una CameraSource con read() -> (frame_bgr, pose, now) dove pose e':
    dict(position=[x,y,z] m, quaternion=[w,x,y,z], R=3x3)  oppure None se nessun tag.
La posa e' quella del tag nel frame camera (corpo->camera), coerente con la convenzione
corpo->mondo dei quaternioni (mondo = frame camera).

cv2 viene importato pigramente, cosi' la modalita' sintetica gira senza OpenCV/Kinect.
"""
import math
import time
from ctypes.util import find_library

import numpy as np

import quaternion as Q

# Kinect v1 RGB: intrinseci datasheet (640x480), come in mouse_control.
CAM_W, CAM_H = 640, 480
FX, FY = 525.0, 525.0
CX, CY = 319.5, 239.5

# Frame marker OpenCV/IPPE: x destra, y su, z fuori dal tag. Ruotando 180 deg attorno a X
# si ottiene x destra, y giu', z dentro il tag: stessi assi della camera (tag frontale -> R~=I).
FLIP = np.diag([1.0, -1.0, -1.0])


def _camera_matrix():
    return np.array([[FX, 0.0, CX], [0.0, FY, CY], [0.0, 0.0, 1.0]], dtype=np.float64)


# ----------------------------------------------------------------- Kinect driver
class _FreenectPython:
    name = "freenect (python)"

    def __init__(self, index):
        import freenect
        import cv2
        self.fn = freenect
        self.cv2 = cv2
        self.index = index

    def read(self):
        res = self.fn.sync_get_video(self.index, self.fn.VIDEO_RGB)
        if res is None:
            raise RuntimeError("freenect.sync_get_video non ha restituito frame")
        return self.cv2.cvtColor(res[0], self.cv2.COLOR_RGB2BGR)

    def close(self):
        self.fn.sync_stop()


class _FreenectCtypes:
    name = "libfreenect_sync (ctypes)"

    def __init__(self, index):
        import ctypes as C
        import cv2
        self.C = C
        self.cv2 = cv2
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
        C = self.C
        ptr, ts = C.c_void_p(), C.c_uint32()
        status = self.lib.freenect_sync_get_video(C.byref(ptr), C.byref(ts), self.index, 0)
        if status != 0 or not ptr.value:
            raise RuntimeError(f"freenect_sync_get_video fallita (codice {status})")
        n = CAM_H * CAM_W * 3
        rgb = np.ctypeslib.as_array(C.cast(ptr, C.POINTER(C.c_uint8)), shape=(n,)).reshape(CAM_H, CAM_W, 3)
        return self.cv2.cvtColor(rgb, self.cv2.COLOR_RGB2BGR)

    def close(self):
        self.lib.freenect_sync_stop()


def _open_kinect(index):
    errors = []
    for cls in (_FreenectPython, _FreenectCtypes):
        cam = None
        try:
            cam = cls(index)
            cam.read()
            return cam
        except Exception as exc:
            errors.append(f"{cls.name}: {exc}")
            if cam is not None:
                try:
                    cam.close()
                except Exception:
                    pass
    raise RuntimeError("Kinect v1 non disponibile.\n  " + "\n  ".join(errors) +
                       "\nControlla alimentazione/USB e che nessun altro programma usi la Kinect.")


# ----------------------------------------------------------------- stima posa tag
class _TagPoseEstimator:
    def __init__(self, tag_size, tag_id=None):
        import cv2
        self.cv2 = cv2
        self.tag_size = tag_size
        self.tag_id = tag_id
        self.cam_mtx = _camera_matrix()
        self.dist = np.zeros(5, dtype=np.float64)
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        if hasattr(cv2.aruco, "ArucoDetector"):
            params = cv2.aruco.DetectorParameters()
            params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
            self.detector = cv2.aruco.ArucoDetector(self.dictionary, params)
        else:
            self.detector = None
            self.params = cv2.aruco.DetectorParameters_create()
            self.params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
        h = tag_size / 2.0
        self.obj_pts = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float64)

    def detect(self, gray):
        if self.detector is not None:
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            corners, ids, _ = self.cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.params)
        return corners, ids

    def pose_of(self, corner):
        cv2 = self.cv2
        ok, rvec, tvec = cv2.solvePnP(self.obj_pts, corner.reshape(4, 2).astype(np.float64),
                                      self.cam_mtx, self.dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            return None
        R, _ = cv2.Rodrigues(rvec)
        R = R @ FLIP
        return dict(position=np.ravel(tvec).tolist(),
                    quaternion=Q.quat_from_matrix(R).tolist(), R=R)

    def first_pose(self, corners, ids):
        if ids is None:
            return None, corners, ids
        for corner, tag_id in zip(corners, ids.ravel()):
            if self.tag_id is None or int(tag_id) == self.tag_id:
                p = self.pose_of(corner)
                if p is not None:
                    return p, corners, ids
        return None, corners, ids


class CameraSource:
    """Kinect reale + stima AprilTag. read() -> (frame_bgr, pose|None, now)."""

    def __init__(self, tag_size, tag_id=None, index=0, clock=time.monotonic):
        self._cam = _open_kinect(index)
        self.name = self._cam.name
        self._est = _TagPoseEstimator(tag_size, tag_id)
        self.clock = clock
        import cv2
        self.cv2 = cv2

    def read(self):
        frame = self._cam.read()
        gray = self.cv2.cvtColor(frame, self.cv2.COLOR_BGR2GRAY)
        corners, ids = self._est.detect(gray)
        pose, corners, ids = self._est.first_pose(corners, ids)
        return frame, pose, self.clock(), corners, ids

    def close(self):
        try:
            self._cam.close()
        except Exception:
            pass


# ----------------------------------------------------------------- sintetica
class SyntheticCamera:
    """Camera finta per test senza hardware: campiona un moto noto a ~30 Hz con rumore.

    truth(t) -> (position, quaternion) e' condiviso con SyntheticImu (vedi imu_ble.py)
    cosi' gyro e camera descrivono lo STESSO movimento e la fusione e' verificabile.
    """

    name = "sintetica (nessuna Kinect)"

    def __init__(self, truth, fps=30.0, pos_noise=0.002, rot_noise_deg=1.5,
                 clock=time.monotonic, rng=None):
        self.truth = truth
        self.period = 1.0 / fps
        self.pos_noise = pos_noise
        self.rot_noise = math.radians(rot_noise_deg)
        self.clock = clock
        self.rng = rng or np.random.default_rng(1)
        self._next = clock()

    def read(self):
        now = self.clock()
        wait = self._next - now
        if wait > 0:
            time.sleep(wait)
        now = self.clock()
        self._next = now + self.period
        p, q = self.truth(now)
        p = np.asarray(p, float) + self.rng.normal(scale=self.pos_noise, size=3)
        dq = Q.quat_from_rotvec(self.rng.normal(scale=self.rot_noise, size=3))
        q = Q.unit(Q.multiply(q, dq))
        pose = dict(position=p.tolist(), quaternion=q.tolist(), R=Q.matrix(q))
        return None, pose, now, None, None

    def close(self):
        pass
