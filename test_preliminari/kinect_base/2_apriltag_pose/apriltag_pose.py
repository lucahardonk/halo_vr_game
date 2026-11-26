"""Real-time AprilTag 6DOF pose with the Kinect v1 RGB camera.

Detects tag36h11 tags (pupil_apriltags), estimates the pose with cv2.solvePnP
and draws the tag frame axes: X = red, Y = green, Z = blue (Z points out of the
tag towards the camera). Press 'q' / ESC or close the window to quit.

    python apriltag_pose.py                 # Kinect via libfreenect
    python apriltag_pose.py --tag-size 0.10 # tag side (black border) = 10 cm
    python apriltag_pose.py --opencv 0      # any webcam via OpenCV (test without Kinect)
"""
import argparse
import ctypes as C
import time
from ctypes.util import find_library

import cv2
import numpy as np
from pupil_apriltags import Detector

# ----------------------------------------------------------------- settings
TAG_SIZE_M = 0.05      # side length of the tag's black square, metres
AXIS_LEN_M = 0.03      # drawn axis length, metres
TAG_FAMILY = "tag36h11"

# Kinect v1 RGB intrinsics (640x480, standard approximate values)
FX, FY = 525.0, 525.0
CX, CY = 319.5, 239.5
CAMERA_MATRIX = np.array([[FX, 0, CX], [0, FY, CY], [0, 0, 1]], dtype=np.float64)
DIST_COEFFS = np.zeros(5)

WINDOW = "AprilTag pose - Kinect v1 (q to quit)"
WIDTH, HEIGHT = 640, 480


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


# Rotation of 180 deg about X: turns the apriltag/IPPE frame (y down, z into the
# tag) into a friendlier one (x right, y up, z out of the tag towards camera).
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


def euler_xyz_deg(R):
    """Roll/pitch/yaw (X, Y, Z) in degrees from R = Rz * Ry * Rx."""
    sy = np.hypot(R[0, 0], R[1, 0])
    if sy > 1e-6:
        x = np.arctan2(R[2, 1], R[2, 2])
        y = np.arctan2(-R[2, 0], sy)
        z = np.arctan2(R[1, 0], R[0, 0])
    else:
        x = np.arctan2(-R[1, 2], R[1, 1])
        y = np.arctan2(-R[2, 0], sy)
        z = 0.0
    return np.degrees([x, y, z])


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
    o = tuple(proj[0])
    for i, color in enumerate(AXIS_COLORS):
        cv2.arrowedLine(frame, o, tuple(proj[i + 1]), color, 3, cv2.LINE_AA, tipLength=0.15)
    dist = float(np.linalg.norm(tvec))
    label = f"ID {det.tag_id}  {dist:.2f} m"
    org = (int(pts[:, 0].min()), int(pts[:, 1].min()) - 8)
    cv2.putText(frame, label, org, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(frame, label, org, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2, cv2.LINE_AA)


def draw_hud(frame, fps, n_tags, source):
    lines = [f"FPS: {fps:5.1f}", f"tags: {n_tags}", source]
    for i, text in enumerate(lines):
        y = 24 + i * 22
        cv2.putText(frame, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(frame, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)


def generate_tag(tag_id, px=800, margin=100):
    """Save a printable tag36h11 PNG in the official AprilTag orientation.

    OpenCV's aruco generator draws AprilTags rotated 180 deg with respect to the
    official images; it is rotated back here so that the drawn X axis points
    right and Y up when the tag is printed upright.
    """
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    marker = cv2.rotate(cv2.aruco.generateImageMarker(dictionary, tag_id, px), cv2.ROTATE_180)
    img = cv2.copyMakeBorder(marker, margin, margin, margin, margin, cv2.BORDER_CONSTANT, value=255)
    out = f"tag36h11_id{tag_id}.png"
    if not cv2.imwrite(out, img):
        raise OSError(f"cannot write {out}")
    print(f"saved {out}: print it and measure the BLACK square side -> --tag-size")


# ----------------------------------------------------------------- main
def main():
    global AXIS_PTS
    parser = argparse.ArgumentParser(description="AprilTag 6DOF pose with Kinect v1")
    parser.add_argument("--tag-size", type=float, default=TAG_SIZE_M, help="tag black-square side in metres")
    parser.add_argument("--axis-len", type=float, default=AXIS_LEN_M, help="drawn axis length in metres")
    parser.add_argument("--index", type=int, default=0, help="Kinect device index")
    parser.add_argument("--opencv", type=int, default=None, metavar="N",
                        help="use OpenCV camera N instead of the Kinect")
    parser.add_argument("--threads", type=int, default=4, help="detector threads")
    parser.add_argument("--decimate", type=float, default=1.0,
                        help="quad_decimate (2.0 = faster, shorter detection range)")
    parser.add_argument("--print-hz", type=float, default=5.0, help="console pose print rate (0 = off)")
    parser.add_argument("--generate", type=int, default=None, metavar="ID",
                        help="save a printable tag36h11 PNG with this ID and exit")
    args = parser.parse_args()

    if args.generate is not None:
        generate_tag(args.generate)
        return

    AXIS_PTS = AXIS_PTS / AXIS_LEN_M * args.axis_len
    obj_pts = tag_object_points(args.tag_size)
    detector = Detector(families=TAG_FAMILY, nthreads=args.threads, quad_decimate=args.decimate,
                        quad_sigma=0.0, refine_edges=1, decode_sharpening=0.25)

    cam = open_camera(args)
    print(f"Camera: {cam.name} | tag {TAG_FAMILY} size {args.tag_size * 100:.1f} cm | q/ESC to quit")
    cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE)

    fps, frames, t_fps = 0.0, 0, time.monotonic()
    t_print = 0.0
    K, K_size = None, None
    print_period = 1.0 / args.print_hz if args.print_hz > 0 else None
    try:
        while True:
            frame = cam.read()
            h, w = frame.shape[:2]
            if K is None or K_size != (w, h):
                K, K_size = camera_matrix_for(w, h), (w, h)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            detections = detector.detect(gray)

            now = time.monotonic()
            do_print = print_period is not None and now - t_print >= print_period
            for det in detections:
                pose = solve_pose(det.corners, obj_pts, K)
                if pose is None:
                    continue
                rvec, tvec, R = pose
                draw_tag(frame, det, rvec, tvec, K)
                if do_print:
                    t = tvec.ravel()
                    rx, ry, rz = euler_xyz_deg(R)
                    rv = np.degrees(rvec.ravel())
                    print(f"tag {det.tag_id:3d} | t = [{t[0]:+.3f} {t[1]:+.3f} {t[2]:+.3f}] m "
                          f"| dist {np.linalg.norm(t):.3f} m | rvec = [{rv[0]:+7.1f} {rv[1]:+7.1f} {rv[2]:+7.1f}] deg "
                          f"| rpy = [{rx:+7.1f} {ry:+7.1f} {rz:+7.1f}] deg")
            if do_print:
                t_print = now

            frames += 1
            if now - t_fps >= 0.5:
                fps, frames, t_fps = frames / (now - t_fps), 0, now
            draw_hud(frame, fps, len(detections), cam.name)
            cv2.imshow(WINDOW, frame)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                break
    except KeyboardInterrupt:
        pass
    finally:
        cam.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
