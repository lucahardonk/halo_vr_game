#!/usr/bin/env python3
"""Fusione AprilTag (Kinect v1) + gyro IMU (BLE) per l'orientamento, con correzione slerp.

La camera (mouse_control) da' la posa ASSOLUTA del tag a ~30 Hz; il gyro dell'IMU, montata
SOLIDALE al tag, propaga l'orientamento ad alta frequenza tra un frame e l'altro. A ogni frame
camera la deriva del gyro viene ritirata con uno slerp dolce verso la posa assoluta.

INVIO  = cattura lo ZERO (posizione + orientamento correnti): da li' l'uscita e' relativa.
q / ESC / Ctrl+C = esci.   (in modalita' --demo tutto e' sintetico, nessun hardware)

    python3 fusion_camera_gyro.py --tag-size 0.05            # Kinect + IMU reali
    python3 fusion_camera_gyro.py --tag-size 0.05 --address 18:FE:03:D9:D4:4B
    python3 fusion_camera_gyro.py --demo --no-preview --duration 6   # prova senza hardware
"""
import argparse
import math
import queue
import select
import sys
import threading
import time

import numpy as np

import quaternion as Q
from fusion import GyroCameraFusion

PREVIEW_WINDOW = "fusion_camera_gyro (INVIO = zero, q = esci)"
TAG_LOST_HOLD_S = 0.3


# ----------------------------------------------------------------- moto sintetico
def make_truth():
    """truth(t) -> (posizione m, quaternione). Moto liscio usato da camera e gyro sintetici."""
    def truth(t):
        roll = 0.35 * math.sin(0.9 * t)
        pitch = 0.25 * math.sin(0.6 * t + 0.5)
        yaw = 0.5 * math.sin(0.4 * t)
        q = Q.unit(_from_euler(roll, pitch, yaw))
        p = np.array([0.03 * math.sin(0.5 * t), 0.02 * math.sin(0.7 * t), 0.6 + 0.05 * math.sin(0.3 * t)])
        return p, q
    return truth


def _from_euler(roll, pitch, yaw):
    r, p, y = roll / 2, pitch / 2, yaw / 2
    cr, cp, cy = math.cos(r), math.cos(p), math.cos(y)
    sr, sp, sy = math.sin(r), math.sin(p), math.sin(y)
    return np.array([cr*cp*cy+sr*sp*sy, sr*cp*cy-cr*sp*sy,
                     cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy])


# ----------------------------------------------------------------- tastiera
def start_terminal_reader(keys):
    def loop():
        while True:
            r, _, _ = select.select([sys.stdin], [], [], 0.5)
            if r:
                line = sys.stdin.readline()
                if line == "":
                    break
                keys.put("enter")
    if sys.stdin is not None and sys.stdin.isatty():
        threading.Thread(target=loop, name="stdin-enter", daemon=True).start()


# ----------------------------------------------------------------- disegno
def draw_preview(cv2, frame, corners, ids, lines):
    if ids is not None and len(ids):
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
    for i, text in enumerate(lines):
        y = 22 + i * 22
        cv2.putText(frame, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(frame, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)


def status_lines(snap):
    if not snap["has_pose"]:
        return ["In attesa della prima posa AprilTag..."]
    bias = "ok" if snap["bias_ready"] else "in stima (tieni la scheda ferma)"
    lines = [f"gyro {snap['gyro_hz']:.0f} Hz | camera {snap['cam_hz']:.0f} Hz | bias {bias}"]
    if not snap["has_zero"]:
        lines.append("Premi INVIO per fissare lo ZERO (posizione + orientamento).")
    else:
        r, p, y = snap["rpy_rel"]
        lines.append(f"orient. rel  R={r:+6.1f}  P={p:+6.1f}  Y={y:+6.1f}  (gradi)")
        if snap["position_rel"] is not None:
            px, py, pz = snap["position_rel"]
            lines.append(f"pos. rel     x={px:+.3f} y={py:+.3f} z={pz:+.3f}  (m)")
    return lines


# ----------------------------------------------------------------- main
def parse_args():
    p = argparse.ArgumentParser(description="Fusione AprilTag + gyro IMU (slerp) per l'orientamento")
    p.add_argument("--tag-size", type=float, default=0.05, help="lato del quadrato nero del tag (m)")
    p.add_argument("--tag-id", type=int, default=None, help="usa solo questo ID tag")
    p.add_argument("--index", type=int, default=0, help="indice device Kinect")
    p.add_argument("--address", help="MAC IMU_BTN gia' connessa dalla GUI Ubuntu")
    p.add_argument("--slerp", type=float, default=0.15,
                   help="guadagno correzione camera 0..1 (basso=piu' liscio/piu' gyro; default 0.15)")
    p.add_argument("--demo", action="store_true", help="camera+IMU SINTETICHE, nessun hardware")
    p.add_argument("--no-preview", action="store_true", help="nessuna finestra camera")
    p.add_argument("--duration", type=float, default=0.0, help="esci dopo N secondi (0 = infinito)")
    p.add_argument("--auto-zero", type=float, default=0.0,
                   help="fissa lo zero automaticamente dopo N secondi (per test non interattivi)")
    args = p.parse_args()
    args.slerp = min(max(args.slerp, 0.0), 1.0)
    if args.tag_size <= 0:
        p.error("--tag-size deve essere > 0")
    return args


def build_sources(args, fusion):
    """Ritorna (camera, imu, info). In --demo sono sintetiche e condividono lo stesso moto."""
    if args.demo:
        from camera_tag import SyntheticCamera
        from imu_ble import SyntheticImu
        truth = make_truth()
        cam = SyntheticCamera(truth)
        imu = SyntheticImu(truth, fusion.predict)
        return cam, imu, "DEMO sintetica (nessuna Kinect, nessun BLE)"
    from camera_tag import CameraSource
    from imu_ble import ImuBle
    cam = CameraSource(args.tag_size, args.tag_id, args.index)
    status_lock = threading.Lock()

    def on_status(msg):
        with status_lock:
            print(f"[IMU] {msg}", flush=True)
    imu = ImuBle(fusion.predict, args.address, on_status=on_status)
    return cam, imu, f"Kinect: {cam.name} | IMU: {imu.name}"


def main():
    args = parse_args()
    fusion = GyroCameraFusion(slerp_gain=args.slerp)

    try:
        cam, imu, info = build_sources(args, fusion)
    except RuntimeError as exc:
        print(f"ERRORE: {exc}", file=sys.stderr)
        return 1

    print(info)
    print(f"slerp gain = {args.slerp:.2f}  (INVIO = zero, q/ESC/Ctrl+C = esci)")
    imu.start()

    preview = not args.no_preview
    cv2 = None
    if preview:
        try:
            import cv2 as _cv2
            cv2 = _cv2
            cv2.namedWindow(PREVIEW_WINDOW, cv2.WINDOW_AUTOSIZE)
        except Exception as exc:
            print(f"[preview] disattivata ({exc})")
            preview = False

    keys = queue.Queue()
    start_terminal_reader(keys)

    last_pose, last_seen = None, 0.0
    last_print = 0.0
    start = time.monotonic()
    zeroed = False
    rc = 0
    try:
        while True:
            now = time.monotonic()
            if args.duration and now - start > args.duration:
                break

            frame, pose, t_cam, corners, ids = cam.read()
            if pose is not None:
                fusion.correct(np.array(pose["quaternion"]), np.array(pose["position"]), t_cam)
                last_pose, last_seen = pose, t_cam

            if args.auto_zero and not zeroed and now - start > args.auto_zero:
                if fusion.set_zero():
                    zeroed = True
                    print(f"[auto] ZERO fissato a t={now - start:.1f}s", flush=True)

            # tasti: finestra preview, poi coda terminale
            if preview:
                k = cv2.waitKey(1) & 0xFF
                if k in (13, 10):
                    keys.put("enter")
                elif k in (ord("q"), 27):
                    break
            while not keys.empty():
                key = keys.get_nowait()
                if key == "enter":
                    if fusion.set_zero():
                        zeroed = True
                        print("ZERO fissato: posizione e orientamento correnti sono il riferimento.", flush=True)
                    else:
                        print("Nessuna posa disponibile: inquadra il tag (e attendi l'IMU), poi INVIO.", flush=True)

            snap = fusion.snapshot(now)
            if now - last_print > 0.1:          # stampa ~10 Hz
                last_print = now
                if snap["has_zero"]:
                    r, p, y = snap["rpy_rel"]
                    pos = snap["position_rel"]
                    pos_s = "   pos n/d" if pos is None else f"   pos=[{pos[0]:+.3f} {pos[1]:+.3f} {pos[2]:+.3f}]m"
                    print(f"[out {snap['gyro_hz']:3.0f}Hz gyro / {snap['cam_hz']:2.0f}Hz cam] "
                          f"RPY=[{r:+6.1f} {p:+6.1f} {y:+6.1f}]deg{pos_s}", flush=True)
                elif snap["has_pose"]:
                    print(f"[attesa zero] gyro {snap['gyro_hz']:.0f} Hz, camera {snap['cam_hz']:.0f} Hz "
                          f"— {'bias ok' if snap['bias_ready'] else 'stima bias...'}", flush=True)

            if preview:
                if frame is None:
                    frame = np.zeros((480, 640, 3), dtype=np.uint8)
                draw_preview(cv2, frame, corners, ids, status_lines(snap))
                cv2.imshow(PREVIEW_WINDOW, frame)
                if cv2.getWindowProperty(PREVIEW_WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                    break
    except KeyboardInterrupt:
        print("\nCtrl+C: uscita.")
    finally:
        imu.stop()
        cam.close()
        if preview and cv2 is not None:
            cv2.destroyAllWindows()
    return rc


if __name__ == "__main__":
    sys.exit(main())
