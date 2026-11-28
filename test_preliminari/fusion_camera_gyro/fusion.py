"""Fusione gyro (alta frequenza) + AprilTag (posa assoluta, ~30 Hz) per l'orientamento.

Idea (filtro complementare minimale, niente Madgwick, niente magnetometro):
  predict()  a ogni campione gyro: q <- q (x) dq(omega*dt)   (integrazione corpo-frame)
  correct()  a ogni frame camera:  q <- slerp(q, q_camera, slerp_gain)

Tra due frame camera il gyro "riempie il buco" ad alta frequenza; quando arriva la
posa della camera la deriva del gyro viene ritirata dolcemente con uno slerp. La camera
e' la verita' assoluta a lungo termine, il gyro la fluidita' a breve termine.

Lo ZERO si cattura con set_zero(): memorizza l'orientamento fuso corrente (q_ref) e la
posizione camera corrente (p_ref). Da quel momento l'uscita e' RELATIVA allo zero:
  q_rel = conj(q_ref) (x) q_fuso
  p_rel = R_ref^T (p_camera - p_ref)   (traslazione espressa nel frame dello zero)

IMU e AprilTag sono considerati SOLIDALI (stesso corpo rigido): per questo test il frame
del gyro coincide con il frame del tag (R_GYRO_TO_TAG = identita'). Se in futuro l'IMU
fosse montata ruotata rispetto al tag, basta impostare qui la matrice costante.
"""
import threading
import time

import numpy as np

import quaternion as Q

# IMU solidale al tag: nessuna rotazione tra frame gyro e frame tag (vedi docstring).
R_GYRO_TO_TAG = np.eye(3)


class GyroCameraFusion:
    """Stato condiviso tra il thread IMU (predict) e il loop camera (correct)."""

    STALE = 0.5          # s: oltre questo vuoto temporale lo stato gyro si azzera
    BIAS_SAMPLES = 150   # campioni fermi per stimare il bias statico del gyro
    BIAS_MAX_RATE = np.radians(3.0)   # |gyro| sotto cui la scheda e' "ferma"

    def __init__(self, slerp_gain=0.15, clock=time.monotonic):
        self.slerp_gain = float(slerp_gain)
        self.clock = clock
        self.lock = threading.RLock()

        self.q = None             # orientamento fuso corrente (corpo->mondo/camera)
        self.t_gyro = None        # timestamp ultimo campione gyro integrato
        self.t_cam = None         # timestamp ultima correzione camera
        self.p_cam = None         # ultima posizione camera (m, frame camera)

        self.q_ref = None         # zero: orientamento di riferimento
        self.p_ref = None         # zero: posizione di riferimento
        self.R_ref_T = None       # trasposta della matrice dello zero (mondo->zero)

        # stima bias gyro statico
        self.bias = np.zeros(3)
        self._bias_acc = []
        self.bias_ready = False

        # statistiche
        self.gyro_count = 0
        self.cam_count = 0
        self._gyro_stamps = []
        self._cam_stamps = []

    # ---------------------------------------------------------------- IMU
    def predict(self, gyro_rad, now=None, values=None):
        """gyro_rad: velocita' angolare (rad/s) nel frame IMU. Integra q nel tempo.

        'values' e' il frame grezzo BLE (btn+acc+gyro+mag); qui non serve ma lo accettiamo
        perche' sia ImuBle sia SyntheticImu passano il payload completo al callback."""
        now = self.clock() if now is None else now
        with self.lock:
            omega = R_GYRO_TO_TAG @ np.asarray(gyro_rad, dtype=float)
            if not np.all(np.isfinite(omega)):
                return
            self.gyro_count += 1
            self._gyro_stamps.append(now)
            self._trim(self._gyro_stamps, now)

            # Stima del bias statico finche' la scheda resta ferma all'avvio.
            if not self.bias_ready:
                if np.linalg.norm(omega) < self.BIAS_MAX_RATE:
                    self._bias_acc.append(omega.copy())
                    if len(self._bias_acc) >= self.BIAS_SAMPLES:
                        self.bias = np.mean(self._bias_acc, axis=0)
                        self.bias_ready = True
                else:
                    self._bias_acc.clear()

            omega = omega - self.bias

            if self.t_gyro is None:
                self.t_gyro = now
                return
            dt = now - self.t_gyro
            self.t_gyro = now
            if dt <= 0:
                return
            if dt > self.STALE:          # vuoto temporale: non integrare attraverso il buco
                return
            if self.q is None:           # nessuna posa camera ancora: niente da propagare
                return
            dq = Q.quat_from_rotvec(omega * dt)
            self.q = Q.unit(Q.multiply(self.q, dq))   # rate nel corpo -> moltiplica a destra

    # ---------------------------------------------------------------- camera
    def correct(self, q_cam, p_cam, now=None):
        """q_cam: orientamento assoluto del tag (corpo->camera). p_cam: posizione (m)."""
        now = self.clock() if now is None else now
        with self.lock:
            q_cam = Q.unit(q_cam)
            self.p_cam = np.asarray(p_cam, dtype=float)
            self.t_cam = now
            self.cam_count += 1
            self._cam_stamps.append(now)
            self._trim(self._cam_stamps, now)
            if self.q is None:
                self.q = q_cam            # primo aggancio: parti dalla posa assoluta
            else:
                self.q = Q.slerp(self.q, q_cam, self.slerp_gain)

    # ---------------------------------------------------------------- zero
    def set_zero(self):
        """Cattura lo zero (orientamento fuso + posizione camera correnti). Serve una posa."""
        with self.lock:
            if self.q is None or self.p_cam is None:
                return False
            self.q_ref = self.q.copy()
            self.p_ref = self.p_cam.copy()
            self.R_ref_T = Q.matrix(self.q_ref).T
            return True

    # ---------------------------------------------------------------- uscita
    def snapshot(self, now=None):
        now = self.clock() if now is None else now
        with self.lock:
            has_pose = self.q is not None
            has_zero = self.q_ref is not None
            q_rel = p_rel = rpy_rel = None
            if has_pose and has_zero:
                q_rel = Q.unit(Q.multiply(Q.conjugate(self.q_ref), self.q))
                rpy_rel = Q.euler_deg(q_rel)
                if self.p_cam is not None:
                    p_rel = (self.R_ref_T @ (self.p_cam - self.p_ref)).tolist()
            cam_age = None if self.t_cam is None else max(0.0, now - self.t_cam)
            return dict(
                has_pose=has_pose,
                has_zero=has_zero,
                bias_ready=self.bias_ready,
                bias_deg=np.degrees(self.bias).tolist(),
                quaternion=None if not has_pose else self.q.tolist(),
                quaternion_rel=None if q_rel is None else q_rel.tolist(),
                rpy_rel=rpy_rel,
                position_rel=p_rel,
                cam_age=cam_age,
                gyro_hz=self._rate(self._gyro_stamps, now),
                cam_hz=self._rate(self._cam_stamps, now),
                gyro_count=self.gyro_count,
                cam_count=self.cam_count,
            )

    # ---------------------------------------------------------------- util
    @staticmethod
    def _trim(stamps, now):
        while stamps and now - stamps[0] > 1.0:
            stamps.pop(0)

    @staticmethod
    def _rate(stamps, now):
        return float(sum(now - t <= 1.0 for t in stamps))
