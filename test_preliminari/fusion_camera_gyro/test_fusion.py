#!/usr/bin/env python3
"""Test della matematica di fusione, eseguibili SENZA hardware (solo numpy).

    python3 test_fusion.py          # dentro test_preliminari/fusion_camera_gyro/

Verifica: round-trip quaternione<->matrice, slerp (estremi + punto medio), che predict()
integri davvero il gyro tra i frame camera, e che set_zero()/snapshot() diano orientamento
e posizione RELATIVI corretti. Non tocca Kinect ne' BLE.
"""
import math
import sys

import numpy as np

import quaternion as Q
from fusion import GyroCameraFusion


def _assert(cond, msg):
    if not cond:
        raise AssertionError(msg)


def _from_euler(roll, pitch, yaw):
    r, p, y = roll / 2, pitch / 2, yaw / 2
    cr, cp, cy = math.cos(r), math.cos(p), math.cos(y)
    sr, sp, sy = math.sin(r), math.sin(p), math.sin(y)
    return Q.unit(np.array([cr*cp*cy+sr*sp*sy, sr*cp*cy-cr*sp*sy,
                            cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy]))


def _quat_close(a, b, tol=1e-9):
    a, b = Q.unit(a), Q.unit(b)
    return min(np.linalg.norm(a - b), np.linalg.norm(a + b)) < tol   # q e -q = stesso orient.


def test_matrix_roundtrip():
    for rpy in [(0, 0, 0), (0.3, -0.2, 1.0), (-1.2, 0.9, -2.5), (math.pi/2, 0, 0)]:
        q = _from_euler(*rpy)
        q2 = Q.quat_from_matrix(Q.matrix(q))
        _assert(_quat_close(q, q2, 1e-8), f"roundtrip matrice fallito per {rpy}")
    print("ok  quaternione <-> matrice (round-trip)")


def test_rotvec_and_multiply():
    # ruotare di +90 deg attorno a Z due volte = +180 deg
    dq = Q.quat_from_rotvec([0, 0, math.pi / 2])
    q = Q.unit(Q.multiply(dq, dq))
    _assert(_quat_close(q, [0, 0, 0, 1]), "due rotazioni da 90 attorno a Z != 180")
    print("ok  quat_from_rotvec + multiply")


def test_slerp_endpoints_and_mid():
    q0 = _from_euler(0, 0, 0)
    q1 = _from_euler(0, 0, math.pi / 2)
    _assert(_quat_close(Q.slerp(q0, q1, 0.0), q0), "slerp(t=0) != q0")
    _assert(_quat_close(Q.slerp(q0, q1, 1.0), q1), "slerp(t=1) != q1")
    mid = Q.slerp(q0, q1, 0.5)
    _assert(_quat_close(mid, _from_euler(0, 0, math.pi / 4), 1e-6), "slerp(t=0.5) != 45 deg")
    print("ok  slerp (estremi + punto medio)")


def test_predict_integrates_gyro():
    """Senza correzioni camera, integrando 2 rad/s attorno a Z per 1 s -> ~2 rad di yaw."""
    f = GyroCameraFusion(slerp_gain=0.2)
    f.bias_ready = True          # salta la stima del bias per il test
    f.q = np.array([1.0, 0.0, 0.0, 0.0])   # parti dall'identita' (come dopo il 1o frame camera)
    rate = 2.0                   # rad/s attorno a Z
    t = 0.0
    f.predict([0, 0, rate], now=t)          # primo campione: fissa solo t_gyro
    dt = 1.0 / 200
    for _ in range(200):
        t += dt
        f.predict([0, 0, rate], now=t)
    yaw = math.radians(Q.euler_deg(f.q)[2])
    _assert(abs(yaw - rate * 1.0) < 1e-2, f"integrazione gyro: yaw={yaw:.4f}, atteso ~2.0")
    print(f"ok  predict integra il gyro (yaw={yaw:.3f} rad dopo 1 s a 2 rad/s)")


def test_zero_relative_output():
    """set_zero() cattura il riferimento: subito dopo RPY relativo ~0; poi ruota di noto."""
    f = GyroCameraFusion(slerp_gain=1.0)   # slerp=1 -> q segue esattamente la camera
    q_ref = _from_euler(0.1, -0.2, 0.3)
    p_ref = np.array([0.5, -0.1, 0.8])
    f.correct(q_ref, p_ref, now=0.0)
    _assert(f.set_zero(), "set_zero deve riuscire dopo una posa camera")
    snap = f.snapshot(now=0.0)
    r, p, y = snap["rpy_rel"]
    _assert(max(abs(r), abs(p), abs(y)) < 1e-6, "RPY relativo non nullo subito dopo lo zero")
    _assert(max(abs(c) for c in snap["position_rel"]) < 1e-9, "posizione relativa non nulla allo zero")

    # ruota di +30 deg in yaw rispetto al riferimento, trasla di 10 cm lungo X mondo
    q_new = Q.unit(Q.multiply(q_ref, _from_euler(0, 0, math.radians(30))))
    f.correct(q_new, p_ref + np.array([0.1, 0.0, 0.0]), now=0.1)
    snap = f.snapshot(now=0.1)
    _assert(abs(snap["rpy_rel"][2] - 30.0) < 1e-3, f"yaw relativo {snap['rpy_rel'][2]:.3f} != 30")
    _assert(snap["position_rel"] is not None, "manca la posizione relativa")
    print("ok  set_zero + snapshot (orientamento e posizione relativi allo zero)")


def test_stale_gap_resets():
    """Un vuoto > STALE non deve integrare attraverso il buco (niente salto di orientamento)."""
    f = GyroCameraFusion()
    f.bias_ready = True
    f.q = np.array([1.0, 0.0, 0.0, 0.0])
    f.predict([0, 0, 5.0], now=0.0)
    f.predict([0, 0, 5.0], now=0.0 + f.STALE + 1.0)   # buco enorme
    yaw = abs(Q.euler_deg(f.q)[2])
    _assert(yaw < 1e-6, f"ha integrato attraverso un vuoto stale (yaw={yaw})")
    print("ok  vuoto temporale > STALE non viene integrato")


def main():
    tests = [test_matrix_roundtrip, test_rotvec_and_multiply, test_slerp_endpoints_and_mid,
             test_predict_integrates_gyro, test_zero_relative_output, test_stale_gap_resets]
    failed = 0
    for t in tests:
        try:
            t()
        except AssertionError as exc:
            failed += 1
            print(f"FALLITO  {t.__name__}: {exc}")
    print()
    if failed:
        print(f"{failed}/{len(tests)} test FALLITI")
        return 1
    print(f"tutti i {len(tests)} test OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
