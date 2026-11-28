"""Quaternioni Hamilton (w, x, y, z), convenzione corpo->mondo come orientation.py.

Solo numpy. Funzioni minime per la fusione gyro+camera:
- multiply / conjugate / unit / matrix  (identiche a bluetooth_test/orientation.py)
- quat_from_matrix : matrice di rotazione 3x3 (corpo->mondo) -> quaternione
- quat_from_rotvec : vettore rotazione (asse*angolo, rad) -> quaternione (mappa esponenziale)
- slerp           : interpolazione sferica tra due quaternioni, con gestione del segno
- euler_deg       : quaternione -> [roll, pitch, yaw] in gradi
"""
import math

import numpy as np


def unit(q):
    q = np.asarray(q, dtype=float)
    n = np.linalg.norm(q)
    if not np.isfinite(n) or n < 1e-12:
        raise ValueError("Quaternione nullo o non finito")
    return q / n


def multiply(a, b):
    w, x, y, z = a
    s, u, v, t = b
    return np.array([w*s-x*u-y*v-z*t, w*u+x*s+y*t-z*v,
                     w*v-x*t+y*s+z*u, w*t+x*v-y*u+z*s])


def conjugate(q):
    return np.asarray(q, dtype=float) * [1, -1, -1, -1]


def matrix(q):
    """Quaternione -> matrice di rotazione 3x3 (corpo->mondo)."""
    w, x, y, z = unit(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
                     [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
                     [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])


def quat_from_matrix(R):
    """Matrice di rotazione 3x3 -> quaternione (w, x, y, z). Metodo di Shepperd, stabile."""
    R = np.asarray(R, dtype=float)
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return unit(np.array([w, x, y, z]))


def quat_from_rotvec(rotvec):
    """Vettore rotazione (asse*angolo, rad) -> quaternione (mappa esponenziale)."""
    rotvec = np.asarray(rotvec, dtype=float)
    angle = float(np.linalg.norm(rotvec))
    if angle < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    axis = rotvec / angle
    half = angle / 2.0
    return np.array([math.cos(half), *(axis * math.sin(half))])


def slerp(q0, q1, t):
    """Interpolazione sferica da q0 a q1 (t in [0,1]). Sceglie il cammino piu' corto."""
    q0 = unit(q0)
    q1 = unit(q1)
    dot = float(np.dot(q0, q1))
    if dot < 0.0:          # stesso orientamento, segno opposto: prendi la via breve
        q1 = -q1
        dot = -dot
    if dot > 0.9995:       # quasi paralleli: lerp + normalizza (evita 0/0)
        return unit(q0 + t * (q1 - q0))
    theta0 = math.acos(max(-1.0, min(1.0, dot)))
    theta = theta0 * t
    sin0 = math.sin(theta0)
    s0 = math.sin(theta0 - theta) / sin0
    s1 = math.sin(theta) / sin0
    return unit(s0 * q0 + s1 * q1)


def euler_deg(q):
    """Quaternione -> [roll, pitch, yaw] in gradi (ZYX, come orientation.py)."""
    w, x, y, z = unit(q)
    return [math.degrees(math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))),
            math.degrees(math.asin(max(-1.0, min(1.0, 2*(w*y-z*x))))),
            math.degrees(math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z)))]
