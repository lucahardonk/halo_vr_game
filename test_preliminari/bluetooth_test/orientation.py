"""Madgwick MARG/IMU, Hamilton wxyz, corpo -> mondo. Vedi README per assi e limiti."""
import math
import threading
import time
from collections import deque

import numpy as np


def unit(v):
    v = np.asarray(v, dtype=float)
    n = np.linalg.norm(v)
    if not np.isfinite(n) or n < 1e-12:
        raise ValueError("Vettore nullo o non finito")
    return v / n


def multiply(a, b):
    w, x, y, z = a
    s, u, v, t = b
    return np.array([w*s-x*u-y*v-z*t, w*u+x*s+y*t-z*v,
                     w*v-x*t+y*s+z*u, w*t+x*v-y*u+z*s])


def conjugate(q):
    return np.array(q) * [1, -1, -1, -1]


def matrix(q):
    w, x, y, z = unit(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
                     [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
                     [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])


def from_euler(roll, pitch, yaw):
    r, p, y = np.asarray([roll, pitch, yaw]) / 2
    cr, cp, cy = np.cos([r, p, y]); sr, sp, sy = np.sin([r, p, y])
    return np.array([cr*cp*cy+sr*sp*sy, sr*cp*cy-cr*sp*sy,
                     cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy])


def euler(q):
    w, x, y, z = unit(q)
    return np.degrees([math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
                       math.asin(np.clip(2*(w*y-z*x), -1, 1)),
                       math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))]).tolist()


def rev2_axes(acc, gyr, mag):
    # Profilo libreria Reefwing-AHRS: setData, BMI270_BMM150 (fonti nel README).
    return np.array(acc), np.radians(gyr), np.array(mag) * [1, -1, 1]


def madgwick(q, gyr, acc, mag, dt, beta=0.06):
    """Equazioni 12, 25/26, 31/32, 33 del filtro Madgwick.

    Gradiente analitico; controllo norma evita 0/0. Corregge anche con gyro=0,
    diversamente dall'uscita anticipata di ahrs 0.4.0. Test di equivalenza inclusi.
    """
    q = unit(q)
    w, x, y, z = q
    derivative = 0.5 * multiply(q, [0, *gyr])
    if np.linalg.norm(acc) > 1e-9:
        a = unit(acc)
        f = [2*(x*z-w*y)-a[0], 2*(w*x+y*z)-a[1], 1-2*(x*x+y*y)-a[2]]
        J = [[-2*y, 2*z, -2*w, 2*x], [2*x, 2*w, 2*z, 2*y], [0, -4*x, -4*y, 0]]
        if mag is not None:
            m = unit(mag)
            h = matrix(q) @ m
            bx, bz = np.linalg.norm(h[:2]), h[2]
            f += [2*bx*(0.5-y*y-z*z)+2*bz*(x*z-w*y)-m[0],
                  2*bx*(x*y-w*z)+2*bz*(w*x+y*z)-m[1],
                  2*bx*(w*y+x*z)+2*bz*(0.5-x*x-y*y)-m[2]]
            J += [[-2*bz*y, 2*bz*z, -4*bx*y-2*bz*w, -4*bx*z+2*bz*x],
                  [-2*bx*z+2*bz*x, 2*bx*y+2*bz*w, 2*bx*x+2*bz*z, -2*bx*w+2*bz*y],
                  [2*bx*y, 2*bx*z-4*bz*x, 2*bx*w-4*bz*y, 2*bx*x]]
        gradient = np.asarray(J).T @ np.asarray(f)
        n = np.linalg.norm(gradient)
        if n > 1e-12:
            derivative -= beta * gradient / n
    return unit(q + derivative * dt)


def initial_attitude(acc, mag):
    a = unit(acc)
    roll = math.atan2(a[1], a[2])
    pitch = math.atan2(-a[0], math.hypot(a[1], a[2]))
    q = from_euler(roll, pitch, 0)
    yaw = 0.0
    if mag is not None:
        horizontal = matrix(q) @ unit(mag)
        if np.linalg.norm(horizontal[:2]) > 1e-6:
            yaw = -math.atan2(horizontal[1], horizontal[0])
    return from_euler(roll, pitch, yaw)


class Orientation:
    STALE = 0.5
    MAG_TIMEOUT = 0.5

    def __init__(self, demo=False, clock=time.monotonic):
        self.clock = clock
        self.demo = demo
        self.lock = threading.RLock()
        self.connected = False
        self.ever_connected = False
        self.error = None
        self.buttons = 0
        self.stamps = deque()
        self.frames = 0
        self.invalid = 0
        self.reset_count = 0
        self.clear()

    def clear(self):
        self.q = None
        self.reference = None
        self.last = None
        self.last_step = None
        self.mag_time = None
        self.last_mag = None
        self.stable = deque()
        self.mode = "IMU"
        self.stamps.clear()

    def connection(self, connected, error=None):
        with self.lock:
            if connected != self.connected or error:
                self.clear()
            self.connected = connected
            self.ever_connected |= connected
            self.error = error

    def feed(self, values, now=None):
        now = self.clock() if now is None else now
        with self.lock:
            if len(values) != 10 or not np.all(np.isfinite(values)):
                self.invalid += 1
                return
            a, g, m = rev2_axes(values[1:4], values[4:7], values[7:10])
            if np.linalg.norm(a) < 1e-6:
                self.invalid += 1
                return
            if self.last is not None and now-self.last > self.STALE:
                self.clear()  # Nessuna integrazione attraverso un vuoto temporale.
            self.connected = self.ever_connected = True
            self.error = None
            self.last = now
            self.buttons = int(values[0]) & 127
            self.frames += 1
            self.stamps.append(now)
            while self.stamps and now-self.stamps[0] > 1:
                self.stamps.popleft()
            if np.linalg.norm(m) > 1e-6:
                if self.last_mag is None or not np.array_equal(m, self.last_mag):
                    self.mag_time = now
                self.last_mag = m.copy()
            else:
                self.mag_time = None
            usable_mag = self.mag_time is not None and now-self.mag_time < self.MAG_TIMEOUT
            self.mode = "MARG" if usable_mag else "IMU"
            mag = m if usable_mag else None
            if self.q is None:
                stable = (abs(np.linalg.norm(a)-9.80665) < 0.8 and np.linalg.norm(g) < math.radians(3))
                if self.stable and np.linalg.norm(a-self.stable[-1][1]) > 0.3:
                    stable = False
                if not stable:
                    self.stable.clear()
                    return
                self.stable.append((now, a.copy(), m.copy()))
                if now-self.stable[0][0] < 1.5:
                    return
                avg_a = np.mean([s[1] for s in self.stable], axis=0)
                avg_m = np.mean([s[2] for s in self.stable], axis=0) if usable_mag else None
                self.q = initial_attitude(avg_a, avg_m)
                self.reference = self.q.copy()
                self.last_step = now
                self.stable.clear()
                return
            dt = now-self.last_step
            if dt < 0.01:  # Fusione <=100 Hz; il display non detta il dt.
                return
            if dt > self.STALE:
                self.clear()
                return
            self.last_step = now
            # Substep per stabilità numerica, non campioni indipendenti.
            steps = max(1, math.ceil(dt/0.02))
            for _ in range(steps):
                self.q = madgwick(self.q, g, a, mag, dt/steps)

    def snapshot(self):
        with self.lock:
            now = self.clock()
            age = None if self.last is None else max(0, now-self.last)
            status = "attesa"
            if not self.connected and self.ever_connected:
                status = "disconnesso"
            elif self.connected and age is not None and age > self.STALE:
                status = "stale"
            elif self.connected and self.q is None:
                status = "inizializzazione"
            elif self.connected and self.q is not None:
                status = "valido"
            valid = status == "valido"
            relative = unit(multiply(conjugate(self.reference), self.q)) if valid else None
            return dict(status=status, valid=valid, demo=self.demo, mode=self.mode,
                        quaternion=None if relative is None else relative.tolist(),
                        rpy=None if relative is None else euler(relative),
                        buttons=[bool(self.buttons & (1 << i)) for i in range(7)] if age is not None and age <= self.STALE and self.connected else None,
                        hz=sum(now-t <= 1 for t in self.stamps) if self.connected else 0,
                        age=age, frames=self.frames, invalid=self.invalid, resets=self.reset_count,
                        warning="Magnetometro assente/non aggiornato: yaw soggetto a deriva" if self.mode == "IMU" else "Campo magnetico non calibrato: possibili interferenze",
                        error=self.error)

    def reset(self):
        with self.lock:
            if not self.snapshot()["valid"]:
                return False
            self.reference = self.q.copy()
            self.reset_count += 1
            return True
