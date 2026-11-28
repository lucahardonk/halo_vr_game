"""Calibrazione PRIMA dell'AHRS; nessuna dipendenza da orientamenti già stimati.

Magnetometro: ellissoide robusto, radice SPD, scala a determinante unitario.
Le soglie sono conservative per un banco di prova, non una certificazione metrologica.
"""
import itertools
import json
import os
from pathlib import Path
import re
import tempfile

import numpy as np

SCHEMA = 1
MAPPING = 'rev2-library-a-g-identity-mag-x-minusy-z-v1'
DIRECTIONS = np.array([v for v in itertools.product([-1, 0, 1], repeat=3) if any(v)], float)
DIRECTIONS /= np.linalg.norm(DIRECTIONS, axis=1)[:, None]


def coverage(points):
    """26 settori sferici, sei facce dominanti; proxy, non misura dell'assetto."""
    p = np.asarray(points, float)
    if len(p) < 12:
        return 0, ['±X', '±Y', '±Z']
    center = (np.percentile(p, 95, axis=0) + np.percentile(p, 5, axis=0)) / 2
    u = p-center
    n = np.linalg.norm(u, axis=1)
    u = u[n > 1e-8] / n[n > 1e-8, None]
    bins = np.bincount(np.argmax(u @ DIRECTIONS.T, axis=1), minlength=26)
    face = np.argmax(np.abs(u), axis=1)
    missing = [f'{sign}{axis}' for i, axis in enumerate('XYZ') for sign in ('+', '-')
               if np.sum((face == i) & (u[:, i] * (1 if sign == '+' else -1) > 0)) < 5]
    return int(np.sum(bins >= 3)), missing


def _ellipsoid(points):
    """Fit normalizzato x'Qx+d'x=1; rifiuto non-SPD e cattivo condizionamento."""
    origin = np.median(points, axis=0)
    scale = np.median(np.linalg.norm(points-origin, axis=1))
    if scale < 1e-6:
        raise ValueError('Campioni degeneri: campo invariato.')
    x, y, z = ((points-origin)/scale).T
    design = np.column_stack((x*x, y*y, z*z, 2*x*y, 2*x*z, 2*y*z, x, y, z))
    if np.linalg.cond(design) > 1e4:
        raise ValueError('Rotazioni insufficienti o raccolta planare: ribalta la scheda.')
    c = np.linalg.lstsq(design, np.ones(len(points)), rcond=None)[0]
    Q = np.array([[c[0], c[3], c[4]], [c[3], c[1], c[5]], [c[4], c[5], c[2]]])
    if np.min(np.linalg.eigvalsh(Q)) <= 0:
        raise ValueError('Fit non ellissoidale: ripeti lontano da metalli.')
    b = -.5*np.linalg.solve(Q, c[6:])
    C = Q/(1 + b@Q@b)
    eigen, vectors = np.linalg.eigh(C)
    if eigen.min() <= 0 or eigen.max()/eigen.min() > 25:
        raise ValueError('Matrice instabile: distorsione eccessiva o copertura incompleta.')
    A = (vectors*np.sqrt(eigen)) @ vectors.T / scale
    return origin + scale*b, A


def fit_magnetometer(points):
    p = np.asarray(points, float)
    if p.ndim != 2 or p.shape[1] != 3 or len(p) < 180 or not np.isfinite(p).all():
        raise ValueError('Servono almeno 180 campioni utili e finiti; continua a ruotare.')
    if len(p) > 1200 or np.max(np.abs(p)) > 1000:
        raise ValueError('Raccolta fuori limite: ripeti la misura.')
    eig = np.linalg.eigvalsh(np.cov(p.T))
    if eig[0] < 1e-4 or eig[0]/eig[-1] < .02:
        raise ValueError('Raccolta planare: ruota su tre assi e ribalta la scheda.')
    # Holdout fisso indipendente dal fit: 20% dei campioni, seme riproducibile.
    rng = np.random.default_rng(713)
    idx = rng.permutation(len(p))
    hold = p[idx[::5]]
    train = p[np.delete(idx, np.arange(0, len(idx), 5))]
    best = None
    for k in range(100):
        subset = train if k == 0 else train[rng.choice(len(train), 24, replace=False)]
        try:
            b, A = _ellipsoid(subset)
        except (ValueError, np.linalg.LinAlgError):
            continue
        residual = np.abs(np.linalg.norm((train-b)@A.T, axis=1)-1)
        mask = residual < .09
        score = (int(mask.sum()), -float(np.median(residual)))
        if best is None or score > best[0]:
            best = (score, mask)
    if best is None or best[1].mean() < .60:
        raise ValueError('Troppi disturbi/outlier: allontana metalli e magneti, poi riprova.')
    mask = best[1]
    for _ in range(4):
        b, A = _ellipsoid(train[mask])
        residual = np.abs(np.linalg.norm((train-b)@A.T, axis=1)-1)
        mask = residual < .09
        if mask.mean() < .60:
            raise ValueError('Fit incoerente: ripeti la raccolta in un campo stabile.')
    b, A = _ellipsoid(train[mask])
    errors = np.abs(np.linalg.norm((hold-b)@A.T, axis=1)-1)
    good = errors < .14
    if good.mean() < .50 or np.sqrt(np.mean(errors[good]**2)) > .07:
        raise ValueError('Verifica indipendente fallita: campo variabile o raccolta incompleta.')
    all_errors = np.abs(np.linalg.norm((p-b)@A.T, axis=1)-1)
    corrected = (p[all_errors < .09]-b)@A.T
    sectors, missing = coverage(corrected)
    covariance = np.linalg.eigvalsh(np.cov(corrected.T))
    if sectors < 18 or missing or covariance[0]/covariance[-1] < .10:
        raise ValueError('Copertura incompleta: ruota su tre assi, includendo tutti i ribaltamenti.')
    if np.any(b < p.min(axis=0)) or np.any(b > p.max(axis=0)):
        raise ValueError('Centro esterno alla raccolta: servono rotazioni più complete.')
    # L'intensità assoluta del campo non è identificabile: conserva il volume.
    radius = float(np.linalg.det(A)**(-1/3))
    if not 5 < radius < 200 or np.linalg.norm(b) > 500:
        raise ValueError('Campo/offset fuori limiti: controlla eventuali magneti vicini.')
    correction = A*radius
    return dict(offset=b.tolist(), matrix=correction.tolist(), radius=radius,
                quality=dict(samples=len(p), sectors=sectors, inlier_fraction=float(np.mean(all_errors < .09)),
                             holdout_rms=float(np.sqrt(np.mean(errors[good]**2))),
                             condition=float(np.linalg.cond(correction))))


class ProfileStore:
    def __init__(self, directory):
        self.directory = Path(directory)

    def path(self, device, demo):
        if (not device or not re.fullmatch(r'(?:[0-9A-F]{2}:){5}[0-9A-F]{2}|SIMULATED', device)
                or (device == 'SIMULATED' and not demo)):
            raise ValueError('Identità della scheda non disponibile.')
        return self.directory / ('demo' if demo else 'reale') / (device.replace(':', '-')+'.json')

    @staticmethod
    def validate(data, device, demo):
        if not isinstance(data, dict):
            raise ValueError('Profilo danneggiato: formato non valido.')
        if (data.get('schema') != SCHEMA or data.get('mapping') != MAPPING or
                data.get('device') != device or data.get('demo') is not demo or data.get('units') != 'uT'):
            raise ValueError('Profilo incompatibile con scheda, modalità o versione.')
        b, A = np.asarray(data['offset'], float), np.asarray(data['matrix'], float)
        if b.shape != (3,) or A.shape != (3, 3) or not np.isfinite(b).all() or not np.isfinite(A).all():
            raise ValueError('Profilo con valori non validi.')
        if not np.allclose(A, A.T, atol=1e-9) or np.linalg.eigvalsh(A).min() <= 0 or np.linalg.cond(A) > 5:
            raise ValueError('Matrice del profilo non SPD o instabile.')
        if np.linalg.norm(b) > 500 or not np.isclose(np.linalg.det(A), 1, rtol=.01):
            raise ValueError('Scala/offset del profilo fuori limite.')
        q = data['quality']
        if (not 180 <= q['samples'] <= 1200 or not 20 <= q['sectors'] <= 26 or
                not .85 <= q['inlier_fraction'] <= 1 or not 0 <= q['holdout_rms'] <= .025 or
                not 5 < data['radius'] < 200):
            raise ValueError('Qualità del profilo insufficiente.')
        return data

    def load(self, device, demo):
        p = self.path(device, demo)
        if not p.exists():
            return None
        if p.stat().st_size > 16000:
            raise ValueError('Profilo troppo grande.')
        try:
            return self.validate(json.loads(p.read_text()), device, demo)
        except (KeyError, TypeError, OverflowError) as exc:
            raise ValueError('Profilo danneggiato.') from exc

    def save(self, profile, device, demo):
        data = {**profile, 'schema': SCHEMA, 'mapping': MAPPING, 'device': device,
                'demo': demo, 'units': 'uT'}
        self.validate(data, device, demo)
        p = self.path(device, demo)
        p.parent.mkdir(parents=True, exist_ok=True)
        fd, temp = tempfile.mkstemp(dir=p.parent, prefix='.profilo-')
        try:
            with os.fdopen(fd, 'w') as stream:
                json.dump(data, stream, allow_nan=False)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temp, p)
        finally:
            if os.path.exists(temp):
                os.unlink(temp)
        return data


class Calibration:
    GYRO_SECONDS = 4.0
    STALE = .5

    def __init__(self, store, demo=False, device=None):
        self.store, self.demo = store, demo
        self.device = None
        self.profile = None
        self.bias = None
        self.phase = 'attesa'
        self.message = 'Connetti la scheda dalla GUI Bluetooth di Ubuntu.'
        self.notice = ''
        self.last = None
        self.started = None
        self.samples = []
        self.window = []
        self.elapsed = 0.
        self.last_gyro = None
        self.last_mag = None
        self.mag_keys = set()
        self.rejected = 0
        self.generation = 0
        self.failed_stage = 'gyro'
        if device:
            self.bind(device)

    def bind(self, device):
        device = device.upper()
        if device == self.device:
            return
        self.store.path(device, self.demo)  # validazione identità prima di cambiare stato
        self.device, self.profile, self.bias = device, None, None
        self.last = None
        self.samples, self.mag_keys, self.last_mag = [], set(), None
        self.notice = ''
        try:
            self.profile = self.store.load(device, self.demo)
        except (ValueError, OSError) as exc:
            self.notice = f'Profilo ignorato: {exc}'
        self.start_gyro()

    def start_gyro(self):
        self.phase = 'gyro'
        self.failed_stage = 'gyro'
        self.window, self.elapsed, self.last_gyro, self.started = [], 0., None, None
        self.generation += 1
        self.message = '1 · Appoggia la scheda su un piano stabile e lasciala ferma per 4 secondi.'

    def interrupt(self, message):
        self.last = None
        if self.phase == 'gyro':
            self.window, self.elapsed, self.last_gyro = [], 0., None
        elif self.phase == 'mag':
            self.samples, self.mag_keys = [], set()
            self.phase, self.failed_stage = 'errore', 'mag'
        self.message = message

    @property
    def allowed(self):
        return self.phase in ('riferimento', 'pronto') and self.bias is not None and self.profile is not None

    def corrected(self, g, m):
        gyro = g if self.bias is None else g-self.bias
        mag = m if self.profile is None or np.linalg.norm(m) < 1e-9 else np.asarray(self.profile['matrix']) @ (m-np.asarray(self.profile['offset']))
        return gyro, mag

    def feed(self, a, g, m, now):
        if self.device is None:
            return
        if not np.isfinite(np.r_[a, g, m]).all():
            self.interrupt('Campione non valido: attendi nuovi dati.')
            return
        if self.last is not None and now-self.last > self.STALE:
            self.interrupt('Dati interrotti: la finestra è stata azzerata. Attendi nuovi campioni.')
        self.last = now
        if self.started is None:
            self.started = now
        if self.phase in ('gyro', 'mag') and now-self.started > 180:
            self.failed_stage = self.phase
            self.phase = 'errore'
            self.message = 'Tempo massimo di 3 minuti raggiunto senza qualità sufficiente. Riprova o annulla.'
            return
        if self.phase == 'gyro':
            self._gyro(a, g, now)
        elif self.phase == 'mag':
            self._mag(m, now)

    def _gyro(self, a, g, now):
        # Usa varianza, non soglia stretta sulla media: permette bias non nullo.
        raw = np.r_[a, g]
        if self.last_gyro is not None:
            if now-self.last_gyro[0] < .019:
                return
            if np.array_equal(raw, self.last_gyro[1]):
                if now-self.last_gyro[0] > self.STALE:
                    self.window, self.elapsed = [], 0.
                    self.message = 'Letture identiche troppo a lungo: attendo campioni nuovi, countdown fermo.'
                return
        if abs(np.linalg.norm(a)-9.80665) > .8 or np.linalg.norm(g) > np.radians(15):
            self.window, self.elapsed = [], 0.
            self.last_gyro = (now, raw.copy())
            self.message = 'Movimento rilevato: appoggia la scheda; countdown azzerato.'
            return
        if self.window:
            rows = np.array([row[1] for row in self.window]+[raw])
            moving = (np.max(np.std(rows[:, :3], axis=0)) > .10 or
                      np.max(np.std(rows[:, 3:], axis=0)) > np.radians(.25) or
                      np.linalg.norm(a-rows[0, :3]) > .35 or
                      np.linalg.norm(g-np.median(rows[:, 3:], axis=0)) > np.radians(.8))
            if moving:
                self.window, self.elapsed = [], 0.
                self.last_gyro = (now, raw.copy())
                self.message = 'Movimento o vibrazioni rilevati: finestra azzerata. Non toccare il piano.'
                return
        if self.last_gyro is not None and self.window:
            self.elapsed += min(now-self.last_gyro[0], .05)
        self.last_gyro = (now, raw.copy())
        self.window.append((now, raw.copy()))
        self.message = f'1 · Scheda ferma: mancano {max(0, self.GYRO_SECONDS-self.elapsed):.1f} s validi.'
        if self.elapsed >= self.GYRO_SECONDS and len(self.window) >= 100:
            self.bias = np.mean([r[1][3:] for r in self.window], axis=0)
            self.window = []
            self.phase = 'riferimento' if self.profile else 'mag_pronto'
            self.generation += 1
            self.message = ('Bias gyro acquisito; profilo magnetico salvato in uso. Tieni ferma la scheda per il riferimento.'
                            if self.profile else 'Bias gyro acquisito. Ora avvia la calibrazione del magnetometro.')

    def _mag(self, m, now):
        if self.last_mag is not None and now-self.last_mag < .095:
            return
        self.last_mag = now
        if not 1 < np.linalg.norm(m) < 500:
            self.rejected += 1
            return
        key = tuple(np.round(m/.2).astype(int))
        if key in self.mag_keys or (self.samples and np.min(np.linalg.norm(np.asarray(self.samples)-m, axis=1)) < .2):
            return
        self.mag_keys.add(key)
        self.samples.append(m.copy())
        sectors, missing = coverage(self.samples)
        self.message = ('2 · Ruota lentamente su tutti e tre gli assi e ribalta la scheda. '
                        + ('Esplora ancora: '+', '.join(missing) if missing else 'Continua a variare inclinazione; poi verifica la raccolta.'))
        if len(self.samples) >= 1200:
            self.phase, self.failed_stage = 'errore', 'mag'
            self.message = 'Limite di 1200 campioni raggiunto. Premi Verifica raccolta oppure Riprova.'

    def action(self, name, now, connected):
        if name == 'annulla':
            self.samples, self.window, self.elapsed = [], [], 0.
            self.phase = 'riferimento' if self.bias is not None and self.profile else 'annullato'
            self.generation += 1
            self.message = 'Procedura annullata; profilo precedente conservato. '+('Tieni ferma la scheda.' if self.allowed else 'Premi Riprova per completare la calibrazione.')
            return
        if not connected or self.last is None or now-self.last > self.STALE:
            raise ValueError('Nessun dato recente: connetti la scheda e attendi lo streaming.')
        if name == 'gyro':
            self.start_gyro()
        elif name == 'mag' or (name == 'riprova' and self.bias is not None and self.failed_stage != 'gyro'):
            if self.bias is None:
                raise ValueError('Completa prima la calibrazione del giroscopio.')
            self.phase, self.samples, self.mag_keys = 'mag', [], set()
            self.failed_stage = 'mag'
            self.last_mag, self.started, self.rejected = None, now, 0
            self.generation += 1
            self.message = '2 · Lontano da metalli e magneti, ruota lentamente su X, Y, Z e ribalta la scheda.'
        elif name == 'riprova':
            self.start_gyro()
        elif name == 'verifica':
            if not (self.phase == 'mag' or self.phase == 'errore' and self.failed_stage == 'mag') or not self.samples:
                raise ValueError('Avvia prima una raccolta magnetica.')
            try:
                candidate = fit_magnetometer(self.samples)
                saved = self.store.save(candidate, self.device, self.demo)
            except (ValueError, OSError, np.linalg.LinAlgError) as exc:
                self.phase, self.failed_stage = 'errore', 'mag'
                self.message = f'Calibrazione non accettata: {exc} Il profilo precedente è conservato.'
                return
            self.profile = saved
            self.phase = 'riferimento'
            self.generation += 1
            self.message = 'Calibrazione accettata e salvata. Appoggia la scheda e tienila ferma per il riferimento iniziale.'
        else:
            raise ValueError('Comando di calibrazione sconosciuto.')

    def snapshot(self, now, connected):
        fresh = connected and self.last is not None and now-self.last <= self.STALE
        if not fresh and self.last is not None:
            self.interrupt('Dati assenti o scaduti: connetti la scheda e attendi nuovi campioni. Nessun avanzamento.')
        sectors, missing = coverage(self.samples)
        return dict(phase=self.phase, message=self.message if fresh else 'In attesa di dati BLE recenti. Nessun avanzamento.',
                    notice=self.notice, fresh=fresh, demo=self.demo, device=self.device,
                    seconds_remaining=round(max(0, self.GYRO_SECONDS-self.elapsed), 1),
                    progress=min(1, self.elapsed/self.GYRO_SECONDS) if self.phase == 'gyro' else min(1, len(self.samples)/180),
                    samples=len(self.samples), sectors=sectors, sectors_total=26, missing=missing,
                    rejected=self.rejected, profile=bool(self.profile),
                    bias_dps=None if self.bias is None else np.degrees(self.bias).tolist(),
                    quality=None if self.profile is None else self.profile['quality'])
