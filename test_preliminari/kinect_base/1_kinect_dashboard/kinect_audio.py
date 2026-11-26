"""Kinect v1 microphone array capture + GCC-PHAT sound direction estimation.

On Linux the Kinect mic array appears as a 4-channel 16 kHz USB audio device
(after the firmware upload done by the ``kinect-audio-setup`` package).

Mic positions along the sensor bar (metres, from the Kinect SDK docs):
    mic0 = -0.113, mic1 = +0.036, mic2 = +0.076, mic3 = +0.113
Positive angle = sound coming from the mic3 side. If the arrow points the wrong
way on your unit, use the "invert" toggle in the UI (channel order can differ).
"""
import contextlib
import os
import threading
import time

import numpy as np

SPEED_OF_SOUND = 343.0
MIC_X = np.array([-0.113, 0.036, 0.076, 0.113])
# (sig, ref) pairs; first is the outermost pair (0-3, 22.6 cm)
PAIRS_4CH = ((0, 3), (0, 2), (1, 3))
MAX_ANGLE = 50.0
BAND_HZ = (300.0, 5000.0)


def gcc_phat(sig, ref, fs, max_tau, interp=16, band=BAND_HZ):
    """Return (tau [s], quality) of ``sig`` relative to ``ref``.

    tau > 0 means ``sig`` is delayed with respect to ``ref``. ``quality`` is the
    peak-to-mean ratio of the PHAT cross-correlation (higher = sharper peak).
    """
    n = sig.size + ref.size
    SIG = np.fft.rfft(sig, n=n)
    REF = np.fft.rfft(ref, n=n)
    R = SIG * np.conj(REF)
    R /= np.abs(R) + 1e-12
    if band is not None:
        freqs = np.fft.rfftfreq(n, 1.0 / fs)
        R[(freqs < band[0]) | (freqs > band[1])] = 0
    cc = np.fft.irfft(R, n=interp * n)
    max_shift = max(1, min(int(interp * fs * max_tau), interp * n // 2))
    cc = np.concatenate((cc[-max_shift:], cc[:max_shift + 1]))
    acc = np.abs(cc)
    peak = int(np.argmax(acc))
    quality = float(acc[peak] / (np.mean(acc) + 1e-12))
    return (peak - max_shift) / float(interp * fs), quality


def estimate_direction(block, fs, pairs, mic_x):
    """Return (angle_deg, quality) for a (samples x channels) float block."""
    angles, weights, qualities = [], [], []
    for a, b in pairs:
        d = abs(mic_x[b] - mic_x[a])
        tau, q = gcc_phat(block[:, a], block[:, b], fs, max_tau=d / SPEED_OF_SOUND * 1.1)
        # sig=a delayed vs ref=b  ->  source is on b's side (b has larger x)
        s = np.clip(tau * SPEED_OF_SOUND / d, -1.0, 1.0)
        if mic_x[b] < mic_x[a]:
            s = -s
        angles.append(np.degrees(np.arcsin(s)))
        weights.append(d * q)
        qualities.append(q)
    w = np.asarray(weights)
    angle = float(np.sum(np.asarray(angles) * w) / (np.sum(w) + 1e-12))
    return angle, float(np.mean(qualities))


KINECT_NAMES = ("kinect", "xbox nui", "nui audio", "microsoft xbox")
NOT_FOUND_MSG = (
    "Kinect microphone array not found. The Kinect v1 mic needs its firmware uploaded: "
    "sudo apt install kinect-audio-setup (if it fails on the MD5 check see README), "
    "then unplug/replug the Kinect and check 'arecord -l' shows 'Kinect USB Audio'. "
    "List devices with: python app.py --list-audio"
)


@contextlib.contextmanager
def quiet_stderr():
    """Temporarily point fd 2 at /dev/null (PortAudio/ALSA/JACK probing noise)."""
    try:
        saved = os.dup(2)
    except OSError:
        yield
        return
    devnull = os.open(os.devnull, os.O_WRONLY)
    try:
        os.dup2(devnull, 2)
        yield
    finally:
        os.dup2(saved, 2)
        os.close(saved)
        os.close(devnull)


def input_devices(pa):
    out = []
    for i in range(pa.get_device_count()):
        info = pa.get_device_info_by_index(i)
        if int(info.get("maxInputChannels", 0)) >= 1:
            out.append(info)
    return out


def find_device(pa, hint=None):
    """Pick the Kinect mic array: explicit hint > Kinect-named device > any raw 4-channel device."""
    devs = input_devices(pa)
    if hint is not None:
        h = str(hint)
        for info in devs:
            if (h.isdigit() and int(h) == int(info["index"])) or h.lower() in str(info["name"]).lower():
                return info
        return None
    named = [d for d in devs if any(k in str(d["name"]).lower() for k in KINECT_NAMES)]
    if named:
        # prefer the raw hw: entry exposing 4 channels over plugin wrappers
        named.sort(key=lambda d: (int(d["maxInputChannels"]) != 4, "hw:" not in str(d["name"])))
        return named[0]
    four = [d for d in devs if int(d["maxInputChannels"]) == 4 and "hw:" in str(d["name"])]
    return four[0] if four else None


def list_audio_devices():
    try:
        import pyaudio
    except ImportError:
        print("pyaudio not installed: sudo apt install portaudio19-dev && pip install pyaudio")
        return
    with quiet_stderr():
        pa = pyaudio.PyAudio()
    try:
        with quiet_stderr():
            devs = input_devices(pa)
            pick = find_device(pa)
        print("idx  ch   rate   name")
        for d in devs:
            mark = "  <== auto" if pick is not None and d["index"] == pick["index"] else ""
            print(f"{int(d['index']):3d}  {int(d['maxInputChannels']):2d}  {int(d['defaultSampleRate']):6d}  {d['name']}{mark}")
        if pick is None:
            print("\n" + NOT_FOUND_MSG)
    finally:
        pa.terminate()


class MicArray(threading.Thread):
    """Background capture thread that calls ``callback(dict)`` for every block."""

    def __init__(self, callback, device=None, rate=16000, chunk=512, window_chunks=2,
                 noise_gate_db=-55.0, on_status=None):
        super().__init__(name="mic-array", daemon=True)
        self.callback = callback
        self.on_status = on_status or (lambda s: None)
        self.device_hint = device
        self.rate = rate
        self.chunk = chunk
        self.window = chunk * window_chunks
        self.noise_gate_db = noise_gate_db
        self.status = {"available": False, "device": None, "channels": 0, "rate": rate, "error": None}
        self.fps = 0.0
        self._running = True
        self._angle = 0.0

    def stop(self):
        self._running = False

    # -------------------------------------------------------------- device
    def _find_device(self, pa):
        return find_device(pa, self.device_hint)

    def _open(self, pa, pyaudio, info):
        channels = min(4, int(info["maxInputChannels"]))
        rates = [self.rate, int(info.get("defaultSampleRate", self.rate))]
        last_exc = None
        for rate in dict.fromkeys(rates):
            try:
                stream = pa.open(format=pyaudio.paInt16, channels=channels, rate=rate, input=True,
                                 input_device_index=int(info["index"]), frames_per_buffer=self.chunk)
                return stream, channels, rate
            except Exception as exc:  # unsupported rate/format -> try next
                last_exc = exc
        raise RuntimeError(f"cannot open '{info['name']}': {last_exc}")

    def _set_status(self, **kw):
        self.status.update(kw)
        self.on_status(dict(self.status))

    # -------------------------------------------------------------- loop
    def run(self):
        try:
            import pyaudio
        except ImportError:
            self._set_status(error="pyaudio not installed (sudo apt install portaudio19-dev && pip install pyaudio)")
            print(f"[audio] {self.status['error']}")
            return
        while self._running:
            # PortAudio only rescans devices on re-init, so re-create it on each retry,
            # but with stderr silenced (ALSA/JACK probing spam).
            with quiet_stderr():
                pa = pyaudio.PyAudio()
            stream = None
            try:
                with quiet_stderr():
                    info = self._find_device(pa)
                    if info is None:
                        raise RuntimeError(NOT_FOUND_MSG)
                    stream, channels, rate = self._open(pa, pyaudio, info)
                self._set_status(available=True, device=info["name"], channels=channels, rate=rate, error=None)
                print(f"[audio] capturing '{info['name']}' {channels} ch @ {rate} Hz")
                self._capture(stream, channels, rate)
            except Exception as exc:
                if self.status.get("error") != str(exc):
                    print(f"[audio] {exc}")
                self._set_status(available=False, error=str(exc))
                for _ in range(50):  # retry every 5 s, stop promptly
                    if not self._running:
                        break
                    time.sleep(0.1)
            finally:
                with quiet_stderr():
                    if stream is not None:
                        try:
                            stream.stop_stream()
                            stream.close()
                        except Exception:
                            pass
                    pa.terminate()

    def _capture(self, stream, channels, rate):
        if channels >= 4:
            pairs, mic_x = PAIRS_4CH, MIC_X
        elif channels >= 2:
            pairs, mic_x = ((0, 1),), np.array([MIC_X[0], MIC_X[3]])
        else:
            pairs, mic_x = (), MIC_X
        buf = np.zeros((self.window, channels), np.float32)
        count, t0 = 0, time.monotonic()
        while self._running:
            data = stream.read(self.chunk, exception_on_overflow=False)
            block = np.frombuffer(data, dtype=np.int16).reshape(-1, channels).astype(np.float32) / 32768.0
            n = block.shape[0]
            buf = np.roll(buf, -n, axis=0)
            buf[-n:] = block

            levels = np.sqrt(np.mean(block ** 2, axis=0) + 1e-12)
            rms_db = float(20 * np.log10(np.mean(levels)))
            confidence = 0.0
            if pairs and rms_db > self.noise_gate_db:
                x = buf - buf.mean(axis=0)
                angle, quality = estimate_direction(x, rate, pairs, mic_x)
                # peak-to-mean quality: ~2.4 for diffuse noise, ~5+ for a single clear source
                q_conf = float(np.clip((quality - 2.6) / 2.4, 0.0, 1.0))
                e_conf = float(np.clip((rms_db - self.noise_gate_db) / 25.0, 0.0, 1.0))
                confidence = q_conf * e_conf
                alpha = 0.15 + 0.6 * confidence  # confident readings move the estimate faster
                self._angle += alpha * (angle - self._angle)
            count += 1
            now = time.monotonic()
            if now - t0 >= 1.0:
                self.fps, count, t0 = count / (now - t0), 0, now
            self.callback({
                "angle": round(float(np.clip(self._angle, -MAX_ANGLE, MAX_ANGLE)), 1),
                "confidence": round(confidence, 3),
                "rms_db": round(rms_db, 1),
                "levels_db": [round(float(20 * np.log10(v)), 1) for v in levels],
                "hz": round(self.fps, 1),
            })
