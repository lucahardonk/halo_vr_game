"""Silence the noisy C libraries (libfreenect, ALSA, JACK) at file-descriptor level.

libfreenect_sync offers no log-level control, and ALSA/JACK print directly to
fd 2 from C code, so Python's sys.stderr cannot catch them. We replace fd 2 with
a pipe, read it in a daemon thread and forward only the lines that are not
known noise. Truncated-USB-packet messages from libfreenect are counted and
summarised every few seconds instead of printed one by one.
"""
import ctypes
import os
import re
import sys
import threading
import time

NOISE = re.compile(
    r"^\[Stream [0-9a-fA-F]+\]"            # libfreenect packet loss / resync
    r"|^ALSA lib "
    r"|jack server|JackShm|Cannot connect to server|jackdmp|JackServer"
    r"|^send_cmd: |^write_register: |^read_register: |^Error: Invalid index \[\d+\]"
    r"|^Expression 'ret' failed|^Expression 'alsa_|^Expression 'AlsaOpen"
)
USB_DROP = re.compile(r"^\[Stream [0-9a-fA-F]+\] (Expected|Invalid|Lost|Inconsistent|.*resync)", re.I)
# libfreenect prints these without a trailing newline while no Kinect is plugged in
INLINE_NOISE = re.compile(r"Failed to open (camera|motor|audio) subd+evice or it is not disabled\.")
CTRL_FAIL = re.compile(r"^(send_cmd|write_register|read_register):")

stats = {"usb_dropped_total": 0, "usb_dropped_rate": 0.0, "ctrl_errors_total": 0}
_lock = threading.Lock()
_installed = False


def silence_alsa():
    """Install a no-op ALSA error handler (stops 'ALSA lib ...' at the source)."""
    try:
        lib = ctypes.cdll.LoadLibrary("libasound.so.2")
    except OSError:
        return
    handler_t = ctypes.CFUNCTYPE(None, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p)
    silence_alsa._handler = handler_t(lambda *a: None)  # keep a reference alive
    try:
        lib.snd_lib_error_set_handler(silence_alsa._handler)
    except Exception:
        pass


def install(summary_period=10.0, verbose=False):
    """Redirect fd 2 through the filter. Safe to call once; no-op if verbose."""
    global _installed
    silence_alsa()
    os.environ.setdefault("JACK_NO_START_SERVER", "1")
    os.environ.setdefault("JACK_NO_AUDIO_RESERVATION", "1")
    if verbose or _installed:
        return
    _installed = True
    real_fd = os.dup(2)
    real = os.fdopen(real_fd, "w", buffering=1, errors="replace")
    r, w = os.pipe()
    os.dup2(w, 2)
    os.close(w)
    sys.stderr = os.fdopen(2, "w", buffering=1, errors="replace")

    def reader():
        with os.fdopen(r, "r", errors="replace") as pipe:
            for line in pipe:
                text = line.rstrip("\n")
                if INLINE_NOISE.search(text):
                    text = INLINE_NOISE.sub("", text)
                    if not text.strip():
                        continue
                    line = text + "\n"
                if NOISE.search(text):
                    with _lock:
                        if USB_DROP.search(text):
                            stats["usb_dropped_total"] += 1
                        elif CTRL_FAIL.search(text):
                            stats["ctrl_errors_total"] += 1
                    continue
                try:
                    real.write(line)
                except Exception:
                    pass

    def summary():
        last = 0
        while True:
            time.sleep(summary_period)
            with _lock:
                total = stats["usb_dropped_total"]
                stats["usb_dropped_rate"] = round((total - last) / summary_period, 1)
            n = total - last
            last = total
            if n:
                try:
                    real.write(f"[usb] {n} pacchetti video troncati negli ultimi {summary_period:.0f}s "
                               "(Kinect v1 su USB3: prova un hub USB 2.0 alimentato o un'altra porta; "
                               "--verbose per il log completo)\n")
                except Exception:
                    pass

    threading.Thread(target=reader, name="stderr-filter", daemon=True).start()
    threading.Thread(target=summary, name="stderr-summary", daemon=True).start()


def snapshot():
    with _lock:
        return dict(stats)
