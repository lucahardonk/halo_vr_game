"""Test motore di inclinazione e accelerometro della Kinect v1 (libfreenect, full context API)."""
import argparse
import ctypes as C
import time
from ctypes.util import find_library

FREENECT_DEVICE_MOTOR = 0x01
STATI_MOTORE = {0: "stopped", 1: "at_limit", 4: "moving"}
SEQUENZA_TILT = ((0, 2), (10, 3), (-10, 3), (0, 2))  # (gradi, attesa in secondi)


class RawTiltState(C.Structure):
    # Layout di freenect_raw_tilt_state in libfreenect.h; tilt_status è un enum (int).
    # Angolo e stato vengono comunque letti con le funzioni di libreria, non dai campi.
    _fields_ = [
        ("accelerometer_x", C.c_int16),
        ("accelerometer_y", C.c_int16),
        ("accelerometer_z", C.c_int16),
        ("tilt_angle", C.c_int8),
        ("tilt_status", C.c_int),
    ]


def carica_libreria():
    name = find_library("freenect")
    if not name:
        raise RuntimeError("Libreria libfreenect non trovata: sudo apt install libfreenect-dev")
    lib = C.CDLL(name)
    p = C.c_void_p
    signatures = {
        "freenect_init": ([C.POINTER(p), p], C.c_int),
        "freenect_select_subdevices": ([p, C.c_int], None),
        "freenect_num_devices": ([p], C.c_int),
        "freenect_open_device": ([p, C.POINTER(p), C.c_int], C.c_int),
        "freenect_close_device": ([p], C.c_int),
        "freenect_shutdown": ([p], C.c_int),
        "freenect_update_tilt_state": ([p], C.c_int),
        "freenect_get_tilt_state": ([p], C.POINTER(RawTiltState)),
        "freenect_get_tilt_degs": ([C.POINTER(RawTiltState)], C.c_double),
        "freenect_get_tilt_status": ([C.POINTER(RawTiltState)], C.c_int),
        "freenect_set_tilt_degs": ([p, C.c_double], C.c_int),
    }
    for fn_name, (argtypes, restype) in signatures.items():
        fn = getattr(lib, fn_name)
        fn.argtypes = argtypes
        fn.restype = restype
    return lib


class MotoreKinect:
    def __init__(self, index=0):
        self.lib = carica_libreria()
        self.ctx = C.c_void_p()
        self.dev = C.c_void_p()
        if self.lib.freenect_init(C.byref(self.ctx), None) < 0:
            raise RuntimeError("freenect_init fallita")
        self.lib.freenect_select_subdevices(self.ctx, FREENECT_DEVICE_MOTOR)
        try:
            if self.lib.freenect_num_devices(self.ctx) < 1:
                raise RuntimeError("Nessuna Kinect trovata (controlla alimentatore e USB: lsusb -d 045e:)")
            if self.lib.freenect_open_device(self.ctx, C.byref(self.dev), index) < 0:
                raise RuntimeError(
                    "Impossibile aprire il motore della Kinect "
                    "(permessi udev? un altro programma la sta usando?)"
                )
        except Exception:
            self.lib.freenect_shutdown(self.ctx)
            raise

    def leggi(self):
        if self.lib.freenect_update_tilt_state(self.dev) < 0:
            raise RuntimeError("freenect_update_tilt_state fallita")
        state = self.lib.freenect_get_tilt_state(self.dev)
        s = state.contents
        angolo = self.lib.freenect_get_tilt_degs(state)
        stato = self.lib.freenect_get_tilt_status(state)
        return s.accelerometer_x, s.accelerometer_y, s.accelerometer_z, angolo, stato

    def inclina(self, gradi):
        if self.lib.freenect_set_tilt_degs(self.dev, float(gradi)) < 0:
            raise RuntimeError(f"freenect_set_tilt_degs({gradi}) fallita")

    def chiudi(self):
        if self.dev:
            self.lib.freenect_close_device(self.dev)
            self.dev = C.c_void_p()
        if self.ctx:
            self.lib.freenect_shutdown(self.ctx)
            self.ctx = C.c_void_p()


def stampa(motore, etichetta=""):
    x, y, z, angolo, stato = motore.leggi()
    nome = STATI_MOTORE.get(stato, f"sconosciuto({stato})")
    print(f"{etichetta:>12} | accel raw x={x:5d} y={y:5d} z={z:5d} | angolo={angolo:6.1f} deg | motore={nome}")


def attendi_e_stampa(motore, secondi, etichetta):
    fine = time.monotonic() + secondi
    while True:
        stampa(motore, etichetta)
        rimanente = fine - time.monotonic()
        if rimanente <= 0:
            break
        time.sleep(min(1.0, rimanente))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--no-tilt", action="store_true",
                        help="legge solo l'accelerometro, senza muovere il motore")
    args = parser.parse_args()

    motore = MotoreKinect()
    mosso = False
    try:
        if args.no_tilt:
            print("Solo lettura accelerometro (Ctrl+C per uscire)")
            while True:
                stampa(motore, "lettura")
                time.sleep(1.0)
        else:
            print("Sequenza tilt: 0 -> +10 -> -10 -> 0 gradi (Ctrl+C per interrompere)")
            for gradi, attesa in SEQUENZA_TILT:
                motore.inclina(gradi)
                mosso = True
                attendi_e_stampa(motore, attesa, f"target {gradi:+d}")
            mosso = False
            print("Sequenza completata.")
    except KeyboardInterrupt:
        print("\nInterrotto dall'utente.")
        if mosso:
            print("Riporto la Kinect a 0 gradi...")
            try:
                motore.inclina(0)
                attendi_e_stampa(motore, 2, "ritorno 0")
            except (RuntimeError, KeyboardInterrupt) as exc:
                print(f"Ritorno a 0 non completato: {exc}")
    finally:
        motore.chiudi()


if __name__ == "__main__":
    main()
