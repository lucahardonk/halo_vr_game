#!/usr/bin/env python3
"""Ricevitore BLE di test per bluetooth_test.ino (Nano 33 BLE Sense Rev2) — solo Linux/BlueZ.

La connessione la fai TU dalla GUI Bluetooth di Ubuntu (Impostazioni -> Bluetooth -> IMU_BTN).
Lo script NON scansiona e NON si connette: trova via D-Bus il dispositivo "IMU_BTN" gia'
connesso, attiva le notifiche sulla characteristic dati e stampa ogni frame:
pulsanti (7 bit), accelerometro, giroscopio, magnetometro e frequenza (Hz, finestra 1 s).
"""
import argparse
import asyncio
import struct
import sys
import time
import signal
import math
from pathlib import Path
from calibration import Calibration, ProfileStore

import numpy as np
from orientation import Orientation, from_euler, conjugate, multiply, matrix
from viewer import Viewer
from collections import deque

from dbus_fast import BusType, Message, MessageType
from dbus_fast.aio import MessageBus

DEVICE_NAME = "IMU_BTN"
DATA_UUID = "19b10001-e8f2-537e-4f6c-d104768a1214"

BLUEZ = "org.bluez"
DEVICE_IFACE = "org.bluez.Device1"
CHAR_IFACE = "org.bluez.GattCharacteristic1"
PROPS_IFACE = "org.freedesktop.DBus.Properties"

# <  = little-endian, nessun padding | B = uint8 pulsanti | 9f = 9 x float32
PAYLOAD_FMT = "<B9f"
PAYLOAD_SIZE = struct.calcsize(PAYLOAD_FMT)
assert PAYLOAD_SIZE == 37, PAYLOAD_SIZE
NUM_BUTTONS = 7


class FrameStats:
    """Conteggio frame e frequenza su finestra mobile di 1 secondo."""

    def __init__(self) -> None:
        self.total = 0
        self.bad = 0
        self.stamps: deque[float] = deque()

    def tick(self) -> float:
        now = time.perf_counter()
        self.total += 1
        self.stamps.append(now)
        while self.stamps and now - self.stamps[0] > 1.0:
            self.stamps.popleft()
        return float(len(self.stamps))


def format_frame(hz: float, values: tuple) -> str:
    btn, ax, ay, az, gx, gy, gz, mx, my, mz = values
    # Ordine stringa: D2 a sinistra ... D8 a destra (bit0 = D2)
    btn_str = "".join("1" if btn & (1 << i) else "0" for i in range(NUM_BUTTONS))
    return (
        f"[Hz: {hz:5.1f}] BTN: {btn_str} | "
        f"Accel: ax={ax:+.3f} ay={ay:+.3f} az={az:+.3f} | "
        f"Gyro: gx={gx:+.2f} gy={gy:+.2f} gz={gz:+.2f} | "
        f"Mag: mx={mx:+.1f} my={my:+.1f} mz={mz:+.1f}"
    )


def _val(props: dict, key: str, default=None):
    v = props.get(key)
    return v.value if v is not None else default


async def call(bus: MessageBus, **kwargs) -> Message:
    reply = await bus.call(Message(**kwargs))
    if reply.message_type == MessageType.ERROR:
        raise RuntimeError(f"{reply.error_name}: {reply.body[0] if reply.body else ''}")
    return reply


async def managed_objects(bus: MessageBus) -> dict:
    reply = await call(
        bus, destination=BLUEZ, path="/",
        interface="org.freedesktop.DBus.ObjectManager", member="GetManagedObjects",
    )
    return reply.body[0]


def find_connected_device(objs: dict, address: str | None):
    """Ritorna (path, props) del dispositivo IMU_BTN connesso, oppure (None, props_o_None)."""
    candidate = None
    for path, ifaces in objs.items():
        props = ifaces.get(DEVICE_IFACE)
        if not props:
            continue
        if address:
            match = str(_val(props, "Address", "")).upper() == address.upper()
        else:
            match = DEVICE_NAME in (_val(props, "Name"), _val(props, "Alias"))
        if not match:
            continue
        if _val(props, "Connected", False):
            return path, props
        candidate = props
    return None, candidate


def find_data_char(objs: dict, dev_path: str):
    for path, ifaces in objs.items():
        props = ifaces.get(CHAR_IFACE)
        if props and path.startswith(dev_path + "/") and str(_val(props, "UUID", "")).lower() == DATA_UUID:
            return path, props
    return None, None


async def wait_for_device(bus: MessageBus, address: str | None):
    """Attende che IMU_BTN sia connesso (dalla GUI) e con i servizi GATT risolti."""
    last_msg = ""
    while True:
        objs = await managed_objects(bus)
        dev_path, props = find_connected_device(objs, address)
        if dev_path is None:
            msg = (
                f"'{address or DEVICE_NAME}' noto a BlueZ ma NON connesso: attiva 'Connection' dalla GUI Bluetooth..."
                if props else
                f"'{address or DEVICE_NAME}' non trovato in BlueZ: associalo/connettilo dalla GUI Bluetooth di Ubuntu..."
            )
        elif not _val(props, "ServicesResolved", False):
            msg = "Connesso, attendo la risoluzione dei servizi GATT..."
        else:
            char_path, char_props = find_data_char(objs, dev_path)
            if char_path:
                return dev_path, props, char_path, char_props
            msg = (f"Connesso ma characteristic {DATA_UUID} non trovata: il firmware caricato e' "
                   "bluetooth_test.ino? Prova 'Forget Device' e riassocia.")
        if msg != last_msg:
            print(msg, flush=True)
            last_msg = msg
        await asyncio.sleep(1.0)


async def add_match(bus: MessageBus, path: str, remove=False) -> None:
    rule = (f"type='signal',sender='{BLUEZ}',interface='{PROPS_IFACE}',"
            f"member='PropertiesChanged',path='{path}'")
    await call(
        bus, destination="org.freedesktop.DBus", path="/org/freedesktop/DBus",
        interface="org.freedesktop.DBus", member="RemoveMatch" if remove else "AddMatch", signature="s", body=[rule],
    )


async def run_session(bus: MessageBus, stats: FrameStats, address: str | None, state=None) -> None:
    dev_path, dev_props, char_path, char_props = await wait_for_device(bus, address)
    if state:
        state.bind_device(_val(dev_props, "Address"))
    print(f"Trovato {_val(dev_props, 'Alias')} [{_val(dev_props, 'Address')}] connesso.")
    mtu = _val(char_props, "MTU")
    if mtu is not None:
        print(f"MTU = {mtu} (servono >= {PAYLOAD_SIZE + 3})")

    disconnected = asyncio.Event()
    warned_short = False

    def handler(msg: Message):
        nonlocal warned_short
        if msg.message_type != MessageType.SIGNAL or msg.member != "PropertiesChanged":
            return None
        iface, changed = msg.body[0], msg.body[1]
        if msg.path == char_path and iface == CHAR_IFACE and "Value" in changed:
            data = bytes(changed["Value"].value)
            if len(data) != PAYLOAD_SIZE:
                stats.bad += 1
                if not warned_short:
                    warned_short = True
                    print(f"ATTENZIONE: ricevuti {len(data)} byte invece di {PAYLOAD_SIZE}. "
                          "Probabile MTU = 23 (max 20 byte): vedi README, sezione MTU.", file=sys.stderr)
                return None
            values = struct.unpack(PAYLOAD_FMT, data)
            hz = stats.tick()
            if state:
                state.feed(values)
            print(format_frame(hz, values), flush=True)
        elif msg.path == dev_path and iface == DEVICE_IFACE and "Connected" in changed:
            if not changed["Connected"].value:
                disconnected.set()
        return None

    matches = []
    bus.add_message_handler(handler)
    try:
        for path in (char_path, dev_path):
            await add_match(bus, path)
            matches.append(path)
        await call(bus, destination=BLUEZ, path=char_path, interface=CHAR_IFACE, member="StartNotify")
        if state:
            state.connection(True)
        print("Streaming avviato — Ctrl+C per uscire.\n", flush=True)
        await disconnected.wait()
        print("\nDispositivo disconnesso: riconnettilo dalla GUI, resto in attesa...")
    finally:
        if state:
            state.connection(False)
        bus.remove_message_handler(handler)
        if not disconnected.is_set():
            try:  # lascia la connessione della GUI aperta, ferma solo le notifiche
                await call(bus, destination=BLUEZ, path=char_path, interface=CHAR_IFACE, member="StopNotify")
            except Exception:
                pass
        for path in matches:
            try:
                await add_match(bus, path, remove=True)
            except Exception:
                pass


async def main(address: str | None, state=None) -> None:
    bus = await MessageBus(bus_type=BusType.SYSTEM).connect()
    stats = FrameStats()
    t_start = time.perf_counter()
    try:
        while True:
            try:
                await run_session(bus, stats, address, state)
            except RuntimeError as exc:
                if state:
                    state.connection(False, str(exc))
                print(f"Errore BlueZ: {exc} — riprovo tra 2s")
            await asyncio.sleep(2.0)
    finally:
        elapsed = time.perf_counter() - t_start
        avg = stats.total / elapsed if elapsed > 0 else 0.0
        print(f"\nFrame ricevuti: {stats.total} | scartati: {stats.bad} | "
              f"durata: {elapsed:.1f}s | media: {avg:.1f} Hz")
        bus.disconnect()


async def demo(state):
    """Sensori sintetici, stesso percorso di fusione e stessa stampa del BLE."""
    print("DEMO SINTETICA: nessun dispositivo BLE utilizzato", flush=True)
    state.connection(True)
    stats = FrameStats()
    start = time.monotonic()
    previous = from_euler(0, 0, 0)
    last = start
    held_mag = np.array([24., 0., 39.])
    last_mag = start
    phase_start = start
    previous_phase = None
    try:
        while True:
            now = time.monotonic()
            phase = state.calibration.phase if state.calibration else 'pronto'
            if phase != previous_phase:
                phase_start, previous_phase = now, phase
            t = now-phase_start
            if phase == 'mag':
                q = from_euler(t*.71, 1.5*math.sin(t*.37), t*1.13)
            elif phase == 'pronto':
                q = from_euler(.4*math.sin(t*.7), .25*math.sin(t*.5), t*.22)
            else:
                q = from_euler(0, 0, 0)
            dt = max(now-last, 1e-6)
            delta = multiply(conjugate(previous), q)
            if delta[0] < 0:
                delta = -delta
            n = np.linalg.norm(delta[1:])
            gyro = np.zeros(3) if n < 1e-12 else delta[1:]/n * (2*math.atan2(n, delta[0])/dt)
            acc = matrix(q).T @ [0, 0, 9.80665] + .001*np.sin(now*31)*np.array([1, .5, .3])
            if now-last_mag >= 0.1:
                held_mag = matrix(q).T @ [24., 0., 39.]
                # Minima variazione sintetica rende osservabile l'aggiornamento a riposo.
                held_mag += np.array([1e-4*math.sin(now), 0, 0])
                last_mag = now
            distortion = np.array([[1.2, .12, .05], [.12, .85, -.08], [.05, -.08, 1.05]])
            measured_mag = distortion @ held_mag + [12, -8, 5]
            measured_gyro = np.degrees(gyro) + [1.1, -.7, .45] + .01*math.sin(now*19)
            values = (1 << (int(t) % 7), *acc, *measured_gyro, *(measured_mag*[1, -1, 1]))
            state.feed(values, now)
            print(format_frame(stats.tick(), values), flush=True)
            previous, last = q, now
            await asyncio.sleep(0.01)
    finally:
        state.connection(False)


async def run(args, state):
    task = asyncio.current_task()
    loop = asyncio.get_running_loop()
    loop.add_signal_handler(signal.SIGTERM, task.cancel)
    try:
        await (demo(state) if args.demo else main(args.address, state))
    finally:
        loop.remove_signal_handler(signal.SIGTERM)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Ricevitore BLE IMU_BTN con visualizzatore locale")
    parser.add_argument("--address", help="MAC scheda già connessa dalla GUI Ubuntu")
    parser.add_argument("--demo", action="store_true", help="Sensori SINTETICI, nessun accesso BLE")
    parser.add_argument("--no-web", action="store_true", help="Solo stampa terminale, come prima")
    parser.add_argument("--port", type=int, default=5000, help="Porta loopback (default: 5000)")
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error("La porta deve essere compresa tra 1 e 65535")
    calibration = Calibration(ProfileStore(Path(__file__).resolve().parent / '.calibration'),
                              demo=args.demo, device='SIMULATED' if args.demo else args.address)
    state = Orientation(demo=args.demo, calibration=calibration)
    web = None
    try:
        if not args.no_web:
            web = Viewer(state, args.port)
            web.start()
            print(f"Visualizzatore sul NUC: http://127.0.0.1:{args.port} — nessun reloader", flush=True)
        asyncio.run(run(args, state))
    except (KeyboardInterrupt, asyncio.CancelledError):
        print("Interrotto: notifiche e visualizzatore arrestati, connessione GUI invariata.")
    except Exception as exc:
        print(f"Errore: {exc}", file=sys.stderr)
        sys.exit(1)
    finally:
        if web:
            web.close()
