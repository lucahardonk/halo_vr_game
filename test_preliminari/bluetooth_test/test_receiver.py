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


async def add_match(bus: MessageBus, path: str) -> None:
    rule = (f"type='signal',sender='{BLUEZ}',interface='{PROPS_IFACE}',"
            f"member='PropertiesChanged',path='{path}'")
    await call(
        bus, destination="org.freedesktop.DBus", path="/org/freedesktop/DBus",
        interface="org.freedesktop.DBus", member="AddMatch", signature="s", body=[rule],
    )


async def run_session(bus: MessageBus, stats: FrameStats, address: str | None) -> None:
    dev_path, dev_props, char_path, char_props = await wait_for_device(bus, address)
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
            print(format_frame(stats.tick(), struct.unpack(PAYLOAD_FMT, data)), flush=True)
        elif msg.path == dev_path and iface == DEVICE_IFACE and "Connected" in changed:
            if not changed["Connected"].value:
                disconnected.set()
        return None

    await add_match(bus, char_path)
    await add_match(bus, dev_path)
    bus.add_message_handler(handler)
    try:
        await call(bus, destination=BLUEZ, path=char_path, interface=CHAR_IFACE, member="StartNotify")
        print("Streaming avviato — Ctrl+C per uscire.\n", flush=True)
        await disconnected.wait()
        print("\nDispositivo disconnesso: riconnettilo dalla GUI, resto in attesa...")
    finally:
        bus.remove_message_handler(handler)
        if not disconnected.is_set():
            try:  # lascia la connessione della GUI aperta, ferma solo le notifiche
                await call(bus, destination=BLUEZ, path=char_path, interface=CHAR_IFACE, member="StopNotify")
            except Exception:
                pass


async def main(address: str | None) -> None:
    bus = await MessageBus(bus_type=BusType.SYSTEM).connect()
    stats = FrameStats()
    t_start = time.perf_counter()
    try:
        while True:
            try:
                await run_session(bus, stats, address)
            except RuntimeError as exc:
                print(f"Errore BlueZ: {exc} — riprovo tra 2s")
            await asyncio.sleep(2.0)
    finally:
        elapsed = time.perf_counter() - t_start
        avg = stats.total / elapsed if elapsed > 0 else 0.0
        print(f"\nFrame ricevuti: {stats.total} | scartati: {stats.bad} | "
              f"durata: {elapsed:.1f}s | media: {avg:.1f} Hz")
        bus.disconnect()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Ricevitore BLE IMU_BTN (connessione gestita dalla GUI di Ubuntu)")
    parser.add_argument("--address", help="indirizzo MAC della scheda (default: cerca per nome IMU_BTN)")
    args = parser.parse_args()
    try:
        asyncio.run(main(args.address))
    except KeyboardInterrupt:
        print("Interrotto dall'utente. Ciao!")
