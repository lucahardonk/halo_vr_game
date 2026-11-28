#!/usr/bin/env python3
"""Ricevitore BLE di test per bluetooth_test.ino (Nano 33 BLE Sense Rev2).

Si connette a "IMU_BTN", si iscrive alla notify e stampa ogni frame:
pulsanti (7 bit), accelerometro, giroscopio, magnetometro e frequenza (Hz, finestra 1 s).
"""
import argparse
import asyncio
import struct
import sys
import time
from collections import deque

from bleak import BleakClient, BleakScanner

DEVICE_NAME = "IMU_BTN"
SERVICE_UUID = "19b10000-e8f2-537e-4f6c-d104768a1214"
DATA_UUID = "19b10001-e8f2-537e-4f6c-d104768a1214"

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


async def find_device(timeout: float):
    print(f"Scansione BLE per '{DEVICE_NAME}' (timeout {timeout:.0f}s)...")
    return await BleakScanner.find_device_by_filter(
        lambda d, adv: d.name == DEVICE_NAME
        or adv.local_name == DEVICE_NAME
        or SERVICE_UUID in [u.lower() for u in adv.service_uuids],
        timeout=timeout,
    )


async def run_session(stats: FrameStats, scan_timeout: float) -> None:
    device = await find_device(scan_timeout)
    if device is None:
        print("Dispositivo non trovato, nuovo tentativo...")
        return

    disconnected = asyncio.Event()
    warned_short = False

    def on_disconnect(_client: BleakClient) -> None:
        disconnected.set()

    def on_notify(_sender, data: bytearray) -> None:
        nonlocal warned_short
        if len(data) != PAYLOAD_SIZE:
            stats.bad += 1
            if not warned_short:
                warned_short = True
                print(
                    f"ATTENZIONE: ricevuti {len(data)} byte invece di {PAYLOAD_SIZE}. "
                    "Probabile MTU = 23 (max 20 byte): vedi README, sezione MTU.",
                    file=sys.stderr,
                )
            return
        hz = stats.tick()
        print(format_frame(hz, struct.unpack(PAYLOAD_FMT, data)), flush=True)

    print(f"Connessione a {device.address}...")
    async with BleakClient(device, disconnected_callback=on_disconnect) as client:
        # Su BlueZ (Linux) mtu_size vale 23 finché non lo si legge esplicitamente.
        backend = getattr(client, "_backend", None)
        if hasattr(backend, "_acquire_mtu"):
            try:
                await backend._acquire_mtu()
            except Exception as exc:  # API privata: non deve mai bloccare il test
                print(f"(MTU non leggibile: {exc})")
        print(f"Connesso. MTU = {client.mtu_size} (servono >= {PAYLOAD_SIZE + 3})")

        await client.start_notify(DATA_UUID, on_notify)
        print("Streaming avviato — Ctrl+C per uscire.\n")
        try:
            await disconnected.wait()
            print("\nDispositivo disconnesso.")
        finally:
            if client.is_connected:
                try:
                    await client.stop_notify(DATA_UUID)
                except Exception:
                    pass


async def main(scan_timeout: float) -> None:
    stats = FrameStats()
    t_start = time.perf_counter()
    try:
        while True:  # riconnessione automatica
            try:
                await run_session(stats, scan_timeout)
            except Exception as exc:
                print(f"Errore BLE: {exc!r} — riprovo tra 2s")
            await asyncio.sleep(2.0)
    finally:
        elapsed = time.perf_counter() - t_start
        avg = stats.total / elapsed if elapsed > 0 else 0.0
        print(
            f"\nFrame ricevuti: {stats.total} | scartati: {stats.bad} | "
            f"durata: {elapsed:.1f}s | media: {avg:.1f} Hz"
        )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Ricevitore BLE IMU_BTN")
    parser.add_argument("--scan-timeout", type=float, default=10.0)
    args = parser.parse_args()
    try:
        asyncio.run(main(args.scan_timeout))
    except KeyboardInterrupt:
        print("Interrotto dall'utente. Ciao!")
