"""Stream gyro via BLE da IMU_BTN (Nano 33 BLE Sense Rev2), adattato da test_receiver.py.

La connessione la fai TU dalla GUI Bluetooth di Ubuntu: lo script NON scansiona e NON si
connette, trova via D-Bus il dispositivo gia' connesso, attiva le notifiche e per ogni frame
chiama callback(gyro_rad, now, values). Qui interessa SOLO il giroscopio (rad/s); accelerometro
e magnetometro non servono alla fusione gyro+camera.

ImuBle gira in un thread di background con il proprio loop asyncio: start()/stop().
SyntheticImu genera gyro coerente con un moto noto, per i test senza hardware.
"""
import asyncio
import math
import struct
import threading
import time

import numpy as np

import quaternion as Q

DEVICE_NAME = "IMU_BTN"
DATA_UUID = "19b10001-e8f2-537e-4f6c-d104768a1214"

BLUEZ = "org.bluez"
DEVICE_IFACE = "org.bluez.Device1"
CHAR_IFACE = "org.bluez.GattCharacteristic1"
PROPS_IFACE = "org.freedesktop.DBus.Properties"

PAYLOAD_FMT = "<B9f"                 # B pulsanti + 9 float32 (acc, gyro, mag)
PAYLOAD_SIZE = struct.calcsize(PAYLOAD_FMT)
assert PAYLOAD_SIZE == 37, PAYLOAD_SIZE


def rev2_gyro_rad(values):
    """values grezzi (10) -> gyro in rad/s, assi Rev2 (come orientation.rev2_axes: solo radianti)."""
    return np.radians(np.array(values[4:7], dtype=float))


# ----------------------------------------------------------------- BLE reale
class ImuBle:
    def __init__(self, callback, address=None, clock=time.monotonic, on_status=None):
        self.callback = callback
        self.address = address
        self.clock = clock
        self.on_status = on_status or (lambda msg: None)
        self._thread = None
        self._loop = None
        self._stop = threading.Event()
        self.name = "BLE IMU_BTN (D-Bus / BlueZ)"

    # ---- API thread
    def start(self):
        self._thread = threading.Thread(target=self._run, name="imu-ble", daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        loop = self._loop
        if loop is not None:
            loop.call_soon_threadsafe(lambda: None)
        if self._thread is not None:
            self._thread.join(timeout=3.0)

    def _run(self):
        try:
            asyncio.run(self._main())
        except Exception as exc:         # il test non deve morire se BlueZ non c'e'
            self.on_status(f"IMU BLE terminata: {exc}")

    # ---- D-Bus
    async def _main(self):
        from dbus_fast import BusType, Message, MessageType
        from dbus_fast.aio import MessageBus
        self._dbus = (Message, MessageType)
        self._loop = asyncio.get_running_loop()
        bus = await MessageBus(bus_type=BusType.SYSTEM).connect()
        try:
            while not self._stop.is_set():
                try:
                    await self._session(bus)
                except RuntimeError as exc:
                    self.on_status(f"Errore BlueZ: {exc} — riprovo tra 2s")
                await asyncio.sleep(2.0)
        finally:
            bus.disconnect()

    async def _call(self, bus, **kwargs):
        Message, MessageType = self._dbus
        reply = await bus.call(Message(**kwargs))
        if reply.message_type == MessageType.ERROR:
            raise RuntimeError(f"{reply.error_name}: {reply.body[0] if reply.body else ''}")
        return reply

    async def _objects(self, bus):
        reply = await self._call(bus, destination=BLUEZ, path="/",
                                 interface="org.freedesktop.DBus.ObjectManager",
                                 member="GetManagedObjects")
        return reply.body[0]

    @staticmethod
    def _val(props, key, default=None):
        v = props.get(key)
        return v.value if v is not None else default

    def _find_device(self, objs):
        candidate = None
        for path, ifaces in objs.items():
            props = ifaces.get(DEVICE_IFACE)
            if not props:
                continue
            if self.address:
                match = str(self._val(props, "Address", "")).upper() == self.address.upper()
            else:
                match = DEVICE_NAME in (self._val(props, "Name"), self._val(props, "Alias"))
            if not match:
                continue
            if self._val(props, "Connected", False):
                return path, props
            candidate = props
        return None, candidate

    def _find_char(self, objs, dev_path):
        for path, ifaces in objs.items():
            props = ifaces.get(CHAR_IFACE)
            if props and path.startswith(dev_path + "/") and \
                    str(self._val(props, "UUID", "")).lower() == DATA_UUID:
                return path, props
        return None, None

    async def _wait_device(self, bus):
        last = ""
        while not self._stop.is_set():
            objs = await self._objects(bus)
            dev_path, props = self._find_device(objs)
            if dev_path is None:
                msg = (f"'{self.address or DEVICE_NAME}' noto a BlueZ ma NON connesso: attiva 'Connection' dalla GUI."
                       if props else
                       f"'{self.address or DEVICE_NAME}' non trovato: associalo/connettilo dalla GUI Bluetooth.")
            elif not self._val(props, "ServicesResolved", False):
                msg = "Connesso, attendo la risoluzione dei servizi GATT..."
            else:
                char_path, char_props = self._find_char(objs, dev_path)
                if char_path:
                    return dev_path, props, char_path, char_props
                msg = f"Connesso ma characteristic {DATA_UUID} assente: firmware bluetooth_test.ino caricato?"
            if msg != last:
                self.on_status(msg)
                last = msg
            await asyncio.sleep(1.0)
        raise RuntimeError("arresto richiesto")

    async def _add_match(self, bus, path, remove=False):
        rule = (f"type='signal',sender='{BLUEZ}',interface='{PROPS_IFACE}',"
                f"member='PropertiesChanged',path='{path}'")
        await self._call(bus, destination="org.freedesktop.DBus", path="/org/freedesktop/DBus",
                         interface="org.freedesktop.DBus",
                         member="RemoveMatch" if remove else "AddMatch", signature="s", body=[rule])

    async def _session(self, bus):
        Message, MessageType = self._dbus
        dev_path, dev_props, char_path, char_props = await self._wait_device(bus)
        self.on_status(f"IMU connessa [{self._val(dev_props, 'Address')}] — streaming gyro avviato.")
        disconnected = asyncio.Event()

        def handler(msg):
            if msg.message_type != MessageType.SIGNAL or msg.member != "PropertiesChanged":
                return None
            iface, changed = msg.body[0], msg.body[1]
            if msg.path == char_path and iface == CHAR_IFACE and "Value" in changed:
                data = bytes(changed["Value"].value)
                if len(data) != PAYLOAD_SIZE:
                    return None
                values = struct.unpack(PAYLOAD_FMT, data)
                if not np.all(np.isfinite(values)):
                    return None
                self.callback(rev2_gyro_rad(values), self.clock(), values)
            elif msg.path == dev_path and iface == DEVICE_IFACE and "Connected" in changed:
                if not changed["Connected"].value:
                    disconnected.set()
            return None

        matches = []
        bus.add_message_handler(handler)
        try:
            for path in (char_path, dev_path):
                await self._add_match(bus, path)
                matches.append(path)
            await self._call(bus, destination=BLUEZ, path=char_path, interface=CHAR_IFACE, member="StartNotify")
            stop_waiter = asyncio.get_running_loop().create_task(self._wait_stop())
            await asyncio.wait({asyncio.create_task(disconnected.wait()), stop_waiter},
                               return_when=asyncio.FIRST_COMPLETED)
            if disconnected.is_set():
                self.on_status("IMU disconnessa: riconnettila dalla GUI, resto in attesa...")
        finally:
            bus.remove_message_handler(handler)
            if not disconnected.is_set():
                try:
                    await self._call(bus, destination=BLUEZ, path=char_path,
                                     interface=CHAR_IFACE, member="StopNotify")
                except Exception:
                    pass
            for path in matches:
                try:
                    await self._add_match(bus, path, remove=True)
                except Exception:
                    pass

    async def _wait_stop(self):
        while not self._stop.is_set():
            await asyncio.sleep(0.2)


# ----------------------------------------------------------------- sintetica
class SyntheticImu:
    """Gyro finto coerente con truth(t) (stesso moto della SyntheticCamera), con bias+rumore."""

    name = "sintetica (nessun BLE)"

    def __init__(self, truth, callback, rate=150.0, bias_deg=(1.1, -0.7, 0.45),
                 noise_deg=0.15, clock=time.monotonic, rng=None):
        self.truth = truth
        self.callback = callback
        self.period = 1.0 / rate
        self.bias = np.radians(np.array(bias_deg, float))
        self.noise = math.radians(noise_deg)
        self.clock = clock
        self.rng = rng or np.random.default_rng(2)
        self._stop = threading.Event()
        self._thread = None

    def start(self):
        self._thread = threading.Thread(target=self._loop, name="imu-synth", daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def _loop(self):
        h = 1e-3
        while not self._stop.is_set():
            now = self.clock()
            # velocita' angolare corpo-frame da differenza finita di truth: omega = 2*(conj(q)⊗q̇)[1:]
            _, q0 = self.truth(now - h)
            _, q1 = self.truth(now + h)
            q0 = Q.unit(q0); q1 = Q.unit(q1)
            qdot = (np.asarray(q1, float) - np.asarray(q0, float)) / (2 * h)
            omega = 2.0 * Q.multiply(Q.conjugate(Q.unit(self.truth(now)[1])), qdot)[1:]
            measured = omega + self.bias + self.rng.normal(scale=self.noise, size=3)
            values = (0, 0.0, 0.0, 9.80665, *np.degrees(measured), 0.0, 0.0, 0.0)
            self.callback(np.radians(np.array(values[4:7])), now, values)
            time.sleep(self.period)
