"""Mouse output and global keyboard input that also work on Wayland.

On a Wayland session pynput only talks to XWayland: it cannot move the real
cursor over native Wayland windows nor see their key presses. The kernel-level
alternative works everywhere (Wayland, X11, even the login screen):

- UInputMouse: a virtual absolute pointer (like the QEMU USB tablet) created
  through /dev/uinput; the compositor maps ABS 0..32767 to the whole screen.
- EvdevKeyboard: reads the real keyboards from /dev/input/event* (no grab, the
  keys still reach the focused application).

Both need the user in the 'input' group and a udev rule for /dev/uinput
(see README). PynputMouse / PynputKeyboard are the X11 fallback.
"""
import os
import select
import threading

ABS_MAX = 32767
DEVICE_NAME = "halo-apriltag-pointer"


def is_wayland():
    return os.environ.get("XDG_SESSION_TYPE", "").lower() == "wayland" or bool(os.environ.get("WAYLAND_DISPLAY"))


# ----------------------------------------------------------------- mouse
class UInputMouse:
    """Absolute virtual pointer. ``position`` is in screen pixels (Screen geometry)."""

    name = "uinput (kernel virtual tablet)"

    def __init__(self, screen):
        from evdev import AbsInfo, UInput, ecodes as e
        self.e = e
        self.screen = screen
        caps = {
            e.EV_KEY: [e.BTN_LEFT, e.BTN_RIGHT, e.BTN_MIDDLE],
            e.EV_ABS: [(e.ABS_X, AbsInfo(0, 0, ABS_MAX, 0, 0, 0)),
                       (e.ABS_Y, AbsInfo(0, 0, ABS_MAX, 0, 0, 0))],
            e.EV_REL: [e.REL_WHEEL],
        }
        self.ui = UInput(caps, name=DEVICE_NAME)
        self.lock = threading.Lock()
        # the real cursor cannot be read on Wayland: remember the last position we set
        self._pos = (screen.x + screen.w // 2, screen.y + screen.h // 2)

    @property
    def position(self):
        return self._pos

    @position.setter
    def position(self, xy):
        self._move(xy, force=False)

    def hold(self, xy):
        """Put the cursor back on xy even if the physical mouse moved it away."""
        self._move(xy, force=True)

    def _move(self, xy, force):
        x, y = xy
        s = self.screen
        ax = int(round((x - s.x) / max(1, s.w - 1) * ABS_MAX))
        ay = int(round((y - s.y) / max(1, s.h - 1) * ABS_MAX))
        ax, ay = min(max(ax, 0), ABS_MAX), min(max(ay, 0), ABS_MAX)
        with self.lock:
            if force:
                # the kernel drops ABS events equal to the previous value: nudge by 1 unit
                # (1/32767 of the screen, invisible) so the compositor gets a fresh position
                self.ui.write(self.e.EV_ABS, self.e.ABS_X, ax - 1 if ax > 0 else ax + 1)
                self.ui.write(self.e.EV_ABS, self.e.ABS_Y, ay - 1 if ay > 0 else ay + 1)
                self.ui.syn()
            self.ui.write(self.e.EV_ABS, self.e.ABS_X, ax)
            self.ui.write(self.e.EV_ABS, self.e.ABS_Y, ay)
            self.ui.syn()
        self._pos = (int(x), int(y))

    def _button(self, value):
        with self.lock:
            self.ui.write(self.e.EV_KEY, self.e.BTN_LEFT, value)
            self.ui.syn()

    def press(self, _button=None):
        self._button(1)

    def release(self, _button=None):
        self._button(0)

    def close(self):
        try:
            self._button(0)
            self.ui.close()
        except Exception:
            pass


class PynputMouse:
    name = "pynput (X11)"

    def __init__(self, screen=None):
        from pynput.mouse import Button, Controller
        self.c = Controller()
        self.left = Button.left

    @property
    def position(self):
        return self.c.position

    @position.setter
    def position(self, xy):
        self.c.position = xy

    def hold(self, xy):
        if tuple(self.c.position) != tuple(xy):
            self.c.position = xy

    def press(self, _button=None):
        self.c.press(self.left)

    def release(self, _button=None):
        self.c.release(self.left)

    def close(self):
        try:
            self.c.release(self.left)
        except Exception:
            pass


def uinput_problem():
    """Return None if /dev/uinput is usable, else a human readable reason."""
    try:
        import evdev  # noqa: F401
    except ImportError:
        return "python 'evdev' not installed (pip install evdev)"
    if not os.path.exists("/dev/uinput"):
        return "/dev/uinput missing (sudo modprobe uinput)"
    if not os.access("/dev/uinput", os.W_OK):
        return "no write permission on /dev/uinput (udev rule + 'input' group, then logout/login)"
    return None


def make_mouse(backend, screen):
    """backend: auto | uinput | pynput."""
    if backend == "auto":
        backend = "uinput" if is_wayland() else "pynput"
        if backend == "uinput" and uinput_problem():
            print(f"[mouse] Wayland detected but uinput unusable: {uinput_problem()}")
            print("[mouse] falling back to pynput: the cursor will move ONLY over XWayland windows")
            backend = "pynput"
    if backend == "uinput":
        problem = uinput_problem()
        if problem:
            raise RuntimeError(f"uinput backend: {problem}")
        return UInputMouse(screen)
    return PynputMouse(screen)


# ----------------------------------------------------------------- keyboard
class EvdevKeyboard:
    """Global key listener on all physical keyboards. Calls on_key(name) on key-down."""

    name = "evdev (/dev/input)"

    def __init__(self, on_key):
        from evdev import InputDevice, ecodes as e, list_devices
        self.e = e
        self.on_key = on_key
        self.devices = []
        for path in list_devices():
            try:
                dev = InputDevice(path)
            except OSError:
                continue
            keys = dev.capabilities().get(e.EV_KEY, [])
            if dev.name != DEVICE_NAME and e.KEY_SPACE in keys and e.KEY_Q in keys:
                self.devices.append(dev)
            else:
                dev.close()
        if not self.devices:
            raise RuntimeError("no readable keyboard in /dev/input (add your user to the 'input' group)")
        self.map = {e.KEY_SPACE: "space", e.KEY_ENTER: "enter", e.KEY_KPENTER: "enter",
                    e.KEY_ESC: "esc", e.KEY_Q: "q", e.KEY_C: "c"}
        self.running = True
        threading.Thread(target=self._loop, name="evdev-keys", daemon=True).start()

    def _loop(self):
        fds = {dev.fd: dev for dev in self.devices}
        while self.running and fds:
            r, _, _ = select.select(list(fds), [], [], 0.5)
            for fd in r:
                dev = fds[fd]
                try:
                    for ev in dev.read():
                        if ev.type == self.e.EV_KEY and ev.value == 1 and ev.code in self.map:
                            self.on_key(self.map[ev.code])
                except OSError:  # keyboard unplugged
                    fds.pop(fd, None)

    def stop(self):
        self.running = False


class PynputKeyboard:
    name = "pynput (X11)"

    def __init__(self, on_key):
        from pynput import keyboard
        special = {keyboard.Key.space: "space", keyboard.Key.enter: "enter", keyboard.Key.esc: "esc"}

        def on_press(key):
            if key in special:
                on_key(special[key])
            else:
                ch = (getattr(key, "char", None) or "").lower()
                if ch in ("q", "c"):
                    on_key(ch)

        self.listener = keyboard.Listener(on_press=on_press)
        self.listener.daemon = True
        self.listener.start()

    def stop(self):
        self.listener.stop()


def make_keyboard(backend, on_key):
    """backend: auto | evdev | pynput. Returns the listener or raises."""
    if backend in ("auto", "evdev"):
        try:
            return EvdevKeyboard(on_key)
        except Exception as exc:
            if backend == "evdev":
                raise
            print(f"[keys] evdev unavailable ({exc})")
            if is_wayland():
                print("[keys] pynput on Wayland sees keys only while an XWayland window (HUD/camera) is focused")
    return PynputKeyboard(on_key)
