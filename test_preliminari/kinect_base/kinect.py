import ctypes as C
from ctypes.util import find_library
import numpy as np

class Kinect:
    WIDTH = 640
    HEIGHT = 480
    VIDEO_RGB = 0
    VIDEO_IR_8BIT = 2
    DEPTH_11BIT = 0

    def __init__(self, index=0):
        self.index = index
        name = find_library("freenect_sync")
        if not name:
            raise RuntimeError("Libreria freenect_sync non trovata.")
        self.lib = C.CDLL(name)
        args = [C.POINTER(C.c_void_p), C.POINTER(C.c_uint32), C.c_int, C.c_int]
        for fn in (self.lib.freenect_sync_get_video, self.lib.freenect_sync_get_depth):
            fn.argtypes = args
            fn.restype = C.c_int
        self.lib.freenect_sync_stop.argtypes = []
        self.lib.freenect_sync_stop.restype = None

    def _frame(self, fn, fmt, shape, ctype):
        pointer = C.c_void_p()
        timestamp = C.c_uint32()
        status = fn(C.byref(pointer), C.byref(timestamp), self.index, fmt)
        if status != 0 or not pointer.value:
            raise RuntimeError(
                f"{fn.__name__} non ha restituito un frame (codice {status}). "
                "Controlla alimentazione, USB, permessi e che glview sia chiuso."
            )
        size = int(np.prod(shape))
        data = C.cast(pointer, C.POINTER(ctype))
        return np.ctypeslib.as_array(data, shape=(size,)).reshape(shape).copy()

    def rgb(self):
        return self._frame(
            self.lib.freenect_sync_get_video, self.VIDEO_RGB,
            (self.HEIGHT, self.WIDTH, 3), C.c_uint8,
        )

    def ir(self):
        return self._frame(
            self.lib.freenect_sync_get_video, self.VIDEO_IR_8BIT,
            (self.HEIGHT, self.WIDTH), C.c_uint8,
        )

    def depth(self):
        return self._frame(
            self.lib.freenect_sync_get_depth, self.DEPTH_11BIT,
            (self.HEIGHT, self.WIDTH), C.c_uint16,
        )

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.lib.freenect_sync_stop()
