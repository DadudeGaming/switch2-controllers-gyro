import ctypes
from ctypes import wintypes

SendInput = ctypes.windll.user32.SendInput

# Mouse event constants
MOUSEEVENTF_MOVE = 0x0001
MOUSEEVENTF_LEFTDOWN = 0x0002
MOUSEEVENTF_LEFTUP = 0x0004

class mouse_input(ctypes.Structure):
    _fields_ = (("dx", wintypes.LONG),
                ("dy", wintypes.LONG),
                ("mouseData", wintypes.DWORD),
                ("dwFlags", wintypes.DWORD),
                ("time", wintypes.DWORD),
                ("dwExtraInfo", ctypes.POINTER(wintypes.ULONG)))

class INPUT(ctypes.Structure):
    class _I(ctypes.Union):
        _fields_ = [("mi", mouse_input)]
    _fields_ = [("type", wintypes.DWORD), ("ii", _I)]

def send_relative_mouse(dx, dy):
    inp = INPUT()
    inp.type = 0  # Mouse input
    inp.ii.mi = mouse_input(dx, dy, 0, MOUSEEVENTF_MOVE, 0, None)
    SendInput(1, ctypes.byref(inp), ctypes.sizeof(inp))

def send_left_click():
    down = INPUT()
    down.type = 0
    down.ii.mi = mouse_input(0, 0, 0, MOUSEEVENTF_LEFTDOWN, 0, None)
    up = INPUT()
    up.type = 0
    up.ii.mi = mouse_input(0, 0, 0, MOUSEEVENTF_LEFTUP, 0, None)
    SendInput(1, ctypes.byref(down), ctypes.sizeof(down))
    SendInput(1, ctypes.byref(up), ctypes.sizeof(up))
