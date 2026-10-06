"""Capture actual visible X11 application windows; never synthesize evidence."""
import ctypes
from ctypes.util import find_library
import re
import subprocess
import time


def aim_gazebo(tcp, cube):
    """Aim the GUI camera at the measured robot/cube; this is not a world pose write."""
    import math
    from scipy.spatial.transform import Rotation
    center = [(a+b)/2 for a, b in zip(tcp, cube)]
    offset = [0.55, -0.65, 0.55] if tcp[2] > 0.4 else [0.36, -0.42, 0.32]
    xyz = [c+d for c, d in zip(center, offset)]
    yaw = math.atan2(-offset[1], -offset[0])
    pitch = math.atan2(offset[2], math.hypot(offset[0], offset[1]))
    q = Rotation.from_euler("xyz", [0.0, pitch, yaw]).as_quat()
    request = ("pose: {position: {x: %s y: %s z: %s} orientation: {x: %s y: %s z: %s w: %s}}"
               % (*xyz, *q))
    result = subprocess.run([
        "ign", "service", "-s", "/gui/move_to/pose", "--reqtype", "ignition.msgs.GUICamera",
        "--reptype", "ignition.msgs.Boolean", "--timeout", "1000", "--req", request,
    ], capture_output=True, text=True, timeout=2)
    if "true" not in result.stdout:
        raise RuntimeError(f"Gazebo GUI camera unavailable: {result.stdout} {result.stderr}")
    time.sleep(0.4)
    return request


def capture_windows(directory, prefix):
    from PIL import ImageGrab
    tree = subprocess.check_output(["xwininfo", "-root", "-tree"], text=True, timeout=5)
    candidates = []
    for line in tree.splitlines():
        if '"' not in line:
            continue
        title = line.split('"')[1]
        if (title == "Gazebo" or title.endswith(" - RViz")) and re.search(r"\b[1-9]\d{2,}x[1-9]\d{2,}", line):
            candidates.append((line.strip().split()[0], "gazebo" if "Gazebo" in title else "rviz"))
    if not candidates:
        raise RuntimeError("No visible Gazebo/RViz window on DISPLAY")
    xlib = ctypes.CDLL(find_library("X11"))
    xlib.XOpenDisplay.restype = ctypes.c_void_p
    xlib.XMapRaised.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
    xlib.XSync.argtypes = [ctypes.c_void_p, ctypes.c_int]
    xlib.XCloseDisplay.argtypes = [ctypes.c_void_p]
    display = xlib.XOpenDisplay(None)
    if not display:
        raise RuntimeError("Cannot open X11 display")
    saved = []
    try:
        for window, app in candidates:
            xlib.XMapRaised(display, int(window, 16))
            xlib.XSync(display, 0)
            time.sleep(0.3)
            info = subprocess.check_output(["xwininfo", "-id", window], text=True, timeout=5)
            def field(label):
                return int(re.search(re.escape(label) + r":\s*(-?\d+)", info)[1])
            x, y = field("Absolute upper-left X"), field("Absolute upper-left Y")
            width, height = field("Width"), field("Height")
            path = directory / f"{prefix}_{app}_{window}.png"
            frame = ImageGrab.grab(bbox=(x, y, x+width, y+height)).convert("RGB")
            frame.save(path)
            if max(high for low, high in frame.getextrema()) <= 2:
                raise RuntimeError(f"Black X11 capture; unusable image retained at {path.name}")
            saved.append(path.name)
    finally:
        xlib.XCloseDisplay(display)
    return saved
