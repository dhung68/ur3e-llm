"""Sparse real desktop captures after confirming the active application window."""
from pathlib import Path
import ctypes
from ctypes.util import find_library
import re
import subprocess
import time
from PIL import ImageGrab


def capture(directory, prefix):
    saved = []
    windows = subprocess.check_output(['xwininfo','-root','-tree'], text=True, timeout=3).splitlines()
    candidates = {}
    for line in windows:
        if '"' not in line: continue
        title = line.split('"')[1]
        app = 'gazebo' if title == 'Gazebo' else 'rviz' if title.endswith(' - RViz') else None
        if not app or not re.search(r"\b[1-9]\d{2,}x[1-9]\d{2,}",line): continue
        window = line.strip().split()[0]
        pid_text = subprocess.check_output(['xprop','-id',window,'_NET_WM_PID'],text=True,timeout=3)
        try: pid = int(pid_text.split()[-1])
        except ValueError: continue
        if app not in candidates or pid > candidates[app][0]: candidates[app] = (pid, window)
    if not candidates: raise RuntimeError('Gazebo/RViz windows unavailable')
    for app, (_, window) in candidates.items():
        activate(int(window,16))
        time.sleep(.5)
        active = subprocess.check_output(['xprop','-root','_NET_ACTIVE_WINDOW'],text=True,timeout=3)
        if int(active.split()[-1],16) != int(window,16):
            raise RuntimeError(f'{app} not active; desktop capture refused')
        info = subprocess.check_output(['xwininfo','-id',window],text=True,timeout=3)
        def field(label): return int(re.search(re.escape(label)+r':\s*(-?\d+)',info)[1])
        x,y,w,h = [field(s) for s in ('Absolute upper-left X','Absolute upper-left Y','Width','Height')]
        path = Path(directory)/f'{prefix}_{app}.png'
        if path.exists(): raise RuntimeError('Evidence image already exists')
        ImageGrab.grab(bbox=(x,y,x+w,y+h)).convert('RGB').save(path)
        saved.append(path.name)
    return saved


def activate(window):
    class Data(ctypes.Union):
        _fields_ = [('b', ctypes.c_char*20), ('s', ctypes.c_short*10), ('l', ctypes.c_long*5)]
    class ClientMessage(ctypes.Structure):
        _fields_ = [('type',ctypes.c_int), ('serial',ctypes.c_ulong), ('send_event',ctypes.c_int),
                    ('display',ctypes.c_void_p), ('window',ctypes.c_ulong), ('message_type',ctypes.c_ulong),
                    ('format',ctypes.c_int), ('data',Data)]
    class Event(ctypes.Union):
        _fields_ = [('xclient',ClientMessage), ('pad',ctypes.c_long*24)]
    lib=ctypes.CDLL(find_library('X11'))
    lib.XOpenDisplay.restype=ctypes.c_void_p
    lib.XDefaultRootWindow.argtypes=[ctypes.c_void_p];lib.XDefaultRootWindow.restype=ctypes.c_ulong
    lib.XInternAtom.argtypes=[ctypes.c_void_p,ctypes.c_char_p,ctypes.c_int];lib.XInternAtom.restype=ctypes.c_ulong
    lib.XSendEvent.argtypes=[ctypes.c_void_p,ctypes.c_ulong,ctypes.c_int,ctypes.c_long,ctypes.POINTER(Event)]
    lib.XMapRaised.argtypes=[ctypes.c_void_p,ctypes.c_ulong]
    lib.XFlush.argtypes=[ctypes.c_void_p];lib.XCloseDisplay.argtypes=[ctypes.c_void_p]
    display=lib.XOpenDisplay(None)
    if not display: raise RuntimeError('Cannot open desktop DISPLAY')
    try:
        event=Event();event.xclient.type=33;event.xclient.send_event=1
        event.xclient.display=display;event.xclient.window=window;event.xclient.format=32
        event.xclient.message_type=lib.XInternAtom(display,b'_NET_ACTIVE_WINDOW',0)
        event.xclient.data.l[0]=2
        lib.XMapRaised(display,window)
        lib.XSendEvent(display,lib.XDefaultRootWindow(display),0,(1<<19)|(1<<20),ctypes.byref(event))
        lib.XFlush(display)
    finally: lib.XCloseDisplay(display)
