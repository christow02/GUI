import os
from picosdk.ps2000a import ps2000a
import ctypes

pico_sdk_path = r"C:\Program Files\Pico Technology\SDK\lib"
os.add_dll_directory(pico_sdk_path)

chandle = ctypes.c_int16()
status = ps2000a.ps2000aOpenUnit(ctypes.byref(chandle), None)
print("OpenUnit status =", status)
