#!/usr/bin/env python3
"""
PicoScope + SIOS GUI

- Keeps SIOS LSV 2500 NG serial driver.
- If picosdk is not installed, a DummyPicoScope is used so you can test the GUI.
"""

import sys
import threading
import queue
import time
from dataclasses import dataclass
from typing import Optional, List, Union

try:
    import tkinter as tk
    from tkinter import ttk, messagebox
except Exception:  # pragma: no cover
    print("Tkinter is needed.")
    sys.exit(1)

#picosdk
try:
    from picosdk.ps2000 import ps2000 as ps
    import ctypes
except Exception:
    ps = None
    ctypes = None

#Serial for SIOS
try:
    import serial
    import serial.tools.list_ports as list_ports
except Exception:
    serial = None
    list_ports = None

# PicoScope driver(s)

class PicoScopeError(RuntimeError):
    pass


class DummyPicoScope:
    """Fallback driver when picosdk not installed: just logs actions."""
    def __init__(self, log):
        self.log = log
        self.connected = False
        self.waveform_cfg = {}

    def connect(self):
        self.connected = True
        self.log("DummyPicoScope: simulated connection established.")

    def close(self):
        self.connected = False
        self.log("DummyPicoScope: simulated connection closed.")

    def set_waveform(self, shape="SIN", f_start=1000.0, f_stop=None,
                     vpp=2.0, offset=0.0, increment=0.0, dwell_time=1.0,
                     sweep_enabled=False):
        self.waveform_cfg = dict(
            shape=shape, f_start=f_start, f_stop=f_stop, vpp=vpp,
            offset=offset, increment=increment, dwell_time=dwell_time,
            sweep_enabled=sweep_enabled
        )
        if sweep_enabled:
            self.log(f"DummyPicoScope: configured HW sweep: {shape} {f_start}->{f_stop} Hz step {increment} Hz dwell {dwell_time}s, {vpp} Vpp, offset {offset} V")
        else:
            self.log(f"DummyPicoScope: configured CW: {shape} {f_start} Hz, {vpp} Vpp, offset {offset} V")

    def start_output(self):
        if not self.connected:
            raise PicoScopeError("Dummy: not connected")
        self.log("DummyPicoScope: output STARTED (simulated).")

    def stop_output(self):
        if not self.connected:
            raise PicoScopeError("Dummy: not connected")
        self.log("DummyPicoScope: output STOPPED (simulated).")


class PicoScope2000:
    """
    PicoScope 2000 AWG wrapper using ps2000.dll.
    """

    def __init__(self, log):
        if ps is None:
            raise PicoScopeError("picosdk not available. Install picosdk and Pico drivers.")
        self.handle = None
        self.log = log
        self.connected = False
        # Map waveform names to PicoSDK built-in waveform codes
        self.waveforms = {
            "SIN": 0,
            "SQU": 1,
            "TRI": 2,
            "DC": 3
        }
        self._last_cfg = None

    def connect(self):
        self.handle = ps.ps2000_open_unit()
        self.log(f"DEBUG: ps2000_open_unit returned {self.handle}")
        if self.handle <= 0:
            raise PicoScopeError(f"Failed to open PicoScope (handle={self.handle})")
        self.connected = True
        self.log("PicoScope connected.")

    def close(self):
        if self.connected:
            ps.ps2000_close_unit(self.handle)
            self.log("PicoScope disconnected.")
            self.connected = False

    def set_waveform(self,
                     shape: str = "SIN",
                     f_start: float = 1000.0,
                     f_stop: float = None,
                     vpp: float = 2.0,
                     offset: float = 0.0,
                     increment: float = 0.0,
                     dwell_time: float = 1.0):
        """
        Configure built-in AWG (CW only for ps2000.dll).
        Sweep is not supported with this wrapper.
        """
        if not self.connected:
            raise PicoScopeError("PicoScope not connected.")

        if shape not in self.waveforms:
            raise ValueError(f"Unsupported waveform: {shape}")

        wave = self.waveforms[shape]
        pk_to_pk_mv = int(round(vpp * 1000.0))
        offset_mv = int(round(offset * 1000.0))

        status = ps.ps2000_set_sig_gen_built_in(
            self.handle,
            offset_mv,  # e.g. 0
            pk_to_pk_mv,  # e.g. 2000 for 2 Vpp
            wave,  # waveform code
            ctypes.c_float(f_start),  # start freq
            ctypes.c_float(f_start),  # stop freq same (no sweep)
            ctypes.c_float(0.0),  # increment
            ctypes.c_float(0.0),  # dwell time
            0,  # sweepType (0 = up)
            0  # operation (0 = normal)
        )
        if status != 0:
            raise PicoScopeError(f"ps2000_set_sig_gen_built_in returned {status}")

        self._last_cfg = dict(shape=shape, f_start=f_start, vpp=vpp, offset=offset)
        self.log(f"PicoScope CW configured: {shape} {f_start} Hz, {vpp} Vpp, offset {offset} V")

    def start_output(self):
        # ps2000 starts output immediately after set_waveform()
        self.log("PicoScope AWG: start_output() called (output starts automatically).")

    def stop_output(self):
        if not self.connected:
            raise PicoScopeError("PicoScope not connected.")
        ps.ps2000_set_sig_gen_built_in(self.handle, 0, 0, 0, ctypes.c_double(0))
        self.log("PicoScope AWG stopped (set amplitude to 0).")


# --------------------------
# SIOS driver (unchanged, minimal)
# --------------------------

@dataclass
class SIOSConfig:
    port: str = "COM4"
    baudrate: int = 9600
    timeout_s: float = 1.0


class SIOSLSV2500NG:
    def __init__(self, cfg: SIOSConfig, log):
        self.cfg = cfg
        self.ser: Optional[serial.Serial] = None
        self.log = log

    def connect(self):
        if serial is None:
            raise RuntimeError("pyserial not installed. 'pip install pyserial'")
        try:
            self.ser = serial.Serial(self.cfg.port, self.cfg.baudrate, timeout=self.cfg.timeout_s)
            self.log(f"SIOS connected to {self.cfg.port}@{self.cfg.baudrate}")
        except Exception as e:
            self.ser = None
            raise RuntimeError(f"Serial connection failed: {e}")

    def close(self):
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.log("SIOS disconnected.")

    def send(self, cmd: str, expect: Optional[bytes] = None, add_crlf: bool = True) -> bytes:
        if not self.ser or not self.ser.is_open:
            raise RuntimeError("SIOS not connected.")
        data = (cmd + ("\r\n" if add_crlf else "")).encode("ascii", errors="ignore")
        self.log(f"> {cmd}")
        self.ser.reset_input_buffer()
        self.ser.write(data)
        self.ser.flush()
        time.sleep(0.02)
        resp = self.ser.read_until(b"\n")
        if resp:
            self.log(f"< {resp.decode(errors='ignore').strip()}")
        if expect and expect not in resp:
            raise RuntimeError(f"Unexpected answer: {resp}")
        return resp

    def start_measurement(self):
        self.send("MEAS:START")

    def stop_measurement(self):
        self.send("MEAS:STOP")

    def set_integration_time(self, ms: int):
        self.send(f"MEAS:INT {ms}")

    def read_value(self) -> Optional[float]:
        resp = self.send("MEAS:READ?")
        try:
            return float(resp.decode().strip())
        except Exception:
            return None


# --------------------------
# GUI (App)
# --------------------------

class App(ttk.Frame):
    def __init__(self, master):
        super().__init__(master)
        self.master = master
        self.pack(fill=tk.BOTH, expand=True)

        self.log_queue: "queue.Queue[str]" = queue.Queue()
        self.after(100, self._drain_log)

        # Create PicoScope driver (or dummy if picosdk missing)
        if ps is None:
            self.pico = DummyPicoScope(self._log)
            self._log("picosdk not found: using DummyPicoScope. Install picosdk + Pico drivers for real device.")
        else:
            try:
                self.pico = PicoScope2000(self._log)
            except Exception as e:
                self.pico = DummyPicoScope(self._log)
                self._log(f"Could not initialize PicoScope driver: {e}. Using dummy driver.")

        # SIOS
        self.sios = SIOSLSV2500NG(SIOSConfig(), self._log)

        self._build_ui()

    # ---------- Logging ----------
    def _log(self, msg: str):
        timestamp = time.strftime("%H:%M:%S")
        self.log_queue.put(f"[{timestamp}] {msg}")

    def _drain_log(self):
        try:
            while True:
                line = self.log_queue.get_nowait()
                self.txt_log.configure(state=tk.NORMAL)
                self.txt_log.insert(tk.END, line + "\n")
                self.txt_log.configure(state=tk.DISABLED)
                self.txt_log.see(tk.END)
        except queue.Empty:
            pass
        self.after(150, self._drain_log)

    # ---------- UI ----------
    def _build_ui(self):
        self.master.title("PicoScope 2000 (AWG) & SIOS LSV 2500 NG - Control")
        self.master.minsize(980, 640)

        frm_conn = ttk.LabelFrame(self, text="Connections")
        frm_conn.pack(fill=tk.X, padx=10, pady=8)

        # PicoScope controls (row 0)
        ttk.Label(frm_conn, text="PicoScope:").grid(row=0, column=0, sticky=tk.W, padx=6, pady=4)
        ttk.Button(frm_conn, text="Connect", command=self._pico_connect).grid(row=0, column=5, padx=4, pady=4)
        ttk.Button(frm_conn, text="Disconnect", command=self._pico_close).grid(row=0, column=6, padx=4, pady=4)

        # SIOS controls (row 1)
        ttk.Label(frm_conn, text="SIOS Port:").grid(row=1, column=0, sticky=tk.W, padx=6, pady=4)
        self.var_sios_port = tk.StringVar(value=self.sios.cfg.port)
        ttk.Entry(frm_conn, textvariable=self.var_sios_port, width=20).grid(row=1, column=1, sticky=tk.W, padx=6, pady=4)

        ttk.Label(frm_conn, text="Baud:").grid(row=1, column=2, sticky=tk.E, padx=6)
        self.var_sios_baud = tk.IntVar(value=self.sios.cfg.baudrate)
        ttk.Entry(frm_conn, textvariable=self.var_sios_baud, width=10).grid(row=1, column=3, sticky=tk.W, padx=6)

        ttk.Button(frm_conn, text="Search Ports", command=self._refresh_ports).grid(row=1, column=4, padx=4, pady=4)
        ttk.Button(frm_conn, text="Connect", command=self._sios_connect).grid(row=1, column=5, padx=4, pady=4)
        ttk.Button(frm_conn, text="Disconnect", command=self._sios_close).grid(row=1, column=6, padx=4, pady=4)

        for i in range(0, 7):
            frm_conn.grid_columnconfigure(i, weight=0)
        frm_conn.grid_columnconfigure(1, weight=1)

        nb = ttk.Notebook(self)
        nb.pack(fill=tk.BOTH, expand=True, padx=10, pady=8)

        self.tab_pico = ttk.Frame(nb)
        self.tab_sios = ttk.Frame(nb)
        self.tab_log = ttk.Frame(nb)
        nb.add(self.tab_pico, text="PicoScope AWG")
        nb.add(self.tab_sios, text="SIOS LSV 2500 NG")
        nb.add(self.tab_log, text="Log & Action")

        self._build_pico_tab(self.tab_pico)
        self._build_sios_tab(self.tab_sios)
        self._build_log_tab(self.tab_log)

    def _build_pico_tab(self, parent):
        frm = ttk.Frame(parent)
        frm.pack(fill=tk.BOTH, expand=True)

        # Waveform controls
        box = ttk.LabelFrame(frm, text="AWG / Waveform")
        box.pack(fill=tk.X, padx=8, pady=8)

        ttk.Label(box, text="Waveform").grid(row=0, column=0, sticky=tk.W, padx=6, pady=6)
        self.var_pico_shape = tk.StringVar(value="SIN")
        cb = ttk.Combobox(box, textvariable=self.var_pico_shape, values=["SIN", "SQU", "TRI", "DC"], state="readonly", width=8)
        cb.grid(row=0, column=1, padx=6, pady=6)

        ttk.Label(box, text="Start freq [Hz]").grid(row=0, column=2, sticky=tk.W)
        self.var_pico_fstart = tk.DoubleVar(value=1000.0)
        ttk.Entry(box, textvariable=self.var_pico_fstart, width=14).grid(row=0, column=3, padx=6)

        ttk.Label(box, text="Stop freq [Hz]").grid(row=0, column=4, sticky=tk.W)
        self.var_pico_fstop = tk.StringVar(value="")  # empty => CW
        ttk.Entry(box, textvariable=self.var_pico_fstop, width=14).grid(row=0, column=5, padx=6)

        ttk.Label(box, text="Increment [Hz]").grid(row=1, column=0, sticky=tk.W, padx=6)
        self.var_pico_inc = tk.StringVar(value="")  # optional
        ttk.Entry(box, textvariable=self.var_pico_inc, width=12).grid(row=1, column=1, padx=6)

        ttk.Label(box, text="Dwell [s]").grid(row=1, column=2, sticky=tk.W)
        self.var_pico_dwell = tk.DoubleVar(value=1.0)
        ttk.Entry(box, textvariable=self.var_pico_dwell, width=12).grid(row=1, column=3, padx=6)

        ttk.Label(box, text="Amplitude [Vpp]").grid(row=1, column=4, sticky=tk.W)
        self.var_pico_vpp = tk.DoubleVar(value=2.0)
        ttk.Entry(box, textvariable=self.var_pico_vpp, width=10).grid(row=1, column=5, padx=6)

        ttk.Label(box, text="Offset [V]").grid(row=1, column=6, sticky=tk.W)
        self.var_pico_offs = tk.DoubleVar(value=0.0)
        ttk.Entry(box, textvariable=self.var_pico_offs, width=10).grid(row=1, column=7, padx=6)

        ttk.Button(box, text="Apply", command=self._pico_apply).grid(row=0, column=8, rowspan=2, padx=8)

        # Output control
        box2 = ttk.LabelFrame(frm, text="Output")
        box2.pack(fill=tk.X, padx=8, pady=8)
        self.var_pico_out = tk.BooleanVar(value=False)
        ttk.Checkbutton(box2, text="Output ON", variable=self.var_pico_out, command=self._pico_toggle_output).pack(anchor=tk.W, padx=8, pady=6)

        # Status
        box3 = ttk.LabelFrame(frm, text="Status / Info")
        box3.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)
        self.var_pico_status = tk.StringVar(value="—")
        ttk.Label(box3, textvariable=self.var_pico_status, anchor="w").pack(fill=tk.X, padx=8, pady=6)
        ttk.Button(box3, text="Refresh status (log)", command=self._pico_status).pack(anchor=tk.W, padx=8, pady=6)

    def _build_sios_tab(self, parent):
        frm = ttk.Frame(parent)
        frm.pack(fill=tk.BOTH, expand=True)

        box = ttk.LabelFrame(frm, text="Measurement")
        box.pack(fill=tk.X, padx=8, pady=8)

        ttk.Label(box, text="Integration time [ms]").grid(row=0, column=0, sticky=tk.W, padx=6, pady=6)
        self.var_int = tk.IntVar(value=10)
        ttk.Entry(box, textvariable=self.var_int, width=10).grid(row=0, column=1, padx=6)

        ttk.Button(box, text="Set", command=self._sios_set_int).grid(row=0, column=2, padx=6)
        ttk.Button(box, text="Start", command=self._sios_start).grid(row=0, column=3, padx=6)
        ttk.Button(box, text="Stop", command=self._sios_stop).grid(row=0, column=4, padx=6)
        ttk.Button(box, text="Read value", command=self._sios_read).grid(row=0, column=5, padx=6)

        box2 = ttk.LabelFrame(frm, text="Measurement value")
        box2.pack(fill=tk.X, padx=8, pady=8)
        self.var_val = tk.StringVar(value="—")
        ttk.Label(box2, textvariable=self.var_val).pack(anchor=tk.W, padx=8, pady=6)

    def _build_log_tab(self, parent):
        frm = ttk.Frame(parent)
        frm.pack(fill=tk.BOTH, expand=True)

        btns = ttk.Frame(frm)
        btns.pack(fill=tk.X, padx=8, pady=4)
        ttk.Button(btns, text="Copy", command=self._copy_log).pack(side=tk.LEFT)
        ttk.Button(btns, text="Clear", command=self._clear_log).pack(side=tk.LEFT, padx=6)

        self.txt_log = tk.Text(frm, height=16, state=tk.DISABLED)
        self.txt_log.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)

    # ---------- Pico actions ----------
    def _pico_connect(self):
        def task():
            try:
                self.pico.connect()
            except Exception as e:
                self._show_error("PicoScope", e)
        threading.Thread(target=task, daemon=True).start()

    def _pico_close(self):
        def task():
            try:
                self.pico.close()
            except Exception as e:
                self._show_error("PicoScope", e)
        threading.Thread(target=task, daemon=True).start()

    def _pico_apply(self):
        try:
            shape = self.var_pico_shape.get()
            f_start = float(self.var_pico_fstart.get())
            f_stop = None
            inc = 0.0
            if self.var_pico_fstop.get().strip() != "":
                f_stop = float(self.var_pico_fstop.get())
            if self.var_pico_inc.get().strip() != "":
                inc = float(self.var_pico_inc.get())
            dwell = float(self.var_pico_dwell.get())
            vpp = float(self.var_pico_vpp.get())
            offs = float(self.var_pico_offs.get())
        except ValueError:
            self._show_error("Eingabe", "Bitte gültige Zahlen eingeben.")
            return

        sweep_enabled = (f_stop is not None and f_stop > f_start and inc > 0.0)

        def task():
            try:
                self.pico.set_waveform(shape=shape, f_start=f_start, f_stop=f_stop,
                                       vpp=vpp, offset=offs, increment=inc, dwell_time=dwell)
                # Note: some PicoSDKs start AWG automatically if shots=0; call start_output explicitly
                self.pico.start_output()
                self._log("PicoScope: AWG konfiguriert und gestartet.")
                if sweep_enabled:
                    self._log("PicoScope: Hardware sweep enabled.")
            except Exception as e:
                self._show_error("PicoScope", e)
        threading.Thread(target=task, daemon=True).start()

    def _pico_toggle_output(self):
        on = self.var_pico_out.get()
        def task():
            try:
                if on:
                    self.pico.start_output()
                else:
                    self.pico.stop_output()
            except Exception as e:
                self._show_error("PicoScope", e)
                # revert checkbox on error
                self.var_pico_out.set(not on)
        threading.Thread(target=task, daemon=True).start()

    def _pico_status(self):
        # Not all picosdk variants provide a "query config" API; we log the last known cfg if available.
        try:
            if hasattr(self.pico, "_last_cfg") and self.pico._last_cfg:
                cfg = self.pico._last_cfg
                self.var_pico_status.set(str(cfg))
                self._log(f"PicoScope status: {cfg}")
            else:
                self.var_pico_status.set("No config known (device may be dummy or SDK does not support query).")
                self._log("PicoScope: No config known.")
        except Exception as e:
            self._show_error("PicoScope", e)

    # ---------- SIOS actions ----------
    def _refresh_ports(self):
        if list_ports is None:
            messagebox.showinfo("Info", "pyserial nicht installiert")
            return
        ports = [p.device for p in list_ports.comports()]
        if ports:
            messagebox.showinfo("Gefundene Ports", "\n".join(ports))
        else:
            messagebox.showinfo("Gefundene Ports", "Keine Ports gefunden.")

    def _sios_connect(self):
        self.sios.cfg.port = self.var_sios_port.get().strip()
        self.sios.cfg.baudrate = int(self.var_sios_baud.get())
        def task():
            try:
                self.sios.connect()
            except Exception as e:
                self._show_error("SIOS", e)
        threading.Thread(target=task, daemon=True).start()

    def _sios_close(self):
        def task():
            try:
                self.sios.close()
            except Exception as e:
                self._show_error("SIOS", e)
        threading.Thread(target=task, daemon=True).start()

    def _sios_set_int(self):
        try:
            ms = int(self.var_int.get())
        except ValueError:
            self._show_error("Eingabe", "Ungültige Integrationszeit")
            return
        def task():
            try:
                self.sios.set_integration_time(ms)
            except Exception as e:
                self._show_error("SIOS", e)
        threading.Thread(target=task, daemon=True).start()

    def _sios_start(self):
        def task():
            try:
                self.sios.start_measurement()
            except Exception as e:
                self._show_error("SIOS", e)
        threading.Thread(target=task, daemon=True).start()

    def _sios_stop(self):
        def task():
            try:
                self.sios.stop_measurement()
            except Exception as e:
                self._show_error("SIOS", e)
        threading.Thread(target=task, daemon=True).start()

    def _sios_read(self):
        def task():
            try:
                val = self.sios.read_value()
                self.var_val.set("—" if val is None else f"{val}")
            except Exception as e:
                self._show_error("SIOS", e)
        threading.Thread(target=task, daemon=True).start()

    # ---------- Log helpers ----------
    def _copy_log(self):
        try:
            self.master.clipboard_clear()
            self.master.clipboard_append(self.txt_log.get("1.0", tk.END))
            messagebox.showinfo("Kopiert", "Log in Zwischenablage kopiert.")
        except Exception:
            pass

    def _clear_log(self):
        self.txt_log.configure(state=tk.NORMAL)
        self.txt_log.delete("1.0", tk.END)
        self.txt_log.configure(state=tk.DISABLED)

    def _show_error(self, title: str, err: Union[Exception, str]):
        self._log(f"ERROR [{title}]: {err}")
        messagebox.showerror(title, str(err))


# --- Main ---

def main():
    root = tk.Tk()
    try:
        root.call("set_theme", "light")  # optional if ttkbootstrap is installed
    except Exception:
        pass
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()
