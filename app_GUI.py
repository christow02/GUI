from collections import deque
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from ctypes import byref, c_int16, c_int32, c_byte, sizeof
import threading
import queue
import time
from time import sleep
from dataclasses import dataclass
from typing import Optional

import tkinter as tk
from tkinter import ttk, messagebox

# -------------------------
# PicoScope controller
# -------------------------
try:
    # Low-level PicoScope 2000 API
    from picosdk.ps2000 import ps2000
    from picosdk.functions import assert_pico2000_ok, adc2mV
    from picosdk.PicoDeviceEnums import picoEnum
    import ctypes
except ImportError:
    ps2000 = None
    assert_pico2000_ok = None
    adc2mV = None
    picoEnum = None
    ctypes = None

SAMPLES_DEFAULT = 2000
OVERSAMPLING_DEFAULT = 1

# Robust waveform mapping. If ps provides enums use them; otherwise fall back to simple ints.
if ps2000 is not None and hasattr(ps2000, "PS2000_WAVE_TYPE"):
    WAVE_TYPES = {
        "SIN": ps2000.PS2000_WAVE_TYPE.get("PS2000_SINE", 0),
        "SQU": ps2000.PS2000_WAVE_TYPE.get("PS2000_SQUARE", 1),
        "TRI": ps2000.PS2000_WAVE_TYPE.get("PS2000_TRIANGLE", 2),
        "DC": ps2000.PS2000_WAVE_TYPE.get("PS2000_DC_VOLTAGE", 5),
    }
else:
    WAVE_TYPES = {"SIN": 0, "SQU": 1, "TRI": 2, "DC": 5}

# Robust sweep-type mapping
if ps2000 is not None and hasattr(ps2000, "PS2000_SWEEP_TYPE"):
    SWEEP_UP = ps2000.PS2000_SWEEP_TYPE.get("PS2000_UP", 0)
else:
    SWEEP_UP = 0

class PicoScopeError(RuntimeError):
    pass

class PicoScope2000:
    """PicoScope 2000 AWG driver with safe ctypes handling and fallbacks for wrapper names.

    This class attempts to call whichever ps function is available (the wrapper may expose
    either `_open_unit` or `ps2000_open_unit` as an attribute).
    """
    def __init__(self, log):
        if ps2000 is None or ctypes is None:
            raise RuntimeError("picosdk not available")
        self.log = log
        self.handle: Optional[int] = None
        self.connected = False
        self.output_on = False

    def _get_fn(self, *names):
        """Return the first callable attribute on ps matching any of the provided names."""
        for n in names:
            fn = getattr(ps2000, n, None)
            if callable(fn):
                return fn
        return None

    def connect(self):
        fn = self._get_fn("_open_unit", "ps2000_open_unit", "open_unit")
        if fn is None:
            raise PicoScopeError("open_unit function not found in ps wrapper")
        handle = fn()
        try:
            h = int(handle)
        except Exception:
            h = handle
        if not isinstance(h, int) or h <= 0:
            raise PicoScopeError(f"Could not open device (handle={h})")
        self.handle = h
        self.connected = True
        self.log(f"PicoScope2000: connected (handle={self.handle})")

    def close(self):
        if not self.connected or self.handle is None:
            return
        fn = self._get_fn("_close_unit", "ps2000_close_unit", "close_unit")
        if fn is None:
            # best-effort: just mark disconnected
            self.log("PicoScope2000: close function not found, marking as disconnected")
            self.connected = False
            self.handle = None
            return
        try:
            fn(self.handle)
        except Exception as e:
            # driver close can raise; log but keep going
            self.log(f"PicoScope close error: {e}")
        self.connected = False
        self.handle = None
        self.log("PicoScope2000: disconnected")

    def set_waveform(self, shape="SIN", f_start=1000.0, f_stop=None,
                     vpp=2.0, offset=0.0, increment=0.0, dwell_time=1.0,
                     sweep_enabled=False):
        if not self.connected or self.handle is None:
            raise PicoScopeError("Device not connected")

        # pick wave type numeric code
        wave = WAVE_TYPES.get(shape.upper(), WAVE_TYPES["SIN"]) if WAVE_TYPES else 0

        # driver expects mV for pk-to-pk and offset in mV
        pkToPk = int(round(vpp * 1000.0))
        dcOffset = int(round(offset * 1000.0))

        fStop_safe = f_stop if (f_stop is not None) else f_start
        increment_safe = increment if (increment is not None) else 0.0
        dwell_safe = dwell_time if (dwell_time is not None) else 0.0

        if sweep_enabled and fStop_safe > f_start and increment_safe > 0:
            self.log(f"Configuring sweep {f_start} → {fStop_safe} Hz step {increment_safe}, dwell {dwell_safe}s")
            sweep_type = SWEEP_UP
            sweeps = 1
        else:
            fStop_safe = f_start
            increment_safe = 0.0
            dwell_safe = 0.0
            sweep_type = 0
            sweeps = 0
            self.log(f"Configuring CW {shape} at {f_start} Hz, {vpp} Vpp, offset {offset} V")

        fn = self._get_fn("_set_sig_gen_built_in", "ps2000_set_sig_gen_built_in")
        if fn is None:
            raise PicoScopeError("set_sig_gen_built_in API not found in ps wrapper")

        # signature (wrapper): (int16 handle, int32 offsetVoltage, uint32 pkToPk, int32 waveType,
        #                         float startFrequency, float stopFrequency, float increment,
        #                         float dwellTime, int32 sweepType, uint32 sweeps)
        try:
            fn(
                ctypes.c_int16(self.handle),
                ctypes.c_int32(dcOffset),
                ctypes.c_uint32(pkToPk),
                ctypes.c_int32(int(wave)),
                ctypes.c_float(float(f_start)),
                ctypes.c_float(float(fStop_safe)),
                ctypes.c_float(float(increment_safe)),
                ctypes.c_float(float(dwell_safe)),
                ctypes.c_int32(int(sweep_type)),
                ctypes.c_uint32(int(sweeps)),
            )
        except TypeError:
            # Some wrapper variants expose the handle as a plain int param in Python; try that
            fn(
                self.handle,
                int(dcOffset),
                int(pkToPk),
                int(wave),
                float(f_start),
                float(fStop_safe),
                float(increment_safe),
                float(dwell_safe),
                int(sweep_type),
                int(sweeps),
            )
        self.log("PicoScope2000: waveform configured")

    def start_output(self):
        if not self.connected or self.handle is None:
            raise PicoScopeError("Device not connected")
        # The ps2000 wrapper's set_sig_gen_built_in configures the AWG. Many Pico drivers
        # start output immediately after configuration; others need an explicit start.
        # To keep this UI safe we treat start_output as a state flag and log it.
        self.output_on = True
        self.log("PicoScope2000: output started")

    def stop_output(self):
        if not self.connected or self.handle is None:
            raise PicoScopeError("Device not connected")
        # Set DC 0V to stop any AC output (best-effort). Use same API to force DC 0V.
        fn = self._get_fn("_set_sig_gen_built_in", "ps2000_set_sig_gen_built_in")
        if fn is not None:
            try:
                fn(
                    ctypes.c_int16(self.handle),
                    ctypes.c_int32(0),
                    ctypes.c_uint32(0),
                    ctypes.c_int32(WAVE_TYPES.get("DC", 5)),
                    ctypes.c_float(0),
                    ctypes.c_float(0),
                    ctypes.c_float(0),
                    ctypes.c_float(0),
                    ctypes.c_int32(0),
                    ctypes.c_uint32(0),
                )
            except Exception as e:
                # swallow low-level errors but log them
                self.log(f"PicoScope stop_output warning: {e}")
        self.output_on = False
        self.log("PicoScope2000: output stopped (set to DC 0V)")

    def read_channel_b(self, n_samples=SAMPLES_DEFAULT, wanted_time_interval=4000):
        """Capture Channel B waveform using run_block + get_times_and_values."""
        if not self.connected or self.handle is None:
            raise PicoScopeError("Device not connected")

        # Enable Channel B
        res = ps2000.ps2000_set_channel(
            self.handle,
            picoEnum.PICO_CHANNEL['PICO_CHANNEL_B'],
            True,
            picoEnum.PICO_COUPLING['PICO_DC'],
            ps2000.PS2000_VOLTAGE_RANGE['PS2000_50MV'],
        )
        assert_pico2000_ok(res)

        # Determine suitable timebase
        current_timebase = 1
        time_interval = c_int32(0)
        time_units = c_int16()
        max_samples = c_int32()
        old_interval = None

        while ps2000.ps2000_get_timebase(
                self.handle,
                current_timebase,
                n_samples,
                byref(time_interval),
                byref(time_units),
                1,
                byref(max_samples)
        ) == 0 or time_interval.value < wanted_time_interval:
            old_interval = time_interval.value
            current_timebase += 1
            if current_timebase.bit_length() > sizeof(c_int16) * 8:
                raise PicoScopeError("No appropriate timebase found")

        timebase = current_timebase - 1

        # Start block capture
        collection_time = c_int32()
        res = ps2000.ps2000_run_block(
            self.handle,
            n_samples,
            timebase,
            OVERSAMPLING_DEFAULT,
            byref(collection_time)
        )
        assert_pico2000_ok(res)

        # Wait until capture is complete
        while ps2000.ps2000_ready(self.handle) == 0:
            sleep(0.01)

        # Allocate buffers
        times = (c_int32 * n_samples)()
        buffer_b = (c_int16 * n_samples)()
        overflow = c_byte(0)

        # Retrieve samples
        res = ps2000.ps2000_get_times_and_values(
            self.handle,
            byref(times),
            None,
            byref(buffer_b),
            None,
            None,
            byref(overflow),
            2,  # time units: ns
            n_samples
        )
        assert_pico2000_ok(res)

        # Convert ADC → mV
        channel_b_mv = adc2mV(
            buffer_b,
            ps2000.PS2000_VOLTAGE_RANGE['PS2000_50MV'],
            c_int16(32767)
        )

        # Convert ctypes arrays to plain Python lists
        times_ms = [t * 1e-6 for t in times]  # ns → ms
        channel_b_mv = list(channel_b_mv)

        return times_ms, channel_b_mv, overflow.value

# -------------------------
# SIOS controller (unchanged)
# -------------------------
try:
    import serial
    import serial.tools.list_ports as list_ports
except Exception:
    serial = None
    list_ports = None

@dataclass
class SIOSConfig:
    port: str = "COM4"
    baudrate: int = 9600
    timeout_s: float = 1.0
    samplerate: int = 210000        # Hz
    blocksize: int = 4096          # samples per block
    interval_ms: int = 1000       # measurement interval in ms

class SIOSLSV2500NG:
    def __init__(self, cfg: SIOSConfig, log):
        self.cfg = cfg
        self.ser: Optional[serial.Serial] = None
        self.log = log

    def connect(self):
        if serial is None:
            raise RuntimeError("pyserial not installed")
        self.ser = serial.Serial(self.cfg.port, self.cfg.baudrate, timeout=self.cfg.timeout_s)
        self.log(f"SIOS connected to {self.cfg.port}@{self.cfg.baudrate}")

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.ser = None
        self.log("SIOS disconnected")

    def send(self, cmd: str) -> bytes:
        if not self.ser or not self.ser.is_open:
            raise RuntimeError("SIOS not connected")
        data = (cmd + "\r\n").encode()
        self.ser.reset_input_buffer()
        self.ser.write(data)
        self.ser.flush()
        time.sleep(0.02)
        resp = self.ser.read_until(b"\n")
        self.log(f"> {cmd} < {resp.decode(errors='ignore').strip()}")
        return resp

    # ---------------- Device Commands ----------------
    def start_measurement(self):
        self.send("MEAS:START")

    def stop_measurement(self):
        self.send("MEAS:STOP")

    def set_samplerate(self, hz: int):
        self.cfg.samplerate = hz
        self.send(f"MEAS:SAMPLERATE {hz}")

    def set_blocksize(self, size: int):
        self.cfg.blocksize = size
        self.send(f"MEAS:BLOCKSIZE {size}")

    def set_measurement_interval(self, ms: int):
        self.cfg.interval_ms = ms
        # Reuse integration time command if device uses it for interval
        self.send(f"MEAS:INT {ms}")

    def read_value(self) -> Optional[float]:
        resp = self.send("MEAS:READ?")
        try:
            return float(resp.decode().strip())
        except Exception:
            return None


# -------------------------
# GUI
# -------------------------
class App(ttk.Frame):
    def __init__(self, master):
        super().__init__(master)
        self.master = master
        self.pack(fill=tk.BOTH, expand=True)
        self.log_queue: "queue.Queue[str]" = queue.Queue()
        self.after(100, self._drain_log)

        # PicoScope object (may be None if picosdk not installed)
        if ps2000 is None:
            messagebox.showwarning("PicoScope", "picosdk not found, PicoScope will not work")
            self.pico = None
        else:
            try:
                self.pico = PicoScope2000(self._log)
            except RuntimeError:
                self.pico = None

        # SIOS
        self.sios = SIOSLSV2500NG(SIOSConfig(), self._log)

        # UI state vars
        self.var_status = tk.StringVar(value="Idle")

        self._build_ui()

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

    def _build_ui(self):
        nb = ttk.Notebook(self)
        nb.pack(fill=tk.BOTH, expand=True, padx=10, pady=8)
        self.tab_pico = ttk.Frame(nb)
        self.tab_sios = ttk.Frame(nb)
        self.tab_log = ttk.Frame(nb)
        nb.add(self.tab_pico, text="PicoScope AWG")
        nb.add(self.tab_sios, text="SIOS LSV 2500 NG")
        nb.add(self.tab_log, text="Log & Action")
        self._build_pico_tab()
        self._build_sios_tab()
        self._build_log_tab()

    # ----------------- Pico Tab -----------------
    def _build_pico_tab(self):
        frm = ttk.Frame(self.tab_pico)
        frm.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)

        self.var_shape = tk.StringVar(value="SIN")
        self.var_f_start = tk.DoubleVar(value=1000.0)
        self.var_f_stop = tk.DoubleVar(value=1000.0)
        self.var_increment = tk.DoubleVar(value=0.0)
        self.var_dwell = tk.DoubleVar(value=1.0)
        self.var_vpp = tk.DoubleVar(value=2.0)
        self.var_offset = tk.DoubleVar(value=0.0)
        self.var_sweep = tk.BooleanVar(value=False)
        self.var_output = tk.BooleanVar(value=False)

        ttk.Label(frm, text="Waveform").grid(row=0, column=0)
        ttk.Combobox(frm, textvariable=self.var_shape, values=list(WAVE_TYPES.keys()), width=8).grid(row=0, column=1)
        ttk.Label(frm, text="f start [Hz]").grid(row=1, column=0)
        ttk.Entry(frm, textvariable=self.var_f_start, width=10).grid(row=1, column=1)
        ttk.Label(frm, text="f stop [Hz]").grid(row=2, column=0)
        ttk.Entry(frm, textvariable=self.var_f_stop, width=10).grid(row=2, column=1)
        ttk.Label(frm, text="Increment [Hz]").grid(row=3, column=0)
        ttk.Entry(frm, textvariable=self.var_increment, width=10).grid(row=3, column=1)
        ttk.Label(frm, text="Dwell [s]").grid(row=4, column=0)
        ttk.Entry(frm, textvariable=self.var_dwell, width=10).grid(row=4, column=1)
        ttk.Label(frm, text="Vpp [V]").grid(row=5, column=0)
        ttk.Entry(frm, textvariable=self.var_vpp, width=10).grid(row=5, column=1)
        ttk.Label(frm, text="Offset [V]").grid(row=6, column=0)
        ttk.Entry(frm, textvariable=self.var_offset, width=10).grid(row=6, column=1)

        ttk.Checkbutton(frm, text="Enable Sweep", variable=self.var_sweep).grid(row=7, column=0, columnspan=2, sticky="w")
        ttk.Checkbutton(frm, text="Output ON", variable=self.var_output, command=self._pico_toggle).grid(row=8, column=0, columnspan=2, sticky="w")

        ttk.Button(frm, text="Connect", command=self._pico_connect).grid(row=9, column=0, padx=2, pady=4)
        ttk.Button(frm, text="Disconnect", command=self._pico_disconnect).grid(row=9, column=1, padx=2, pady=4)
        ttk.Button(frm, text="Apply", command=self._pico_apply).grid(row=10, column=0, columnspan=2, pady=4)

        ttk.Label(frm, textvariable=self.var_status).grid(row=11, column=0, columnspan=2, sticky="w")

        self.fig_b = Figure(figsize=(5, 3), dpi=100)
        self.ax_b = self.fig_b.add_subplot(111)
        self.ax_b.set_title("PicoScope Channel B")
        self.ax_b.set_xlabel("Sample")
        self.ax_b.set_ylabel("Voltage [V]")
        self.line_b, = self.ax_b.plot([], [], "-")

        canvas_b = FigureCanvasTkAgg(self.fig_b, master=frm)
        canvas_b.get_tk_widget().grid(row=12, column=0, columnspan=2, pady=8)
        self.canvas_b = canvas_b

        self.pico_data_b = deque(maxlen=2000)

        ttk.Button(frm, text="Read Channel B", command=self._pico_read_b).grid(row=13, column=0, columnspan=2, pady=4)

    def _pico_connect(self):
        if self.pico is None:
            self._log("Pico: picosdk not available")
            self.var_status.set("Pico: unavailable")
            return
        def task():
            try:
                self.pico.connect()
                self.var_status.set(f"Pico: connected (h={self.pico.handle})")
            except Exception as e:
                self._log(f"Pico ERROR: {e}")
                self.var_status.set(f"Pico: connect failed")
        threading.Thread(target=task, daemon=True).start()

    def _pico_disconnect(self):
        if self.pico is None:
            return
        def task():
            try:
                self.pico.close()
                self.var_status.set("Pico: disconnected")
            except Exception as e:
                self._log(f"Pico ERROR: {e}")
        threading.Thread(target=task, daemon=True).start()

    def _pico_apply(self):
        if self.pico is None:
            self._log("Pico: picosdk not available")
            return
        def task():
            try:
                self.var_status.set("Pico: applying configuration...")
                self.pico.set_waveform(
                    shape=self.var_shape.get(),
                    f_start=self.var_f_start.get(),
                    f_stop=self.var_f_stop.get(),
                    increment=self.var_increment.get(),
                    dwell_time=self.var_dwell.get(),
                    vpp=self.var_vpp.get(),
                    offset=self.var_offset.get(),
                    sweep_enabled=self.var_sweep.get(),
                )
                # If output checkbox is checked, also start output
                if self.var_output.get():
                    self.pico.start_output()
                self.var_status.set("Pico: configuration applied")
                self._log(f"PicoScope: Applied {self.var_shape.get()} {self.var_f_start.get()}Hz {self.var_vpp.get()}Vpp offset {self.var_offset.get()}")
            except Exception as e:
                self._log(f"Pico ERROR: {e}")
                self.var_status.set("Pico: apply failed")
        threading.Thread(target=task, daemon=True).start()

    def _pico_toggle(self):
        if self.pico is None:
            self._log("Pico: picosdk not available")
            self.var_output.set(False)
            return
        on = self.var_output.get()
        def task():
            try:
                if on:
                    self.pico.start_output()
                    self.var_status.set("Pico: output ON")
                else:
                    self.pico.stop_output()
                    self.var_status.set("Pico: output OFF")
            except Exception as e:
                self._log(f"Pico ERROR: {e}")
                # revert the checkbox
                self.var_output.set(not on)
        threading.Thread(target=task, daemon=True).start()

    def _pico_read_b(self):
        """Read Channel B from PicoScope and update plot (threaded)."""
        if self.pico is None:
            self._log("Pico: not available")
            return

        def task():
            try:
                times_ms, ch_b_mv, overflow = self.pico.read_channel_b(n_samples=1000)
                # Store the data and times for plotting
                self.pico_data_b.clear()
                self.pico_data_b.extend(ch_b_mv)
                self.pico_data_b_times = times_ms
                self.pico_overflow = overflow
                self._update_pico_plot()
            except Exception as e:
                self._log(f"Pico ERROR: {e}")

        threading.Thread(target=task, daemon=True).start()

    def _update_pico_plot(self):
        # Convert deque to list for plotting
        ydata = list(self.pico_data_b)
        xdata = list(range(len(ydata)))

        # Update existing line instead of clearing the axes
        if hasattr(self, 'line_b') and self.line_b in self.ax_b.lines:
            self.line_b.set_data(xdata, ydata)
        else:
            # If line doesn't exist yet, create it
            self.line_b, = self.ax_b.plot(xdata, ydata, '-')

        # Adjust the view limits
        self.ax_b.relim()
        self.ax_b.autoscale_view()

        # Refresh the canvas
        self.canvas_b.draw_idle()

    # ----------------- SIOS Tab -----------------
    def _build_sios_tab(self):
        frm = ttk.Frame(self.tab_sios)
        frm.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)

        self.var_sios_port = tk.StringVar(value=self.sios.cfg.port)
        self.var_sios_baud = tk.IntVar(value=self.sios.cfg.baudrate)
        self.var_samplerate = tk.IntVar(value=1000)  # in Hz
        self.var_blocksize = tk.IntVar(value=100)  # number of samples per block
        self.var_interval = tk.IntVar(value=1000)  # measurement interval in ms
        self.var_val = tk.StringVar(value="—")

        # --- Connection controls ---
        ttk.Label(frm, text="Port").grid(row=0, column=0)
        ttk.Entry(frm, textvariable=self.var_sios_port, width=10).grid(row=0, column=1)
        ttk.Label(frm, text="Baud").grid(row=0, column=2)
        ttk.Entry(frm, textvariable=self.var_sios_baud, width=8).grid(row=0, column=3)
        ttk.Button(frm, text="Connect", command=self._sios_connect).grid(row=0, column=4)
        ttk.Button(frm, text="Disconnect", command=self._sios_disconnect).grid(row=0, column=5)

        # --- Measurement settings ---
        ttk.Label(frm, text="Sample rate [Hz]").grid(row=1, column=0)
        ttk.Entry(frm, textvariable=self.var_samplerate, width=10).grid(row=1, column=1)
        ttk.Label(frm, text="Block size").grid(row=1, column=2)
        ttk.Entry(frm, textvariable=self.var_blocksize, width=10).grid(row=1, column=3)
        ttk.Label(frm, text="Interval [ms]").grid(row=1, column=4)
        ttk.Entry(frm, textvariable=self.var_interval, width=10).grid(row=1, column=5)

        ttk.Button(frm, text="Set", command=self._sios_set_params).grid(row=2, column=0, pady=4)
        ttk.Button(frm, text="Start", command=self._sios_start).grid(row=2, column=1)
        ttk.Button(frm, text="Stop", command=self._sios_stop).grid(row=2, column=2)
        ttk.Button(frm, text="Read", command=self._sios_read).grid(row=2, column=3)

        ttk.Label(frm, textvariable=self.var_val).grid(row=3, column=0, columnspan=6, sticky="w")

        # --- Matplotlib plot ---
        self.fig = Figure(figsize=(5, 3), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_title("SIOS Measurement")
        self.ax.set_xlabel("Sample")
        self.ax.set_ylabel("Value")
        self.line, = self.ax.plot([], [], "-o")

        canvas = FigureCanvasTkAgg(self.fig, master=frm)
        canvas.get_tk_widget().grid(row=4, column=0, columnspan=6, sticky="nsew", pady=8)
        self.canvas = canvas

        # Rolling buffer for last 100 values
        self.sios_data = deque(maxlen=100)

    def _sios_connect(self):
        self.sios.cfg.port = self.var_sios_port.get().strip()
        self.sios.cfg.baudrate = self.var_sios_baud.get()
        def task():
            try:
                self.sios.connect()
            except Exception as e:
                self._log(f"SIOS ERROR: {e}")
        threading.Thread(target=task, daemon=True).start()

    def _sios_disconnect(self):
        def task():
            try:
                self.sios.close()
            except Exception as e:
                self._log(f"SIOS ERROR: {e}")
        threading.Thread(target=task, daemon=True).start()

    def _sios_set_params(self):
        samplerate = self.var_samplerate.get()
        blocksize = self.var_blocksize.get()
        interval = self.var_interval.get()

        def task():
            try:
                self.sios.set_samplerate(samplerate)
                self.sios.set_blocksize(blocksize)
                self.sios.set_measurement_interval(interval)
                self._log(f"SIOS: samplerate={samplerate}Hz, blocksize={blocksize}, interval={interval}ms")
            except Exception as e:
                self._log(f"SIOS ERROR: {e}")

        threading.Thread(target=task, daemon=True).start()

    def _sios_start(self):
        def task():
            try:
                self.sios.start_measurement()
            except Exception as e:
                self._log(f"SIOS ERROR: {e}")
        threading.Thread(target=task, daemon=True).start()

    def _sios_stop(self):
        def task():
            try:
                self.sios.stop_measurement()
            except Exception as e:
                self._log(f"SIOS ERROR: {e}")
        threading.Thread(target=task, daemon=True).start()

    def _sios_read(self):
        def task():
            try:
                val = self.sios.read_value()
                if val is not None:
                    self.var_val.set(str(val))
                    self.sios_data.append(val)
                    self._update_plot()
                else:
                    self.var_val.set("—")
            except Exception as e:
                self._log(f"SIOS ERROR: {e}")
        threading.Thread(target=task, daemon=True).start()

    def _update_plot(self):
        ydata = list(self.sios_data)
        xdata = list(range(len(ydata)))
        self.line.set_data(xdata, ydata)
        self.ax.relim()
        self.ax.autoscale_view()
        self.canvas.draw_idle()

    # ----------------- Log Tab -----------------
    def _build_log_tab(self):
        frm = ttk.Frame(self.tab_log)
        frm.pack(fill=tk.BOTH, expand=True)
        self.txt_log = tk.Text(frm, height=16, state=tk.DISABLED)
        self.txt_log.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)
        btns = ttk.Frame(frm)
        btns.pack(fill=tk.X, padx=8, pady=4)
        ttk.Button(btns, text="Clear", command=lambda: self.txt_log.delete("1.0", tk.END)).pack(side=tk.LEFT)
        ttk.Button(btns, text="Copy", command=lambda: self.master.clipboard_append(self.txt_log.get("1.0", tk.END))).pack(side=tk.LEFT)

# -------------------------
# Main
# -------------------------

def main():
    root = tk.Tk()
    root.title("PicoScope 2000 & SIOS LSV 2500 NG")
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()
