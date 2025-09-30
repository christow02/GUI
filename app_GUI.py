import tkinter as tk
from tkinter import ttk, messagebox
import threading
import queue
import time
import ctypes
from ctypes import c_int32, c_double
import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from collections import deque

# -------------------------
# PicoScope controller
# -------------------------
try:
    from picosdk.ps2000 import ps2000
    from picosdk.functions import assert_pico2000_ok, adc2mV
    from picosdk.PicoDeviceEnums import picoEnum
    from ctypes import c_int16, c_int32 as c_int32b, c_byte, byref
except ImportError:
    ps2000 = None
    assert_pico2000_ok = None
    adc2mV = None
    picoEnum = None
    c_int16 = None
    c_int32b = None
    c_byte = None
    byref = None

SAMPLES_DEFAULT = 10000
OVERSAMPLING_DEFAULT = 1
WAVE_TYPES = {"SIN": 0, "SQU": 1, "TRI": 2, "DC": 5}


class PicoScopeError(RuntimeError):
    pass


class PicoScope2000:
    def __init__(self, log):
        if ps2000 is None:
            raise RuntimeError("picosdk not available")
        self.log = log
        self.handle = None
        self.connected = False
        self.output_on = False

    def connect(self):
        h = ps2000.ps2000_open_unit()
        if not isinstance(h, int) or h <= 0:
            raise PicoScopeError(f"Could not open device (handle={h})")
        self.handle = h
        self.connected = True
        self.log(f"PicoScope2000: connected (handle={self.handle})")

    def close(self):
        if self.connected and self.handle:
            ps2000.ps2000_close_unit(self.handle)
        self.connected = False
        self.handle = None
        self.log("PicoScope2000: disconnected")

    def set_waveform(self, shape="SIN", f_start=1000.0, vpp=2.0, offset=0.0):
        if not self.connected or self.handle is None:
            raise PicoScopeError("Device not connected")
        wave = WAVE_TYPES.get(shape.upper(), 0)
        pkToPk = int(round(vpp * 1_000_000.0))
        dcOffset = int(round(offset * 1_000_000.0))
        ps2000.ps2000_set_sig_gen_built_in(
            self.handle,
            dcOffset,
            pkToPk,
            wave,
            float(f_start),
            float(f_start),
            0.0,
            0.0,
            0,
            0,
        )
        self.log(f"Configured {shape} {f_start} Hz {vpp} Vpp offset {offset} V")

    def start_output(self):
        if not self.connected:
            raise PicoScopeError("Device not connected")
        self.output_on = True
        self.log("PicoScope2000: output started")

    def stop_output(self):
        if not self.connected:
            return
        ps2000.ps2000_set_sig_gen_built_in(
            self.handle, 0, 0, WAVE_TYPES["DC"], 0, 0, 0, 0, 0, 0
        )
        self.output_on = False
        self.log("PicoScope2000: output stopped (DC 0V)")

    def read_channel_b(self, f_target=1000.0, cycles=10, points_per_cycle=100):
        if not self.connected or self.handle is None:
            raise PicoScopeError("Device not connected")

        n_samples = int(cycles * points_per_cycle)
        wanted_dt = 1.0 / (f_target * points_per_cycle)

        ps2000.ps2000_set_channel(
            self.handle,
            picoEnum.PICO_CHANNEL["PICO_CHANNEL_B"],
            True,
            picoEnum.PICO_COUPLING["PICO_DC"],
            ps2000.PS2000_VOLTAGE_RANGE["PS2000_10V"],
        )

        time_interval = c_int32b()
        time_units = c_int16()
        max_samples = c_int32b()
        timebase = 1

        while True:
            ok = ps2000.ps2000_get_timebase(
                self.handle,
                timebase,
                n_samples,
                byref(time_interval),
                byref(time_units),
                OVERSAMPLING_DEFAULT,
                byref(max_samples),
            )
            if ok and time_interval.value >= wanted_dt * 1e9:
                break
            timebase += 1
            if timebase > 20000:
                raise PicoScopeError("No suitable timebase found")

        collection_time = c_int32b()
        ps2000.ps2000_run_block(
            self.handle, n_samples, timebase, OVERSAMPLING_DEFAULT, byref(collection_time)
        )

        while ps2000.ps2000_ready(self.handle) == 0:
            time.sleep(0.01)

        buffer_b = (c_int16 * n_samples)()
        overflow = c_byte(0)
        ps2000.ps2000_get_values(self.handle, None, buffer_b, None, None, byref(overflow), n_samples)

        channel_b_mv = adc2mV(buffer_b, ps2000.PS2000_VOLTAGE_RANGE["PS2000_10V"], c_int16(32767))
        channel_b_v = [v / 1000.0 for v in channel_b_mv]
        dt = time_interval.value * 1e-9
        times = [i * dt for i in range(n_samples)]
        return times, channel_b_v, overflow.value


# -------------------------
# SIOS controller (DLL-based)
# -------------------------
DLL_PATH = r"C:\Users\ge42tak\Downloads\GUI-main\GUI-main\siosifm.dll"
IFM_MEAS_ONECHANNEL = 0x0100
IFM_MEAS_LENGTH = 0x0002


class SIOS:
    def __init__(self, log):
        self.dll = None
        self.dev = None
        self.log = log
        self.running = False

    def connect(self):
        try:
            if self.dll is None:
                self.dll = ctypes.WinDLL(DLL_PATH)

                # Signatures
                self.dll.IfmInit.restype = c_int32
                self.dll.IfmEnumSearchDevices.restype = c_int32
                self.dll.IfmEnumOpen.argtypes = [c_int32]
                self.dll.IfmEnumOpen.restype = c_int32
                self.dll.IfmCloseDevice.argtypes = [c_int32]
                self.dll.IfmCloseDevice.restype = None
                self.dll.IfmClose.restype = None

                self.dll.IfmSetMeasurement.argtypes = [c_int32, ctypes.c_uint32, c_double]
                self.dll.IfmSetMeasurement.restype = c_int32
                self.dll.IfmStart.argtypes = [c_int32]; self.dll.IfmStart.restype = c_int32
                self.dll.IfmStop.argtypes = [c_int32]; self.dll.IfmStop.restype = c_int32

                self.dll.IfmValueCount.argtypes = [c_int32]; self.dll.IfmValueCount.restype = c_int32
                self.dll.IfmGetValues.argtypes = [c_int32]; self.dll.IfmGetValues.restype = c_int32

                self.dll.IfmLengthValue.argtypes = [c_int32, c_int32]
                self.dll.IfmLengthValue.restype = c_double

            ret = self.dll.IfmInit()
            self.log(f"IfmInit -> {ret}")
            cnt = self.dll.IfmEnumSearchDevices()
            self.log(f"Devices found: {cnt}")
            if cnt <= 0:
                raise RuntimeError("No SIOS devices found")
            self.dev = self.dll.IfmEnumOpen(0)
            self.log(f"Opened device handle: {self.dev}")
        except Exception as e:
            self.log(f"SIOS connect error: {e}")
            raise

    def disconnect(self):
        try:
            if self.dll and self.dev is not None:
                self.dll.IfmCloseDevice(self.dev)
                self.dll.IfmClose()
                self.log("SIOS: device closed")
                self.dev = None
            self.running = False
        except Exception as e:
            self.log(f"SIOS disconnect error: {e}")

    def start_measurement(self, samplerate):
        flags = IFM_MEAS_ONECHANNEL | IFM_MEAS_LENGTH
        ret = self.dll.IfmSetMeasurement(self.dev, flags, float(samplerate))
        self.log(f"IfmSetMeasurement -> {ret}")
        ret = self.dll.IfmStart(self.dev)
        self.log(f"IfmStart -> {ret}")
        self.running = True

    def stop_measurement(self):
        if self.dev is not None:
            self.dll.IfmStop(self.dev)
            self.log("Measurement stopped")
        self.running = False

    def read_value(self, channel=0):
        if not self.running:
            return None
        cnt = self.dll.IfmValueCount(self.dev)
        if cnt > 0:
            self.dll.IfmGetValues(self.dev)
            val = self.dll.IfmLengthValue(self.dev, channel)
            return val  # nanometers from DLL
        return None


# -------------------------
# GUI
# -------------------------
class App(ttk.Frame):
    def __init__(self, master):
        super().__init__(master)
        self.master = master
        self.pack(fill=tk.BOTH, expand=True)
        self.log_queue = queue.Queue()
        self.after(100, self._drain_log)

        # Instruments
        try:
            self.pico = PicoScope2000(self._log) if ps2000 else None
        except Exception as e:
            self._log(f"Pico init error: {e}")
            self.pico = None
        self.sios = SIOS(self._log)

        self._build_ui()

        # SIOS state
        self.sios_data = []
        self.offset = 0.0
        self.auto_zero_done = False
        self.after(100, self._sios_update_plot)

    # -------- Logging --------
    def _log(self, msg):
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

    # -------- UI --------
    def _build_ui(self):
        nb = ttk.Notebook(self)
        nb.pack(fill=tk.BOTH, expand=True, padx=10, pady=8)
        self.tab_pico = ttk.Frame(nb)
        self.tab_sios = ttk.Frame(nb)
        self.tab_log = ttk.Frame(nb)
        nb.add(self.tab_pico, text="PicoScope AWG")
        nb.add(self.tab_sios, text="SIOS LSV 2500 NG (DLL)")
        nb.add(self.tab_log, text="Log & Action")
        self._build_pico_tab()
        self._build_sios_tab()
        self._build_log_tab()

    # -------- Pico Tab --------
    def _build_pico_tab(self):
        frm = ttk.Frame(self.tab_pico)
        frm.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)

        self.var_shape = tk.StringVar(value="SIN")
        self.var_f_start = tk.DoubleVar(value=1000.0)
        self.var_vpp = tk.DoubleVar(value=2.0)
        self.var_offset = tk.DoubleVar(value=0.0)
        self.var_output = tk.BooleanVar(value=False)

        ttk.Label(frm, text="Waveform").grid(row=0, column=0)
        ttk.Combobox(frm, textvariable=self.var_shape,
                     values=list(WAVE_TYPES.keys()), width=8, state="readonly").grid(row=0, column=1)

        ttk.Label(frm, text="Frequency [Hz]").grid(row=1, column=0)
        ttk.Entry(frm, textvariable=self.var_f_start, width=10).grid(row=1, column=1)

        ttk.Label(frm, text="Vpp [V]").grid(row=2, column=0)
        ttk.Entry(frm, textvariable=self.var_vpp, width=10).grid(row=2, column=1)

        ttk.Label(frm, text="Offset [V]").grid(row=3, column=0)
        ttk.Entry(frm, textvariable=self.var_offset, width=10).grid(row=3, column=1)

        ttk.Checkbutton(frm, text="Output ON", variable=self.var_output,
                        command=self._pico_toggle).grid(row=4, column=0, columnspan=2, sticky="w")

        ttk.Button(frm, text="Connect", command=self._pico_connect).grid(row=9, column=0, padx=2, pady=4)
        ttk.Button(frm, text="Disconnect", command=self._pico_disconnect).grid(row=9, column=1, padx=2, pady=4)
        ttk.Button(frm, text="Apply", command=self._pico_apply).grid(row=10, column=0, columnspan=2, pady=4)

        self.fig_b = Figure(figsize=(5, 3), dpi=100)
        self.ax_b = self.fig_b.add_subplot(111)
        self.ax_b.set_title("PicoScope Channel B")
        self.ax_b.set_xlabel("Time [s]")
        self.ax_b.set_ylabel("Voltage [V]")
        self.fig_b.subplots_adjust(left=0.17, bottom=0.2)
        self.line_b, = self.ax_b.plot([], [], "-")

        canvas_b = FigureCanvasTkAgg(self.fig_b, master=frm)
        canvas_b.get_tk_widget().grid(row=12, column=0, columnspan=2, pady=8)
        self.canvas_b = canvas_b

        self.pico_data_b = deque(maxlen=2000)
        ttk.Button(frm, text="Read Channel B", command=self._pico_read_b).grid(row=13, column=0, columnspan=2, pady=4)

    def _pico_connect(self):
        if self.pico is None:
            messagebox.showwarning("PicoScope", "picosdk not available")
            return
        def task():
            try:
                self.pico.connect()
                self._log("Pico: connected")
            except Exception as e:
                self._log(f"Pico ERROR: {e}")
        threading.Thread(target=task, daemon=True).start()

    def _pico_disconnect(self):
        if self.pico is None:
            return
        threading.Thread(target=self.pico.close, daemon=True).start()

    def _pico_apply(self):
        if self.pico is None:
            return
        def task():
            try:
                shape = self.var_shape.get()
                f_start = float(self.var_f_start.get())
                vpp = float(self.var_vpp.get())
                offset = float(self.var_offset.get())
                if not self.var_output.get():
                    self._log("Pico: output OFF (checkbox not ticked)")
                    return
                self.pico.set_waveform(shape, f_start, vpp, offset)
                self._log("Pico: configuration applied")
            except Exception as e:
                self._log(f"Pico ERROR: {e}")
        threading.Thread(target=task, daemon=True).start()

    def _pico_toggle(self):
        if self.pico is None:
            self.var_output.set(False)
            return
        def task():
            try:
                if self.var_output.get():
                    self.pico.start_output()
                else:
                    self.pico.stop_output()
            except Exception as e:
                self._log(f"Pico ERROR: {e}")
        threading.Thread(target=task, daemon=True).start()

    def _pico_read_b(self):
        if self.pico is None:
            return
        def task():
            try:
                f_target = float(self.var_f_start.get())
                times, ch_b_v, overflow = self.pico.read_channel_b(f_target=f_target)
                self.pico_data_b.clear()
                self.pico_data_b.extend(ch_b_v)
                self.line_b.set_data(times, list(self.pico_data_b))
                self.ax_b.relim(); self.ax_b.autoscale_view()
                self.canvas_b.draw_idle()
            except Exception as e:
                self._log(f"Pico ERROR: {e}")
        threading.Thread(target=task, daemon=True).start()

    # -------- SIOS Tab --------
    # -------- SIOS Tab --------
    def _build_sios_tab(self):
        frm = ttk.Frame(self.tab_sios)
        frm.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)

        # Parameters (defaults as requested)
        self.var_samplerate = tk.IntVar(value=210000)
        self.var_blocksize = tk.IntVar(value=4096)
        self.var_interval = tk.IntVar(value=0)

        ttk.Button(frm, text="Connect", command=self._sios_connect).grid(row=0, column=0, padx=5, pady=5)
        ttk.Button(frm, text="Disconnect", command=self._sios_disconnect).grid(row=0, column=1, padx=5, pady=5)

        ttk.Label(frm, text="Sample rate [Hz]").grid(row=1, column=0, sticky="e")
        ttk.Entry(frm, textvariable=self.var_samplerate, width=10).grid(row=1, column=1, sticky="w")

        ttk.Label(frm, text="Block size").grid(row=2, column=0, sticky="e")
        blocksize_options = [256, 512, 1024, 2048, 4096, 8192]
        ttk.Combobox(frm, textvariable=self.var_blocksize,
                     values=blocksize_options, width=10, state="readonly").grid(row=2, column=1, sticky="w")

        ttk.Label(frm, text="Interval [ms]").grid(row=3, column=0, sticky="e")
        ttk.Entry(frm, textvariable=self.var_interval, width=10).grid(row=3, column=1, sticky="w")

        ttk.Button(frm, text="Start Measurement", command=self._sios_start).grid(row=4, column=0, padx=5, pady=5)
        ttk.Button(frm, text="Stop Measurement", command=self._sios_stop).grid(row=4, column=1, padx=5, pady=5)

        # Figure: time + FFT
        self.fig = Figure(figsize=(6, 6), dpi=100)
        gs = self.fig.add_gridspec(2, 1, hspace=0.5)

        self.ax_time = self.fig.add_subplot(gs[0, 0])
        self.ax_time.set_title("Time Domain")
        self.ax_time.set_xlabel("Time [ms]")
        self.ax_time.set_ylabel("Displacement [nm]")
        self.line_time, = self.ax_time.plot([], [], "-")

        self.ax_fft = self.fig.add_subplot(gs[1, 0])
        self.ax_fft.set_title("FFT")
        self.ax_fft.set_xlabel("Frequency [Hz]")
        self.ax_fft.set_ylabel("Amplitude [nm]")
        self.line_fft, = self.ax_fft.plot([], [], "-")

        self.fig.subplots_adjust(bottom=0.15)

        canvas = FigureCanvasTkAgg(self.fig, master=frm)
        canvas.get_tk_widget().grid(row=5, column=0, columnspan=2, pady=10, sticky="nsew")
        self.canvas = canvas

        frm.rowconfigure(5, weight=1)
        frm.columnconfigure(1, weight=1)

    def _sios_start(self):
        sr = self.var_samplerate.get()
        bs = self.var_blocksize.get()
        iv = self.var_interval.get()
        self.sios_data.clear()
        self.offset = 0.0
        self.auto_zero_done = False

        def task():
            try:
                # Pass blocksize + interval if your DLL supports it
                self.sios.start_measurement(sr)  # base call
                self._log(f"SIOS started: samplerate={sr} Hz, blocksize={bs}, interval={iv} ms")
            except Exception as e:
                self._log(f"SIOS ERROR: {e}")

        threading.Thread(target=task, daemon=True).start()

    def _sios_connect(self):
        def task():
            try:
                self.sios.connect()
                self._log("SIOS: connected")
            except Exception as e:
                self._log(f"SIOS ERROR: {e}")
                messagebox.showerror("SIOS", str(e))
        threading.Thread(target=task, daemon=True).start()

    def _sios_disconnect(self):
        threading.Thread(target=self.sios.disconnect, daemon=True).start()

    def _sios_start(self):
        sr = self.var_samplerate.get()
        self.sios_data.clear()
        self.offset = 0.0
        self.auto_zero_done = False
        def task():
            try:
                self.sios.start_measurement(sr)
            except Exception as e:
                self._log(f"SIOS ERROR: {e}")
        threading.Thread(target=task, daemon=True).start()

    def _sios_stop(self):
        threading.Thread(target=self.sios.stop_measurement, daemon=True).start()

    def _sios_update_plot(self):
        try:
            val = self.sios.read_value(0)
        except Exception as e:
            self._log(f"SIOS read error: {e}")
            val = None

        if val is not None:
            # DLL already returns nm
            self.sios_data.append(val)
            self.sios_data = self.sios_data[-20000:]  # rolling buffer

        if self.sios_data:
            sr = max(1, self.var_samplerate.get())
            y = np.array(self.sios_data, dtype=float)  # nm
            N = len(y)

            # Auto-zero using first 200 samples
            if not self.auto_zero_done and N >= 200:
                self.offset = float(np.mean(y[:200]))
                self.auto_zero_done = True

            y_zeroed = y - self.offset

            # Time domain
            x_ms = (np.arange(N) / sr) * 1000.0
            self.line_time.set_data(x_ms, y_zeroed)
            self.ax_time.relim(); self.ax_time.autoscale_view()

            # FFT (linear amplitude, Hann window)
            if N > 1:
                window = np.hanning(N)
                yw = (y_zeroed - np.mean(y_zeroed)) * window
                Y = np.fft.rfft(yw)
                f = np.fft.rfftfreq(N, 1.0 / sr)
                spectrum = np.abs(Y) / (N / 2)  # linear amplitude
                self.line_fft.set_data(f, spectrum)
                self.ax_fft.relim(); self.ax_fft.autoscale_view()

            self.canvas.draw_idle()

        # schedule next update
        self.after(100, self._sios_update_plot)

    # -------- Log Tab --------
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
    root.title("PicoScope 2000 & SIOS LSV 2500 NG (DLL)")
    root.geometry("1000x720")
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()
