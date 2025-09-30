import tkinter as tk
from tkinter import ttk, messagebox
import ctypes
from ctypes import c_int32, c_double
import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

DLL_PATH = r"C:\Users\ge42tak\Downloads\GUI-main\GUI-main\siosifm.dll"

# --- Flags (from siosifmdef.h) ---
IFM_MEAS_ONECHANNEL = 0x0100
IFM_MEAS_LENGTH     = 0x0002

class SIOS:
    def __init__(self, log):
        self.dll = None
        self.dev = None
        self.log = log
        self.running = False

    def connect(self):
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
        return True

    def disconnect(self):
        if self.dll and self.dev is not None:
            self.dll.IfmCloseDevice(self.dev)
            self.dll.IfmClose()
            self.log("Device closed")
            self.dev = None

    def start_measurement(self, samplerate, blocksize, interval_ms):
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
        """Consume new values and return the latest length."""
        if not self.running:
            return None
        cnt = self.dll.IfmValueCount(self.dev)
        if cnt > 0:
            self.dll.IfmGetValues(self.dev)
            val = self.dll.IfmLengthValue(self.dev, channel)
            return val
        return None

# --- Tkinter GUI ---
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("SIOS LSV 2500 NG GUI with FFT")

        self.sios = SIOS(self._log)
        self.data = []
        self.offset = 0.0
        self.auto_zero_done = False

        # Parameters
        self.var_samplerate = tk.IntVar(value=210000)
        self.var_blocksize = tk.IntVar(value=4096)
        self.var_interval = tk.IntVar(value=0)

        # Build UI
        frm = ttk.Frame(self)
        frm.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        ttk.Button(frm, text="Connect", command=self._connect).grid(row=0, column=0, padx=5, pady=5)
        ttk.Button(frm, text="Disconnect", command=self._disconnect).grid(row=0, column=1, padx=5, pady=5)

        ttk.Label(frm, text="Sample rate [Hz]").grid(row=1, column=0, sticky="e")
        ttk.Entry(frm, textvariable=self.var_samplerate, width=10).grid(row=1, column=1, sticky="w")

        ttk.Label(frm, text="Block size").grid(row=2, column=0, sticky="e")
        blocksize_options = [256, 512, 1024, 2048, 4096, 8192]
        ttk.Combobox(frm, textvariable=self.var_blocksize,
                     values=blocksize_options, width=10, state="readonly").grid(row=2, column=1, sticky="w")

        ttk.Label(frm, text="Interval [ms]").grid(row=3, column=0, sticky="e")
        ttk.Entry(frm, textvariable=self.var_interval, width=10).grid(row=3, column=1, sticky="w")

        ttk.Button(frm, text="Start Measurement", command=self._start).grid(row=4, column=0, padx=5, pady=5)
        ttk.Button(frm, text="Stop Measurement", command=self._stop).grid(row=4, column=1, padx=5, pady=5)

        # Log box
        self.txt_log = tk.Text(frm, height=8, state=tk.DISABLED)
        self.txt_log.grid(row=5, column=0, columnspan=2, sticky="nsew", pady=10)

        # Create figure with extra vertical spacing
        self.fig = Figure(figsize=(6, 6), dpi=100)
        gs = self.fig.add_gridspec(2, 1, hspace=0.5)  # hspace = spacing between plots

        # Time-domain subplot
        self.ax_time = self.fig.add_subplot(gs[0, 0])
        self.ax_time.set_title("Time Domain")
        self.ax_time.set_xlabel("Time [ms]")
        self.ax_time.set_ylabel("Displacement [nm]")
        self.line_time, = self.ax_time.plot([], [], "-")

        # FFT subplot
        self.ax_fft = self.fig.add_subplot(gs[1, 0])
        self.ax_fft.set_title("FFT")
        self.ax_fft.set_xlabel("Frequency [Hz]")
        self.ax_fft.set_ylabel("Amplitude [nm]")
        self.line_fft, = self.ax_fft.plot([], [], "-")

        # Adjust layout margins
        self.fig.subplots_adjust(bottom=0.15)  # more space for FFT x-label

        # Embed in Tkinter
        canvas = FigureCanvasTkAgg(self.fig, master=frm)
        canvas.get_tk_widget().grid(row=6, column=0, columnspan=2, pady=10)
        self.canvas = canvas

        frm.rowconfigure(6, weight=1)
        frm.columnconfigure(1, weight=1)

        # Periodic update
        self.after(100, self._update_plot)

    def _log(self, msg):
        self.txt_log.configure(state=tk.NORMAL)
        self.txt_log.insert(tk.END, msg + "\n")
        self.txt_log.configure(state=tk.DISABLED)
        self.txt_log.see(tk.END)

    def _connect(self):
        try:
            self.sios.connect()
            self._log("Connected")
        except Exception as e:
            messagebox.showerror("Error", str(e))

    def _disconnect(self):
        self.sios.disconnect()
        self._log("Disconnected")

    def _start(self):
        sr = self.var_samplerate.get()
        bs = self.var_blocksize.get()
        iv = self.var_interval.get()
        self.data.clear()
        self.offset = 0.0
        self.auto_zero_done = False  # reset before each measurement
        self.sios.start_measurement(sr, bs, iv)

    def _stop(self):
        self.sios.stop_measurement()

    def _update_plot(self):
        val = self.sios.read_value(channel=0)  # only channel 0 has valid data
        if val is not None:
            self.data.append(val)
            self.data = self.data[-20000:]  # keep last ~20k samples

        if self.data:
            sr = max(1, self.var_samplerate.get())
            N = len(self.data)

            y = np.array(self.data)  # already in nanometers

            # --- Auto-zero offset (first 200 samples) ---
            if not self.auto_zero_done and N >= 200:
                self.offset = np.mean(y[:200])
                self.auto_zero_done = True

            # Apply offset
            y_zeroed = y - self.offset

            # --- Time domain plot ---
            x = np.arange(N) / sr * 1000.0  # ms
            self.line_time.set_data(x, y_zeroed)
            self.ax_time.set_xlabel("Time [ms]")
            self.ax_time.set_ylabel("Displacement [nm]")
            self.ax_time.relim()
            self.ax_time.autoscale_view()

            # --- FFT plot with Hann window ---
            if N > 1:
                window = np.hanning(N)
                y_windowed = (y_zeroed - np.mean(y_zeroed)) * window

                Y = np.fft.rfft(y_windowed)
                f = np.fft.rfftfreq(N, 1.0 / sr)

                spectrum = np.abs(Y) / (N / 2)  # normalize amplitude
                self.line_fft.set_data(f, spectrum)
                self.ax_fft.set_xlabel("Frequency [Hz]")
                self.ax_fft.set_ylabel("Amplitude [nm]")
                self.ax_fft.relim()
                self.ax_fft.autoscale_view()

        self.canvas.draw_idle()
        self.after(100, self._update_plot)

if __name__ == "__main__":
    app = App()
    app.mainloop()
