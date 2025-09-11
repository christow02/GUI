import sys
import threading
import queue
import time
from dataclasses import dataclass
from typing import Optional, List

try:
    import tkinter as tk
    from tkinter import ttk, messagebox
except Exception:  # pragma: no cover
    print("Tkinter wird benötigt.")
    sys.exit(1)

# --- Instrument Treiber (platzhalterhaft, aber funktionsfähig strukturiert) ---

# VISA/SCPI für R&S AM300
try:
    import pyvisa
except ImportError:
    pyvisa = None

# Serial für SIOS
try:
    import serial
    import serial.tools.list_ports as list_ports
except ImportError:
    serial = None
    list_ports = None


class VisaError(RuntimeError):
    pass


class SerialError(RuntimeError):
    pass


@dataclass
class AM300Config:
    resource: str = "TCPIP0::192.168.0.100::INSTR"  # Beispiel
    timeout_ms: int = 2000


class AM300:
    """Minimaler SCPI-Treiber für R&S AM300 (Beispiel-Kommandos)."""
    def __init__(self, cfg: AM300Config, log):
        self.cfg = cfg
        self.rm = None
        self.inst = None
        self.log = log

    def connect(self):
        if pyvisa is None:
            raise VisaError("pyvisa nicht installiert. 'pip install pyvisa pyvisa-py'")
        self.rm = pyvisa.ResourceManager()
        try:
            self.inst = self.rm.open_resource(self.cfg.resource)
            self.inst.timeout = self.cfg.timeout_ms
            # Identifikation
            idn = self.query("*IDN?").strip()
            self.log(f"AM300 verbunden: {idn}")
        except Exception as e:
            self.inst = None
            raise VisaError(f"VISA-Verbindung fehlgeschlagen: {e}")

    def close(self):
        try:
            if self.inst:
                self.inst.close()
        finally:
            self.inst = None
            if self.rm:
                try:
                    self.rm.close()
                except Exception:
                    pass
                self.rm = None
            self.log("AM300 getrennt.")

    def write(self, cmd: str):
        if not self.inst:
            raise VisaError("AM300 nicht verbunden.")
        self.log(f"> {cmd}")
        self.inst.write(cmd)

    def query(self, cmd: str) -> str:
        if not self.inst:
            raise VisaError("AM300 nicht verbunden.")
        self.log(f"? {cmd}")
        return self.inst.query(cmd)

    # --- Beispiel-Funktionen; bitte mit Handbuch gegenprüfen ---
    def set_output(self, on: bool):
        # TODO: Prüfe, ob 'OUTP ON/OFF' für AM300 korrekt ist
        self.write(f"OUTP {'ON' if on else 'OFF'}")

    def set_waveform(self, shape: str):
        # TODO: Prüfe Form: SIN, SQU, RAMP, NOIS, ARB, etc.
        self.write(f"FUNC {shape.upper()}")

    def set_frequency(self, hz: float):
        self.write(f"FREQ {hz}")

    def set_amplitude(self, vpp: float):
        # Einheit als VPP; ggf. "VOLT" oder "VOLT:UNIT VPP" prüfen
        self.write(f"VOLT {vpp}")

    def set_offset(self, v: float):
        self.write(f"VOLT:OFFS {v}")

    def get_status(self) -> str:
        # Standard-Query-Beispiel
        try:
            freq = self.query("FREQ?").strip()
            ampl = self.query("VOLT?").strip()
            offs = self.query("VOLT:OFFS?").strip()
            func = self.query("FUNC?").strip()
            outp = self.query("OUTP?").strip()
            return f"FUNC={func}, FREQ={freq} Hz, AMPL={ampl} V, OFFS={offs} V, OUTP={outp}"
        except Exception as e:
            return f"Fehler bei Statusabfrage: {e}"


@dataclass
class SIOSConfig:
    port: str = "COM3"  # unter Linux: '/dev/ttyUSB0'
    baudrate: int = 115200
    timeout_s: float = 1.0


class SIOSLSV2500NG:
    """Beispielhafter serieller Treiber für SIOS LSV 2500 NG.
    HINWEIS: Das echte Protokoll bitte im Handbuch nachlesen und hier anpassen.
    """

    def __init__(self, cfg: SIOSConfig, log):
        self.cfg = cfg
        self.ser: Optional[serial.Serial] = None
        self.log = log

    def connect(self):
        if serial is None:
            raise SerialError("pyserial nicht installiert. 'pip install pyserial'")
        try:
            self.ser = serial.Serial(
                self.cfg.port,
                self.cfg.baudrate,
                timeout=self.cfg.timeout_s,
            )
            self.log(f"SIOS verbunden auf {self.cfg.port}@{self.cfg.baudrate}")
        except Exception as e:
            self.ser = None
            raise SerialError(f"Serielle Verbindung fehlgeschlagen: {e}")

    def close(self):
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.log("SIOS getrennt.")

    def send(self, cmd: str, expect: Optional[bytes] = None, add_crlf: bool = True) -> bytes:
        if not self.ser or not self.ser.is_open:
            raise SerialError("SIOS nicht verbunden.")
        data = (cmd + ("\r\n" if add_crlf else "")).encode("ascii", errors="ignore")
        self.log(f"> {cmd}")
        self.ser.reset_input_buffer()
        self.ser.write(data)
        self.ser.flush()
        time.sleep(0.02)
        resp = self.ser.read_until(b"\n")  # einfache Lese-Logik
        if resp:
            self.log(f"< {resp.decode(errors='ignore').strip()}")
        if expect and expect not in resp:
            raise SerialError(f"Unerwartete Antwort: {resp}")
        return resp

    # --- Beispiel-Funktionen; bitte echte Kommandos ergänzen ---
    def start_measurement(self):
        # TODO: durch echtes Kommando ersetzen
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


# --- GUI ---

class App(ttk.Frame):
    def __init__(self, master):
        super().__init__(master)
        self.master = master
        self.pack(fill=tk.BOTH, expand=True)

        self.log_queue: "queue.Queue[str]" = queue.Queue()
        self.after(100, self._drain_log)

        # Instrumente
        self.am300 = AM300(AM300Config(), self._log)
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

    # ---------- UI Aufbau ----------
    def _build_ui(self):
        self.master.title("AM300 & SIOS LSV 2500 NG - Steuerung")
        self.master.minsize(980, 640)

        # Top: Verbindungen
        frm_conn = ttk.LabelFrame(self, text="Verbindungen")
        frm_conn.pack(fill=tk.X, padx=10, pady=8)

        # AM300 Verbindung
        ttk.Label(frm_conn, text="AM300 VISA-Resource:").grid(row=0, column=0, sticky=tk.W, padx=6, pady=4)
        self.var_am_res = tk.StringVar(value=self.am300.cfg.resource)
        ttk.Entry(frm_conn, textvariable=self.var_am_res, width=40).grid(row=0, column=1, padx=6, pady=4)
        ttk.Button(frm_conn, text="Verbinden", command=self._am_connect).grid(row=0, column=2, padx=4)
        ttk.Button(frm_conn, text="Trennen", command=self._am_close).grid(row=0, column=3, padx=4)
        ttk.Button(frm_conn, text="Status", command=self._am_status).grid(row=0, column=4, padx=4)

        # SIOS Verbindung
        ttk.Label(frm_conn, text="SIOS Port:").grid(row=1, column=0, sticky=tk.W, padx=6, pady=4)
        self.var_sios_port = tk.StringVar(value=self.sios.cfg.port)
        ttk.Entry(frm_conn, textvariable=self.var_sios_port, width=20).grid(row=1, column=1, sticky=tk.W, padx=6, pady=4)

        ttk.Label(frm_conn, text="Baud:").grid(row=1, column=2, sticky=tk.E)
        self.var_sios_baud = tk.IntVar(value=self.sios.cfg.baudrate)
        ttk.Entry(frm_conn, textvariable=self.var_sios_baud, width=10).grid(row=1, column=3, sticky=tk.W)

        ttk.Button(frm_conn, text="Ports suchen", command=self._refresh_ports).grid(row=1, column=4, padx=4)
        ttk.Button(frm_conn, text="Verbinden", command=self._sios_connect).grid(row=1, column=5, padx=4)
        ttk.Button(frm_conn, text="Trennen", command=self._sios_close).grid(row=1, column=6, padx=4)

        for i in range(0, 7):
            frm_conn.grid_columnconfigure(i, weight=0)
        frm_conn.grid_columnconfigure(1, weight=1)

        # Notebook Tabs
        nb = ttk.Notebook(self)
        nb.pack(fill=tk.BOTH, expand=True, padx=10, pady=8)

        self.tab_am = ttk.Frame(nb)
        self.tab_sios = ttk.Frame(nb)
        self.tab_log = ttk.Frame(nb)
        nb.add(self.tab_am, text="AM300")
        nb.add(self.tab_sios, text="SIOS LSV 2500 NG")
        nb.add(self.tab_log, text="Log & Aktionen")

        self._build_am_tab(self.tab_am)
        self._build_sios_tab(self.tab_sios)
        self._build_log_tab(self.tab_log)

    def _build_am_tab(self, parent):
        frm = ttk.Frame(parent)
        frm.pack(fill=tk.BOTH, expand=True)

        # Waveform Controls
        box = ttk.LabelFrame(frm, text="Signal")
        box.pack(fill=tk.X, padx=8, pady=8)

        ttk.Label(box, text="Form").grid(row=0, column=0, sticky=tk.W, padx=6, pady=6)
        self.var_shape = tk.StringVar(value="SIN")
        cb_shape = ttk.Combobox(box, textvariable=self.var_shape, values=["SIN", "SQU", "RAMP", "NOIS", "ARB"], state="readonly")
        cb_shape.grid(row=0, column=1, padx=6, pady=6)

        ttk.Label(box, text="Frequenz [Hz]").grid(row=0, column=2, sticky=tk.W)
        self.var_freq = tk.DoubleVar(value=1000.0)
        ttk.Entry(box, textvariable=self.var_freq, width=14).grid(row=0, column=3, padx=6)

        ttk.Label(box, text="Amplitude [Vpp]").grid(row=0, column=4, sticky=tk.W)
        self.var_ampl = tk.DoubleVar(value=1.0)
        ttk.Entry(box, textvariable=self.var_ampl, width=10).grid(row=0, column=5, padx=6)

        ttk.Label(box, text="Offset [V]").grid(row=0, column=6, sticky=tk.W)
        self.var_offs = tk.DoubleVar(value=0.0)
        ttk.Entry(box, textvariable=self.var_offs, width=10).grid(row=0, column=7, padx=6)

        ttk.Button(box, text="Übernehmen", command=self._am_apply).grid(row=0, column=8, padx=8)

        # Output Controls
        box2 = ttk.LabelFrame(frm, text="Ausgang")
        box2.pack(fill=tk.X, padx=8, pady=8)
        self.var_out = tk.BooleanVar(value=False)
        ttk.Checkbutton(box2, text="Output EIN", variable=self.var_out, command=self._am_toggle_output).pack(anchor=tk.W, padx=8, pady=6)

        # Status
        box3 = ttk.LabelFrame(frm, text="Status")
        box3.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)
        self.var_am_status = tk.StringVar(value="—")
        ttk.Label(box3, textvariable=self.var_am_status, anchor="w").pack(fill=tk.X, padx=8, pady=6)
        ttk.Button(box3, text="Aktualisieren", command=self._am_status).pack(anchor=tk.W, padx=8, pady=6)

    def _build_sios_tab(self, parent):
        frm = ttk.Frame(parent)
        frm.pack(fill=tk.BOTH, expand=True)

        box = ttk.LabelFrame(frm, text="Messung")
        box.pack(fill=tk.X, padx=8, pady=8)

        ttk.Label(box, text="Integrationszeit [ms]").grid(row=0, column=0, sticky=tk.W, padx=6, pady=6)
        self.var_int = tk.IntVar(value=10)
        ttk.Entry(box, textvariable=self.var_int, width=10).grid(row=0, column=1, padx=6)

        ttk.Button(box, text="Setzen", command=self._sios_set_int).grid(row=0, column=2, padx=6)
        ttk.Button(box, text="Start", command=self._sios_start).grid(row=0, column=3, padx=6)
        ttk.Button(box, text="Stop", command=self._sios_stop).grid(row=0, column=4, padx=6)
        ttk.Button(box, text="Wert lesen", command=self._sios_read).grid(row=0, column=5, padx=6)

        box2 = ttk.LabelFrame(frm, text="Letzter Messwert")
        box2.pack(fill=tk.X, padx=8, pady=8)
        self.var_val = tk.StringVar(value="—")
        ttk.Label(box2, textvariable=self.var_val).pack(anchor=tk.W, padx=8, pady=6)

    def _build_log_tab(self, parent):
        frm = ttk.Frame(parent)
        frm.pack(fill=tk.BOTH, expand=True)

        btns = ttk.Frame(frm)
        btns.pack(fill=tk.X, padx=8, pady=4)
        ttk.Button(btns, text="Kopieren", command=self._copy_log).pack(side=tk.LEFT)
        ttk.Button(btns, text="Leeren", command=self._clear_log).pack(side=tk.LEFT, padx=6)

        self.txt_log = tk.Text(frm, height=16, state=tk.DISABLED)
        self.txt_log.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)

    # ---------- Aktionen AM300 ----------
    def _am_connect(self):
        self.am300.cfg.resource = self.var_am_res.get().strip()
        def task():
            try:
                self.am300.connect()
            except Exception as e:
                self._show_error("AM300", e)
        threading.Thread(target=task, daemon=True).start()

    def _am_close(self):
        def task():
            try:
                self.am300.close()
            except Exception as e:
                self._show_error("AM300", e)
        threading.Thread(target=task, daemon=True).start()

    def _am_apply(self):
        shape = self.var_shape.get()
        try:
            freq = float(self.var_freq.get())
            ampl = float(self.var_ampl.get())
            offs = float(self.var_offs.get())
        except ValueError:
            self._show_error("Eingabe", "Bitte gültige Zahlen eingeben.")
            return

        def task():
            try:
                self.am300.set_waveform(shape)
                self.am300.set_frequency(freq)
                self.am300.set_amplitude(ampl)
                self.am300.set_offset(offs)
                self._log("AM300: Parameter gesetzt.")
            except Exception as e:
                self._show_error("AM300", e)
        threading.Thread(target=task, daemon=True).start()

    def _am_toggle_output(self):
        on = self.var_out.get()
        def task():
            try:
                self.am300.set_output(on)
            except Exception as e:
                self._show_error("AM300", e)
        threading.Thread(target=task, daemon=True).start()

    def _am_status(self):
        def task():
            try:
                status = self.am300.get_status()
                self.var_am_status.set(status)
                self._log(status)
            except Exception as e:
                self._show_error("AM300", e)
        threading.Thread(target=task, daemon=True).start()

    # ---------- Aktionen SIOS ----------
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

    # ---------- Log Helpers ----------
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

    def _show_error(self, title: str, err: Exception | str):
        self._log(f"ERROR [{title}]: {err}")
        messagebox.showerror(title, str(err))


# --- Main ---

def main():
    root = tk.Tk()
    # Native ttk-Theme
    try:
        root.call("set_theme", "light")  # falls ttkbootstrap vorhanden
    except Exception:
        pass
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()

