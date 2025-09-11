import tkinter as tk
from tkinter import ttk, messagebox
import serial                   # pyserial → Kommunikation mit dem LSV über USB/COM-Port
import serial.tools.list_ports   # Hilfsfunktion, um verfügbare Ports anzuzeigen
import pyvisa                   # VISA-Library → Kommunikation mit dem R&S AM300


# Window
root = tk.Tk()
root.title("Laser Vibrometer + AM300 GUI")
root.geometry("600x400")

# Notebook (Tabs for Generator + Vibrometer)
notebook = ttk.Notebook(root)
notebook.pack(expand=True, fill="both")


# Global connection flags
gen_connected = False   # Generator (AM300)
lsv_connected = False   # Vibrometer (LSV)


# Generator (AM300)
frame_gen = ttk.Frame(notebook)
notebook.add(frame_gen, text="Generator (AM300)")


def list_resources():
    rm = pyvisa.ResourceManager()
    devices = rm.list_resources()
    if devices:
        messagebox.showinfo("VISA Devices", "\n".join(devices))
    else:
        messagebox.showinfo("VISA Devices", "Keine Geräte gefunden.")


def connect_generator():
    global gen_connected, gen_session
    try:
        rm = pyvisa.ResourceManager()
        devices = rm.list_resources()
        if not devices:
            messagebox.showwarning("Warnung", "Kein VISA-Gerät gefunden")
            return

        # try all devices until one responds with AM300
        for d in devices:
            try:
                inst = rm.open_resource(d)
                idn = inst.query("*IDN?").strip()
                if "AM300" in idn:   # adjust string to match your device
                    gen_session = inst
                    gen_connected = True
                    btn_connect_gen.config(state="disabled")
                    messagebox.showinfo("Info", f"Generator verbunden: {idn}")
                    return
            except Exception:
                continue

        messagebox.showwarning("Warnung", "Kein AM300 Generator gefunden")

    except Exception as e:
        messagebox.showerror("Fehler", f"VISA Fehler:\n{e}")
        gen_connected = False


def set_generator_values():
    if not gen_connected:
        messagebox.showwarning("Warnung", "Gerät nicht verbunden")
        return
    freq = entry_freq.get()
    amp = entry_amp.get()
    offset = entry_offset.get()
    # Hier SCPI-Befehle an Generator senden...
    print(f"Generator set: {freq} Hz, {amp} Vpp, Offset {offset} V")


# Buttons and entries for generator
btn_list = ttk.Button(frame_gen, text="List VISA Devices", command=list_resources)
btn_list.pack(pady=10)

ttk.Label(frame_gen, text="Frequency (Hz):").pack()
entry_freq = ttk.Entry(frame_gen)
entry_freq.pack()

ttk.Label(frame_gen, text="Amplitude (Vpp):").pack()
entry_amp = ttk.Entry(frame_gen)
entry_amp.pack()

ttk.Label(frame_gen, text="Offset (V):").pack()
entry_offset = ttk.Entry(frame_gen)
entry_offset.pack()

btn_connect_gen = ttk.Button(frame_gen, text="Connect Generator", command=connect_generator)
btn_connect_gen.pack(pady=10)

btn_set_gen = ttk.Button(frame_gen, text="Set Generator Values", command=set_generator_values)
btn_set_gen.pack(pady=10)


# Vibrometer (LSV)
frame_vib = ttk.Frame(notebook)
notebook.add(frame_vib, text="Vibrometer (LSV)")


def list_serial_ports():
    ports = serial.tools.list_ports.comports()
    if ports:
        devices = [f"{p.device} - {p.description}" for p in ports]
        messagebox.showinfo("Serial Ports", "\n".join(devices))
    else:
        messagebox.showinfo("Serial Ports", "Keine COM-Ports gefunden.")


def connect_vibrometer():
    global lsv_connected, lsv_serial
    try:
        ports = serial.tools.list_ports.comports()
        if not ports:
            messagebox.showwarning("Warnung", "Kein COM-Port gefunden")
            return

        for p in ports:
            try:
                ser = serial.Serial(p.device, 9600, timeout=1)  # adjust baudrate
                # Send an identification command if available
                # For Polytec LSV 2500 this could be something like "*IDN?\r\n" or "VER\r\n"
                ser.write(b"*IDN?\r\n")
                reply = ser.readline().decode(errors="ignore").strip()
                if "LSV" in reply:   # adjust check for your device
                    lsv_serial = ser
                    lsv_connected = True
                    btn_connect_vib.config(state="disabled")
                    messagebox.showinfo("Info", f"Vibrometer verbunden: {reply}")
                    return
                ser.close()
            except Exception:
                continue

        messagebox.showwarning("Warnung", "Kein LSV Vibrometer gefunden")

    except Exception as e:
        messagebox.showerror("Fehler", f"Serial Fehler:\n{e}")
        lsv_connected = False


def set_integration_time():
    if not lsv_connected:
        messagebox.showwarning("Warnung", "Gerät nicht verbunden")
        return
    value = entry_inttime.get()
    # hier SCPI/Serielle Befehle senden...
    print(f"Integration time set to {value} ms")


def start_measurement():
    if not lsv_connected:
        messagebox.showwarning("Warnung", "Gerät nicht verbunden")
        return
    print("LSV started")


def stop_measurement():
    if not lsv_connected:
        messagebox.showwarning("Warnung", "Gerät nicht verbunden")
        return
    print("LSV stopped")


# Buttons and entries for vibrometer
btn_ports = ttk.Button(frame_vib, text="List Serial Ports", command=list_serial_ports)
btn_ports.pack(pady=10)

btn_connect_vib = ttk.Button(frame_vib, text="Connect Vibrometer", command=connect_vibrometer)
btn_connect_vib.pack(pady=10)

ttk.Label(frame_vib, text="Integrationszeit (ms):").pack()
entry_inttime = ttk.Entry(frame_vib)
entry_inttime.pack()

btn_setzen = ttk.Button(frame_vib, text="Setzen", command=set_integration_time)
btn_setzen.pack(pady=10)

frame_buttons = ttk.Frame(frame_vib)
frame_buttons.pack(pady=10)

btn_start = ttk.Button(frame_buttons, text="Start", command=start_measurement)
btn_start.pack(side="left", padx=5)

btn_stop = ttk.Button(frame_buttons, text="Stop", command=stop_measurement)
btn_stop.pack(side="left", padx=5)


# Mainloop
root.mainloop()


