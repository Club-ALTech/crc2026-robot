from tkinter import Tk, ttk as tk, StringVar
from reaktiv import Signal, Computed, Effect
import serial.tools.list_ports
import sys

window = Tk()
window.title("dashboard")
window.bind("<Escape>", sys.exit)


def get_ports():
    available = []
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if "usb" in port.description.lower():
            available.append(port.device)
    return available


port_name = Signal(None)


select_port = StringVar()
selectsOptions_value = StringVar()
selectsOptions = tk.Combobox(
    window, values=get_ports(), textvariable=selectsOptions_value
)
selectsOptions.bind(
    "<<ComboboxSelected>>", lambda _e: port_name.set(selectsOptions_value.get())
)
selectsOptions.pack()

port = Signal(None)


def update_port_fn():
    if port_name() is not None:
        ser = serial.Serial(port_name(), 9600, 5)
        ser.write(b'ping\n')
        # Read response
        ser.flushInput()
        response = ser.readline()[:-2]
        print(response.strip())
        # Close connection
        ser.close()
update_port = Effect(update_port_fn)

# ser = serial.Serial("/dev/ttyUSB0", 115200, timeout=1)


# def read_lines_forever(ser):
#     """Read lines continuously with error handling"""
#     while True:
#         try:
#             line = ser.readline()
#             if line:
#                 text = line.decode('utf-8', errors='ignore').strip()
#                 if text:  # Skip empty lines
#                     yield text
#         except KeyboardInterrupt:
#             break
#         except Exception as e:
#             print(f"Read error: {e}")
#             break



# tk.Button(window, text="ping", command=lambda: ser.write(b"ping\n")).pack()

# # Usage
# for line in read_lines_forever(ser):
#     print(f"Got: {line}")
#     if line == "QUIT":
#         break

window.mainloop()