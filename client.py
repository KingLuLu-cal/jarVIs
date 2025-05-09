import tkinter as tk
from tkinter import ttk, messagebox
import threading
import asyncio
import time
import sys
from bleak import BleakScanner, BleakClient

# ===== CONFIGURATION =====
TARGET_NAME = "ESP32-Robot"
CHAR_UUID = "0000dead-0000-1000-8000-00805f9b34fb"
CHAR_UUID_TOP = "12345678-90ab-cdef-fedc-ba0987654321"
CHAR_UUID_FRONT = "21436587-09ba-dcfe-efcd-ab9078563412"

COMMANDS = {
    'Forward': 'FORWARD',
    'Backward': 'BACKWARD',
    'Left': 'LEFT',
    'Right': 'RIGHT',
    'Rotate CW': 'ROTATE_CW',
    'Rotate CCW': 'ROTATE_CCW',
    'Stop': 'STOP',
    'MANUAL_START': 'MANUAL_START',
    'MANUAL_STOP': 'MANUAL_STOP'
}

ble_client = None
device_address = None
is_connected = False
continuous_movement = False
continuous_thread = None
loop = None

def create_asyncio_loop():
    global loop
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    return loop

def run_async(coro):
    if loop is None:
        create_asyncio_loop()
    return asyncio.run_coroutine_threadsafe(coro, loop)

async def scan_for_devices():
    try:
        root.after(0, lambda: status_label.configure(text="Scanning for BLE devices...", foreground="skyblue"))
        devices = await BleakScanner.discover(timeout=5)
        named_devices = {d.name: d for d in devices if d.name}
        def update_ui():
            device_dropdown['values'] = list(named_devices.keys())
            if TARGET_NAME in named_devices:
                device_var.set(TARGET_NAME)
                status_label.configure(text=f"Found {TARGET_NAME}! Select it and connect.", foreground="lime")
            else:
                status_label.configure(text="Scan complete. Select a device.", foreground="orange")
        root.after(0, update_ui)
        return named_devices
    except Exception as e:
        root.after(0, lambda: messagebox.showerror("Scan Error", str(e)))
        root.after(0, lambda: status_label.configure(text=f"Scan error: {str(e)}", foreground="red"))
        return {}

def start_scanning():
    def scan_thread():
        global device_dict
        future = run_async(scan_for_devices())
        device_dict = future.result()
    threading.Thread(target=scan_thread, daemon=True).start()

async def connect_to_device_async(device_name):
    global ble_client, device_address, is_connected
    if not device_name or not device_name in device_dict:
        raise ValueError("Please select a valid device")
    device = device_dict[device_name]
    device_address = device.address
    try:
        client = BleakClient(device_address)
        await client.connect()
        if client.is_connected:
            ble_client = client
            is_connected = True
            return True
        else:
            return False
    except Exception as e:
        raise Exception(f"Connection failed: {str(e)}")

def connect_to_device():
    selected_device = device_var.get()
    def connect_thread():
        try:
            future = run_async(connect_to_device_async(selected_device))
            if future.result():
                root.after(0, lambda: status_label.configure(text=f"Connected to {selected_device}", foreground="lime"))
                root.after(0, lambda: connect_btn.configure(text="Disconnect", command=disconnect_from_device))
                root.after(0, lambda: toggle_control_buttons(True))
            else:
                root.after(0, lambda: status_label.configure(text="Connection failed", foreground="red"))
        except Exception as e:
            root.after(0, lambda: messagebox.showerror("Connection Error", str(e)))
    threading.Thread(target=connect_thread, daemon=True).start()

async def disconnect_from_device_async():
    global ble_client, is_connected
    if ble_client and ble_client.is_connected:
        try:
            await ble_client.write_gatt_char(CHAR_UUID, COMMANDS['Stop'].encode(), response=True)
        except:
            pass
        await ble_client.disconnect()
    ble_client = None
    is_connected = False

def disconnect_from_device():
    global continuous_movement
    continuous_movement = False
    def disconnect_thread():
        try:
            future = run_async(disconnect_from_device_async())
            future.result()
            root.after(0, lambda: status_label.configure(text="Disconnected", foreground="orange"))
            root.after(0, lambda: connect_btn.configure(text="Connect", command=connect_to_device))
            root.after(0, lambda: toggle_control_buttons(False))
        except Exception as e:
            root.after(0, lambda: messagebox.showerror("Disconnection Error", str(e)))
    threading.Thread(target=disconnect_thread, daemon=True).start()

async def send_command_async(cmd):
    global ble_client
    if not is_connected or not ble_client or not ble_client.is_connected:
        raise ValueError("Not connected to a device")
    try:
        await ble_client.write_gatt_char(CHAR_UUID, cmd.encode(), response=True)
        return f"Sent: {cmd}"
    except Exception as e:
        raise Exception(f"Send error: {str(e)}")

def on_control_press(direction):
    if direction in COMMANDS:
        command = COMMANDS[direction]
        def send_thread():
            try:
                future = run_async(send_command_async(command))
                response = future.result()
                def update_ui():
                    log_text.config(state=tk.NORMAL)
                    log_text.insert(tk.END, f"{response}\n")
                    log_text.see(tk.END)
                    log_text.config(state=tk.DISABLED)
                    movement_label.configure(text=f"Movement: {direction}")
                root.after(0, update_ui)
            except Exception as e:
                def show_error():
                    messagebox.showerror("Communication Error", str(e))
                    log_text.config(state=tk.NORMAL)
                    log_text.insert(tk.END, f"Error: {str(e)}\n")
                    log_text.see(tk.END)
                    log_text.config(state=tk.DISABLED)
                root.after(0, show_error)
        threading.Thread(target=send_thread, daemon=True).start()

def on_control_hold(direction):
    global continuous_movement, continuous_thread
    continuous_movement = False
    if continuous_thread and continuous_thread.is_alive():
        continuous_thread.join(0.5)
    continuous_movement = True
    continuous_thread = threading.Thread(target=continuous_send, args=(direction,))
    continuous_thread.daemon = True
    continuous_thread.start()

def on_control_release():
    global continuous_movement
    continuous_movement = False
    on_control_press('Stop')

def continuous_send(direction):
    global continuous_movement
    if direction in COMMANDS:
        command = COMMANDS[direction]
        while continuous_movement and is_connected:
            try:
                future = run_async(send_command_async(command))
                future.result()
                time.sleep(0.1)
            except:
                break

def toggle_control_buttons(enable):
    state = tk.NORMAL if enable else tk.DISABLED
    for button in control_buttons:
        button.config(state=state)

def clear_log():
    log_text.config(state=tk.NORMAL)
    log_text.delete(1.0, tk.END)
    log_text.config(state=tk.DISABLED)

def handle_speed_change(event):
    speed = int(speed_slider.get())
    def speed_thread():
        try:
            future = run_async(send_command_async(f"V{speed}"))
            future.result()
        except Exception as e:
            root.after(0, lambda: messagebox.showerror("Speed Change Error", str(e)))
    threading.Thread(target=speed_thread, daemon=True).start()

def start_asyncio_thread():
    def run_loop_in_thread():
        loop = create_asyncio_loop()
        loop.run_forever()
    threading.Thread(target=run_loop_in_thread, daemon=True).start()

# ===== UI SETUP =====
root = tk.Tk()
root.title("JARVIS BLE Controller 🕸️")
root.geometry("800x600")
root.configure(bg="#102542")

style = ttk.Style()
style.theme_use('default')
style.configure('TButton', font=('Arial', 10), background="#ff3b3f", foreground="white")
style.configure('TFrame', background="#102542")
style.configure('TLabel', background="#102542", font=('Arial', 10), foreground="white")
style.configure('TScale', background="#102542")

tk.Label(root, text="Team JarVIs 🕸️", font=("Arial", 18, "bold"), fg="#ff3b3f", bg="#102542").pack(pady=10)

main_frame = ttk.Frame(root, padding="10")
main_frame.pack(fill=tk.BOTH, expand=True)

left_panel = ttk.Frame(main_frame, padding="5")
left_panel.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

conn_frame = ttk.LabelFrame(left_panel, text="BLE Connection", padding="10")
conn_frame.pack(fill=tk.X, pady=5)

device_var = tk.StringVar()
device_dict = {}

ttk.Label(conn_frame, text="Device:").grid(row=0, column=0, sticky=tk.W, pady=5)
device_dropdown = ttk.Combobox(conn_frame, textvariable=device_var, width=15)
device_dropdown.grid(row=0, column=1, sticky=tk.W, padx=5, pady=5)
ttk.Button(conn_frame, text="Scan", command=start_scanning).grid(row=0, column=2, padx=5, pady=5)
connect_btn = ttk.Button(conn_frame, text="Connect", command=connect_to_device)
connect_btn.grid(row=0, column=3, padx=5, pady=5)

status_label = tk.Label(conn_frame, text="Not Connected", fg="red", bg="#102542")
status_label.grid(row=1, column=0, columnspan=4, sticky=tk.W, pady=5)
movement_label = tk.Label(conn_frame, text="Movement: None", font=('Arial', 12, 'bold'), bg="#102542", fg="white")
movement_label.grid(row=2, column=0, columnspan=4, sticky=tk.W, pady=10)

log_frame = ttk.LabelFrame(left_panel, text="Command Log", padding="10")
log_frame.pack(fill=tk.BOTH, expand=True, pady=5)
log_text = tk.Text(log_frame, height=10, width=40, state=tk.DISABLED)
log_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
log_scrollbar = ttk.Scrollbar(log_frame, orient=tk.VERTICAL, command=log_text.yview)
log_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
log_text.config(yscrollcommand=log_scrollbar.set)
ttk.Button(log_frame, text="Clear Log", command=clear_log).pack(pady=5)

right_panel = ttk.Frame(main_frame, padding="5")
right_panel.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

control_frame = ttk.LabelFrame(right_panel, text="Vehicle Controls", padding="10")
control_frame.pack(fill=tk.BOTH, expand=True, pady=5)
ttk.Label(control_frame, text="Control Mode: Hold Button to Move").pack(anchor=tk.W, pady=5)

buttons_frame = ttk.Frame(control_frame, padding="5")
buttons_frame.pack(pady=10)
control_buttons = []

def create_btn(row, text, direction):
    btn = ttk.Button(row, text=text, width=10)
    btn.pack(side=tk.LEFT, padx=5)
    control_buttons.append(btn)
    btn.bind("<ButtonPress-1>", lambda e: on_control_hold(direction))
    btn.bind("<ButtonRelease-1>", lambda e: on_control_release())

top_row = ttk.Frame(buttons_frame); top_row.pack(pady=5)
create_btn(top_row, "↺ CCW", "Rotate CCW")
create_btn(top_row, "↑ Forward", "Forward")
create_btn(top_row, "↻ CW", "Rotate CW")

middle_row = ttk.Frame(buttons_frame); middle_row.pack(pady=5)
create_btn(middle_row, "← Left", "Left")
create_btn(middle_row, "■ Stop", "Stop")
create_btn(middle_row, "→ Right", "Right")

bottom_row = ttk.Frame(buttons_frame); bottom_row.pack(pady=5)
create_btn(bottom_row, "↓ Backward", "Backward")

# Manual mode frame
manual_frame = ttk.LabelFrame(right_panel, text="Manual Control", padding="10")
manual_frame.pack(fill=tk.X, pady=5)

manual_start_btn = ttk.Button(manual_frame, text="Enable Manual Mode",
    command=lambda: on_control_press('MANUAL_START'))
manual_start_btn.pack(side=tk.LEFT, padx=5, pady=5, expand=True, fill=tk.X)

manual_stop_btn = ttk.Button(manual_frame, text="Disable Manual Mode",
    command=lambda: on_control_press('MANUAL_STOP'))
manual_stop_btn.pack(side=tk.LEFT, padx=5, pady=5, expand=True, fill=tk.X)


def read_sensor(uuid_label):
    def thread():
        try:
            result = run_async(ble_client.read_gatt_char(uuid_label)).result().decode("utf-8")
            def update_log():
                log_text.config(state=tk.NORMAL)
                log_text.insert(tk.END, f"{result}\n")
                log_text.see(tk.END)
                log_text.config(state=tk.DISABLED)
            root.after(0, update_log)
        except Exception as e:
            root.after(0, lambda: messagebox.showerror("Read Error", str(e)))
    threading.Thread(target=thread, daemon=True).start()


ttk.Button(right_panel, text="Read FRONT Sensor", command=lambda: read_sensor(CHAR_UUID_TOP)).pack(pady=3)
ttk.Button(right_panel, text="Read TOP Sensor", command=lambda: read_sensor(CHAR_UUID_FRONT)).pack(pady=3)

special_frame = ttk.LabelFrame(right_panel, text="Speed Control", padding="10")
special_frame.pack(fill=tk.X, pady=5)
ttk.Label(special_frame, text="Speed:").pack(anchor=tk.W)
speed_slider = ttk.Scale(special_frame, from_=120, to=250, orient=tk.HORIZONTAL)
speed_slider.set(185)
speed_slider.pack(fill=tk.X, pady=5)
speed_slider.bind("<ButtonRelease-1>", handle_speed_change)

toggle_control_buttons(False)
start_asyncio_thread()

def on_closing():
    global loop, continuous_movement
    continuous_movement = False
    if is_connected:
        run_async(disconnect_from_device_async())
    if loop:
        loop.call_soon_threadsafe(loop.stop)
    root.destroy()
    sys.exit(0)

root.protocol("WM_DELETE_WINDOW", on_closing)
root.mainloop()
