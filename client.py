import tkinter as tk
from tkinter import ttk, messagebox
import threading
import asyncio
import time
import sys
from bleak import BleakScanner, BleakClient

# ===== CONFIGURATION =====
TARGET_NAME = "BLE-Server"  # Change to your ESP32 BLE device name
CHAR_UUID = "0000dead-0000-1000-8000-00805f9b34fb"  # Characteristic UUID

# Command mapping for mecanum car controls
COMMANDS = {
    'Forward': 'W',
    'Backward': 'S',
    'Left': 'A',
    'Right': 'D',
    'Rotate CW': 'E',     # Clockwise rotation
    'Rotate CCW': 'Q',    # Counter-clockwise rotation
    'Stop': 'X',
    'Boost Mode': 'B',    # Increased speed mode
    'Precision Mode': 'P'  # Slow/precise movement mode
}

# ===== VARIABLES =====
ble_client = None
device_address = None
is_connected = False
continuous_movement = False
continuous_thread = None
loop = None

# Required for asyncio in tkinter
def create_asyncio_loop():
    global loop
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    return loop

def run_async(coro):
    """Run an asyncio coroutine from the tkinter main thread"""
    if loop is None:
        create_asyncio_loop()
    
    return asyncio.run_coroutine_threadsafe(coro, loop)

async def scan_for_devices():
    """Scan for BLE devices asynchronously"""
    try:
        # Use root.after to modify UI elements safely
        root.after(0, lambda: status_label.configure(text="Scanning for BLE devices...", foreground="blue"))
        
        devices = await BleakScanner.discover(timeout=5)
        
        # Filter for devices with names
        named_devices = {d.name: d for d in devices if d.name}
        
        # Update the dropdown using root.after
        def update_ui():
            device_dropdown['values'] = list(named_devices.keys())
            # Preselect target device if found
            if TARGET_NAME in named_devices:
                device_var.set(TARGET_NAME)
                status_label.configure(text=f"Found {TARGET_NAME}! Select it and connect.", foreground="green")
            else:
                status_label.configure(text="Scan complete. Select a device.", foreground="orange")
        
        root.after(0, update_ui)
        return named_devices
    except Exception as e:
        # Use root.after to show error safely
        root.after(0, lambda: messagebox.showerror("Scan Error", str(e)))
        root.after(0, lambda: status_label.configure(text=f"Scan error: {str(e)}", foreground="red"))
        return {}

def start_scanning():
    """Start BLE scanning from UI thread"""
    global device_dict
    
    # Run in a thread to avoid blocking UI
    def scan_thread():
        global device_dict
        future = run_async(scan_for_devices())
        device_dict = future.result()
    
    threading.Thread(target=scan_thread, daemon=True).start()

async def connect_to_device_async(device_name):
    """Connect to the selected BLE device asynchronously"""
    global ble_client, device_address, is_connected
    
    if not device_name or device_name not in device_dict:
        raise ValueError("Please select a valid device")
    
    device = device_dict[device_name]
    device_address = device.address
    
    try:
        # Create a new client
        client = BleakClient(device_address)
        
        # Connect to the device
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
    """Establish connection to the selected BLE device from UI"""
    selected_device = device_var.get()
    
    # Run in a thread to avoid blocking UI
    def connect_thread():
        try:
            future = run_async(connect_to_device_async(selected_device))
            if future.result():
                root.after(0, lambda: status_label.configure(text=f"Connected to {selected_device}", foreground="green"))
                root.after(0, lambda: connect_btn.configure(text="Disconnect", command=disconnect_from_device))
                
                # Enable control buttons
                root.after(0, lambda: toggle_control_buttons(True))
            else:
                root.after(0, lambda: status_label.configure(text="Connection failed", foreground="red"))
        except Exception as e:
            root.after(0, lambda: messagebox.showerror("Connection Error", str(e)))
            root.after(0, lambda: status_label.configure(text=f"Connection failed: {str(e)}", foreground="red"))
    
    threading.Thread(target=connect_thread, daemon=True).start()

async def disconnect_from_device_async():
    """Disconnect from the BLE device asynchronously"""
    global ble_client, is_connected
    
    if ble_client and ble_client.is_connected:
        # Send stop command before disconnecting
        try:
            await ble_client.write_gatt_char(CHAR_UUID, COMMANDS['Stop'].encode(), response=True)
        except:
            pass  # Ignore errors during disconnect
        
        await ble_client.disconnect()
    
    ble_client = None
    is_connected = False

def disconnect_from_device():
    """Disconnect from the device from UI"""
    global continuous_movement
    
    # Stop any continuous movement
    continuous_movement = False
    
    # Run in a thread to avoid blocking UI
    def disconnect_thread():
        try:
            future = run_async(disconnect_from_device_async())
            future.result()
            root.after(0, lambda: status_label.configure(text="Disconnected", foreground="orange"))
            root.after(0, lambda: connect_btn.configure(text="Connect", command=connect_to_device))
            
            # Disable control buttons
            root.after(0, lambda: toggle_control_buttons(False))
        except Exception as e:
            root.after(0, lambda: messagebox.showerror("Disconnection Error", str(e)))
    
    threading.Thread(target=disconnect_thread, daemon=True).start()

async def send_command_async(cmd):
    """Send command to the connected BLE device asynchronously"""
    global ble_client
    
    if not is_connected or not ble_client or not ble_client.is_connected:
        raise ValueError("Not connected to a device")
    
    try:
        await ble_client.write_gatt_char(CHAR_UUID, cmd.encode(), response=True)
        return f"Sent: {cmd}"
    except Exception as e:
        raise Exception(f"Send error: {str(e)}")

def on_control_press(direction):
    """Handler for button press events"""
    if direction in COMMANDS:
        command = COMMANDS[direction]
        
        # Run in a thread to avoid blocking UI
        def send_thread():
            try:
                future = run_async(send_command_async(command))
                response = future.result()
                
                def update_ui():
                    log_text.config(state=tk.NORMAL)
                    log_text.insert(tk.END, f"{response}\n")
                    log_text.see(tk.END)
                    log_text.config(state=tk.DISABLED)
                    
                    # Update the status display
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
    """Start continuous movement in the specified direction"""
    global continuous_movement, continuous_thread
    
    # Stop any existing continuous movement
    continuous_movement = False
    if continuous_thread and continuous_thread.is_alive():
        continuous_thread.join(0.5)
    
    # Start new continuous movement
    continuous_movement = True
    continuous_thread = threading.Thread(target=continuous_send, args=(direction,))
    continuous_thread.daemon = True
    continuous_thread.start()

def on_control_release():
    """Stop continuous movement when button is released"""
    global continuous_movement
    
    continuous_movement = False
    on_control_press('Stop')

def continuous_send(direction):
    """Send command continuously while button is held"""
    global continuous_movement
    
    if direction in COMMANDS:
        command = COMMANDS[direction]
        while continuous_movement and is_connected:
            try:
                future = run_async(send_command_async(command))
                future.result()
                time.sleep(0.1)  # Adjust this delay as needed
            except:
                break

def toggle_control_buttons(enable):
    """Enable or disable control buttons based on connection status"""
    state = tk.NORMAL if enable else tk.DISABLED
    for button in control_buttons:
        button.config(state=state)
    
    # Also update the special function buttons
    for button in special_buttons:
        button.config(state=state)

def clear_log():
    """Clear the command log"""
    log_text.config(state=tk.NORMAL)
    log_text.delete(1.0, tk.END)
    log_text.config(state=tk.DISABLED)

def toggle_continuous_mode():
    """Toggle between continuous and single-press modes"""
    global movement_mode
    
    if movement_mode.get() == "continuous":
        mode_btn.config(text="Mode: Continuous")
    else:
        mode_btn.config(text="Mode: Single Press")

def create_preset(preset_name):
    """Save a preset command sequence"""
    # TODO: Implement custom preset functionality
    messagebox.showinfo("Preset", f"Preset '{preset_name}' functionality not implemented yet")

async def handle_speed_change_async(speed):
    """Handle speed changes asynchronously"""
    if not is_connected or not ble_client:
        return
    
    try:
        # Assuming 'V' is your speed command prefix followed by speed value
        speed_cmd = f"V{speed}"
        await ble_client.write_gatt_char(CHAR_UUID, speed_cmd.encode(), response=True)
        
        def update_log():
            log_text.config(state=tk.NORMAL)
            log_text.insert(tk.END, f"Speed set to: {speed}%\n")
            log_text.see(tk.END)
            log_text.config(state=tk.DISABLED)
        
        root.after(0, update_log)
    except Exception as e:
        def show_error():
            log_text.config(state=tk.NORMAL)
            log_text.insert(tk.END, f"Speed change error: {str(e)}\n")
            log_text.see(tk.END)
            log_text.config(state=tk.DISABLED)
        
        root.after(0, show_error)

def handle_speed_change(event):
    """Handle speed slider changes"""
    speed = int(speed_slider.get())
    
    # Run in a thread to avoid blocking UI
    def speed_thread():
        try:
            future = run_async(handle_speed_change_async(speed))
            future.result()
        except Exception as e:
            root.after(0, lambda: messagebox.showerror("Speed Change Error", str(e)))
    
    threading.Thread(target=speed_thread, daemon=True).start()

def start_asyncio_thread():
    """Start the asyncio event loop in a separate thread"""
    def run_loop_in_thread():
        loop = create_asyncio_loop()
        loop.run_forever()
    
    threading.Thread(target=run_loop_in_thread, daemon=True).start()

# ===== UI SETUP =====
root = tk.Tk()
root.title("Mecanum Car BLE Controller")
root.geometry("800x600")
root.configure(bg="#f0f0f0")

# Start asyncio loop in a separate thread
start_asyncio_thread()

# Create a modern style
style = ttk.Style()
style.theme_use('default')  # Use default theme for better compatibility
style.configure('TButton', font=('Arial', 10))
style.configure('TFrame', background="#f0f0f0")
style.configure('TLabel', background="#f0f0f0", font=('Arial', 10))
style.configure('TScale', background="#f0f0f0")

# Main frame
main_frame = ttk.Frame(root, padding="10")
main_frame.pack(fill=tk.BOTH, expand=True)

# Left panel (Connection and Log)
left_panel = ttk.Frame(main_frame, padding="5")
left_panel.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

# Connection frame
conn_frame = ttk.LabelFrame(left_panel, text="BLE Connection", padding="10")
conn_frame.pack(fill=tk.X, pady=5)

# Device selection
device_var = tk.StringVar()
device_dict = {}  # Will store name->device mapping

ttk.Label(conn_frame, text="Device:").grid(row=0, column=0, sticky=tk.W, pady=5)
device_dropdown = ttk.Combobox(conn_frame, textvariable=device_var, width=15)
device_dropdown.grid(row=0, column=1, sticky=tk.W, padx=5, pady=5)

# Scan and Connect buttons
ttk.Button(conn_frame, text="Scan", command=start_scanning).grid(row=0, column=2, padx=5, pady=5)
connect_btn = ttk.Button(conn_frame, text="Connect", command=connect_to_device)
connect_btn.grid(row=0, column=3, padx=5, pady=5)

# Status label (using a regular tk.Label to avoid ttk styling issues)
status_label = tk.Label(conn_frame, text="Not Connected", fg="red", bg="#f0f0f0")
status_label.grid(row=1, column=0, columnspan=4, sticky=tk.W, pady=5)

# Movement status (using a regular tk.Label)
movement_label = tk.Label(conn_frame, text="Movement: None", font=('Arial', 12, 'bold'), bg="#f0f0f0")
movement_label.grid(row=2, column=0, columnspan=4, sticky=tk.W, pady=10)

# Log frame
log_frame = ttk.LabelFrame(left_panel, text="Command Log", padding="10")
log_frame.pack(fill=tk.BOTH, expand=True, pady=5)

log_text = tk.Text(log_frame, height=10, width=40, state=tk.DISABLED)
log_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

log_scrollbar = ttk.Scrollbar(log_frame, orient=tk.VERTICAL, command=log_text.yview)
log_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
log_text.config(yscrollcommand=log_scrollbar.set)

ttk.Button(log_frame, text="Clear Log", command=clear_log).pack(pady=5)

# Right panel (Controls)
right_panel = ttk.Frame(main_frame, padding="5")
right_panel.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

# Control frame
control_frame = ttk.LabelFrame(right_panel, text="Vehicle Controls", padding="10")
control_frame.pack(fill=tk.BOTH, expand=True, pady=5)

# Movement mode (continuous or single press)
movement_mode = tk.StringVar(value="continuous")
mode_btn = ttk.Button(control_frame, text="Mode: Continuous", command=toggle_continuous_mode)
mode_btn.pack(pady=5)

ttk.Radiobutton(control_frame, text="Continuous (hold button)", variable=movement_mode, 
               value="continuous").pack(anchor=tk.W)
ttk.Radiobutton(control_frame, text="Single Press", variable=movement_mode, 
               value="single").pack(anchor=tk.W)

# Control buttons layout
buttons_frame = ttk.Frame(control_frame, padding="5")
buttons_frame.pack(pady=10)

# Initialize list to store control buttons for enabling/disabling
control_buttons = []

# Create top row (Rotate CCW, Forward, Rotate CW)
top_row = ttk.Frame(buttons_frame)
top_row.pack(pady=5)

ccw_btn = ttk.Button(top_row, text="↺ CCW", width=10)
ccw_btn.pack(side=tk.LEFT, padx=5)
control_buttons.append(ccw_btn)

forward_btn = ttk.Button(top_row, text="↑ Forward", width=10)
forward_btn.pack(side=tk.LEFT, padx=5)
control_buttons.append(forward_btn)

cw_btn = ttk.Button(top_row, text="↻ CW", width=10)
cw_btn.pack(side=tk.LEFT, padx=5)
control_buttons.append(cw_btn)

# Create middle row (Left, Stop, Right)
middle_row = ttk.Frame(buttons_frame)
middle_row.pack(pady=5)

left_btn = ttk.Button(middle_row, text="← Left", width=10)
left_btn.pack(side=tk.LEFT, padx=5)
control_buttons.append(left_btn)

stop_btn = ttk.Button(middle_row, text="■ Stop", width=10)
stop_btn.pack(side=tk.LEFT, padx=5)
control_buttons.append(stop_btn)

right_btn = ttk.Button(middle_row, text="→ Right", width=10)
right_btn.pack(side=tk.LEFT, padx=5)
control_buttons.append(right_btn)

# Create bottom row (backward only)
bottom_row = ttk.Frame(buttons_frame)
bottom_row.pack(pady=5)

backward_btn = ttk.Button(bottom_row, text="↓ Backward", width=10)
backward_btn.pack()
control_buttons.append(backward_btn)

# Configure button events
def setup_button_events(button, direction):
    """Set up button events based on movement mode"""
    def on_press(event):
        if movement_mode.get() == "continuous":
            on_control_hold(direction)
        else:
            on_control_press(direction)
    
    def on_release(event):
        if movement_mode.get() == "continuous":
            on_control_release()
    
    button.bind("<ButtonPress-1>", on_press)
    button.bind("<ButtonRelease-1>", on_release)

setup_button_events(forward_btn, 'Forward')
setup_button_events(backward_btn, 'Backward')
setup_button_events(left_btn, 'Left')
setup_button_events(right_btn, 'Right')
setup_button_events(cw_btn, 'Rotate CW')
setup_button_events(ccw_btn, 'Rotate CCW')
setup_button_events(stop_btn, 'Stop')

# Special functions frame
special_frame = ttk.LabelFrame(right_panel, text="Special Functions", padding="10")
special_frame.pack(fill=tk.X, pady=5)

# Initialize list to store special function buttons for enabling/disabling
special_buttons = []

# Speed control slider
ttk.Label(special_frame, text="Speed:").pack(anchor=tk.W)
speed_slider = ttk.Scale(special_frame, from_=0, to=100, orient=tk.HORIZONTAL)
speed_slider.set(50)  # Default to 50%
speed_slider.pack(fill=tk.X, pady=5)

# Special function buttons
special_btn_frame = ttk.Frame(special_frame)
special_btn_frame.pack(fill=tk.X, pady=5)

boost_btn = ttk.Button(special_btn_frame, text="Boost Mode", 
                      command=lambda: on_control_press('Boost Mode'))
boost_btn.pack(side=tk.LEFT, padx=5, pady=5, fill=tk.X, expand=True)
special_buttons.append(boost_btn)

precision_btn = ttk.Button(special_btn_frame, text="Precision Mode", 
                          command=lambda: on_control_press('Precision Mode'))
precision_btn.pack(side=tk.LEFT, padx=5, pady=5, fill=tk.X, expand=True)
special_buttons.append(precision_btn)

# Presets frame
presets_frame = ttk.LabelFrame(right_panel, text="Preset Movements", padding="10")
presets_frame.pack(fill=tk.X, pady=5)

# TODO: Add your custom presets here
ttk.Button(presets_frame, text="Square Pattern", 
          command=lambda: create_preset("Square Pattern")).pack(fill=tk.X, pady=2)
ttk.Button(presets_frame, text="Circle Pattern", 
          command=lambda: create_preset("Circle Pattern")).pack(fill=tk.X, pady=2)
ttk.Button(presets_frame, text="Custom 1", 
          command=lambda: create_preset("Custom 1")).pack(fill=tk.X, pady=2)

# Handle speed slider changes
speed_slider.bind("<ButtonRelease-1>", handle_speed_change)

# Disable control buttons initially (until connected)
toggle_control_buttons(False)

# TODO: Add keyboard controls for easier operation
# TODO: Add battery level indicator (if your ESP32 supports reading battery level)
# TODO: Add automatic movement sequence recording/playback
# TODO: Add signal strength indicator
# TODO: Implement custom characteristic handling for device feedback

# Cleanup on exit
def on_closing():
    """Clean up on application exit"""
    global loop, continuous_movement
    
    # Stop continuous movement
    continuous_movement = False
    
    # Disconnect BLE if connected
    if is_connected:
        run_async(disconnect_from_device_async())
    
    # Stop asyncio loop
    if loop:
        loop.call_soon_threadsafe(loop.stop)
    
    root.destroy()
    sys.exit(0)

root.protocol("WM_DELETE_WINDOW", on_closing)

# Start the application
root.mainloop()