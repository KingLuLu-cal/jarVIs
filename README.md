# 🤖 jaVIs: ESP32 BLE-Controlled Mecanum Robot with Python GUI

This project is a BLE-based mobile robot built using an ESP32 microcontroller and a mecanum drive system. It supports both **autonomous navigation** and **manual control** via a Python GUI over Bluetooth.

---

## 🗂 Project Structure

```
├── main.c            # FreeRTOS setup, task creation, BLE init
├── manual.c/h        # BLE GATT server + command processing
├── drivers.c/h       # Motor control utilities (PWM + GPIO)
├── globals.h         # Global variables, states, handles
├── pins.h            # GPIO pin mappings for motors and sensors
├── client.py         # Python GUI app (Tkinter + Bleak)
├── CMakeLists.txt    # ESP-IDF build config
├── idf_component.yml # Dependency manifest
└── Kconfig.projbuild # IDF project options
```

---

## 🧠 Features

### ✅ Robot Firmware (ESP32)
- Mecanum motor drive with PWM control
- BLE GATT Server with custom characteristics
- Ultrasonic obstacle detection (front + top)
- Dual-mode:
  - **Manual Control** via GUI
  - **Autonomous Mode** via distance sensors
- Real-time FreeRTOS task scheduling

### 💻 Python GUI (client.py)
- Cross-platform GUI using Tkinter
- BLE connection via [`bleak`](https://github.com/hbldh/bleak)
- Command buttons:
  - Directional: `Forward`, `Backward`, `Left`, `Right`
  - Rotation: `CW`, `CCW`
  - Mode Switch: `Manual Start/Stop`
  - Velocity setting (default: 180)
- Responsive interface using threads
- Log panel for feedback
- **Hold-to-repeat** movement logic for smooth control

---

## 🛠 Requirements

### Hardware
- ESP32 Development Board
- 4 Mecanum Wheels + Motor Driver (PWM capable)
- 2× Ultrasonic Sensors (e.g., HC-SR04)
- 5V Power Supply or Battery Pack

### Software
- [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/)
- Python 3.7+
  - `bleak`, `tkinter`, `asyncio` (included in Python)
- iOS App (optional): Swift app provided for BLE control

---

## 🧪 Build & Flash (ESP32)

```bash
idf.py set-target esp32
idf.py build
idf.py flash -p /dev/ttyUSB0
idf.py monitor
```

---

## 🕹 Running the GUI

Install dependencies:

```bash
pip install bleak
```

Then launch:

```bash
python client.py
```

The GUI will:
- Scan for `ESP32-Robot`
- Connect over BLE
- Enable button controls with visual feedback (Red=Active, Green=Success)

---

## 📶 BLE Characteristics

| Name         | UUID                                   | Type  | Description                    |
|--------------|----------------------------------------|-------|--------------------------------|
| TOP Sensor   | `12345678-90ab-cdef-fedc-ba0987654321` | READ  | Returns top distance (cm)     |
| FRONT Sensor | `21436587-09ba-dcfe-efcd-ab9078563412` | READ  | Returns front distance (cm)   |
| Motor State  | `ABCDEF12-3456-7890-9078-563412EFCDAB` | READ  | Robot motion state (enum)     |
| Command      | `0000dead-0000-1000-8000-00805f9b34fb` | WRITE | Movement/Mode/Speed commands  |

---

## 🎮 BLE Commands Accepted

- `FORWARD`, `BACKWARD`, `LEFT`, `RIGHT`
- `ROTATE_CW`, `ROTATE_CCW`, `STOP`
- `MANUAL_START`, `MANUAL_STOP`
- `V<speed>` e.g. `V180` to change velocity (120–250 allowed)


