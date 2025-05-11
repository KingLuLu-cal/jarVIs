from flask import Flask, jsonify, request, render_template
from bleak import BleakClient, BleakScanner
import asyncio
import threading

app = Flask(__name__)

TARGET_NAME = "ESP32-Robot"
CHAR_UUID = "0000dead-0000-1000-8000-00805f9b34fb"
ble_client = None
loop = asyncio.new_event_loop()

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

def start_loop():
    asyncio.set_event_loop(loop)
    loop.run_forever()

threading.Thread(target=start_loop, daemon=True).start()

@app.route('/')
def index():
    return render_template("index.html")

@app.route('/scan', methods=['GET'])
def scan():
    async def do_scan():
        devices = await BleakScanner.discover(timeout=5)
        return [d.name for d in devices if d.name]

    future = asyncio.run_coroutine_threadsafe(do_scan(), loop)
    result = future.result()
    return jsonify(result)

@app.route('/connect', methods=['POST'])
def connect():
    global ble_client

    name = request.json.get("name")
    async def do_connect():
        devices = await BleakScanner.discover(timeout=5)
        for d in devices:
            if d.name == name:
                client = BleakClient(d.address)
                await client.connect()
                ble_client = client
                return client.is_connected
        return False

    future = asyncio.run_coroutine_threadsafe(do_connect(), loop)
    connected = future.result()
    return jsonify({"connected": connected})

@app.route('/command', methods=['POST'])
def send_command():
    global ble_client
    cmd = request.json.get("command")
    if cmd not in COMMANDS:
        return jsonify({"error": "Invalid command"}), 400

    async def do_send():
        if ble_client and ble_client.is_connected:
            await ble_client.write_gatt_char(CHAR_UUID, COMMANDS[cmd].encode(), response=True)
            return True
        return False

    future = asyncio.run_coroutine_threadsafe(do_send(), loop)
    success = future.result()
    return jsonify({"sent": success})

if __name__ == '__main__':
    app.run(debug=True)
