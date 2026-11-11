import os
import json
import time
import threading
from collections import deque
from flask import Flask, jsonify, request
import paho.mqtt.client as mqtt

app = Flask(__name__, static_folder='static', static_url_path='')
HOST = os.getenv('MQTT_BROKER_HOST', '192.168.137.1')
PORT = int(os.getenv('MQTT_BROKER_PORT', '1883'))
TELEMETRY = 'inf2004/car/telemetry'
ROUTE = 'inf2004/car/route'
CONTROL = 'inf2004/car/control'
lock = threading.Lock()
state = {'connected': False, 'latest': None, 'received_at': None, 'error': None}
history = deque(maxlen=120)
# The robot keeps its last barcode until reboot, so after a reset the first
# value seen is treated as old and hidden until a different one arrives.
barcode_baseline = {'value': None, 'pending': False}
client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id='buddy1-dashboard')

def on_connect(c, userdata, flags, reason_code, properties):
    with lock:
        state['connected'] = not reason_code.is_failure
        state['error'] = None if state['connected'] else str(reason_code)
    if not reason_code.is_failure:
        c.subscribe(TELEMETRY, qos=0)

def on_disconnect(c, userdata, disconnect_flags, reason_code, properties):
    with lock:
        state['connected'] = False

PACKET_SIZES = (4, 5)     # 4: before the barcode byte was added


def decode_packet(payload, previous):
    """Decode the robot's 5-byte binary telemetry packet.

    Layout (app_program/subsystem1/telemetry_codec.h, bit 0 = LSB):
      byte 0: bits 0-2 state (0 idle, 1 following, 2 avoiding, 3 fault,
              4 START countdown, 5 START waiting for IMU), bits 3-7 faults
      byte 1: commanded speed, cm/s
      byte 2: line error, offset-binary (128 = centre), error = (b-128)/128
      byte 3: bit 7 = 1 obstacle (bits 0-6 x 2 cm), 0 hump (bits 0-6 mm)
      byte 4: last decoded barcode character (ASCII), 0 = none
    Byte 3 alternates between hump and obstacle, so the other value is
    carried over from the previous packet.
    """
    b0, b1, b2, b3 = payload[:4]
    value = {
        'mission': b0 & 0x07,
        'fault': (b0 >> 3) & 0x1F,
        'speed': b1 / 100.0,
        'line': (b2 - 128) / 128.0,
        'hump_m': (previous or {}).get('hump_m'),
        'obstacle_cm': (previous or {}).get('obstacle_cm'),
        'binary': True,
    }
    if len(payload) >= 5:
        value['barcode'] = payload[4]     # number, as in the JSON payload
    if b3 & 0x80:
        value['obstacle_cm'] = (b3 & 0x7F) * 2.0
    else:
        value['hump_m'] = (b3 & 0x7F) / 1000.0
    return value


def on_message(c, userdata, msg):
    print('MQTT MESSAGE RECEIVED:', msg.topic, repr(msg.payload), flush=True)
    if len(msg.payload) in PACKET_SIZES:
        with lock:
            previous = state['latest']
        value = decode_packet(msg.payload, previous)
    else:
        # PACK=0 firmware still publishes the JSON text payload.
        try:
            value = json.loads(msg.payload.decode('utf-8-sig'))
            if not isinstance(value, dict):
                return
        except (UnicodeDecodeError, ValueError):
            return
    with lock:
        if barcode_baseline['pending']:
            barcode_baseline['value'] = value.get('barcode')
            barcode_baseline['pending'] = False
        if barcode_baseline['value'] and value.get('barcode') == barcode_baseline['value']:
            value['barcode'] = 0
        state['latest'] = value
        state['received_at'] = time.time()
        history.append(value)

client.on_connect = on_connect
client.on_disconnect = on_disconnect
client.on_message = on_message

def mqtt_loop():
    while True:
        try:
            client.connect(HOST, PORT, keepalive=60)
            client.loop_forever(retry_first_connection=True)
        except Exception as exc:
            with lock:
                state['connected'] = False
                state['error'] = str(exc)
            time.sleep(3)

threading.Thread(target=mqtt_loop, daemon=True).start()

@app.get('/')
def index():
    return app.send_static_file('index.html')

@app.get('/api/status')
def status():
    with lock:
        return jsonify({**state, 'history': list(history)})

@app.post('/api/reset')
def reset():
    with lock:
        state['latest'] = None
        state['received_at'] = None
        history.clear()
        barcode_baseline['value'] = None
        barcode_baseline['pending'] = True
    return jsonify({'ok': True})

@app.after_request
def no_cache(response):
    response.headers['Cache-Control'] = 'no-store'
    return response

def publish(topic, payload, retain=False):
    with lock:
        connected = state['connected']
    if not connected:
        return jsonify({'error': 'MQTT broker is disconnected'}), 503
    info = client.publish(topic, payload, qos=1, retain=retain)
    if info.rc != mqtt.MQTT_ERR_SUCCESS:
        return jsonify({'error': f'MQTT publish failed: {info.rc}'}), 503
    return jsonify({'ok': True, 'topic': topic, 'payload': payload,
                    'note': 'MQTT publish queued; robot application is not acknowledged.'})

CODE39 = set('0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-.$/+%')

@app.post('/api/route')
def route():
    data = request.get_json(silent=True) or {}
    keys = ('straight', 'left', 'right', 'uturn')
    values = []
    for key in keys:
        val = data.get(key)
        if not isinstance(val, str) or len(val) != 1 or val.upper() not in CODE39:
            return jsonify({'error': f'{key} must be one valid Code 39 character'}), 400
        values.append(val.upper())
    if len(set(values)) != 4:
        return jsonify({'error': 'Each turn action needs a different character'}), 400
    payload = 'CLEAR,' + ','.join(f'{symbol}:{action}' for symbol, action in zip(values, ('S', 'L', 'R', 'U')))
    return publish(ROUTE, payload, retain=True)

@app.post('/api/speed')
def speed():
    data = request.get_json(silent=True) or {}
    value = data.get('speed')
    if isinstance(value, bool) or not isinstance(value, (float, int)) or not 0.10 <= value <= 0.70:
        return jsonify({'error': 'Speed must be between 0.10 and 0.70 m/s'}), 400
    return publish(CONTROL, f'SPEED:{value:.2f}', retain=False)

@app.post('/api/start')
def start():
    return publish(CONTROL, 'START', retain=False)

@app.post('/api/stop')
def stop():
    return publish(CONTROL, 'STOP', retain=False)

if __name__ == '__main__':
    app.run(host='127.0.0.1', port=8080, debug=False)
