import os
import re
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

def on_message(c, userdata, msg):
    print("MQTT MESSAGE RECEIVED:", msg.topic, repr(msg.payload), flush=True)
    try:
        value = json.loads(msg.payload.decode('utf-8-sig'))
        if not isinstance(value, dict):
            return
    except (UnicodeDecodeError, ValueError):
        return
    except (UnicodeDecodeError, ValueError):
        return
    with lock:
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

def publish(topic, payload, retain=False):
    with lock:
        connected = state['connected']
    if not connected:
        return jsonify({'error': 'MQTT broker is disconnected'}), 503
    info = client.publish(topic, payload, qos=1, retain=retain)
    if info.rc != mqtt.MQTT_ERR_SUCCESS:
        return jsonify({'error': f'MQTT publish failed: {info.rc}'}), 503
    return jsonify({'ok': True, 'topic': topic, 'payload': payload,
                    'note': 'Broker accepted the publish request; robot application is not acknowledged.'})

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

if __name__ == '__main__':
    app.run(host='127.0.0.1', port=8080, debug=False)
