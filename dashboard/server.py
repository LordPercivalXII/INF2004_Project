import os
import json
import time
import threading
import sqlite3
from pathlib import Path
from collections import deque
from flask import Flask, jsonify, request
import paho.mqtt.client as mqtt

app = Flask(__name__, static_folder='static', static_url_path='')
HOST = os.getenv('MQTT_BROKER_HOST', '192.168.137.1')
PORT = int(os.getenv('MQTT_BROKER_PORT', '1883'))
TELEMETRY = 'inf2004/car/telemetry'
ROUTE = 'inf2004/car/route'
ROUTE_ACTIVE = 'inf2004/car/route/active'
CONTROL = 'inf2004/car/control'
lock = threading.Lock()
state = {'connected': False, 'latest': None, 'received_at': None, 'error': None,
         'route_active': None}
history = deque(maxlen=120)
mission_events = deque(maxlen=100)
DEMO_MODE = os.getenv('ROBO_DEMO_MODE') == '1'
DEFAULT_DB_NAME = 'mission_history_demo.sqlite3' if DEMO_MODE else 'mission_history.sqlite3'
DB_PATH = Path(os.getenv('MISSION_DB_PATH', str(Path(__file__).resolve().parent / DEFAULT_DB_NAME)))
MAX_MISSIONS = 30
_last_mission_save = 0.0
mission = {'id': 0, 'status': 'No mission recorded', 'started_at': None, 'ended_at': None, 'initial_speed': None, 'last_speed': None, 'final_distance_m': None, 'peak_hump_m': None, 'barcode_count': 0, 'obstacle_count': 0, 'hump_count': None}
mission_tracker = {'last_mission': None, 'last_barcode': None, 'last_junction_count': None, 'last_hump_count': None, 'last_hump_active': False, 'last_obstacle': False, 'last_speed': None, 'last_seen_at': None, 'last_hump_valid': False}
network_events = deque(maxlen=80)
network_stats = {'packet_count': 0, 'last_topic': None}
robot_wifi = {'ssid': None, 'ip': None, 'rssi_dbm': None}
barcode_baseline = {'value': None, 'pending': False}
client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id='buddy1-dashboard')



def db_connect():
    db = sqlite3.connect(str(DB_PATH), timeout=5)
    db.execute("PRAGMA busy_timeout=5000")
    return db


def init_mission_db():
    DB_PATH.parent.mkdir(parents=True, exist_ok=True)
    with db_connect() as db:
        db.execute("CREATE TABLE IF NOT EXISTS missions (id INTEGER PRIMARY KEY, started_at REAL NOT NULL, data TEXT NOT NULL)")
        db.execute("CREATE TABLE IF NOT EXISTS mission_events (id INTEGER PRIMARY KEY AUTOINCREMENT, mission_id INTEGER NOT NULL, at REAL NOT NULL, data TEXT NOT NULL)")
        db.execute("CREATE INDEX IF NOT EXISTS mission_events_by_mission ON mission_events(mission_id, id)")
        row = db.execute("SELECT data FROM missions ORDER BY id DESC LIMIT 1").fetchone()
        if row:
            restored = json.loads(row[0])
            if restored.get('status') in ('Completed', 'Success', 'Failed'):
                restored['status'] = 'Ended (unconfirmed)'
            mission.update(restored)
            mission_events.extend(json.loads(r[0]) for r in db.execute(
                "SELECT data FROM mission_events WHERE mission_id=? ORDER BY id DESC LIMIT 100", (mission['id'],)
            ).fetchall()[::-1])
            # A server restart cannot establish that the old run is still active.
            if mission.get('started_at') is not None and mission.get('ended_at') is None:
                mission['ended_at'] = time.time()
                mission['status'] = 'Interrupted (server restart)'
                save_mission()


def save_mission():
    if not mission.get('id') or mission.get('started_at') is None:
        return
    with db_connect() as db:
        db.execute("INSERT OR REPLACE INTO missions(id, started_at, data) VALUES(?,?,?)",
                   (mission['id'], mission['started_at'], json.dumps(mission)))
        old = db.execute("SELECT id FROM missions ORDER BY id DESC LIMIT -1 OFFSET ?", (MAX_MISSIONS,)).fetchall()
        if old:
            ids = [r[0] for r in old]
            db.executemany("DELETE FROM mission_events WHERE mission_id=?", [(i,) for i in ids])
            db.executemany("DELETE FROM missions WHERE id=?", [(i,) for i in ids])


def save_mission_event(event):
    if not mission.get('id'):
        return
    with db_connect() as db:
        db.execute("INSERT INTO mission_events(mission_id, at, data) VALUES(?,?,?)",
                   (mission['id'], event['at'], json.dumps(event)))
        db.execute("DELETE FROM mission_events WHERE mission_id=? AND id NOT IN "
                   "(SELECT id FROM mission_events WHERE mission_id=? ORDER BY id DESC LIMIT 100)",
                   (mission['id'], mission['id']))


init_mission_db()

def add_event(kind, message):
    # Caller must hold lock.
    network_events.append({'at': time.time(), 'type': kind, 'message': str(message)})


def on_connect(c, userdata, flags, reason_code, properties):
    connected = not reason_code.is_failure
    with lock:
        state['connected'] = connected
        state['error'] = None if connected else str(reason_code)
        add_event('MQTT', 'Broker connected' if connected else f'Connection rejected: {reason_code}')
    if connected:
        c.subscribe(TELEMETRY, qos=0)
        c.subscribe(ROUTE_ACTIVE, qos=1)


def on_disconnect(c, userdata, disconnect_flags, reason_code, properties):
    with lock:
        state['connected'] = False
        add_event('MQTT', f'Broker disconnected: {reason_code}')


PACKET_SIZES = (4, 5, 7)
ACTIONS = ('STRAIGHT', 'LEFT', 'RIGHT', 'U-TURN')


def decode_packet(payload, previous):
    """Decode the project's 4-byte status/ramp/IMU/metadata frames and legacy 5/7-byte frames."""
    b0, b1, b2, b3 = payload[:4]
    if len(payload) == 4 and (b0 & 0x07) == 6:
        value = dict(previous or {})
        flags = (b0 >> 3) & 0x1F
        rise_mm = b1 if b1 < 0x80 else b1 - 0x100
        value.update({
            'imu_valid': bool(flags & 0x01),
            'tilt_valid': bool(flags & 0x01),
            'imu_calibrated': bool(flags & 0x02),
            'height_tracking_valid': bool(flags & 0x04),
            'hump_active': bool(flags & 0x08),
            'hump_valid': bool(flags & 0x10),
            'height_approx': True,
            'rise_m': rise_mm / 1000.0,
            'terrain_peak_m': b2 / 1000.0,
            'hump_m': b3 / 1000.0,
            'binary': True,
        })
        return value

    if len(payload) == 4 and (b0 & 0x07) == 7 and (b0 & 0x08):
        value = dict(previous or {})
        pitch_qdeg = b2 if b2 < 0x80 else b2 - 0x100
        value.update({
            'imu_valid': bool(b0 & 0x10),
            'motion': b1,
            'pitch_deg': pitch_qdeg / 4.0,
            'hump_count': b3,
            'height_approx': bool(b0 & 0x20),
            'height_gap_pending': bool(b0 & 0x40),
            'pickup_detected': bool(b0 & 0x80),
            'binary': True,
        })
        return value

    if len(payload) == 4 and (b0 & 0x07) == 7:
        value = dict(previous or {})
        value.update({
            'simulation': bool(b0 & 0x80),
            'barcode': b1 & 0x7F,
            'junction_char': chr(b2) if b2 else '',
            'junction_action': ACTIONS[b3 & 0x03],
            'junction_count': b3 >> 2,
            'binary': True,
        })
        return value

    value = {
        'mission': b0 & 0x07,
        'fault': (b0 >> 3) & 0x1F,
        'speed': b1 / 100.0,
        'line': (b2 - 128) / 128.0,
        'hump_m': (previous or {}).get('hump_m'),
        'obstacle_cm': (previous or {}).get('obstacle_cm'),
        'barcode': (previous or {}).get('barcode', 0),
        'simulation': (previous or {}).get('simulation', False),
        'junction_char': (previous or {}).get('junction_char', ''),
        'junction_action': (previous or {}).get('junction_action'),
        'junction_count': (previous or {}).get('junction_count', 0),
        'binary': True,
    }
    if len(payload) >= 5:
        value['barcode'] = payload[4] & 0x7F
        value['simulation'] = bool(payload[4] & 0x80)
    if len(payload) >= 7:
        value['junction_char'] = chr(payload[5]) if payload[5] else ''
        value['junction_action'] = ACTIONS[payload[6] & 0x03]
        value['junction_count'] = payload[6] >> 2
    if b3 & 0x80:
        value['obstacle_cm'] = (b3 & 0x7F) * 2.0
    else:
        value['hump_m'] = (b3 & 0x7F) / 1000.0
    return value


def on_message(c, userdata, msg):
    # Avoid synchronous printing of every packet in the MQTT callback.
    with lock:
        network_stats['packet_count'] += 1
        network_stats['last_topic'] = msg.topic
    if msg.topic == ROUTE_ACTIVE:
        with lock:
            state['route_active'] = msg.payload.decode('ascii', 'replace')
        return
    if msg.topic != TELEMETRY:
        return
    if len(msg.payload) in PACKET_SIZES:
        with lock:
            previous = state['latest']
        value = decode_packet(msg.payload, previous)
    else:
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
        record_mission(value, state['received_at'])



def _num(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and float('-inf') < value < float('inf')


def _event(kind, message, at, **details):
    # Caller holds lock. Store only bounded, meaningful events.
    event = {'at': at, 'type': kind, 'message': message, 'details': details}
    mission_events.append(event)
    save_mission_event(event)


def record_mission(t, at):
    """Build a compact history only from confirmed incoming telemetry.

    Mission start is based on a robot-reported state transition, never the
    browser Start click. Hump and barcode events need counters or transitions.
    """
    global _last_mission_save
    if not isinstance(t, dict):
        return
    prev = mission_tracker
    state_code = t.get('mission')
    if isinstance(state_code, str):
        names = {'idle': 0, 'following': 1, 'avoiding': 2, 'fault': 3, 'starting': 4}
        state_code = names.get(state_code.lower())
    if not isinstance(state_code, int) or isinstance(state_code, bool) or state_code not in range(6):
        state_code = None
    active = state_code in (1, 2, 4, 5)
    previous_code = prev['last_mission']
    if active and (mission['started_at'] is None or mission['ended_at'] is not None) and previous_code in (0, None):
        mission['id'] += 1
        mission.update({'status': 'Running', 'started_at': at, 'ended_at': None,
                        'initial_speed': t.get('speed') if _num(t.get('speed')) else None,
                        'last_speed': t.get('speed') if _num(t.get('speed')) else None,
                        'final_distance_m': None, 'peak_hump_m': None,
                        'barcode_count': 0, 'obstacle_count': 0, 'hump_count': None})
        mission_events.clear()
        prev.update({'last_barcode': None, 'last_junction_count': None,
                     'last_hump_count': None, 'last_hump_active': False,
                     'last_obstacle': False, 'last_speed': None, 'last_hump_valid': False})
        save_mission()
        _event('MISSION', 'Robot reported mission started', at, mission_state=state_code,
               speed_m_s=mission['initial_speed'])
    running = mission['started_at'] is not None and mission['ended_at'] is None
    if running:
        if previous_code != state_code and state_code is not None:
            if state_code == 2:
                _event('OBSTACLE', 'Robot entered avoidance state', at)
                mission['obstacle_count'] += 1
            elif state_code == 1 and previous_code == 2:
                _event('MISSION', 'Robot returned to line following', at)
            elif state_code == 3:
                _event('ERROR', 'Robot reported fault state', at, fault=t.get('fault'))
        speed = t.get('speed')
        if _num(speed):
            if _num(prev['last_speed']) and abs(speed - prev['last_speed']) >= 0.015:
                _event('SPEED', 'Reported speed changed', at,
                       previous_m_s=prev['last_speed'], speed_m_s=speed)
            prev['last_speed'] = speed
            mission['last_speed'] = speed
        distance = t.get('distance_m', t.get('distance'))
        if _num(distance):
            mission['final_distance_m'] = distance
        # Barcode: prefer an increasing junction counter, since a barcode can
        # stay in multiple snapshots. Without a counter, use 0 -> code edge.
        code = t.get('junction_char') or t.get('barcode')
        if isinstance(code, int):
            code = chr(code) if 32 <= code <= 126 else None
        if not isinstance(code, str) or not code.strip():
            code = None
        count = t.get('junction_count')
        count_valid = isinstance(count, int) and not isinstance(count, bool)
        changed_count = count_valid and prev['last_junction_count'] is not None and count > prev['last_junction_count']
        edge = code is not None and prev['last_barcode'] is None
        # Never treat a repeated unchanged barcode as another detection.
        if code and (changed_count or (not count_valid and edge)):
            _event('BARCODE', f'Barcode {code} reported', at, code=code,
                   action=t.get('junction_action'), junction_count=count if count_valid else None)
            mission['barcode_count'] += 1
        if count_valid:
            prev['last_junction_count'] = count
        prev['last_barcode'] = code
        hump_count = t.get('hump_count')
        count_ok = isinstance(hump_count, int) and not isinstance(hump_count, bool)
        hump_active = t.get('hump_active') is True
        hump_valid = t.get('hump_valid') is True
        new_hump = (count_ok and prev['last_hump_count'] is not None and hump_count > prev['last_hump_count'])
        # A confirmed active -> inactive transition is another usable event
        # if no count is provided. Do not invent an event from a peak alone.
        if not count_ok and prev['last_hump_active'] and not hump_active and prev['last_hump_valid']:
            new_hump = True
        if new_hump:
            h = t.get('hump_m')
            if not _num(h):
                h = t.get('terrain_peak_m')
            _event('RAMP', 'Hump detection completed', at,
                   height_m=h if _num(h) and (hump_valid or count_ok) else None,
                   hump_count=hump_count if count_ok else None)
        if count_ok:
            mission['hump_count'] = hump_count
            prev['last_hump_count'] = hump_count
        prev['last_hump_active'] = hump_active
        prev['last_hump_valid'] = hump_valid
        # Peak height is shown only when firmware marks it valid.
        peak = t.get('terrain_peak_m')
        if t.get('height_tracking_valid') is True and _num(peak):
            if mission['peak_hump_m'] is None or peak > mission['peak_hump_m']:
                mission['peak_hump_m'] = peak
        if state_code in (0, 3) and previous_code in (1, 2, 4, 5):
            mission['ended_at'] = at
            mission['status'] = 'Stopped' if state_code == 0 else 'Fault'
            _event('MISSION', f"Robot reported {mission['status'].lower()}", at,
                   distance_m=mission['final_distance_m'])
    if running and (at - _last_mission_save >= 2 or mission['ended_at'] is not None):
        save_mission()
        _last_mission_save = at
    prev['last_mission'] = state_code
    prev['last_seen_at'] = at


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
                add_event('MQTT', f'Connection error: {exc}')
            time.sleep(3)


if not DEMO_MODE:
    threading.Thread(target=mqtt_loop, daemon=True).start()


@app.get('/')
def index():
    return app.send_static_file('index.html')


@app.get('/api/status')
def status():
    with lock:
        return jsonify({**state, 'history': list(history)})


@app.get('/api/network')
def network():
    with lock:
        return jsonify({
            'broker': {
                'host': HOST, 'port': PORT, 'client_id': 'buddy1-dashboard',
                'connected': state['connected'], 'error': state['error'],
            },
            'robot_wifi': dict(robot_wifi),
            'telemetry': {
                'received_at': state['received_at'],
                'packet_count': network_stats['packet_count'],
                'last_topic': network_stats['last_topic'],
            },
            'topics': {
                'telemetry': TELEMETRY, 'route': ROUTE,
                'route_active': ROUTE_ACTIVE, 'control': CONTROL,
            },
            'events': list(network_events),
        })


@app.get('/api/mission-history')
def mission_history():
    requested_id = request.args.get('id', type=int)
    with lock:
        with db_connect() as db:
            rows = db.execute("SELECT id, data FROM missions ORDER BY id DESC LIMIT ?", (MAX_MISSIONS,)).fetchall()
            missions = [json.loads(row[1]) for row in rows]
            for run in missions:
                if run.get('status') in ('Completed', 'Success', 'Failed'):
                    run['status'] = 'Ended (unconfirmed)'
            selected = next((m for m in missions if m['id'] == requested_id), None) if requested_id is not None else (missions[0] if missions else None)
            if selected:
                events = [json.loads(row[0]) for row in db.execute(
                    "SELECT data FROM mission_events WHERE mission_id=? ORDER BY id ASC LIMIT 100", (selected['id'],)
                ).fetchall()]
            else:
                events = []
        return jsonify({'missions': missions, 'mission': selected, 'events': events,
                        'source': 'local simulator' if os.getenv('ROBO_DEMO_MODE') == '1' else 'received MQTT telemetry',
                        'limits': {'missions': MAX_MISSIONS, 'events_per_mission': 100},
                        'note': 'Saved in local SQLite. Demo records are synthetic when ROBO_DEMO_MODE=1; unknown values remain unknown.'})


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
    if DEMO_MODE:
        return jsonify({'error': 'Robot commands are disabled in local demo mode'}), 503
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


# Local-only development simulator. Disabled unless explicitly enabled.
# Uses the same telemetry callback as MQTT, but never publishes to the robot.
_simulation_busy = threading.Event()


def _demo_mission():
    from types import SimpleNamespace
    def emit(mission_code, speed, distance, barcode=None, junction_count=0,
             action=None, hump_count=0, peak=0.0, height_valid=False):
        data = {'mission': mission_code, 'speed': speed, 'distance_m': distance,
                'barcode': barcode or 0, 'junction_char': barcode or '',
                'junction_count': junction_count, 'junction_action': action,
                'hump_count': hump_count, 'hump_active': False,
                'hump_valid': height_valid, 'height_tracking_valid': height_valid,
                'terrain_peak_m': peak, 'simulation': True}
        msg = SimpleNamespace(topic=TELEMETRY, payload=json.dumps(data).encode('utf-8'))
        on_message(None, None, msg)
    try:
        # Establish a known idle baseline, then send staged robot-like packets.
        emit(0, 0.0, 0.0)
        time.sleep(0.7)
        emit(1, 0.20, 0.0)
        time.sleep(1.3)
        emit(1, 0.20, 0.8, 'A', 1, 'STRAIGHT')
        time.sleep(1.3)
        emit(1, 0.20, 1.2, None, 1)
        emit(1, 0.28, 1.7, None, 1)
        time.sleep(1.3)
        emit(1, 0.28, 2.4, None, 1, hump_count=1, peak=0.038, height_valid=True)
        time.sleep(1.3)
        emit(2, 0.18, 3.1, None, 1, hump_count=1, peak=0.038, height_valid=True)
        time.sleep(1.3)
        emit(1, 0.20, 4.2, None, 1, hump_count=1, peak=0.038, height_valid=True)
        time.sleep(1.3)
        emit(1, 0.20, 5.4, 'B', 2, 'LEFT', 1, 0.038, True)
        time.sleep(1.3)
        emit(1, 0.20, 6.7, None, 2, hump_count=1, peak=0.038, height_valid=True)
        time.sleep(1.3)
        emit(0, 0.0, 7.8, None, 2, hump_count=1, peak=0.038, height_valid=True)
    finally:
        _simulation_busy.clear()


@app.get('/api/dev/info')
def dev_info():
    return jsonify({'demo_mode': DEMO_MODE, 'simulation_running': _simulation_busy.is_set()})


@app.post('/api/dev/simulate')
def dev_simulate():
    if os.getenv('ROBO_DEMO_MODE') != '1':
        return jsonify({'error': 'Simulation mode disabled'}), 404
    if _simulation_busy.is_set():
        return jsonify({'error': 'Simulation already running'}), 409
    _simulation_busy.set()
    threading.Thread(target=_demo_mission, daemon=True).start()
    return jsonify({'ok': True, 'mode': 'SIMULATED',
                    'note': 'Synthetic telemetry; no robot or broker required.'}), 202


if __name__ == '__main__':
    app.run(host='127.0.0.1', port=8080, debug=False)
