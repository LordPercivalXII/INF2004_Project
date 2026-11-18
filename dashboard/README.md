# RoboControl Dashboard | INF2004 CSC19

This README explains the **laptop-side dashboard** maintained for Buddy 1 (WiFi, MQTT and telemetry), including how to run it, what teammates can see, and how mission records are saved automatically.

> **Scope:** These instructions describe the updated Flask dashboard with Dashboard, Network & MQTT, and Mission History pages. They are not instructions for compiling or flashing the Pico W firmware. Keep the dashboard's four source files together (`server.py`, `static/index.html`, `static/app.js`, `static/style.css`) because the frontend and API must match.

## 1. What the dashboard does

The dashboard is a local web application. The **Pico W robot** sends telemetry over WiFi to the **MQTT broker** on the laptop. The **Flask server** subscribes to that telemetry, exposes API endpoints and stores mission records. The **browser** displays the data.

```text
Pico W robot  --WiFi/MQTT-->  Mosquitto broker (laptop)
                                  |
                                  v
                           Flask server.py
                            |          |
                            v          v
                       Browser UI   SQLite history
```

**Real mode never invents sensor readings.** Missing or unsupported telemetry fields remain unavailable (`--` or a waiting message). The browser being open does not mean the robot is connected.

## 2. Dashboard pages

### Dashboard

- **Live readings:** speed, straight-ahead obstacle distance/state, mission state, line error, distance travelled, last barcode and junction decision, fault code, and pickup/collision indication when the firmware supplies these values.
- **Ramp / IMU:** motion classification, pitch, estimated rise, peak terrain height, last hump height, validity/tracking/calibration indicators, and hump count when reported.
- **Controls:** Start, Stop, speed request, and Code 39 barcode-to-turn mapping (straight, left, right, U-turn).
- Commands are **requests published over MQTT**, not proof the robot carried them out. The displayed mission state changes only when telemetry confirms it. **STOP is not an emergency stop.** Secure the robot during testing.

### Network & MQTT

- Displays Flask's MQTT broker connection status, telemetry freshness and connection-related diagnostics/events.
- Helps identify whether the laptop can reach the broker and whether robot packets are arriving.
- Robot WiFi SSID, IP address or RSSI must not be assumed if firmware does not report them.

### Mission History

- Keeps a **list of recorded runs** so teammates can select and review earlier missions.
- Shows a summary and chronological timeline of **meaningful telemetry-derived events**, such as mission state changes, barcode/junction actions, speed changes, ramp/hump information, and obstacle/avoidance transitions where supported.
- Shows barcode sequence, connection diagnostics and recent MQTT events, and supports exporting history as JSON in the UI.
- Avoids logging every identical telemetry packet as a new mission event.
- **Does not claim a run was “completed successfully”** just because the robot stopped. A stop/idle transition is not proof of task completion.
- If the robot has never reported a mission, an empty list and `--` values are normal. An **HTTP 404 API error is not normal** and usually means the running Flask backend does not match the frontend files.

## 3. Requirements

- Windows laptop with Python 3 and access to the robot's MQTT broker.
- Python dependencies from `dashboard/requirements.txt` (Flask and `paho-mqtt`).
- Mosquitto MQTT broker, running on the laptop or another reachable host, for **real robot mode**.
- Pico W robot flashed with firmware that publishes the expected MQTT telemetry.

## 4. Start the dashboard (Windows PowerShell)

From the **repository root** in VS Code:

```powershell
python -m pip install -r dashboard/requirements.txt
python dashboard/server.py
```

Open **http://127.0.0.1:8080** in a browser.

The default broker configuration in `server.py` is:

```text
MQTT_BROKER_HOST = 192.168.137.1
MQTT_BROKER_PORT = 1883
```

If your laptop/broker uses a different address, set the environment variables **before** starting Flask:

```powershell
$env:MQTT_BROKER_HOST = "192.168.137.1"
$env:MQTT_BROKER_PORT = "1883"
python dashboard/server.py
```

The IP `192.168.137.1` is the team's previously used Windows hotspot address. **Verify the current broker IP** using `ipconfig`; do not assume it stays the same on every laptop. Start Mosquitto with your team's approved configuration and ensure the robot and laptop can reach each other.

Stop the dashboard with **Ctrl+C** in the terminal running Flask. Refresh the browser with **Ctrl+Shift+R** after updating HTML/JS/CSS.

## 5. Run a safe dashboard-only demo (no robot)

The updated backend includes an **explicit opt-in local demo mode** to preview Mission History. This is **synthetic test data**, not real telemetry.

In PowerShell, from the repository root:

```powershell
$env:ROBO_DEMO_MODE = "1"
python dashboard/server.py
```

Open **http://127.0.0.1:8080**, navigate to **Mission History**, and select **Simulate a mission** if the button appears. Alternatively, from a second PowerShell terminal:

```powershell
Invoke-RestMethod -Method POST -Uri http://127.0.0.1:8080/api/dev/simulate
```

Demo mode runs without a connected robot or broker. Robot Start/Stop/Speed/Route publishing is **disabled** in this mode to prevent confusing test actions with live commands.

To return to **real robot mode**, stop Flask and run:

```powershell
Remove-Item Env:ROBO_DEMO_MODE -ErrorAction SilentlyContinue
python dashboard/server.py
```

Do not use the demo database as evidence of actual robot performance.

## 6. Mission history storage (automatic setup)

**No database download or manual setup is needed.** When Flask initializes mission history, it creates its local SQLite storage automatically. Each teammate gets their own local mission records. Normal operation stores real telemetry-derived history; optional demo mode uses separate test records so they do not mix with real runs.

Generated database files are ignored by Git and **should not be committed**. They may appear in VS Code Explorer after running the dashboard, which is expected. History survives Flask restarts. To clear local records, stop Flask first and remove the generated database, understanding that this permanently deletes that machine's saved history.

The repository `.gitignore` should include `dashboard/*.sqlite3`, `dashboard/*.sqlite3-wal`, and `dashboard/*.sqlite3-shm`.

## 7. MQTT topics and API checks

The backend uses these topics:

| Topic | Direction | Purpose |
| --- | --- | --- |
| `inf2004/car/telemetry` | Robot → Flask | Live telemetry, including supported compact binary or JSON payloads |
| `inf2004/car/route/active` | Robot → Flask | Robot's active barcode route table |
| `inf2004/car/control` | Flask → Robot | Start, Stop, speed commands |
| `inf2004/car/route` | Flask → Robot | Barcode route mapping |

Useful local API checks while Flask is running:

```text
http://127.0.0.1:8080/api/status
http://127.0.0.1:8080/api/network
http://127.0.0.1:8080/api/mission-history
http://127.0.0.1:8080/api/dev/info
```

`/api/mission-history` should return JSON with a mission list and selected mission/events, even if no missions have been recorded. The `/api/dev/simulate` POST endpoint is available only when demo mode is enabled.

## 8. Common issues

| Symptom | What to check |
| --- | --- |
| `MQTT disconnected` / connection timeout | Mosquitto running, broker IP/port, Windows firewall, laptop hotspot and robot WiFi |
| `No missions recorded yet` | Normal if no mission telemetry has been received; real runs are not fabricated |
| Mission History API returned `404` | Ensure `dashboard/server.py` is the updated version with `/api/mission-history`; restart Flask, hard refresh browser |
| Dashboard values stay `--` | Check MQTT connection and whether the firmware publishes those exact fields |
| SQLite database is locked | Stop other Flask/Python instances using the same database before moving or deleting it |
| Old mission history still appears | Real and demo databases are separate; confirm `ROBO_DEMO_MODE` and which DB is in use |
| Browser UI doesn't reflect new changes | Stop/restart Flask, then hard refresh; verify `server.py` and `static/` files came from the same update |

## 9. Notes for teammates

1. **Dashboard files run on the laptop.** Changes to HTML/CSS/JS/Python do not require reflashing the Pico W. Firmware changes do require a new firmware build and flash.
2. The dashboard is a **monitor and command interface**, not the source of truth for robot behavior. Confirmed telemetry is the source of displayed mission states.
3. Real mission history and demo history are **separate**. The demo is for UI testing only.
4. A disconnected MQTT broker does not necessarily mean Flask is broken. The webpage and local APIs can still load.
5. When changing telemetry packet formats, MQTT topics or command strings, coordinate firmware and dashboard changes so the decoder remains compatible.
6. Before a live test, confirm the correct Pico W firmware, hotspot/broker settings, and safe physical setup.

**Maintained for:** INF2004 Embedded Systems Programming, CSC19, Buddy 1 dashboard integration.
