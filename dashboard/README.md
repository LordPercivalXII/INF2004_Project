# Robot Control & Telemetry Dashboard

INF2004 Embedded Systems Programming, Team CSC19

## 1. Overview

This dashboard provides a web-based interface for monitoring and configuring the Pico W robot car through MQTT.

It supports:

- Live robot telemetry monitoring
- Robot speed configuration
- Barcode-to-turn mapping configuration
- MQTT broker connection status
- Robot mission and obstacle status

The dashboard is implemented using Python Flask, Paho MQTT, HTML, CSS, and JavaScript.

## 2. Requirements

Install the following software:

| Software | Purpose |
|---|---|
| Python 3 | Run the Flask dashboard |
| pip | Install Python dependencies |
| Mosquitto MQTT Broker | Exchange messages between the dashboard and robot |
| Web browser | Access the dashboard |

### Python dependencies

The required Python packages are listed in `requirements.txt`.

From the project root, run:

```powershell
python -m pip install -r .\dashboard\requirements.txt
```

### Mosquitto installation

Download and install Mosquitto:

https://mosquitto.org/download/

On Windows, the default installation directory is typically:

```text
C:\Program Files\mosquitto
```

Ensure that Mosquitto is installed before starting the dashboard.

## 3. Project Structure

```text
dashboard/
├── README.md
├── requirements.txt
├── server.py
└── static/
    ├── index.html
    ├── style.css
    └── app.js
```

- `server.py`: Flask server, MQTT connection, and API endpoints
- `requirements.txt`: Python dependencies
- `static/index.html`: Dashboard interface
- `static/style.css`: Dashboard styling
- `static/app.js`: Frontend logic

## 4. Running the Dashboard

Open separate PowerShell terminals from the repository root.

### Terminal 1: Start Mosquitto

For the Windows installation used during development:

```powershell
& "C:\Program Files\mosquitto\mosquitto.exe" -c "C:\Program Files\mosquitto\mosquitto.conf" -v
```

Keep this terminal running.

The broker configuration must include a listener on port `1883`. For local testing, the broker may be restricted to localhost.

For a physical Pico W connection, configure the broker to accept connections from the robot's network and apply appropriate access controls.

### Terminal 2: Start Flask

For local dashboard testing:

```powershell
$env:MQTT_BROKER_HOST = "127.0.0.1"
python .\dashboard\server.py
```

The dashboard connects to the MQTT broker on port `1883` by default.

Open the following URL:

http://127.0.0.1:8080

### Terminal 3: Monitor MQTT Messages (Optional)

```powershell
& "C:\Program Files\mosquitto\mosquitto_sub.exe" -h 127.0.0.1 -t "inf2004/car/#" -v
```

This displays MQTT messages exchanged through the broker.

## 5. MQTT Topics

| Topic | Direction | Purpose |
|---|---|---|
| `inf2004/car/telemetry` | Robot → Dashboard | Live robot telemetry |
| `inf2004/car/route` | Dashboard → Robot | Barcode route configuration |
| `inf2004/car/control` | Dashboard → Robot | Speed configuration |

### Telemetry

The dashboard subscribes to:

```text
inf2004/car/telemetry
```

Example payload:

```json
{
  "t_ms": 28000,
  "mission": 2,
  "speed": 0.35,
  "line": -1.0,
  "distance": 5.41,
  "obstacle_cm": 25.0,
  "barcode": 65,
  "fault": 0
}
```

The dashboard displays the latest telemetry received.

### Speed Control

The dashboard publishes speed commands to:

```text
inf2004/car/control
```

Example:

```text
SPEED:0.50
```

The dashboard currently accepts values between `0.10` and `0.70` m/s.

The firmware validates incoming commands against its configured minimum and maximum speed limits.

### Barcode Route Configuration

The dashboard publishes mappings to:

```text
inf2004/car/route
```

Example:

```text
CLEAR,A:S,B:L,C:R,D:U
```

| Barcode | Action |
|---|---|
| A | Straight |
| B | Left |
| C | Right |
| D | U-turn |

The dashboard supports selecting valid Code 39 characters.

`CLEAR` removes previous mappings before applying the new configuration.

Route messages are published with MQTT retain enabled, allowing the broker to provide the latest configuration to the robot after reconnection.

## 6. Testing Without the Physical Robot

The dashboard can be tested by publishing simulated telemetry.

Start the broker and Flask server first.

From another PowerShell terminal, run:

```powershell
'{"t_ms":28000,"mission":2,"speed":0.35,"line":-1.0,"distance":5.41,"obstacle_cm":25.0,"barcode":65,"fault":0}' | & "C:\Program Files\mosquitto\mosquitto_pub.exe" -h 127.0.0.1 -t "inf2004/car/telemetry" -s
```

Refresh or inspect the dashboard.

Expected results:

- MQTT connection status shows connected.
- Latest telemetry values appear.
- Speed displays approximately `0.350 m/s`.
- Distance displays `5.41 m`.
- Straight-ahead distance displays `25.0 cm`.

To test dashboard publishing, use the speed and barcode configuration controls while monitoring messages in Terminal 3.

Successful MQTT publication confirms that messages reach the broker, not that the physical robot has executed the commands.

## 7. Connecting to the Physical Pico W

During development, the laptop hotspot broker address was:

```text
192.168.137.1
```

This address is environment-specific and may differ on another laptop.

For physical testing:

1. Connect the Pico W and laptop to the intended network.
2. Configure Mosquitto to accept connections from the Pico W.
3. Ensure the firewall permits the required MQTT traffic.
4. Set the firmware's broker address to the laptop's reachable IP address.
5. Start Mosquitto and the Flask dashboard.
6. Flash the compatible firmware onto the Pico W.
7. Verify MQTT subscriptions and telemetry through the serial monitor and dashboard.

Do not commit Wi-Fi passwords or private MQTT credentials to Git.

**Important:** Source-code integration has been inspected, but the complete dashboard-to-physical-robot command path still requires runtime testing.

## 8. Troubleshooting

### Dashboard shows MQTT disconnected

- Check that Mosquitto is running.
- Confirm the broker host and port.
- Check firewall and network configuration.

### Dashboard shows connected but no telemetry

- Confirm that messages are published to `inf2004/car/telemetry`.
- Verify that the payload is valid JSON.
- Use `mosquitto_sub` to inspect incoming messages.
- Check whether the Pico W is connected and publishing.

### Speed or route commands do not affect the robot

- Verify that the messages appear in `mosquitto_sub`.
- Check the Pico W serial logs for successful MQTT subscriptions.
- Confirm the firmware is running the updated command handlers.
- Verify that the controller and route-processing tasks are active.

### Port 8080 is already in use

Stop the existing dashboard process before starting another instance.

## 9. Implementation Notes

The dashboard is intended for local development and demonstration.

It does not currently provide robot command acknowledgements. A successful API response indicates that the MQTT client accepted the publish request, not that the robot completed the requested action.

Do not expose the Flask development server or an unauthenticated MQTT broker directly to the public internet.

## 10. Current Verification Status

- Dashboard UI: Implemented
- Flask server: Tested
- MQTT broker connection: Tested
- Simulated telemetry display: Tested
- Speed command publication: Tested
- Barcode route publication: Tested
- Firmware MQTT subscriptions: Source-code verified
- Firmware speed and route handlers: Source-code verified
- Physical robot integration: Pending
