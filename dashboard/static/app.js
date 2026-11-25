const $ = (id) => document.getElementById(id);

let brokerConnected = false;
let pendingCommand = null;
let commandSequence = 0;
let speedUserEdited = false;
let activePage = "dashboard";
let networkFetchPending = false;
let missionFetchPending = false;
let lastMissionData = null;
let selectedMissionId = null;

const COMMAND_TIMEOUT_MS = 10000;
const TELEMETRY_FRESH_MS = 10000;

const missions = [
  "Idle",
  "Following",
  "Avoiding",
  "Fault",
  "Starting (3 s countdown)",
  "Waiting for IMU calibration",
];

const motionNames = {
  0: "Stationary",
  1: "Accelerating",
  2: "Turning",
  3: "Climbing",
  4: "Descending",
  5: "Impact",
  6: "Unknown",
};

const ROUTE_LETTER = {
  straight: "S",
  left: "L",
  right: "R",
  uturn: "U",
};

const hasNumber = (v) => typeof v === "number" && Number.isFinite(v);

const number = (v, digits = 2) => (hasNumber(v) ? v.toFixed(digits) : "--");

const isTrue = (v) => v === true || v === 1;

// The car measures and sends whole millimetres: show exactly that (0.1 cm),
// the same figure as the serial console's metres with three decimals.
const formatCm = (v) => (hasNumber(v) ? `${(v * 100).toFixed(1)} cm` : "--");

const setText = (id, value) => {
  const el = $(id);
  if (el) el.textContent = String(value);
};

const feedback = (message) => setText("feedback", message);

function setSystemStatus(label, state = "unknown") {
  const el = $("system-status");
  if (!el) return;
  el.textContent = label;
  el.classList.remove(
    "status-idle",
    "status-running",
    "status-avoiding",
    "status-fault",
    "status-starting",
    "status-calibrating",
    "status-unknown",
  );
  el.classList.add(`status-${state}`);
}

function setRampBadge(label, state = "neutral") {
  const el = $("ramp-status");
  if (!el) return;
  el.textContent = label;
  el.classList.remove(
    "badge-neutral",
    "badge-info",
    "badge-success",
    "badge-warning",
    "badge-danger",
  );
  el.classList.add(`badge-${state}`);
}

function formatDate(timestamp) {
  if (!hasNumber(timestamp)) return "Time unknown";

  return new Date(timestamp * 1000).toLocaleString("en-GB", {
    day: "2-digit",
    month: "2-digit",
    year: "numeric",
    hour: "numeric",
    minute: "2-digit",
    second: "2-digit",
    hour12: true,
  });
}

function missionTime(seconds) {
  if (!hasNumber(seconds)) return "--";

  const s = Math.max(0, Math.floor(seconds));

  return `${String(Math.floor(s / 60)).padStart(2, "0")}:${String(s % 60).padStart(2, "0")}`;
}

// ---------------------------------------------
// SPEED CONTROL
// ---------------------------------------------

const slider = $("speed-slider");

if (slider) {
  slider.value = "0.15";
  setText("speed-value", "0.15");

  slider.addEventListener("input", () => {
    speedUserEdited = true;
    setText("speed-value", Number(slider.value).toFixed(2));
  });
}

function syncSpeedControl(t, fresh) {
  if (!fresh || !slider || !hasNumber(t?.speed)) return;

  const speed = t.speed;

  if (speed < Number(slider.min) || speed > Number(slider.max)) return;

  if (speedUserEdited) {
    if (Math.abs(Number(slider.value) - speed) > 0.005) {
      return;
    }
    speedUserEdited = false;
  }

  if (document.activeElement === slider) return;

  slider.value = String(speed);
  setText("speed-value", speed.toFixed(2));
}

// ---------------------------------------------
// MQTT COMMANDS
// ---------------------------------------------

async function post(path, body = {}) {
  const sequence = ++commandSequence;

  try {
    const response = await fetch(path, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body),
    });

    const result = await response.json();

    if (!response.ok) {
      throw new Error(result.error || "Request failed");
    }

    if (sequence !== commandSequence) return;

    const payload = result.payload;

    if (payload === "START" || payload === "STOP") {
      pendingCommand = {
        type: payload,
        sentAt: Date.now() / 1000,
        startedAt: Date.now(),
      };

      feedback(`${payload} command sent. Waiting for robot confirmation...`);
    } else {
      feedback(
        result.topic
          ? `Published ${payload ?? "command"} to ${result.topic}.`
          : "Command request accepted.",
      );
    }
  } catch (error) {
    if (sequence !== commandSequence) return;

    pendingCommand = null;
    feedback(`Error: ${error.message}`);
  }
}

function requireBroker() {
  if (brokerConnected) return true;
  feedback("MQTT broker is disconnected.");
  return false;
}

$("route-form")?.addEventListener("submit", async (event) => {
  event.preventDefault();

  if (!requireBroker()) return;

  const form = new FormData(event.currentTarget);
  const values = Object.fromEntries(form.entries());

  routePublishPending = true;

  try {
    const response = await fetch("/api/route", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(values),
    });

    const result = await response.json();

    if (!response.ok) {
      throw new Error(result.error || "Request failed");
    }

    feedback("Route settings published. Waiting for robot confirmation...");

    setTimeout(() => {
      if (routePublishPending) {
        routePublishPending = false;

        feedback(
          "Route confirmation timed out. Check the robot's active route.",
        );
      }
    }, 10000);
  } catch (error) {
    routePublishPending = false;
    feedback(`Error publishing route: ${error.message}`);
  }
});

$("send-speed")?.addEventListener("click", () => {
  if (!requireBroker()) return;

  post("/api/speed", {
    speed: Number(slider.value),
  });
});

$("start-robot")?.addEventListener("click", () => {
  if (requireBroker()) post("/api/start");
});

$("stop-robot")?.addEventListener("click", () => {
  if (requireBroker()) post("/api/stop");
});

function checkCommandConfirmation(t, receivedAt) {
  if (!pendingCommand || !hasNumber(receivedAt)) return;
  if (receivedAt <= pendingCommand.sentAt) return;

  if (pendingCommand.type === "START") {
    if (t.mission === 4) {
      feedback("Robot received START. Countdown running...");
    } else if (t.mission === 5) {
      feedback(
        "Robot waiting for IMU calibration. Keep the car still and level.",
      );
    } else if (t.mission === 1 || t.mission === 2) {
      feedback("Robot START confirmed. Mission is running.");
      pendingCommand = null;
    }
  } else if (pendingCommand.type === "STOP" && t.mission === 0) {
    feedback("Robot STOP confirmed. Mission is idle.");
    pendingCommand = null;
  }
}

function checkCommandTimeout() {
  if (!pendingCommand) return;

  if (Date.now() - pendingCommand.startedAt >= COMMAND_TIMEOUT_MS) {
    const type = pendingCommand.type;
    pendingCommand = null;

    feedback(
      `${type} command sent, but robot confirmation timed out. Check MQTT and telemetry.`,
    );
  }
}

// ---------------------------------------------
// OBSTACLE WIDTH
// ---------------------------------------------

function updateObstacleWidth(t, fresh) {
  if (!fresh || !t) {
    setText("obstacle-width", "--");
    setText("width-detail", "Waiting for fresh telemetry");
    return;
  }

  const width = t.obstacle_width_cm ?? t.estimated_width_cm;

  const explicitlyInvalid =
    t.obstacle_width_valid !== undefined && !isTrue(t.obstacle_width_valid);

  if (explicitlyInvalid) {
    setText("obstacle-width", "Not measured");
    setText("width-detail", "Robot reports width measurement as invalid");
    return;
  }

  if (hasNumber(width) && width >= 0) {
    setText("obstacle-width", `${width.toFixed(1)} cm`);
    setText("width-detail", "Reported by robot telemetry");
    return;
  }

  setText("obstacle-width", "Not reported");

  setText(
    "width-detail",
    width === undefined || width === null
      ? "Firmware has not sent an obstacle width"
      : "Firmware sent a non-numeric or negative width",
  );
}

// ---------------------------------------------
// RAMP TELEMETRY
// ---------------------------------------------

function clearRampTelemetry(status = "Waiting for telemetry") {
  setRampBadge(status, "neutral");

  [
    "ramp-direction",
    "ramp-pitch",
    "ramp-rise",
    "ramp-peak",
    "ramp-height",
    "ramp-count",
  ].forEach((id) => setText(id, "--"));

  [
    "ramp-validity",
    "ramp-tracking",
    "ramp-calibration",
    "ramp-quality",
  ].forEach((id) => setText(id, "Unknown"));
}

function updateRampTelemetry(t, fresh) {
  if (!t || !fresh) {
    clearRampTelemetry(t ? "Telemetry stale" : "Waiting for telemetry");
    return;
  }

  const imuValid = isTrue(t.imu_valid);
  const tiltValid = isTrue(t.tilt_valid);
  const active = isTrue(t.hump_active);
  const valid = isTrue(t.hump_valid);

  let motion = hasNumber(t.motion)
    ? (motionNames[t.motion] ?? "Unknown")
    : "--";

  if (t.imu_valid !== undefined && !imuValid) {
    motion = "IMU unavailable";
  }

  setText("ramp-direction", motion);

  setText(
    "ramp-pitch",
    tiltValid && hasNumber(t.pitch_deg) ? `${t.pitch_deg.toFixed(1)}°` : "--",
  );

  setText("ramp-rise", formatCm(t.rise_m));
  setText("ramp-peak", formatCm(t.terrain_peak_m));
  setText("ramp-height", formatCm(t.hump_m));

  setText(
    "ramp-validity",
    t.hump_valid === undefined
      ? "Unknown"
      : valid
        ? "Valid"
        : active
          ? "Measurement in progress"
          : "Not validated",
  );

  setText(
    "ramp-tracking",
    t.height_tracking_valid === undefined
      ? "Unknown"
      : isTrue(t.height_tracking_valid)
        ? "Tracking valid"
        : "Not tracking",
  );

  setText(
    "ramp-calibration",
    t.imu_calibrated === undefined
      ? "Unknown"
      : isTrue(t.imu_calibrated)
        ? "Calibrated"
        : "Not calibrated",
  );

  setText("ramp-count", number(t.hump_count, 0));

  setText(
    "ramp-quality",
    t.height_gap_pending === undefined || t.height_approx === undefined
      ? "Unknown"
      : isTrue(t.height_gap_pending)
        ? "Timing gap pending"
        : isTrue(t.height_approx)
          ? "Approximate estimate"
          : "No quality warning",
  );

  const rampLabel =
    t.imu_valid !== undefined && !imuValid
      ? "IMU unavailable"
      : active
        ? "Hump crossing active"
        : valid
          ? "Height measurement available"
          : hasNumber(t.motion)
            ? "Monitoring terrain"
            : "Waiting for ramp telemetry";

  const rampState =
    rampLabel === "IMU unavailable"
      ? "warning"
      : rampLabel === "Hump crossing active"
        ? "info"
        : rampLabel === "Height measurement available"
          ? "success"
          : rampLabel === "Monitoring terrain"
            ? "info"
            : "neutral";

  setRampBadge(rampLabel, rampState);
}

// ---------------------------------------------
// ROUTE STATUS
// ---------------------------------------------

let lastConfirmedRoute = null;
let routeUserEdited = false;
let routePublishPending = false;

const routeForm = $("route-form");

routeForm?.addEventListener("input", () => {
  routeUserEdited = true;
});

function parseActiveRoute(active) {
  if (typeof active !== "string" || !active.trim()) return null;

  const mapping = {};
  const allowed = /^[0-9A-Z\-.$/+%]$/;

  for (const part of active.split(/[;,]/)) {
    const match = part.trim().match(/^([^:]+):\s*([SLRU])$/i);
    if (!match) continue;

    const symbol = match[1].trim().toUpperCase();
    const action = match[2].toUpperCase();
    if (symbol === "*" || !allowed.test(symbol)) continue;
    if (mapping[action] && mapping[action] !== symbol) return null;
    mapping[action] = symbol;
  }

  if (!Object.values(ROUTE_LETTER).every((action) => mapping[action])) {
    return null;
  }
  if (new Set(Object.values(mapping)).size !== 4) return null;
  return mapping;
}

function routeMatchesForm(mapping) {
  if (!routeForm || !mapping) return false;
  const form = new FormData(routeForm);
  return Object.entries(ROUTE_LETTER).every(([field, action]) => {
    const symbol = String(form.get(field) || "")
      .trim()
      .toUpperCase();
    return symbol === mapping[action];
  });
}

function restoreConfirmedRoute(mapping) {
  if (!routeForm || !mapping) return;
  for (const [field, action] of Object.entries(ROUTE_LETTER)) {
    const input = routeForm.elements.namedItem(field);
    if (input) input.value = mapping[action];
  }
}

function showRouteActive(active) {
  if (typeof active !== "string" || !active.trim()) {
    setText("route-active", "Unknown: waiting for robot route table");
    return;
  }

  const mapping = parseActiveRoute(active);
  if (!mapping) {
    setText("route-active", `${active} · Cannot restore all four actions`);
    return;
  }

  if (active !== lastConfirmedRoute) lastConfirmedRoute = active;

  if (routePublishPending && routeMatchesForm(mapping)) {
    routePublishPending = false;
    routeUserEdited = false;
    feedback("Dashboard route matches the last reported robot configuration.");
  }

  if (
    !routeUserEdited &&
    !routePublishPending &&
    !routeForm?.contains(document.activeElement)
  ) {
    restoreConfirmedRoute(mapping);
  }

  const matches = routeMatchesForm(mapping);
  setText(
    "route-active",
    matches
      ? `${active} ✓ matches the form`
      : `${active} ⚠ form differs from robot`,
  );
}

// ---------------------------------------------
// LIVE TELEMETRY
// ---------------------------------------------

function clearLiveTelemetry() {
  setText("current-speed", "--");
  setText("distance", "--");
  setText("mission", "--");
  setText("obstacle-state", "--");
  setText("obstacle-detail", "Waiting for fresh telemetry");
  setText("timestamp", "No uptime");
  setText("line", "--");
  setText("travelled", "--");
  setText("barcode", "--");
  setText("fault", "--");
  setText("junction-decision", "No fresh telemetry");
  setText("collision-state", "Status unavailable");
  setSystemStatus("Waiting for telemetry", "unknown");

  const collisionStatus = $("collision-status");
  collisionStatus?.classList.remove("detected", "clear");
  collisionStatus?.classList.add("unknown");

  updateObstacleWidth(null, false);
  clearRampTelemetry("Waiting for fresh telemetry");
}

function updateLiveTelemetry(t, receivedAt) {
  setText("current-speed", number(t.speed, 3));
  setText("distance", number(t.obstacle_cm, 1));

  setText(
    "mission",
    Number.isInteger(t.mission)
      ? (missions[t.mission] ?? "Unknown")
      : "Unknown",
  );

  const systemStates = {
    0: ["Idle", "idle"],
    1: ["Following", "running"],
    2: ["Avoiding", "avoiding"],
    3: ["Fault", "fault"],
    4: ["Starting", "starting"],
    5: ["Calibrating", "calibrating"],
  };
  const [systemLabel, systemState] = systemStates[t.mission] || [
    "Unknown",
    "unknown",
  ];
  setSystemStatus(systemLabel, systemState);

  const obstacleState =
    t.mission === 2
      ? "Avoiding"
      : t.mission === 3
        ? "Fault"
        : t.mission === 0
          ? "Idle"
          : t.mission === 1
            ? "No active avoidance"
            : "Unknown";

  setText("obstacle-state", obstacleState);

  setText(
    "obstacle-detail",
    t.mission === 2
      ? "Exact avoidance step not yet transmitted"
      : "Based on mission state",
  );

  setText(
    "timestamp",
    hasNumber(t.t_ms)
      ? `Robot uptime: ${(t.t_ms / 1000).toFixed(1)} s`
      : "No uptime",
  );

  setText("line", number(t.line, 3));
  setText("travelled", `${number(t.distance, 2)} m`);

  setText(
    "barcode",
    hasNumber(t.barcode) && t.barcode >= 33 && t.barcode < 127
      ? String.fromCharCode(t.barcode)
      : "--",
  );

  setText("fault", t.fault ?? "--");

  if (typeof t.junction_action === "string") {
    setText(
      "junction-decision",
      t.junction_count
        ? `#${t.junction_count}: ${
            t.junction_char ? `'${t.junction_char}'` : "no barcode"
          } → ${t.junction_action}`
        : "No junction yet",
    );
  }

  const pickupDetected = isTrue(t.pickup_detected) || t.fault_code === 8;

  setText(
    "collision-state",
    pickupDetected
      ? "Collision detected: robot lifted / stopped"
      : "No collision detected",
  );

  const collisionStatus = $("collision-status");
  collisionStatus?.classList.toggle("detected", pickupDetected);
  collisionStatus?.classList.toggle("clear", !pickupDetected);
  collisionStatus?.classList.remove("unknown");

  updateObstacleWidth(t, true);
  updateRampTelemetry(t, true);
  syncSpeedControl(t, true);
  checkCommandConfirmation(t, receivedAt);
}

let pollPending = false;

async function poll() {
  if (pollPending) return;
  pollPending = true;

  try {
    const response = await fetch("/api/status", {
      cache: "no-store",
    });

    if (!response.ok) {
      throw new Error(`Status API returned ${response.status}`);
    }

    const data = await response.json();

    brokerConnected = !!data.connected;

    const receivedAt = hasNumber(data.received_at) ? data.received_at : null;

    const ageMs =
      receivedAt === null ? Infinity : Date.now() - receivedAt * 1000;

    const fresh = brokerConnected && ageMs >= 0 && ageMs < TELEMETRY_FRESH_MS;

    const firmwareSimulation = isTrue(data.latest?.simulation);

    setText(
      "connection",
      firmwareSimulation
        ? "SIMULATION firmware: motors not driven"
        : fresh
          ? "Live telemetry"
          : brokerConnected
            ? "Broker connected · waiting"
            : "MQTT disconnected",
    );

    const connection = $("connection");

    if (connection) {
      connection.className = `status ${brokerConnected && !firmwareSimulation ? "online" : "offline"}`;
    }

    ["send-speed", "start-robot", "stop-robot"].forEach((id) => {
      const button = $(id);
      if (button) button.disabled = !brokerConnected;
    });

    const routeButton = $("route-form")?.querySelector("button");
    if (routeButton) routeButton.disabled = !brokerConnected;

    showRouteActive(data.route_active);

    if (fresh && data.latest) {
      updateLiveTelemetry(data.latest, receivedAt);
    } else {
      clearLiveTelemetry();
    }

    updateSidebarConnection(brokerConnected, fresh);
    checkCommandTimeout();
  } catch (error) {
    brokerConnected = false;

    setText("connection", "Server unavailable");

    if ($("connection")) {
      $("connection").className = "status offline";
    }

    ["send-speed", "start-robot", "stop-robot"].forEach((id) => {
      if ($(id)) $(id).disabled = true;
    });

    const routeButton = $("route-form")?.querySelector("button");
    if (routeButton) routeButton.disabled = true;

    clearLiveTelemetry();
    setSystemStatus("Server unavailable", "unknown");
    updateSidebarConnection(false, false);
    checkCommandTimeout();
  } finally {
    pollPending = false;
  }
}

// ---------------------------------------------
// NAVIGATION
// ---------------------------------------------

const pages = ["dashboard", "network", "diagnostics"];

function navigate(page) {
  if (!pages.includes(page)) return;

  activePage = page;

  pages.forEach((name) => {
    const el = $(`page-${name}`);
    if (el) el.hidden = name !== page;
  });

  document.querySelectorAll(".nav-link").forEach((button) => {
    const selected = button.dataset.page === page;

    button.classList.toggle("active", selected);

    if (selected) {
      button.setAttribute("aria-current", "page");
    } else {
      button.removeAttribute("aria-current");
    }
  });

  if (page === "network" || page === "diagnostics") {
    refreshNetwork();
  }

  if (page === "diagnostics") {
    refreshMissionHistory();
  }
}

document.querySelectorAll(".nav-link").forEach((button) => {
  button.addEventListener("click", () => {
    navigate(button.dataset.page);
  });
});

$("sidebar-toggle")?.addEventListener("click", () => {
  const sidebar = $("sidebar");
  if (!sidebar) return;

  const collapsed = sidebar.classList.toggle("collapsed");

  $("sidebar-toggle").setAttribute(
    "aria-label",
    collapsed ? "Expand sidebar" : "Collapse sidebar",
  );
});

$("refresh-network")?.addEventListener("click", refreshNetwork);

// ---------------------------------------------
// NETWORK & MQTT
// ---------------------------------------------

function telemetryHealth(data) {
  if (!data.connected) return "Broker disconnected";

  if (!hasNumber(data.received_at)) {
    return "Waiting for robot";
  }

  const age = Date.now() - data.received_at * 1000;

  return age >= 0 && age < TELEMETRY_FRESH_MS
    ? "Receiving live telemetry"
    : "Telemetry stale";
}

function updateSidebarConnection(connected, healthy) {
  $("sidebar-status-dot")?.classList.toggle("online", !!connected);

  setText(
    "sidebar-status-text",
    connected
      ? healthy
        ? "MQTT · live telemetry"
        : "MQTT · connected"
      : "MQTT disconnected",
  );
}

async function refreshNetwork() {
  if (networkFetchPending) return;
  networkFetchPending = true;

  try {
    const response = await fetch("/api/network", {
      cache: "no-store",
    });

    if (!response.ok) {
      throw new Error(`Network API returned ${response.status}`);
    }

    const data = await response.json();

    const broker = data.broker || {};
    const robot = data.robot_wifi || {};
    const telem = data.telemetry || {};

    setText(
      "net-broker-status",
      broker.connected ? "Connected" : "Disconnected",
    );

    setText("net-broker-host", broker.host || "Not configured");
    setText("net-broker-port", broker.port ?? "--");
    setText("net-client-id", broker.client_id || "--");
    setText("net-error", broker.error || "None reported");

    setText("net-robot-ssid", robot.ssid ?? "Not reported");
    setText("net-robot-ip", robot.ip ?? "Not reported");

    setText(
      "net-robot-rssi",
      hasNumber(robot.rssi_dbm) ? `${robot.rssi_dbm} dBm` : "Not reported",
    );

    setText(
      "net-telemetry-status",
      telemetryHealth({
        connected: broker.connected,
        received_at: telem.received_at,
      }),
    );

    setText(
      "net-last-received",
      hasNumber(telem.received_at)
        ? formatDate(telem.received_at)
        : "Never received",
    );

    setText(
      "net-telemetry-age",
      hasNumber(telem.received_at)
        ? `${Math.max(0, Date.now() / 1000 - telem.received_at).toFixed(1)} s`
        : "--",
    );

    setText("net-packet-count", telem.packet_count ?? "--");
    setText("net-last-topic", telem.last_topic || "None received");

    const topics = $("net-topics");

    if (topics && activePage === "network") {
      topics.replaceChildren();

      Object.entries(data.topics || {}).forEach(([key, value]) => {
        const entry = document.createElement("div");
        entry.className = "topic-entry";

        const label = document.createElement("small");
        label.textContent = key.replaceAll("_", " ");

        const code = document.createElement("code");
        code.textContent = value;

        entry.append(label, code);
        topics.append(entry);
      });
    }

    setText("diag-broker", broker.connected ? "Connected" : "Disconnected");

    setText(
      "diag-telemetry",
      telemetryHealth({
        connected: broker.connected,
        received_at: telem.received_at,
      }),
    );

    setText("diag-error", broker.error || "None reported");
    setText("network-error", "");
  } catch (error) {
    setText(
      "network-error",
      `Cannot fetch live network information: ${error.message}`,
    );

    setText("net-broker-status", "Server unavailable");
    setText("diag-broker", "Server unavailable");
  } finally {
    networkFetchPending = false;
  }
}

// ---------------------------------------------
// MISSION HISTORY
// ---------------------------------------------

function missionEventRow(event) {
  const row = document.createElement("div");
  row.className = "event-entry";

  const when = document.createElement("time");

  when.textContent = hasNumber(event.at)
    ? new Date(event.at * 1000).toLocaleTimeString("en-GB")
    : "Time unavailable";

  const label = document.createElement("span");
  const extra = event.details || {};
  const details = [];

  if (hasNumber(extra.speed_m_s)) {
    details.push(`${extra.speed_m_s.toFixed(2)} m/s`);
  }

  if (hasNumber(extra.height_m)) {
    details.push(`${(extra.height_m * 100).toFixed(1)} cm`);
  }

  if (extra.action) {
    details.push(`Action: ${extra.action}`);
  }

  if (hasNumber(extra.distance_m)) {
    details.push(`${extra.distance_m.toFixed(2)} m`);
  }

  const type = String(event.type || "EVENT").toUpperCase();

  const typeLabel = document.createElement("span");
  typeLabel.className = `event-label event-label-${type.toLowerCase()}`;
  typeLabel.textContent = `${type}: `;

  const description = document.createElement("span");
  description.textContent =
    event.message + (details.length ? ` · ${details.join(" · ")}` : "");

  label.replaceChildren(typeLabel, description);

  row.append(when, label);

  return row;
}

async function refreshMissionHistory() {
  if (activePage !== "diagnostics" || missionFetchPending) return;

  missionFetchPending = true;

  try {
    const query =
      selectedMissionId === null
        ? ""
        : `?id=${encodeURIComponent(selectedMissionId)}`;

    const response = await fetch(`/api/mission-history${query}`, {
      cache: "no-store",
    });

    if (!response.ok) {
      throw new Error(`Mission API returned ${response.status}`);
    }

    const data = await response.json();
    lastMissionData = data;

    const runs = Array.isArray(data.missions) ? data.missions : [];

    setText(
      "mh-run-count",
      `${runs.length} run${runs.length === 1 ? "" : "s"}`,
    );

    // Compact mission dropdown
    const runList = $("mh-runs");
    let select = $("mh-run-select");

    if (!select && runList) {
      runList.replaceChildren();

      select = document.createElement("select");
      select.id = "mh-run-select";
      select.setAttribute("aria-label", "Select recorded mission");

      select.addEventListener("change", () => {
        selectedMissionId = select.value ? Number(select.value) : null;

        refreshMissionHistory();
      });

      runList.append(select);
    }

    const selectedId = data.mission?.id ?? null;

    if (select) {
      const signature = runs
        .map((run) => [run.id, run.status, run.ended_at].join(":"))
        .join("|");

      if (select.dataset.signature !== signature) {
        select.replaceChildren();

        if (!runs.length) {
          const option = document.createElement("option");
          option.textContent = "No recorded missions yet";
          option.value = "";
          select.append(option);
        }

        for (const run of runs) {
          const option = document.createElement("option");
          option.value = String(run.id);

          const duration =
            hasNumber(run.ended_at) && hasNumber(run.started_at)
              ? missionTime(run.ended_at - run.started_at)
              : "In progress";

          option.textContent =
            `Run #${String(run.id).padStart(3, "0")}` +
            ` | ${run.status}` +
            ` | ${duration}` +
            ` | ${formatDate(run.started_at)}`;

          select.append(option);
        }

        select.dataset.signature = signature;
      }

      select.value = selectedId === null ? "" : String(selectedId);

      select.disabled = !runs.length;
    }

    // Mission summary
    const m = data.mission || {};
    const started = hasNumber(m.started_at);

    const end = hasNumber(m.ended_at) ? m.ended_at : Date.now() / 1000;

    setText("mh-status", m.status || "No mission recorded");

    setText("mh-duration", started ? missionTime(end - m.started_at) : "--");

    setText(
      "mh-speed",
      hasNumber(m.initial_speed) ? `${m.initial_speed.toFixed(2)} m/s` : "--",
    );

    setText(
      "mh-distance",
      hasNumber(m.final_distance_m)
        ? `${m.final_distance_m.toFixed(2)} m`
        : "--",
    );

    setText("mh-barcodes", started ? (m.barcode_count ?? "--") : "--");

    setText(
      "mh-peak",
      hasNumber(m.peak_hump_m)
        ? `${(m.peak_hump_m * 100).toFixed(1)} cm`
        : "--",
    );

    setText("mh-humps", m.hump_count ?? "--");

    setText("mh-obstacles", started ? (m.obstacle_count ?? "--") : "--");

    // Event timeline and barcode sequence
    const events = Array.isArray(data.events) ? data.events : [];

    const timeline = $("mh-events");
    const barcodeList = $("mh-barcode-list");

    const timelineSignature = `${selectedId}:${events.length}:${events.at(-1)?.at ?? ""}`;

    if (timeline && timeline.dataset.signature !== timelineSignature) {
      timeline.replaceChildren();

      if (!events.length) {
        timeline.textContent = "No mission events recorded yet.";
      } else {
        events.forEach((event) => {
          timeline.append(missionEventRow(event));
        });
      }

      timeline.dataset.signature = timelineSignature;

      if (barcodeList) {
        barcodeList.replaceChildren();

        const barcodes = events.filter((event) => event.type === "BARCODE");

        if (!barcodes.length) {
          barcodeList.textContent = "No barcode events recorded yet.";
        } else {
          barcodes.forEach((event) => {
            barcodeList.append(missionEventRow(event));
          });
        }
      }
    }

    if ($("download-mission")) {
      $("download-mission").disabled = !started;
    }

    setText("mh-error", "");
  } catch (error) {
    setText("mh-error", `Unable to load mission history: ${error.message}`);
  } finally {
    missionFetchPending = false;
  }
}

// ---------------------------------------------
// DOWNLOAD MISSION JSON
// ---------------------------------------------

$("download-mission")?.addEventListener("click", () => {
  if (!lastMissionData?.mission) return;

  const blob = new Blob([JSON.stringify(lastMissionData, null, 2)], {
    type: "application/json",
  });

  const url = URL.createObjectURL(blob);
  const a = document.createElement("a");

  a.href = url;
  a.download = `robot-mission-${lastMissionData.mission.id || "history"}.json`;

  a.click();

  setTimeout(() => URL.revokeObjectURL(url), 1000);
});

// ---------------------------------------------
// INITIALISE
// ---------------------------------------------

// Never reset backend telemetry on page load.
clearLiveTelemetry();
updateSidebarConnection(false, false);

poll();
// 4 Hz: the car sends ramp height every 200 ms while on a slope.
setInterval(poll, 250);

// Network and mission history update only when relevant.
setInterval(() => {
  if (activePage === "network" || activePage === "diagnostics") {
    refreshNetwork();
  }

  if (activePage === "diagnostics") {
    refreshMissionHistory();
  }
}, 2000);
