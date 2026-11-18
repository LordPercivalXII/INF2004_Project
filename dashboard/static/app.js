const $ = (id) => document.getElementById(id);

let brokerConnected = false;
let pendingCommand = null;
let commandSequence = 0;

const COMMAND_TIMEOUT_MS = 10000;
const TELEMETRY_FRESH_MS = 10000;

// 4 and 5 exist only in the binary packet: START received, not moving yet.
const missions = [
  "Idle",
  "Following",
  "Avoiding",
  "Fault",
  "Starting (3 s countdown)",
  "Waiting for IMU calibration",
];
const MISSION_STARTING = 4;
const MISSION_WAIT_IMU = 5;

const motionNames = {
  0: "Stationary",
  1: "Accelerating",
  2: "Turning",
  3: "Climbing",
  4: "Descending",
  5: "Impact",
  6: "Unknown",
};

const number = (v, digits = 2) =>
  typeof v === "number" && Number.isFinite(v) ? v.toFixed(digits) : "--";

const hasNumber = (value) =>
  typeof value === "number" && Number.isFinite(value);

const isTrue = (value) => value === 1 || value === true;

const formatCm = (value) =>
  hasNumber(value) ? `${(value * 100).toFixed(2)} cm` : "--";

const formatDegrees = (value) =>
  hasNumber(value) ? `${value.toFixed(1)}°` : "--";

const setText = (id, value) => {
  const element = $(id);
  if (element) element.textContent = value;
};

const feedback = (message) => {
  $("feedback").textContent = message;
};

// --------------------------------------------------
// Speed control, default 0.15 m/s
// --------------------------------------------------

const slider = $("speed-slider");
let speedUserEdited = false;

// Match the firmware's configured default until live telemetry arrives.
slider.value = "0.15";
setText("speed-value", "0.15");

function syncSpeedControl(t, fresh) {
  if (!fresh || !t || !hasNumber(t.speed)) return;

  const speed = t.speed;
  const min = Number(slider.min);
  const max = Number(slider.max);

  if (speed < min || speed > max) return;

  // Keep a manually selected value until the robot reports that speed.
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

slider.addEventListener("input", () => {
  speedUserEdited = true;
  setText("speed-value", Number(slider.value).toFixed(2));
});

// --------------------------------------------------
// Obstacle width
// --------------------------------------------------

function updateObstacleWidth(t, fresh) {
  const width = t && (t.obstacle_width_cm ?? t.estimated_width_cm);
  const valid =
    t &&
    (t.obstacle_width_valid === undefined || isTrue(t.obstacle_width_valid));

  if (fresh && valid && hasNumber(width) && width >= 0) {
    setText("obstacle-width", `${width.toFixed(1)} cm`);
    setText("width-detail", "Reported by robot telemetry");
  } else {
    setText("obstacle-width", "Unavailable");
    setText(
      "width-detail",
      fresh
        ? "No valid width measurement received"
        : "Waiting for fresh telemetry",
    );
  }
}

// --------------------------------------------------
// START / STOP command confirmation
// --------------------------------------------------

function checkCommandConfirmation(telemetry, receivedAt) {
  if (!pendingCommand) return;

  const pending = pendingCommand;

  // Do not confirm using telemetry received before the command was sent.
  if (typeof receivedAt !== "number" || receivedAt <= pending.sentAt) {
    return;
  }

  if (pending.type === "START" && telemetry.mission === MISSION_STARTING) {
    feedback("Robot received START. Countdown running...");
    pending.startedAt = Date.now();
    return;
  }

  if (pending.type === "START" && telemetry.mission === MISSION_WAIT_IMU) {
    feedback(
      "Robot received START but is waiting for IMU calibration. " +
        "Keep the car still and level.",
    );
    pending.startedAt = Date.now();
    return;
  }

  if (
    pending.type === "START" &&
    (telemetry.mission === 1 || telemetry.mission === 2)
  ) {
    feedback("Robot START confirmed. Mission is running.");
    pendingCommand = null;
    return;
  }

  if (pending.type === "STOP" && telemetry.mission === 0) {
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
      `${type} command sent, but robot confirmation timed out. ` +
        "Check MQTT reception and robot telemetry.",
    );
  }
}

// --------------------------------------------------
// MQTT command publishing
// --------------------------------------------------

async function post(path, body = {}) {
  const sequence = ++commandSequence;

  try {
    const response = await fetch(path, {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
      },
      body: JSON.stringify(body),
    });

    const result = await response.json();

    if (!response.ok) {
      throw new Error(result.error || "Request failed");
    }

    const payload = result.payload;

    if (sequence !== commandSequence) return;

    if (payload === "START" || payload === "STOP") {
      pendingCommand = {
        type: payload,
        sentAt: Date.now() / 1000,
        startedAt: Date.now(),
      };

      feedback(`${payload} command sent. Waiting for robot confirmation...`);
    } else {
      feedback(`Published ${payload} to ${result.topic}.`);
    }
  } catch (error) {
    if (sequence !== commandSequence) return;

    pendingCommand = null;
    feedback(`Error: ${error.message}`);
  }
}

// --------------------------------------------------
// Route configuration
// --------------------------------------------------

$("route-form").addEventListener("submit", (event) => {
  event.preventDefault();

  if (!brokerConnected) {
    return feedback("MQTT broker is disconnected.");
  }

  const form = new FormData(event.currentTarget);
  post("/api/route", Object.fromEntries(form.entries()));
});

// --------------------------------------------------
// Speed control
// --------------------------------------------------

$("send-speed").addEventListener("click", () => {
  if (!brokerConnected) {
    return feedback("MQTT broker is disconnected.");
  }

  post("/api/speed", {
    speed: Number(slider.value),
  });
});

// --------------------------------------------------
// START robot
// --------------------------------------------------

$("start-robot").addEventListener("click", () => {
  if (!brokerConnected) {
    return feedback("MQTT broker is disconnected.");
  }

  post("/api/start");
});

// --------------------------------------------------
// STOP robot
// --------------------------------------------------

$("stop-robot").addEventListener("click", () => {
  if (!brokerConnected) {
    return feedback("MQTT broker is disconnected.");
  }

  post("/api/stop");
});

// --------------------------------------------------
// Ramp Detection & Height telemetry
// --------------------------------------------------

function clearRampTelemetry(status = "Waiting for telemetry") {
  setText("ramp-status", status);
  setText("ramp-direction", "--");
  setText("ramp-pitch", "--");
  setText("ramp-rise", "--");
  setText("ramp-peak", "--");
  setText("ramp-height", "--");
  setText("ramp-validity", "Unknown");
  setText("ramp-tracking", "Unknown");
  setText("ramp-calibration", "Unknown");
  setText("ramp-count", "--");
  setText("ramp-quality", "Unknown");
}

function updateRampTelemetry(t, isFresh = true) {
  if (!t || !isFresh) {
    clearRampTelemetry(t ? "Telemetry stale" : "Waiting for telemetry");
    return;
  }

  const sensorValid = isTrue(t.imu_valid);
  const tiltValid = isTrue(t.tilt_valid);

  let motionLabel = "--";

  if (hasNumber(t.motion)) {
    motionLabel = motionNames[t.motion] ?? "Unknown";

    if (t.imu_valid !== undefined && !sensorValid) {
      motionLabel = "IMU unavailable";
    }
  }

  setText("ramp-direction", motionLabel);

  setText("ramp-pitch", tiltValid ? formatDegrees(t.pitch_deg) : "--");

  setText("ramp-rise", formatCm(t.rise_m));
  setText("ramp-peak", formatCm(t.terrain_peak_m));
  setText("ramp-height", formatCm(t.hump_m));

  const active = isTrue(t.hump_active);
  const valid = isTrue(t.hump_valid);
  const tracking = isTrue(t.height_tracking_valid);
  const calibrated = isTrue(t.imu_calibrated);
  const approximate = isTrue(t.height_approx);
  const gapPending = isTrue(t.height_gap_pending);

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
      : tracking
        ? "Tracking valid"
        : "Not tracking",
  );

  setText(
    "ramp-calibration",
    t.imu_calibrated === undefined
      ? "Unknown"
      : calibrated
        ? "Calibrated"
        : "Not calibrated",
  );

  setText("ramp-count", hasNumber(t.hump_count) ? String(t.hump_count) : "--");

  setText(
    "ramp-quality",
    t.height_gap_pending === undefined || t.height_approx === undefined
      ? "Unknown"
      : gapPending
        ? "Timing gap pending"
        : approximate
          ? "Approximate estimate"
          : "No quality warning",
  );

  if (t.imu_valid !== undefined && !sensorValid) {
    setText("ramp-status", "IMU unavailable");
  } else if (active) {
    setText("ramp-status", "Hump crossing active");
  } else if (valid) {
    setText("ramp-status", "Height measurement available");
  } else if (hasNumber(t.motion)) {
    setText("ramp-status", "Monitoring terrain");
  } else {
    setText("ramp-status", "Waiting for ramp telemetry");
  }
}

// --------------------------------------------------
// Poll robot telemetry
// --------------------------------------------------

const ROUTE_LETTER = {
  straight: "S",
  left: "L",
  right: "R",
  uturn: "U",
};

function showRouteActive(active) {
  const element = $("route-active");
  if (!element) return;

  if (typeof active !== "string") {
    element.textContent = "unknown (robot not connected yet)";
    return;
  }

  const entries = active.split(",").filter((e) => e && !e.startsWith("*:"));

  if (entries.length === 0) {
    element.textContent =
      "EMPTY: every junction goes straight. Press Publish route settings.";
    return;
  }

  const form = new FormData($("route-form"));

  const mismatched = Object.entries(ROUTE_LETTER).some(([field, letter]) => {
    const symbol = String(form.get(field) || "").toUpperCase();
    return !entries.includes(`${symbol}:${letter}`);
  });

  element.textContent = mismatched
    ? `${active}  ⚠ differs from the form: press Publish route settings`
    : `${active}  ✓ matches the form`;
}

async function poll() {
  try {
    const response = await fetch("/api/status", {
      cache: "no-store",
    });

    if (!response.ok) {
      throw new Error("Server unavailable");
    }

    const data = await response.json();

    brokerConnected = !!data.connected;

    const receivedAt =
      typeof data.received_at === "number" ? data.received_at : null;

    const ageMs =
      receivedAt !== null ? Date.now() - receivedAt * 1000 : Infinity;

    const recent = brokerConnected && ageMs >= 0 && ageMs < TELEMETRY_FRESH_MS;

    $("connection").textContent = brokerConnected
      ? recent
        ? "Live telemetry"
        : "Broker connected · waiting"
      : "MQTT disconnected";

    $("connection").className =
      `status ${brokerConnected ? "online" : "offline"}`;

    if (data.latest && data.latest.simulation) {
      $("connection").textContent +=
        " · SIMULATION firmware: motors not driven (rebuild with make SIM=0)";
      $("connection").className = "status offline";
    }

    $("send-speed").disabled = !brokerConnected;
    $("start-robot").disabled = !brokerConnected;
    $("stop-robot").disabled = !brokerConnected;

    $("route-form").querySelector("button").disabled = !brokerConnected;

    showRouteActive(data.route_active);

    const t = data.latest;

    if (!t) {
      clearRampTelemetry();
      updateObstacleWidth(null, false);
      checkCommandTimeout();
      return;
    }

    updateObstacleWidth(t, recent);
    syncSpeedControl(t, recent);

    // --------------------------------------------------
    // Robot telemetry
    // --------------------------------------------------

    $("current-speed").textContent = number(t.speed, 3);

    $("distance").textContent = number(t.obstacle_cm, 1);

    const mission = missions[t.mission] || "Unknown";

    $("mission").textContent = mission;

    $("obstacle-state").textContent =
      t.mission === 2
        ? "Avoiding"
        : t.mission === 3
          ? "Fault"
          : t.mission === 0
            ? "Idle"
            : t.mission === 1
              ? "No active avoidance"
              : "Unknown";

    if (typeof t.junction_action === "string") {
      setText(
        "junction-decision",
        t.junction_count
          ? `#${t.junction_count}: ` +
              (t.junction_char ? `'${t.junction_char}'` : "no barcode") +
              ` → ${t.junction_action}`
          : "No junction yet",
      );
    }

    $("obstacle-detail").textContent =
      t.mission === 2
        ? "Exact avoidance step not yet transmitted"
        : "Based on mission state";

    $("timestamp").textContent =
      typeof t.t_ms === "number"
        ? `Robot uptime: ${(t.t_ms / 1000).toFixed(1)} s`
        : "No uptime";

    $("line").textContent = number(t.line, 3);

    $("travelled").textContent = `${number(t.distance, 2)} m`;

    $("barcode").textContent =
      hasNumber(t.barcode) && t.barcode >= 33 && t.barcode < 127
        ? String.fromCharCode(t.barcode)
        : "--";

    $("fault").textContent = t.fault ?? "--";

    const pickupDetected = isTrue(t.pickup_detected) || t.fault_code === 8;

    $("collision-state").textContent = pickupDetected
      ? "Collision detected: robot lifted / stopped"
      : "No collision detected";

    $("collision-status").classList.toggle("detected", pickupDetected);

    // --------------------------------------------------
    // Ramp Detection & Height
    // --------------------------------------------------

    updateRampTelemetry(t, recent);

    // --------------------------------------------------
    // START / STOP confirmation
    // --------------------------------------------------

    if (recent) {
      checkCommandConfirmation(t, receivedAt);
    }

    checkCommandTimeout();
  } catch (error) {
    brokerConnected = false;

    $("connection").textContent = "Server unavailable";
    $("connection").className = "status offline";

    $("send-speed").disabled = true;
    $("start-robot").disabled = true;
    $("stop-robot").disabled = true;

    $("route-form").querySelector("button").disabled = true;

    clearRampTelemetry("Server unavailable");
    updateObstacleWidth(null, false);

    checkCommandTimeout();
  }
}

// --------------------------------------------------
// Start polling
// --------------------------------------------------

fetch("/api/reset", { method: "POST" })
  .catch(() => {})
  .finally(() => {
    poll();
    setInterval(poll, 1000);
  });


// --------------------------------------------------
// Navigation and live Network & MQTT information
// --------------------------------------------------
const pages = ["dashboard", "network", "diagnostics"];
let activePage = "dashboard";
function navigate(page) {
  if (!pages.includes(page)) return;
  activePage = page;
  pages.forEach((name) => {
    $("page-" + name).hidden = name !== page;
  });
  document.querySelectorAll(".nav-link").forEach((button) => {
    const selected = button.dataset.page === page;
    button.classList.toggle("active", selected);
    if (selected) button.setAttribute("aria-current", "page");
    else button.removeAttribute("aria-current");
  });
  if (page === "network" || page === "diagnostics") refreshNetwork();
  if (page === "diagnostics") refreshMissionHistory();
}
document.querySelectorAll(".nav-link").forEach((button) => {
  button.addEventListener("click", () => navigate(button.dataset.page));
});
$("sidebar-toggle").addEventListener("click", () => {
  const collapsed = $("sidebar").classList.toggle("collapsed");
  $("sidebar-toggle").setAttribute("aria-label", collapsed ? "Expand sidebar" : "Collapse sidebar");
});
$("refresh-network").addEventListener("click", refreshNetwork);

function formatReceivedTime(value) {
  if (typeof value !== "number" || !Number.isFinite(value)) return "Never received";
  return new Date(value * 1000).toLocaleString();
}
function telemetryHealth(data) {
  if (!data.connected) return "Broker disconnected";
  if (typeof data.received_at !== "number") return "Waiting for robot";
  const age = Date.now() - data.received_at * 1000;
  return age >= 0 && age < TELEMETRY_FRESH_MS ? "Receiving live telemetry" : "Telemetry stale";
}
function updateSidebarConnection(connected, healthy) {
  $("sidebar-status-dot").classList.toggle("online", !!connected);
  setText("sidebar-status-text", connected ? (healthy ? "MQTT · live telemetry" : "MQTT · connected") : "MQTT disconnected");
}
let networkFetchPending = false;
async function refreshNetwork() {
  if (networkFetchPending) return;
  networkFetchPending = true;
  try {
    const response = await fetch("/api/network", { cache: "no-store" });
    if (!response.ok) throw new Error(`Network API returned ${response.status}`);
    const data = await response.json();
    const broker = data.broker || {};
    const robot = data.robot_wifi || {};
    const telem = data.telemetry || {};
    setText("net-broker-status", broker.connected ? "Connected" : "Disconnected");
    setText("net-broker-host", broker.host || "Not configured");
    setText("net-broker-port", broker.port ?? "--");
    setText("net-client-id", broker.client_id || "--");
    setText("net-error", broker.error || "None reported");
    setText("net-robot-ssid", robot.ssid ?? "Not reported");
    setText("net-robot-ip", robot.ip ?? "Not reported");
    setText("net-robot-rssi", typeof robot.rssi_dbm === "number" ? `${robot.rssi_dbm} dBm` : "Not reported");
    setText("net-telemetry-status", telemetryHealth({ connected: broker.connected, received_at: telem.received_at }));
    setText("net-last-received", formatReceivedTime(telem.received_at));
    setText("net-telemetry-age", typeof telem.received_at === "number" ? `${Math.max(0, (Date.now() / 1000 - telem.received_at)).toFixed(1)} s` : "--");
    setText("net-packet-count", telem.packet_count ?? "--");
    setText("net-last-topic", telem.last_topic || "None received");
    const topics = $("net-topics");
    if (activePage === "network") {
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
    setText("diag-telemetry", telemetryHealth({ connected: broker.connected, received_at: telem.received_at }));
    setText("diag-error", broker.error || "None reported");
    const events = $("diag-events");
    if (activePage === "diagnostics") {
    events.replaceChildren();
    if (!Array.isArray(data.events) || data.events.length === 0) {
      events.textContent = "No connection events recorded yet.";
    } else {
      [...data.events].reverse().forEach((event) => {
        const item = document.createElement("div");
        item.className = "event-entry";
        const time = document.createElement("time");
        time.textContent = formatReceivedTime(event.at);
        const detail = document.createElement("span");
        detail.textContent = `${event.type}: ${event.message}`;
        item.append(time, detail);
        events.append(item);
      });
    }
    }
    setText("network-error", "");
  } catch (error) {
    setText("network-error", `Cannot fetch live network information: ${error.message}`);
    setText("net-broker-status", "Server unavailable");
    setText("diag-broker", "Server unavailable");
  } finally {
    networkFetchPending = false;
  }
}
// Refresh background network status independently of the existing dashboard poll.
setInterval(() => {
  updateSidebarConnection(brokerConnected, $("connection").textContent.startsWith("Live telemetry"));
  if (activePage === "network" || activePage === "diagnostics") refreshNetwork();
  if (activePage === "diagnostics") refreshMissionHistory();
}, 2000);
// Network details load when their page is opened.



// Mission history is recorded by Flask from incoming MQTT telemetry.
// The page only renders when opened, and never generates fake readings.
let missionFetchPending = false;
let lastMissionData = null;
let selectedMissionId = null;
function missionTime(seconds) {
  if (typeof seconds !== "number" || !Number.isFinite(seconds)) return "--";
  const s = Math.max(0, Math.floor(seconds));
  return `${String(Math.floor(s / 60)).padStart(2, "0")}:${String(s % 60).padStart(2, "0")}`;
}
function missionEventRow(e) {
  const row = document.createElement("div");
  row.className = "event-entry";
  const when = document.createElement("time");
  when.textContent = typeof e.at === "number" ? new Date(e.at * 1000).toLocaleTimeString() : "Time unavailable";
  const label = document.createElement("span");
  const extra = e.details || {};
  const details = [];
  if (typeof extra.speed_m_s === "number") details.push(`${extra.speed_m_s.toFixed(2)} m/s`);
  if (typeof extra.height_m === "number") details.push(`${(extra.height_m * 100).toFixed(2)} cm`);
  if (extra.action) details.push(`Action: ${extra.action}`);
  if (typeof extra.distance_m === "number") details.push(`${extra.distance_m.toFixed(2)} m`);
  label.textContent = `${e.type}: ${e.message}${details.length ? " · " + details.join(" · ") : ""}`;
  row.append(when, label);
  return row;
}
async function refreshMissionHistory() {
  if (activePage !== "diagnostics" || missionFetchPending) return;
  missionFetchPending = true;
  try {
    const query = selectedMissionId === null ? "" : `?id=${encodeURIComponent(selectedMissionId)}`;
    const response = await fetch(`/api/mission-history${query}`, { cache: "no-store" });
    if (!response.ok) throw new Error(`Mission API returned ${response.status}`);
    const data = await response.json();
    lastMissionData = data;
    const runs = Array.isArray(data.missions) ? data.missions : [];
    setText("mh-run-count", `${runs.length} run${runs.length === 1 ? "" : "s"}`);
    const runList = $("mh-runs");
    runList.replaceChildren();
    if (!runs.length) runList.textContent = "No missions recorded yet. Start a robot mission to create the first record.";
    const selectedId = data.mission?.id ?? null;
    for (const run of runs) {
      const button = document.createElement("button");
      button.type = "button";
      button.className = `mission-run-button${run.id === selectedId ? " selected" : ""}`;
      const start = typeof run.started_at === "number" ? new Date(run.started_at * 1000).toLocaleString() : "Time unknown";
      const duration = typeof run.started_at === "number"
        ? missionTime((typeof run.ended_at === "number" ? run.ended_at : Date.now() / 1000) - run.started_at)
        : "--";
      // Keep the run list factual and simple. Do not infer completion or success.
      button.textContent = `Run #${String(run.id).padStart(3, "0")}  ·  ${start}  ·  ${duration}  ·  ${run.barcode_count ?? 0} barcode events`;
      button.addEventListener("click", () => { selectedMissionId = run.id; refreshMissionHistory(); });
      runList.append(button);
    }
    const m = data.mission || {};
    const started = typeof m.started_at === "number";
    setText("mh-status", m.status || "No mission recorded");
    const end = typeof m.ended_at === "number" ? m.ended_at : Date.now() / 1000;
    setText("mh-duration", started ? missionTime(end - m.started_at) : "--");
    setText("mh-speed", typeof m.initial_speed === "number" ? `${m.initial_speed.toFixed(2)} m/s` : "--");
    setText("mh-distance", typeof m.final_distance_m === "number" ? `${m.final_distance_m.toFixed(2)} m` : "--");
    setText("mh-barcodes", started ? m.barcode_count : "--");
    setText("mh-peak", typeof m.peak_hump_m === "number" ? `${(m.peak_hump_m * 100).toFixed(2)} cm` : "--");
    setText("mh-humps", m.hump_count ?? "--");
    setText("mh-obstacles", started ? m.obstacle_count : "--");
    const events = Array.isArray(data.events) ? data.events : [];
    const timeline = $("mh-events");
    timeline.replaceChildren();
    if (!events.length) timeline.textContent = "No mission events recorded yet.";
    else events.forEach(e => timeline.append(missionEventRow(e)));
    const barcodeList = $("mh-barcode-list");
    barcodeList.replaceChildren();
    const barcodes = events.filter(e => e.type === "BARCODE");
    if (!barcodes.length) barcodeList.textContent = "No barcode events recorded yet.";
    else barcodes.forEach(e => barcodeList.append(missionEventRow(e)));
    $("download-mission").disabled = !started;
    setText("mh-error", "");
  } catch (error) {
    setText("mh-error", `Unable to load mission history: ${error.message}`);
  } finally {
    missionFetchPending = false;
  }
}
$("download-mission").addEventListener("click", () => {
  if (!lastMissionData) return;
  const blob = new Blob([JSON.stringify(lastMissionData, null, 2)], { type: "application/json" });
  const url = URL.createObjectURL(blob);
  const a = document.createElement("a");
  a.href = url;
  a.download = `robot-mission-${lastMissionData.mission?.id || "history"}.json`;
  a.click();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
});
