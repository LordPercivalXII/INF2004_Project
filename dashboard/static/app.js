
const $ = (id) => document.getElementById(id);

let brokerConnected = false;
let pendingCommand = null;
let commandSequence = 0;

const COMMAND_TIMEOUT_MS = 10000;
const TELEMETRY_FRESH_MS = 10000;

const missions = ["Idle", "Following", "Avoiding", "Fault"];

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
  typeof v === "number" && Number.isFinite(v)
    ? v.toFixed(digits)
    : "--";

const hasNumber = (value) =>
  typeof value === "number" && Number.isFinite(value);

const isTrue = (value) =>
  value === 1 || value === true;

const formatCm = (value) =>
  hasNumber(value)
    ? `${(value * 100).toFixed(2)} cm`
    : "--";

const formatDegrees = (value) =>
  hasNumber(value)
    ? `${value.toFixed(1)}°`
    : "--";

const setText = (id, value) => {
  const element = $(id);

  if (element) {
    element.textContent = value;
  }
};

const feedback = (message) => {
  $("feedback").textContent = message;
};

const slider = $("speed-slider");

slider.addEventListener("input", () => {
  $("speed-value").textContent =
    Number(slider.value).toFixed(2);
});

// --------------------------------------------------
// START / STOP command confirmation
// --------------------------------------------------

function checkCommandConfirmation(telemetry, receivedAt) {
  if (!pendingCommand) return;

  const pending = pendingCommand;

  // Do not confirm using telemetry received before
  // the command was sent.
  if (
    typeof receivedAt !== "number" ||
    receivedAt <= pending.sentAt
  ) {
    return;
  }

  // START is confirmed by a running mission state.
  if (
    pending.type === "START" &&
    (telemetry.mission === 1 || telemetry.mission === 2)
  ) {
    feedback("Robot START confirmed. Mission is running.");
    pendingCommand = null;
    return;
  }

  // STOP is confirmed by the robot reporting IDLE.
  if (
    pending.type === "STOP" &&
    telemetry.mission === 0
  ) {
    feedback("Robot STOP confirmed. Mission is idle.");
    pendingCommand = null;
  }
}

function checkCommandTimeout() {
  if (!pendingCommand) return;

  if (
    Date.now() - pendingCommand.startedAt >=
    COMMAND_TIMEOUT_MS
  ) {
    const type = pendingCommand.type;

    pendingCommand = null;

    feedback(
      `${type} command sent, but robot confirmation timed out. ` +
      "Check MQTT reception and robot telemetry."
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

    // Do not overwrite feedback from a newer request.
    if (sequence !== commandSequence) return;

    if (payload === "START" || payload === "STOP") {
      pendingCommand = {
        type: payload,
        sentAt: Date.now() / 1000,
        startedAt: Date.now(),
      };

      feedback(
        `${payload} command sent. Waiting for robot confirmation...`
      );
    } else {
      feedback(
        `Published ${payload} to ${result.topic}.`
      );
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
    clearRampTelemetry(
      t ? "Telemetry stale" : "Waiting for telemetry"
    );
    return;
  }

  // IMU motion classification from robot_types.h.
  // This is different from the controller's
  // smoothed ramp_state.

  const sensorValid = isTrue(t.imu_valid);
  const tiltValid = isTrue(t.tilt_valid);

  let motionLabel = "--";

  if (hasNumber(t.motion)) {
    motionLabel =
      motionNames[t.motion] ?? "Unknown";

    if (
      t.imu_valid !== undefined &&
      !sensorValid
    ) {
      motionLabel = "IMU unavailable";
    }
  }

  setText("ramp-direction", motionLabel);

  // Pitch angle
  setText(
    "ramp-pitch",
    tiltValid
      ? formatDegrees(t.pitch_deg)
      : "--"
  );

  // Terrain height estimates
  setText("ramp-rise", formatCm(t.rise_m));

  setText(
    "ramp-peak",
    formatCm(t.terrain_peak_m)
  );

  setText(
    "ramp-height",
    formatCm(t.hump_m)
  );

  // Height measurement state
  const active = isTrue(t.hump_active);
  const valid = isTrue(t.hump_valid);

  const tracking =
    isTrue(t.height_tracking_valid);

  const calibrated =
    isTrue(t.imu_calibrated);

  const approximate =
    isTrue(t.height_approx);

  const gapPending =
    isTrue(t.height_gap_pending);

  setText(
    "ramp-validity",
    t.hump_valid === undefined
      ? "Unknown"
      : valid
        ? "Valid"
        : active
          ? "Measurement in progress"
          : "Not validated"
  );

  setText(
    "ramp-tracking",
    t.height_tracking_valid === undefined
      ? "Unknown"
      : tracking
        ? "Tracking valid"
        : "Not tracking"
  );

  setText(
    "ramp-calibration",
    t.imu_calibrated === undefined
      ? "Unknown"
      : calibrated
        ? "Calibrated"
        : "Not calibrated"
  );

  setText(
    "ramp-count",
    hasNumber(t.hump_count)
      ? String(t.hump_count)
      : "--"
  );

  // Measurement quality indicators
  setText(
    "ramp-quality",
    t.height_gap_pending === undefined ||
    t.height_approx === undefined
      ? "Unknown"
      : gapPending
        ? "Timing gap pending"
        : approximate
          ? "Approximate estimate"
          : "No quality warning"
  );

  // Overall ramp status
  if (
    t.imu_valid !== undefined &&
    !sensorValid
  ) {
    setText("ramp-status", "IMU unavailable");
  } else if (active) {
    setText("ramp-status", "Hump crossing active");
  } else if (valid) {
    setText(
      "ramp-status",
      "Height measurement available"
    );
  } else if (hasNumber(t.motion)) {
    setText("ramp-status", "Monitoring terrain");
  } else {
    setText(
      "ramp-status",
      "Waiting for ramp telemetry"
    );
  }
}

// --------------------------------------------------
// Poll robot telemetry
// --------------------------------------------------

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

    // Flask server.py reports received_at as a
    // Unix timestamp in seconds.
    const receivedAt =
      typeof data.received_at === "number"
        ? data.received_at
        : null;

    const ageMs =
      receivedAt !== null
        ? Date.now() - receivedAt * 1000
        : Infinity;

    const recent =
      brokerConnected &&
      ageMs >= 0 &&
      ageMs < TELEMETRY_FRESH_MS;

    // Connection status
    $("connection").textContent = brokerConnected
      ? recent
        ? "Live telemetry"
        : "Broker connected · waiting"
      : "MQTT disconnected";

    $("connection").className =
      `status ${brokerConnected ? "online" : "offline"}`;

    // Enable or disable controls
    $("send-speed").disabled = !brokerConnected;
    $("start-robot").disabled = !brokerConnected;
    $("stop-robot").disabled = !brokerConnected;

    $("route-form").querySelector("button").disabled =
      !brokerConnected;

    const t = data.latest;

    if (!t) {
      clearRampTelemetry();
      checkCommandTimeout();
      return;
    }

    // --------------------------------------------------
    // Existing robot telemetry
    // --------------------------------------------------

    $("current-speed").textContent =
      number(t.speed, 3);

    $("distance").textContent =
      number(t.obstacle_cm, 1);

    const mission =
      missions[t.mission] || "Unknown";

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

    $("obstacle-detail").textContent =
      t.mission === 2
        ? "Exact avoidance step not yet transmitted"
        : "Based on mission state";

    $("timestamp").textContent =
      typeof t.t_ms === "number"
        ? `Robot uptime: ${(t.t_ms / 1000).toFixed(1)} s`
        : "No uptime";

    $("line").textContent =
      number(t.line, 3);

    $("travelled").textContent =
      `${number(t.distance, 2)} m`;

    // Firmware sends the decoded character as its ASCII code.
    $("barcode").textContent =
      hasNumber(t.barcode) && t.barcode >= 33 && t.barcode < 127
        ? String.fromCharCode(t.barcode)
        : "--";

    $("fault").textContent =
      t.fault ?? "--";

    // --------------------------------------------------
    // New Ramp Detection & Height telemetry
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

    $("connection").textContent =
      "Server unavailable";

    $("connection").className =
      "status offline";

    $("send-speed").disabled = true;
    $("start-robot").disabled = true;
    $("stop-robot").disabled = true;

    $("route-form").querySelector("button").disabled =
      true;

    clearRampTelemetry("Server unavailable");

    checkCommandTimeout();
  }
}

// --------------------------------------------------
// Start polling
// --------------------------------------------------

// Clear server-side cached telemetry so a refresh starts from a clean state.
fetch("/api/reset", { method: "POST" })
  .catch(() => {})
  .finally(() => {
    poll();
    setInterval(poll, 1000);
  });
