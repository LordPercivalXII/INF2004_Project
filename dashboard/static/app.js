const $ = (id) => document.getElementById(id);

let brokerConnected = false;
let pendingCommand = null;
let commandSequence = 0;

const COMMAND_TIMEOUT_MS = 10000;
const missions = ["Idle", "Following", "Avoiding", "Fault"];

const number = (v, digits = 2) =>
  typeof v === "number" && Number.isFinite(v)
    ? v.toFixed(digits)
    : "--";

const feedback = (message) => {
  $("feedback").textContent = message;
};

const slider = $("speed-slider");

slider.addEventListener("input", () => {
  $("speed-value").textContent = Number(slider.value).toFixed(2);
});

function checkCommandConfirmation(telemetry, receivedAt) {
  if (!pendingCommand) return;

  const pending = pendingCommand;

  // Do not confirm using telemetry received before the command.
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
      "Check MQTT reception and robot telemetry."
    );
  }
}

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

// Route configuration
$("route-form").addEventListener("submit", (event) => {
  event.preventDefault();

  if (!brokerConnected) {
    return feedback("MQTT broker is disconnected.");
  }

  const form = new FormData(event.currentTarget);

  post("/api/route", Object.fromEntries(form.entries()));
});

// Speed control
$("send-speed").addEventListener("click", () => {
  if (!brokerConnected) {
    return feedback("MQTT broker is disconnected.");
  }

  post("/api/speed", {
    speed: Number(slider.value),
  });
});

// START robot
$("start-robot").addEventListener("click", () => {
  if (!brokerConnected) {
    return feedback("MQTT broker is disconnected.");
  }

  post("/api/start");
});

// STOP robot
$("stop-robot").addEventListener("click", () => {
  if (!brokerConnected) {
    return feedback("MQTT broker is disconnected.");
  }

  post("/api/stop");
});

// Poll robot telemetry
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

    const recent =
      typeof data.received_at === "number" &&
      Date.now() / 1000 - data.received_at < 10;

    $("connection").textContent = brokerConnected
      ? recent
        ? "Live telemetry"
        : "Broker connected · waiting"
      : "MQTT disconnected";

    $("connection").className =
      `status ${brokerConnected ? "online" : "offline"}`;

    $("send-speed").disabled = !brokerConnected;
    $("start-robot").disabled = !brokerConnected;
    $("stop-robot").disabled = !brokerConnected;

    $("route-form").querySelector("button").disabled =
      !brokerConnected;

    const t = data.latest;

    if (!t) {
      checkCommandTimeout();
      return;
    }

    // Display current telemetry
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

    $("obstacle-detail").textContent =
      t.mission === 2
        ? "Exact avoidance step not yet transmitted"
        : "Based on mission state";

    $("timestamp").textContent =
      typeof t.t_ms === "number"
        ? `Robot uptime: ${(t.t_ms / 1000).toFixed(1)} s`
        : "No uptime";

    $("line").textContent = number(t.line, 3);

    $("travelled").textContent =
      `${number(t.distance, 2)} m`;

    $("barcode").textContent = t.barcode ?? "--";
    $("fault").textContent = t.fault ?? "--";

    // Confirm START/STOP only using newer telemetry.
    if (recent) {
      checkCommandConfirmation(t, data.received_at);
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

    checkCommandTimeout();
  }
}

poll();
setInterval(poll, 1000);
