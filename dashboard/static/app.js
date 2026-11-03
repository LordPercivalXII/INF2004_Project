const $ = (id) => document.getElementById(id);
let brokerConnected = false;
const missions = ["Idle", "Following", "Avoiding", "Fault"];
const number = (v, digits = 2) =>
  typeof v === "number" && Number.isFinite(v) ? v.toFixed(digits) : "--";
const feedback = (message) => {
  $("feedback").textContent = message;
};
const slider = $("speed-slider");
slider.addEventListener("input", () => {
  $("speed-value").textContent = Number(slider.value).toFixed(2);
});
async function post(path, body = {}) {
  try {
    const response = await fetch(path, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body),
    });
    const result = await response.json();
    if (!response.ok) throw new Error(result.error || "Request failed");
    feedback(
      `Published ${result.payload} to ${result.topic}. Robot acknowledgment is not available.`,
    );
  } catch (error) {
    feedback(`Error: ${error.message}`);
  }
}
$("route-form").addEventListener("submit", (event) => {
  event.preventDefault();
  if (!brokerConnected) return feedback("MQTT broker is disconnected.");
  const form = new FormData(event.currentTarget);
  post("/api/route", Object.fromEntries(form.entries()));
});
$("send-speed").addEventListener("click", () => {
  if (!brokerConnected) return feedback("MQTT broker is disconnected.");
  post("/api/speed", { speed: Number(slider.value) });
});
$("start-robot").addEventListener("click", () => {
  if (!brokerConnected) return feedback("MQTT broker is disconnected.");
  post("/api/start");
});
$("stop-robot").addEventListener("click", () => {
  if (!brokerConnected) return feedback("MQTT broker is disconnected.");
  post("/api/stop");
});
async function poll() {
  try {
    const response = await fetch("/api/status", { cache: "no-store" });
    if (!response.ok) throw new Error("Server unavailable");
    const data = await response.json();
    brokerConnected = !!data.connected;
    const recent =
      data.received_at && Date.now() / 1000 - data.received_at < 10;
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
    $("route-form").querySelector("button").disabled = !brokerConnected;
    const t = data.latest;
    if (!t) return;
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
    $("travelled").textContent = `${number(t.distance, 2)} m`;
    $("barcode").textContent = t.barcode ?? "--";
    $("fault").textContent = t.fault ?? "--";
  } catch (error) {
    brokerConnected = false;
    $("connection").textContent = "Server unavailable";
    $("connection").className = "status offline";
    $("send-speed").disabled = true;
    $("start-robot").disabled = true;
    $("stop-robot").disabled = true;
    $("route-form").querySelector("button").disabled = true;
  }
}
poll();
setInterval(poll, 1000);
