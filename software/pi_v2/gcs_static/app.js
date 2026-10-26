const enableOutput = document.getElementById("enableOutput");
const throttle = document.getElementById("throttle");
const steering = document.getElementById("steering");
const leftPwmValue = document.getElementById("leftPwmValue");
const rightPwmValue = document.getElementById("rightPwmValue");
const leftActualValue = document.getElementById("leftActualValue");
const rightActualValue = document.getElementById("rightActualValue");
const commandFreshValue = document.getElementById("commandFreshValue");
const controlSourceValue = document.getElementById("controlSourceValue");
const rcThrottleValue = document.getElementById("rcThrottleValue");
const rcSteeringValue = document.getElementById("rcSteeringValue");
const rcModeValue = document.getElementById("rcModeValue");
const rcLinkValue = document.getElementById("rcLinkValue");
const stopButton = document.getElementById("stopButton");
const linkStatus = document.getElementById("linkStatus");
const mainStatus = document.getElementById("mainStatus");
const leftStatus = document.getElementById("leftStatus");
const rightStatus = document.getElementById("rightStatus");
const gpsStatus = document.getElementById("gpsStatus");
const imuStatus = document.getElementById("imuStatus");
const magStatus = document.getElementById("magStatus");
const batteryStatus = document.getElementById("batteryStatus");
const rawLog = document.getElementById("rawLog");
const attitudeCanvas = document.getElementById("attitudeCanvas");
const compassCanvas = document.getElementById("compassCanvas");
const trendCanvas = document.getElementById("trendCanvas");
const rollValue = document.getElementById("rollValue");
const pitchValue = document.getElementById("pitchValue");
const headingValue = document.getElementById("headingValue");
const headingSource = document.getElementById("headingSource");
const mainBatteryVoltage = document.getElementById("mainBatteryVoltage");
const mainBatteryBar = document.getElementById("mainBatteryBar");
const leftBatteryVoltage = document.getElementById("leftBatteryVoltage");
const rightBatteryVoltage = document.getElementById("rightBatteryVoltage");
const leftBatteryBar = document.getElementById("leftBatteryBar");
const rightBatteryBar = document.getElementById("rightBatteryBar");
const mapLink = document.getElementById("mapLink");
const mapPosition = document.getElementById("mapPosition");
const mapFix = document.getElementById("mapFix");
const waypointLat = document.getElementById("waypointLat");
const waypointLon = document.getElementById("waypointLon");
const addWaypoint = document.getElementById("addWaypoint");
const addCurrentPosition = document.getElementById("addCurrentPosition");
const applyMission = document.getElementById("applyMission");
const revertMission = document.getElementById("revertMission");
const clearMission = document.getElementById("clearMission");
const missionCount = document.getElementById("missionCount");
const missionFeedback = document.getElementById("missionFeedback");
const waypointList = document.getElementById("waypointList");
const startMission = document.getElementById("startMission");
const pauseMission = document.getElementById("pauseMission");
const stopMission = document.getElementById("stopMission");
const missionRuntimeStatus = document.getElementById("missionRuntimeStatus");
const missionRuntimeNavigation = document.getElementById("missionRuntimeNavigation");
const missionRuntimeTarget = document.getElementById("missionRuntimeTarget");
const missionRuntimeDistance = document.getElementById("missionRuntimeDistance");
const missionRuntimeReason = document.getElementById("missionRuntimeReason");
const cruiseThrottle = document.getElementById("cruiseThrottle");
const applyCruiseThrottle = document.getElementById("applyCruiseThrottle");
const testLogStatus = document.getElementById("testLogStatus");
const testLogSamples = document.getElementById("testLogSamples");
const testLogFile = document.getElementById("testLogFile");
const startTestLog = document.getElementById("startTestLog");
const stopTestLog = document.getElementById("stopTestLog");
const downloadTestCsv = document.getElementById("downloadTestCsv");
const downloadTestSummary = document.getElementById("downloadTestSummary");
const trendToggles = Array.from(document.querySelectorAll(".trend-toggle"));

const history = {
  maxPoints: 240,
  heading: [],
  speed: [],
  battery: [],
  roll: [],
  pitch: [],
};

const trendSeries = {
  battery: { label: "Battery %", min: 0, max: 100, color: "#2a9d8f", unit: "%" },
  speed: { label: "Speed", min: 0, max: 2, color: "#8ecae6", unit: "m/s" },
  heading: { label: "Heading", min: 0, max: 360, color: "#f4a261", unit: "deg" },
  roll: { label: "Roll", min: -45, max: 45, color: "#e76f51", unit: "deg" },
  pitch: { label: "Pitch", min: -45, max: 45, color: "#cdb4db", unit: "deg" },
};

let activeTrends = new Set(["battery", "speed", "heading"]);
let latestGps = {};
let missionWaypoints = [];
let activeMissionWaypoints = [];
let map;
let vesselMarker;
let waypointLayer;
let routeLine;
let activeRouteLine;
let mapCenteredOnVessel = false;

function initializeMap() {
  if (!window.L) {
    missionFeedback.textContent = "Map library could not be loaded.";
    return;
  }

  map = L.map("map", { zoomControl: true }).setView([37.5665, 126.9780], 14);
  L.tileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png", {
    maxZoom: 19,
    attribution: "© OpenStreetMap contributors",
  }).addTo(map);
  waypointLayer = L.layerGroup().addTo(map);
  routeLine = L.polyline([], { color: "#f4a261", weight: 3, opacity: 0.9 }).addTo(map);
  activeRouteLine = L.polyline([], { color: "#8ecae6", weight: 3, dashArray: "8 6", opacity: 0.9 }).addTo(map);

  map.on("click", (event) => {
    const { lat, lng } = event.latlng;
    waypointLat.value = lat.toFixed(7);
    waypointLon.value = lng.toFixed(7);
    updateMission({ action: "add", lat, lon: lng });
  });
}

function unitValue(input) {
  return Number(input.value) / 100;
}

function formatPwm(value) {
  const number = numeric(value);
  return number === null ? "-" : `${number.toFixed(0)} us`;
}

function updateOutputMonitor(telemetry) {
  const main = telemetry.main || {};
  const left = telemetry.left || {};
  const right = telemetry.right || {};

  leftPwmValue.textContent = formatPwm(main.left_pwm_us);
  rightPwmValue.textContent = formatPwm(main.right_pwm_us);
  leftActualValue.textContent = formatPwm(left.actual_pwm_us);
  rightActualValue.textContent = formatPwm(right.actual_pwm_us);
  commandFreshValue.textContent = main.pi_command_fresh === true
    ? "fresh"
    : main.pi_command_fresh === false
      ? "stale"
      : "-";
  controlSourceValue.textContent = main.control_source || "-";
  rcThrottleValue.textContent = formatRc(main.rc_throttle, main.ch2_us);
  rcSteeringValue.textContent = formatRc(main.rc_steering, main.ch1_us);
  rcModeValue.textContent = formatRc(null, main.ch4_us);
  rcLinkValue.textContent = main.rc_ok === true ? "connected" : main.rc_ok === false ? "lost" : "-";
}

async function sendCommand() {
  const payload = {
    enabled: enableOutput.checked,
    throttle: unitValue(throttle),
    steering: unitValue(steering),
  };

  await fetch("/api/command", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(payload),
  });
}

async function stopOutput() {
  enableOutput.checked = false;
  throttle.value = 0;
  steering.value = 0;
  await fetch("/api/stop", { method: "POST" });
}

function renderDl(node, data) {
  node.textContent = "";
  const entries = Object.entries(formatValues(data || {}));
  if (entries.length === 0) {
    const dt = document.createElement("dt");
    const dd = document.createElement("dd");
    dt.textContent = "status";
    dd.textContent = "waiting";
    node.append(dt, dd);
    return;
  }

  for (const [key, value] of entries) {
    const dt = document.createElement("dt");
    const dd = document.createElement("dd");
    dt.textContent = key;
    dd.textContent = value;
    node.append(dt, dd);
  }
}

function formatValues(data) {
  const formatted = {};
  for (const [key, value] of Object.entries(data || {})) {
    if (typeof value === "number") {
      formatted[key] = Number.isInteger(value) ? value : value.toFixed(3);
    } else if (value === null) {
      formatted[key] = "-";
    } else {
      formatted[key] = value;
    }
  }
  return formatted;
}

function numeric(value) {
  return typeof value === "number" && Number.isFinite(value) ? value : null;
}

function formatUnit(value) {
  const number = numeric(value);
  return number === null ? "-" : number.toFixed(2);
}

function formatRc(unit, pulseUs) {
  const unitValue = numeric(unit);
  const pulse = numeric(pulseUs);
  if (unitValue === null && pulse === null) return "-";
  if (unitValue === null) return `${pulse.toFixed(0)} us`;
  if (pulse === null) return unitValue.toFixed(2);
  return `${unitValue.toFixed(2)} / ${pulse.toFixed(0)} us`;
}

function pushHistory(list, value) {
  const number = numeric(value);
  if (number === null) return;
  list.push({ time: Date.now(), value: number });
  if (list.length > history.maxPoints) {
    list.splice(0, list.length - history.maxPoints);
  }
}

function drawCompass(headingDeg) {
  const ctx = compassCanvas.getContext("2d");
  const width = compassCanvas.width;
  const height = compassCanvas.height;
  const cx = width / 2;
  const cy = height / 2;
  const radius = Math.min(width, height) * 0.34;
  const heading = numeric(headingDeg);

  ctx.clearRect(0, 0, width, height);
  ctx.lineWidth = 2;
  ctx.strokeStyle = "#3b4a51";
  ctx.fillStyle = "#121719";
  ctx.beginPath();
  ctx.arc(cx, cy, radius, 0, Math.PI * 2);
  ctx.fill();
  ctx.stroke();

  const labels = [
    ["N", 0],
    ["E", 90],
    ["S", 180],
    ["W", 270],
  ];

  ctx.font = "700 18px Arial";
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  for (const [label, deg] of labels) {
    const rad = (deg - 90) * Math.PI / 180;
    const x = cx + Math.cos(rad) * (radius + 28);
    const y = cy + Math.sin(rad) * (radius + 28);
    ctx.fillStyle = label === "N" ? "#e5383b" : "#c9d6dc";
    ctx.fillText(label, x, y);
  }

  ctx.strokeStyle = "#53656d";
  for (let deg = 0; deg < 360; deg += 30) {
    const rad = (deg - 90) * Math.PI / 180;
    const outer = radius;
    const inner = deg % 90 === 0 ? radius - 14 : radius - 8;
    ctx.beginPath();
    ctx.moveTo(cx + Math.cos(rad) * inner, cy + Math.sin(rad) * inner);
    ctx.lineTo(cx + Math.cos(rad) * outer, cy + Math.sin(rad) * outer);
    ctx.stroke();
  }

  if (heading === null) {
    headingValue.textContent = "-";
    headingSource.textContent = "Compass waiting";
    return;
  }

  const needle = (heading - 90) * Math.PI / 180;
  const tipX = cx + Math.cos(needle) * (radius - 14);
  const tipY = cy + Math.sin(needle) * (radius - 14);
  const tailX = cx - Math.cos(needle) * (radius - 42);
  const tailY = cy - Math.sin(needle) * (radius - 42);
  const leftX = cx + Math.cos(needle + Math.PI / 2) * 12;
  const leftY = cy + Math.sin(needle + Math.PI / 2) * 12;
  const rightX = cx + Math.cos(needle - Math.PI / 2) * 12;
  const rightY = cy + Math.sin(needle - Math.PI / 2) * 12;

  ctx.fillStyle = "#e5383b";
  ctx.beginPath();
  ctx.moveTo(tipX, tipY);
  ctx.lineTo(leftX, leftY);
  ctx.lineTo(rightX, rightY);
  ctx.closePath();
  ctx.fill();

  ctx.strokeStyle = "#8ecae6";
  ctx.lineWidth = 3;
  ctx.beginPath();
  ctx.moveTo(cx, cy);
  ctx.lineTo(tailX, tailY);
  ctx.stroke();

  ctx.fillStyle = "#edf2f4";
  ctx.beginPath();
  ctx.arc(cx, cy, 6, 0, Math.PI * 2);
  ctx.fill();

  headingValue.textContent = `${heading.toFixed(1)}°`;
  headingSource.textContent = "Compass heading";
}

function drawAttitude(imu) {
  const ctx = attitudeCanvas.getContext("2d");
  const width = attitudeCanvas.width;
  const height = attitudeCanvas.height;
  const roll = numeric(imu?.roll_deg);
  const pitch = numeric(imu?.pitch_deg);
  const cx = width / 2;
  const cy = height / 2;

  ctx.clearRect(0, 0, width, height);
  ctx.fillStyle = "#121719";
  ctx.fillRect(0, 0, width, height);

  if (roll === null || pitch === null) {
    ctx.fillStyle = "#9fb1bc";
    ctx.font = "16px Arial";
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.fillText("IMU waiting", cx, cy);
    rollValue.textContent = "-";
    pitchValue.textContent = "-";
    return;
  }

  const clampedPitch = Math.max(-45, Math.min(45, pitch));
  const pitchOffset = clampedPitch * 2.2;
  const rollRad = roll * Math.PI / 180;

  ctx.save();
  ctx.translate(cx, cy);
  ctx.rotate(-rollRad);

  ctx.fillStyle = "#264653";
  ctx.fillRect(-width, -height * 2 + pitchOffset, width * 2, height * 2);
  ctx.fillStyle = "#7f5539";
  ctx.fillRect(-width, pitchOffset, width * 2, height * 2);

  ctx.strokeStyle = "#edf2f4";
  ctx.lineWidth = 3;
  ctx.beginPath();
  ctx.moveTo(-width, pitchOffset);
  ctx.lineTo(width, pitchOffset);
  ctx.stroke();

  ctx.strokeStyle = "rgba(237, 242, 244, 0.45)";
  ctx.lineWidth = 1;
  for (let deg = -30; deg <= 30; deg += 10) {
    if (deg === 0) continue;
    const y = pitchOffset + deg * 2.2;
    ctx.beginPath();
    ctx.moveTo(-42, y);
    ctx.lineTo(42, y);
    ctx.stroke();
  }
  ctx.restore();

  ctx.strokeStyle = "#f4a261";
  ctx.lineWidth = 4;
  ctx.beginPath();
  ctx.moveTo(cx - 58, cy);
  ctx.lineTo(cx - 16, cy);
  ctx.moveTo(cx + 16, cy);
  ctx.lineTo(cx + 58, cy);
  ctx.stroke();

  ctx.fillStyle = "#f4a261";
  ctx.beginPath();
  ctx.arc(cx, cy, 5, 0, Math.PI * 2);
  ctx.fill();

  ctx.strokeStyle = "#3b4a51";
  ctx.lineWidth = 2;
  ctx.strokeRect(1, 1, width - 2, height - 2);

  rollValue.textContent = `${roll.toFixed(1)} deg`;
  pitchValue.textContent = `${pitch.toFixed(1)} deg`;
}

function mainBatteryData(telemetry) {
  const main = telemetry?.main || {};
  const voltage = numeric(main.vbat_v);
  const valid = main.battery_valid !== false && voltage !== null && voltage > 0;
  if (!valid) return { ok: false, source: "Main ESP32-S3 ADC", voltage_v: null, soc_percent: null };
  return {
    ok: true,
    source: "Main ESP32-S3 ADC",
    voltage_v: voltage,
    cell_v: voltage / 2,
    soc_percent: Math.max(0, Math.min(100, estimateLipoSoc(voltage / 2))),
  };
}

function updateBatteryVisual(telemetry) {
  const battery = mainBatteryData(telemetry);
  const percent = numeric(battery.soc_percent);
  const voltage = numeric(battery.voltage_v);
  if (percent === null || voltage === null) {
    mainBatteryVoltage.textContent = "Battery waiting";
    mainBatteryBar.style.width = "0%";
  } else {
    const clamped = Math.max(0, Math.min(100, percent));
    mainBatteryVoltage.textContent = voltage === null
      ? `${clamped.toFixed(0)}% / voltage waiting`
      : `${voltage.toFixed(2)} V / ${clamped.toFixed(0)}%`;
    mainBatteryBar.style.width = `${clamped}%`;
    mainBatteryBar.style.background = clamped < 25 ? "#e5383b" : clamped < 45 ? "#f4a261" : "#2a9d8f";
  }

  updateSubBatteryVisual(leftBatteryVoltage, leftBatteryBar, telemetry?.left);
  updateSubBatteryVisual(rightBatteryVoltage, rightBatteryBar, telemetry?.right);
}

function updateSubBatteryVisual(node, bar, status) {
  const voltage = numeric(status?.vbat_v);
  const faultFlags = numeric(status?.fault_flags) || 0;
  const adcFault = (faultFlags & 4) !== 0;
  if (voltage === null || voltage <= 0 || adcFault) {
    node.textContent = "Voltage unavailable";
    bar.style.width = "0%";
    return;
  }
  const cellVoltage = voltage / 4;
  const percent = Math.max(0, Math.min(100, estimateLipoSoc(cellVoltage)));
  node.textContent = `${voltage.toFixed(2)} V / ${percent.toFixed(0)}% est.`;
  bar.style.width = `${percent}%`;
  bar.style.background = percent < 25 ? "#e5383b" : percent < 45 ? "#f4a261" : "#2a9d8f";
}

function estimateLipoSoc(cellVoltage) {
  const table = [
    [4.20, 100], [4.10, 90], [4.00, 80], [3.92, 70], [3.85, 60],
    [3.79, 50], [3.75, 40], [3.70, 30], [3.60, 20], [3.50, 10], [3.30, 0],
  ];
  if (cellVoltage >= table[0][0]) return 100;
  if (cellVoltage <= table[table.length - 1][0]) return 0;
  for (let index = 0; index < table.length - 1; index += 1) {
    const [highV, highSoc] = table[index];
    const [lowV, lowSoc] = table[index + 1];
    if (lowV <= cellVoltage && cellVoltage <= highV) {
      return lowSoc + ((cellVoltage - lowV) / (highV - lowV)) * (highSoc - lowSoc);
    }
  }
  return 0;
}

function updateMap(gps) {
  const lat = numeric(gps?.lat);
  const lon = numeric(gps?.lon);
  const sats = numeric(gps?.satellites);
  const hdop = numeric(gps?.hdop);
  const fix = gps?.fix_quality;

  if (lat === null || lon === null) {
    mapPosition.textContent = "GPS waiting";
    mapFix.textContent = "-";
    return;
  }

  const position = [lat, lon];
  if (map) {
    if (!vesselMarker) {
      vesselMarker = L.circleMarker(position, {
        radius: 8, color: "#edf2f4", weight: 2, fillColor: "#2a9d8f", fillOpacity: 1,
      }).bindTooltip("Vessel", { direction: "top" }).addTo(map);
    } else {
      vesselMarker.setLatLng(position);
    }
    if (!mapCenteredOnVessel) {
      map.setView(position, 17);
      mapCenteredOnVessel = true;
    }
  }

  mapLink.href = `https://www.openstreetmap.org/?mlat=${lat.toFixed(6)}&mlon=${lon.toFixed(6)}#map=17/${lat.toFixed(6)}/${lon.toFixed(6)}`;

  mapPosition.textContent = `${lat.toFixed(6)}, ${lon.toFixed(6)}`;
  mapFix.textContent = `fix ${fix ?? "-"} / sats ${sats === null ? "-" : sats.toFixed(0)} / hdop ${hdop === null ? "-" : hdop.toFixed(1)}`;
}

function missionPositions(waypoints) {
  return waypoints.flatMap((waypoint) => {
    const lat = numeric(waypoint.lat);
    const lon = numeric(waypoint.lon);
    return lat === null || lon === null ? [] : [[lat, lon]];
  });
}

function renderMissionMap() {
  if (!map || !waypointLayer || !routeLine || !activeRouteLine) return;
  waypointLayer.clearLayers();
  const positions = [];
  for (const [index, waypoint] of missionWaypoints.entries()) {
    const lat = numeric(waypoint.lat);
    const lon = numeric(waypoint.lon);
    if (lat === null || lon === null) continue;
    const position = [lat, lon];
    positions.push(position);
    L.circleMarker(position, {
      radius: 8, color: "#edf2f4", weight: 2, fillColor: "#e76f51", fillOpacity: 1,
    }).bindTooltip(`Waypoint ${index + 1}`, { direction: "top" }).addTo(waypointLayer);
  }
  routeLine.setLatLngs(positions);
  activeRouteLine.setLatLngs(missionPositions(activeMissionWaypoints));
}

function drawTrend() {
  const ctx = trendCanvas.getContext("2d");
  const width = trendCanvas.width;
  const height = trendCanvas.height;
  const padLeft = 52;
  const padRight = 18;
  const padTop = 20;
  const padBottom = 36;
  const plotW = width - padLeft - padRight;
  const plotH = height - padTop - padBottom;

  ctx.clearRect(0, 0, width, height);
  ctx.fillStyle = "#121719";
  ctx.fillRect(0, 0, width, height);
  ctx.strokeStyle = "#2e3a40";
  ctx.lineWidth = 1;

  const selected = Object.entries(trendSeries)
    .filter(([key]) => activeTrends.has(key));

  if (selected.length === 0) {
    ctx.fillStyle = "#9fb1bc";
    ctx.font = "16px Arial";
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.fillText("Select trend values", width / 2, height / 2);
    return;
  }

  let minY = Math.min(...selected.map(([, item]) => item.min));
  let maxY = Math.max(...selected.map(([, item]) => item.max));
  if (selected.length === 1) {
    minY = selected[0][1].min;
    maxY = selected[0][1].max;
  }
  const ySpan = maxY - minY || 1;

  ctx.font = "12px Arial";
  ctx.textAlign = "right";
  ctx.textBaseline = "middle";
  for (let i = 0; i <= 4; i++) {
    const value = maxY - (ySpan / 4) * i;
    const y = padTop + (plotH / 4) * i;
    ctx.beginPath();
    ctx.moveTo(padLeft, y);
    ctx.lineTo(width - padRight, y);
    ctx.stroke();
    ctx.fillStyle = "#9fb1bc";
    ctx.fillText(formatAxisValue(value), padLeft - 8, y);
  }

  ctx.strokeStyle = "#3b4a51";
  ctx.beginPath();
  ctx.moveTo(padLeft, padTop);
  ctx.lineTo(padLeft, padTop + plotH);
  ctx.lineTo(width - padRight, padTop + plotH);
  ctx.stroke();

  for (const [key, item] of selected) {
    const data = history[key] || [];
    if (data.length < 2) continue;
    ctx.strokeStyle = item.color;
    ctx.lineWidth = 2;
    ctx.beginPath();
    data.forEach((point, index) => {
      const value = Math.max(minY, Math.min(maxY, point.value));
      const x = padLeft + (index / (history.maxPoints - 1)) * plotW;
      const y = padTop + plotH - ((value - minY) / ySpan) * plotH;
      if (index === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    });
    ctx.stroke();
  }

  ctx.font = "12px Arial";
  ctx.textBaseline = "top";
  ctx.textAlign = "left";
  let x = padLeft;
  for (const [, item] of selected) {
    ctx.fillStyle = item.color;
    ctx.fillRect(x, height - 22, 10, 10);
    ctx.fillStyle = "#c9d6dc";
    ctx.fillText(`${item.label} ${item.unit}`, x + 16, height - 24);
    x += item.label.length * 8 + item.unit.length * 7 + 54;
  }
}

function formatAxisValue(value) {
  if (Math.abs(value) >= 100) return value.toFixed(0);
  if (Math.abs(value) >= 10) return value.toFixed(1);
  return value.toFixed(2);
}

function updateVisuals(sensors, telemetry) {
  const gps = sensors.gps || {};
  const imu = sensors.imu || {};
  const mag = sensors.mag || {};
  const battery = mainBatteryData(telemetry);
  const heading = numeric(mag.heading_deg);
  latestGps = gps;

  pushHistory(history.heading, heading);
  pushHistory(history.speed, gps.speed_mps);
  pushHistory(history.battery, battery.soc_percent);
  pushHistory(history.roll, imu.roll_deg);
  pushHistory(history.pitch, imu.pitch_deg);

  drawAttitude(imu);
  drawCompass(heading);
  updateBatteryVisual(telemetry);
  updateMap(gps);
  drawTrend();
}

function renderMission(mission) {
  const waypoints = Array.isArray(mission?.waypoints) ? mission.waypoints : [];
  missionWaypoints = waypoints;
  activeMissionWaypoints = Array.isArray(mission?.active_waypoints) ? mission.active_waypoints : [];
  missionCount.textContent = `${waypoints.length} waypoint${waypoints.length === 1 ? "" : "s"}`;
  const pending = mission?.pending_changes === true;
  missionFeedback.textContent = pending
    ? `Draft changed — apply to update the Pi mission file (revision ${mission?.revision ?? 0}).`
    : `Pi mission file applied (revision ${mission?.revision ?? 0}). Select Start to run it.`;
  waypointList.textContent = "";
  for (const [index, waypoint] of waypoints.entries()) {
    const item = document.createElement("li");
    const position = document.createElement("span");
    position.textContent = `${Number(waypoint.lat).toFixed(7)}, ${Number(waypoint.lon).toFixed(7)}`;
    const remove = document.createElement("button");
    remove.type = "button";
    remove.className = "secondary";
    remove.textContent = "Remove";
    remove.addEventListener("click", () => updateMission({ action: "remove", index }));
    item.append(position, remove);
    waypointList.append(item);
  }
  renderMissionMap();
}

async function updateMission(payload) {
  try {
    const response = await fetch("/api/mission", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(payload),
    });
    const result = await response.json();
    if (!response.ok || !result.ok) throw new Error(result.error || "Mission update failed.");
    renderMission(result.mission);
  } catch (error) {
    missionFeedback.textContent = `Mission plan error: ${error.message}`;
  }
}

function renderMissionRuntime(runtime) {
  const state = runtime || {};
  missionRuntimeStatus.textContent = state.status || "IDLE";
  missionRuntimeNavigation.textContent = state.navigation_state || "IDLE";
  const index = numeric(state.waypoint_index);
  const total = numeric(state.waypoint_total);
  missionRuntimeTarget.textContent = state.status !== "RUNNING" && state.status !== "PAUSED"
    ? "-"
    : index === null || total === null || total === 0 ? "-" : `${index + 1} / ${total}`;
  const distance = numeric(state.distance_m);
  missionRuntimeDistance.textContent = distance === null ? "-" : `${distance.toFixed(1)} m`;
  missionRuntimeReason.textContent = state.reason || "-";
  const cruise = numeric(state.cruise_throttle);
  if (cruise !== null && document.activeElement !== cruiseThrottle) {
    cruiseThrottle.value = cruise.toFixed(2);
  }
}

async function applyCruiseOutput() {
  try {
    const response = await fetch("/api/mission/tuning", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ cruise_throttle: Number(cruiseThrottle.value) }),
    });
    const result = await response.json();
    if (!response.ok || !result.ok) throw new Error(result.error || "Cruise output update failed.");
    renderMissionRuntime(result.mission_runtime);
  } catch (error) {
    missionFeedback.textContent = `Cruise output error: ${error.message}`;
  }
}

function renderTestLog(testLog) {
  const state = testLog || {};
  const active = state.active === true;
  testLogStatus.textContent = active ? "Recording · 5 Hz" : "Log idle";
  testLogSamples.textContent = numeric(state.sample_count) === null ? "0" : Number(state.sample_count).toFixed(0);
  testLogFile.textContent = state.csv_file || "-";
  startTestLog.disabled = active;
  stopTestLog.disabled = !active;
  const hasCsv = Boolean(state.csv_file);
  const hasSummary = Boolean(state.summary_file) && !active;
  downloadTestCsv.style.display = hasCsv ? "inline-flex" : "none";
  downloadTestSummary.style.display = hasSummary ? "inline-flex" : "none";
}

async function controlTestLog(action) {
  try {
    const response = await fetch("/api/test-log", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ action }),
    });
    const result = await response.json();
    if (!response.ok || !result.ok) throw new Error(result.error || "Test log command failed.");
    renderTestLog(result.test_log);
  } catch (error) {
    missionFeedback.textContent = `Test log error: ${error.message}`;
  }
}

async function controlMission(action) {
  try {
    const response = await fetch("/api/mission/control", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ action }),
    });
    const result = await response.json();
    if (!response.ok || !result.ok) throw new Error(result.error || "Mission control failed.");
    renderMissionRuntime(result.mission_runtime);
  } catch (error) {
    missionFeedback.textContent = `Mission control error: ${error.message}`;
  }
}

async function refreshStatus() {
  try {
    const response = await fetch("/api/status");
    const data = await response.json();
    const gcs = data.gcs || {};
    const telemetry = data.telemetry || {};
    const sensors = data.sensors || {};

    linkStatus.textContent = gcs.demo
      ? "DEMO — Simulated data / No hardware output"
      : gcs.esp_connected
      ? `Main connected: ${gcs.esp_port}`
      : `Main disconnected: ${gcs.esp_error || "waiting"}`;

    renderDl(mainStatus, telemetry.main);
    renderDl(leftStatus, telemetry.left);
    renderDl(rightStatus, telemetry.right);
    renderDl(gpsStatus, sensors.gps);
    renderDl(imuStatus, sensors.imu);
    renderDl(magStatus, sensors.mag);
    renderDl(batteryStatus, mainBatteryData(telemetry));
    updateOutputMonitor(telemetry);
    updateVisuals(sensors, telemetry);
    renderMission(data.mission);
    renderMissionRuntime(data.mission_runtime);
    renderTestLog(data.test_log);
    rawLog.textContent = (telemetry.raw || []).join("\n");
  } catch (error) {
    linkStatus.textContent = `GCS server disconnected: ${error}`;
  }
}

for (const input of [enableOutput, throttle, steering]) {
  input.addEventListener("input", () => {
    sendCommand().catch(() => {});
  });
}

stopButton.addEventListener("click", () => {
  stopOutput().catch(() => {});
});

addWaypoint.addEventListener("click", () => {
  updateMission({ action: "add", lat: waypointLat.value, lon: waypointLon.value });
});

addCurrentPosition.addEventListener("click", () => {
  if (numeric(latestGps.lat) === null || numeric(latestGps.lon) === null) {
    missionFeedback.textContent = "Current GPS position is unavailable.";
    return;
  }
  waypointLat.value = Number(latestGps.lat).toFixed(7);
  waypointLon.value = Number(latestGps.lon).toFixed(7);
  updateMission({ action: "add", lat: latestGps.lat, lon: latestGps.lon });
});

clearMission.addEventListener("click", () => updateMission({ action: "clear" }));
applyMission.addEventListener("click", () => updateMission({ action: "apply" }));
revertMission.addEventListener("click", () => updateMission({ action: "revert" }));
startMission.addEventListener("click", () => controlMission("start"));
pauseMission.addEventListener("click", () => controlMission("pause"));
stopMission.addEventListener("click", () => controlMission("stop"));
applyCruiseThrottle.addEventListener("click", applyCruiseOutput);
startTestLog.addEventListener("click", () => controlTestLog("start"));
stopTestLog.addEventListener("click", () => controlTestLog("stop"));

for (const button of trendToggles) {
  button.addEventListener("click", () => {
    const series = button.dataset.series;
    if (!series) return;
    if (activeTrends.has(series)) {
      activeTrends.delete(series);
      button.classList.remove("active");
    } else {
      activeTrends.add(series);
      button.classList.add("active");
    }
    drawTrend();
  });
}

initializeMap();

setInterval(() => {
  if (enableOutput.checked) {
    sendCommand().catch(() => {});
  }
}, 100);
setInterval(refreshStatus, 250);
refreshStatus();
