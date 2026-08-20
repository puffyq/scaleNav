"use strict";

const state = {
  sessions: [],
  session: null,
  frames: [],
  position: 0,
  payload: null,
  mode: "body",
  playing: false,
  timer: null,
  requestSerial: 0,
};

const elements = Object.fromEntries([
  "session-select", "session-meta", "frame-slider", "frame-readout", "first-frame",
  "previous-frame", "play-pause", "next-frame", "last-frame", "mode-body", "mode-world",
  "scene", "scene-status", "scene-subtitle", "raw-depth", "model-depth", "depth-range",
  "frame-time", "frame-metrics", "candidate-count", "candidate-table", "event-json", "error-banner",
].map((id) => [id, document.getElementById(id)]));

function showError(error) {
  elements["error-banner"].textContent = error instanceof Error ? error.message : String(error);
  elements["error-banner"].hidden = false;
  window.clearTimeout(showError.timeout);
  showError.timeout = window.setTimeout(() => { elements["error-banner"].hidden = true; }, 7000);
}

async function getJson(path) {
  const response = await fetch(path, { cache: "no-store" });
  const value = await response.json();
  if (!response.ok) throw new Error(value.error || `${response.status} ${response.statusText}`);
  return value;
}

function query(values) {
  return new URLSearchParams(values).toString();
}

function number(value, digits = 2) {
  return Number.isFinite(Number(value)) ? Number(value).toFixed(digits) : "-";
}

function vector(value, digits = 2) {
  return Array.isArray(value) ? `[${value.map((item) => number(item, digits)).join(", ")}]` : "-";
}

function bytes(value) {
  if (!Number.isFinite(value)) return "-";
  const units = ["B", "KB", "MB", "GB"];
  let amount = value;
  let unit = 0;
  while (amount >= 1024 && unit < units.length - 1) { amount /= 1024; unit += 1; }
  return `${amount.toFixed(unit === 0 ? 0 : 1)} ${units[unit]}`;
}

function add(a, b) { return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]; }
function subtract(a, b) { return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]; }
function scale(a, amount) { return [a[0] * amount, a[1] * amount, a[2] * amount]; }

function normalizeQuaternion(value) {
  const q = Array.isArray(value) && value.length === 4 ? value : [0, 0, 0, 1];
  const norm = Math.hypot(...q) || 1;
  return q.map((item) => item / norm);
}

function rotate(point, quaternion) {
  const [x, y, z, w] = normalizeQuaternion(quaternion);
  const [px, py, pz] = point;
  const tx = 2 * (y * pz - z * py);
  const ty = 2 * (z * px - x * pz);
  const tz = 2 * (x * py - y * px);
  return [
    px + w * tx + (y * tz - z * ty),
    py + w * ty + (z * tx - x * tz),
    pz + w * tz + (x * ty - y * tx),
  ];
}

function inverseRotate(point, quaternion) {
  const [x, y, z, w] = normalizeQuaternion(quaternion);
  return rotate(point, [-x, -y, -z, w]);
}

function xyz(points) {
  return {
    x: points.map((point) => point === null ? null : point[0]),
    y: points.map((point) => point === null ? null : point[1]),
    z: points.map((point) => point === null ? null : point[2]),
  };
}

function lineTrace(points, name, color, width = 4, dash = "solid") {
  return {
    type: "scatter3d", mode: "lines", name, ...xyz(points),
    line: { color, width, dash }, hoverinfo: "name+x+y+z",
  };
}

async function loadSessions() {
  const data = await getJson("/api/sessions");
  state.sessions = data.sessions;
  elements["session-select"].replaceChildren();
  for (const session of state.sessions) {
    const option = document.createElement("option");
    option.value = session.id;
    const count = session.frame_count == null ? "按需索引" : `${session.frame_count} frames`;
    option.textContent = `${session.id} · ${count}`;
    elements["session-select"].append(option);
  }
  if (!state.sessions.length) throw new Error("log_event 中没有 openseek_events_*.jsonl");
  await loadSession(state.sessions[0].id);
}

async function loadSession(id) {
  stopPlayback();
  elements["scene-status"].textContent = "索引日志";
  state.session = await getJson(`/api/session?${query({ id })}`);
  state.frames = state.session.frames || [];
  state.position = 0;
  elements["session-select"].value = id;
  elements["frame-slider"].max = Math.max(0, state.frames.length - 1);
  elements["frame-slider"].value = "0";
  elements["session-meta"].textContent = `${state.frames.length} 帧 · ${bytes(state.session.size_bytes)} · ${state.session.file}`;
  if (!state.frames.length) throw new Error("该日志没有 model 帧");
  await loadFrame(0);
}

async function loadFrame(position) {
  if (!state.frames.length) return;
  state.position = Math.max(0, Math.min(state.frames.length - 1, position));
  const serial = ++state.requestSerial;
  const frame = state.frames[state.position];
  elements["frame-slider"].value = String(state.position);
  elements["frame-readout"].textContent = `Frame ${frame.frame_index} / ${state.frames.length}`;
  elements["scene-status"].textContent = "读取帧";
  try {
    const payload = await getJson(`/api/frame?${query({ id: state.session.id, frame: frame.frame_index })}`);
    const pointCloud = await getJson(`/api/depth-points?${query({ id: state.session.id, frame: frame.frame_index, kind: "model", stride: 4, max_m: 20 })}`);
    if (serial !== state.requestSerial) return;
    state.payload = payload;
    updateImages(frame.frame_index);
    updateInspector(payload);
    renderScene(payload, pointCloud);
  } catch (error) {
    if (serial === state.requestSerial) showError(error);
  }
}

function updateImages(frameIndex) {
  const base = { id: state.session.id, frame: frameIndex, max_m: 20, nonce: Date.now() };
  elements["raw-depth"].src = `/api/depth-image?${query({ ...base, kind: "raw" })}`;
  elements["model-depth"].src = `/api/depth-image?${query({ ...base, kind: "model" })}`;
  const maxDepth = state.payload?.startup?.max_depth_m || 20;
  elements["depth-range"].textContent = `0-${number(maxDepth, 0)} m`;
}

function metric(label, value) {
  const wrapper = document.createElement("div");
  const term = document.createElement("dt");
  const detail = document.createElement("dd");
  term.textContent = label;
  detail.textContent = value;
  wrapper.append(term, detail);
  return wrapper;
}

function updateInspector(payload) {
  const { model = {}, depth = {}, trajectory = {}, controls = [], odom = {} } = payload;
  const timestamp = new Date(payload.frame.wall_time * 1000);
  elements["frame-time"].textContent = Number.isNaN(timestamp.valueOf()) ? "-" : timestamp.toLocaleTimeString();
  const latestControl = controls.at(-1) || {};
  const rawStats = depth?.raw_depth_m_p05_median_center;
  const modelStats = depth?.model_depth_m_p05_median_center;
  const tensorStats = model?.depth_tensor_min_max_mean;
  elements["frame-metrics"].replaceChildren(
    metric("raw p05 / median / center m", vector(rawStats)),
    metric("model p05 / median / center m", vector(modelStats)),
    metric("tensor min / max / mean", tensorStats ? vector(tensorStats, 3) : "旧日志未记录"),
    metric("inference", `${number(model?.inference_ms)} ms`),
    metric("motion [v, a, goal] body", vector(model?.motion)),
    metric("selected / score", `${model?.selected ?? "-"} / ${number(model?.selected_score, 4)}`),
    metric("selected P body m", vector(model?.selected_state_body?.slice(0, 3))),
    metric("selected V body m/s", vector(model?.selected_state_body?.slice(3, 6))),
    metric("planned end world m", vector(trajectory?.end_position_world)),
    metric("actual odom world m", vector(odom?.position_world)),
    metric("executed V world m/s", vector(latestControl?.velocity_world)),
    metric("yaw / yaw rate", `${number(latestControl?.yaw_deg)} deg / ${number(latestControl?.yaw_rate_rad_s)} rad/s`),
  );

  const scores = model?.candidate_scores || [model?.selected_score];
  const states = model?.candidate_states_body || [model?.selected_state_body];
  elements["candidate-count"].textContent = `${states.filter(Boolean).length} candidates`;
  elements["candidate-table"].replaceChildren();
  states.forEach((candidate, index) => {
    if (!candidate) return;
    const row = document.createElement("tr");
    if (index === model?.selected || states.length === 1) row.className = "selected";
    for (const value of [String(index), number(scores[index], 4), vector(candidate.slice(0, 3))]) {
      const cell = document.createElement("td");
      cell.textContent = value;
      row.append(cell);
    }
    elements["candidate-table"].append(row);
  });

  const lidarSummary = payload.lidar ? { ...payload.lidar, points_xyz_m: `[${payload.lidar.point_count} points]` } : null;
  elements["event-json"].textContent = JSON.stringify({
    depth, model, trajectory, controls, odom, lidar: lidarSummary,
  }, null, 2);
  elements["scene-subtitle"].textContent = `frame ${payload.frame.frame_index} · score ${number(model?.selected_score, 4)} · ${number(model?.inference_ms)} ms`;
}

function renderScene(payload, pointCloud) {
  const { model = {}, trajectory = {}, odom = {}, lidar = {}, controls = [] } = payload;
  const quaternion = odom?.orientation_xyzw || [0, 0, 0, 1];
  const odomPosition = odom?.position_world || trajectory?.start_position_world || [0, 0, 0];
  const originWorld = trajectory?.start_position_world || model?.reference_position_world || odomPosition;

  const worldPoint = (point) => state.mode === "world" ? point : inverseRotate(subtract(point, originWorld), quaternion);
  const bodyPoint = (point, base = odomPosition) => state.mode === "world" ? add(base, rotate(point, quaternion)) : add(worldPoint(base), point);
  const traces = [];

  const depthPoints = (pointCloud.points || []).map((point) => bodyPoint(point));
  if (depthPoints.length) {
    traces.push({
      type: "scatter3d", mode: "markers", name: "Depth input", ...xyz(depthPoints),
      marker: { size: 2.1, opacity: 0.58, color: pointCloud.depth_m, colorscale: "Turbo", cmin: 0, cmax: 20, showscale: false },
      hovertemplate: "depth %{marker.color:.2f} m<extra></extra>",
    });
  }

  const lidarPoints = (lidar?.points_xyz_m || []).map((point) => bodyPoint(point));
  if (lidarPoints.length) {
    traces.push({
      type: "scatter3d", mode: "markers", name: "LiDAR", ...xyz(lidarPoints),
      marker: { size: 3.5, color: "#90cc66", opacity: 0.9 }, hoverinfo: "name+x+y+z",
    });
  }

  const candidateStates = model?.candidate_states_body || (model?.selected_state_body ? [model.selected_state_body] : []);
  const candidateScores = model?.candidate_scores || [model?.selected_score];
  const candidateLines = [];
  const candidateEnds = [];
  for (const candidate of candidateStates) {
    if (!candidate) continue;
    const start = state.mode === "world" ? originWorld : [0, 0, 0];
    const end = state.mode === "world" ? add(originWorld, rotate(candidate.slice(0, 3), quaternion)) : candidate.slice(0, 3);
    candidateLines.push(start, end, null);
    candidateEnds.push(end);
  }
  if (candidateLines.length) {
    traces.push(lineTrace(candidateLines, "Candidates", "rgba(151,160,166,0.38)", 2));
    traces.push({
      type: "scatter3d", mode: "markers", name: "Candidate score", ...xyz(candidateEnds),
      marker: { size: 4, color: candidateScores, colorscale: "RdYlGn", reversescale: true, showscale: false, opacity: 0.9 },
      customdata: candidateScores, hovertemplate: "score %{customdata:.4f}<extra></extra>",
    });
  }

  const planned = (payload.planned_path_world || []).map(worldPoint);
  if (planned.length) traces.push(lineTrace(planned, "Planned P/V/A polynomial", "#f0a94b", 7));

  const actual = (payload.odom_context || []).map((record) => worldPoint(record.position_world));
  if (actual.length) traces.push(lineTrace(actual, "Actual odom", "#39c8d4", 7));

  const commandPoints = controls.filter((control) => control.position_world).map((control) => worldPoint(control.position_world));
  if (commandPoints.length) {
    traces.push({
      type: "scatter3d", mode: "markers", name: "Executed command samples", ...xyz(commandPoints),
      marker: { size: 4, color: "#df6ac0" }, hoverinfo: "name+x+y+z",
    });
  }

  const drone = worldPoint(odomPosition);
  traces.push({
    type: "scatter3d", mode: "markers", name: "Drone", ...xyz([drone]),
    marker: { size: 7, color: "#ffffff", symbol: "diamond" }, hoverinfo: "name+x+y+z",
  });

  const axisLength = 1.0;
  const axes = [
    [[axisLength, 0, 0], "Body X", "#ef5656"],
    [[0, axisLength, 0], "Body Y", "#58c979"],
    [[0, 0, axisLength], "Body Z", "#568de8"],
  ];
  for (const [direction, name, color] of axes) {
    const end = state.mode === "world" ? add(odomPosition, rotate(direction, quaternion)) : add(drone, direction);
    traces.push(lineTrace([drone, end], name, color, 8));
  }

  const goalBody = model?.goal_body;
  const goalWorld = model?.goal_world;
  const goal = state.mode === "world" ? goalWorld : goalBody;
  if (goal) {
    traces.push({
      type: "scatter3d", mode: "markers", name: "Goal", ...xyz([goal]),
      marker: { size: 7, color: "#f2d453", symbol: "diamond-open" }, hoverinfo: "name+x+y+z",
    });
  }

  const layout = {
    autosize: true,
    margin: { l: 0, r: 0, t: 49, b: 0 },
    paper_bgcolor: "#111416", plot_bgcolor: "#111416",
    font: { color: "#96a0a6", family: "ui-monospace, monospace", size: 10 },
    showlegend: false,
    uirevision: `${state.session.id}-${state.mode}`,
    scene: {
      bgcolor: "#111416", aspectmode: "data",
      xaxis: { title: state.mode === "world" ? "X east (m)" : "X forward (m)", color: "#a7afb4", gridcolor: "#30363a", zerolinecolor: "#687178" },
      yaxis: { title: state.mode === "world" ? "Y north (m)" : "Y left (m)", color: "#a7afb4", gridcolor: "#30363a", zerolinecolor: "#687178" },
      zaxis: { title: "Z up (m)", color: "#a7afb4", gridcolor: "#30363a", zerolinecolor: "#687178" },
      camera: { eye: { x: 1.35, y: -1.55, z: 1.0 }, up: { x: 0, y: 0, z: 1 } },
    },
  };
  const config = { responsive: true, displaylogo: false, scrollZoom: true, modeBarButtonsToRemove: ["toImage", "sendDataToCloud", "resetCameraLastSave3d"] };
  Plotly.react(elements.scene, traces, layout, config);
  elements["scene-status"].textContent = `${state.mode === "world" ? "WORLD ENU" : "BODY FLU"} · depth ${depthPoints.length} · lidar ${lidarPoints.length} · odom ${actual.length}`;
}

function setMode(mode) {
  state.mode = mode;
  elements["mode-body"].classList.toggle("active", mode === "body");
  elements["mode-world"].classList.toggle("active", mode === "world");
  if (state.payload) {
    const frame = state.frames[state.position];
    getJson(`/api/depth-points?${query({ id: state.session.id, frame: frame.frame_index, kind: "model", stride: 4, max_m: 20 })}`)
      .then((points) => renderScene(state.payload, points))
      .catch(showError);
  }
}

function stopPlayback() {
  state.playing = false;
  window.clearInterval(state.timer);
  state.timer = null;
  if (elements["play-pause"]) {
    elements["play-pause"].innerHTML = "&#9654;";
    elements["play-pause"].title = "播放";
  }
}

function togglePlayback() {
  if (state.playing) { stopPlayback(); return; }
  state.playing = true;
  elements["play-pause"].innerHTML = "&#10074;&#10074;";
  elements["play-pause"].title = "暂停";
  state.timer = window.setInterval(() => {
    if (state.position >= state.frames.length - 1) { stopPlayback(); return; }
    loadFrame(state.position + 1);
  }, 500);
}

elements["session-select"].addEventListener("change", (event) => loadSession(event.target.value).catch(showError));
elements["frame-slider"].addEventListener("input", (event) => { stopPlayback(); loadFrame(Number(event.target.value)); });
elements["first-frame"].addEventListener("click", () => { stopPlayback(); loadFrame(0); });
elements["previous-frame"].addEventListener("click", () => { stopPlayback(); loadFrame(state.position - 1); });
elements["play-pause"].addEventListener("click", togglePlayback);
elements["next-frame"].addEventListener("click", () => { stopPlayback(); loadFrame(state.position + 1); });
elements["last-frame"].addEventListener("click", () => { stopPlayback(); loadFrame(state.frames.length - 1); });
elements["mode-body"].addEventListener("click", () => setMode("body"));
elements["mode-world"].addEventListener("click", () => setMode("world"));
window.addEventListener("keydown", (event) => {
  if (event.target.matches("input, select")) return;
  if (event.key === "ArrowLeft") { stopPlayback(); loadFrame(state.position - 1); }
  if (event.key === "ArrowRight") { stopPlayback(); loadFrame(state.position + 1); }
  if (event.key === " ") { event.preventDefault(); togglePlayback(); }
});

loadSessions().catch(showError);
