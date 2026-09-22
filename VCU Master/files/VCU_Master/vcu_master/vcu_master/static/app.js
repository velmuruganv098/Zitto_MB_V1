/* VCU Master - browser side. Talks to the Python backend over REST + WebSocket. */
(() => {
"use strict";
const { StripChart, Ribbon, fmt } = window.VCUCharts;
const $ = (s, r = document) => r.querySelector(s);
const $$ = (s, r = document) => Array.from(r.querySelectorAll(s));
const esc = (s) => String(s ?? "").replace(/[&<>"']/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]));
const now = () => Date.now() / 1000;
const LS = {
  get(k, d) { try { const v = localStorage.getItem("vcum." + k); return v === null ? d : JSON.parse(v); } catch { return d; } },
  set(k, v) { try { localStorage.setItem("vcum." + k, JSON.stringify(v)); } catch { /* ignore */ } },
};

/* ------------------------------------------------------------------ channels */
const CH = {
  CAN1:   { label: "CAN1",   c: "--c-can1" },
  CAN2:   { label: "CAN2",   c: "--c-can2" },
  IMU:    { label: "IMU",    c: "--c-imu" },
  CSA:    { label: "CSA",    c: "--c-csa" },
  FLASH:  { label: "Flash",  c: "--c-flash" },
  SYSTEM: { label: "System", c: "--c-sys" },
  GPIO:   { label: "GPIO",   c: "--c-gpio" },
  OTA:    { label: "OTA",    c: "--c-ota" },
  ESP32:  { label: "ESP32",  c: "--c-esp" },
  CMD:    { label: "Command", c: "--c-sys" },
  TX:     { label: "Sent",   c: "--c-tx" },
  LOG:    { label: "Log",    c: "--c-sys" },
  RAW:    { label: "Raw",    c: "--c-sys" },
  OTHER:  { label: "Other",  c: "--ink-3" },
};
const CH_ORDER = ["CAN1", "CAN2", "IMU", "CSA", "FLASH", "OTA", "GPIO", "SYSTEM", "ESP32", "CMD", "TX", "LOG", "RAW", "OTHER"];
const channelOf = (r) => CH_ORDER.find((t) => r.tags.includes(t)) || "OTHER";

/* ------------------------------------------------------------------ api */
async function api(method, url, body) {
  const res = await fetch(url, {
    method, headers: body ? { "Content-Type": "application/json" } : undefined,
    body: body ? JSON.stringify(body) : undefined,
  });
  let data = null;
  try { data = await res.json(); } catch { /* not json */ }
  if (!res.ok) throw new Error((data && data.detail) || `${res.status} ${res.statusText}`);
  return data;
}
const GET = (u) => api("GET", u), POST = (u, b) => api("POST", u, b || {}), DEL = (u) => api("DELETE", u);
let toastT;
function toast(msg, err = false) {
  const t = $("#toast");
  t.textContent = msg; t.className = "show" + (err ? " err" : "");
  clearTimeout(toastT); toastT = setTimeout(() => (t.className = ""), err ? 5000 : 2200);
}
async function act(fn, okMsg) {
  try { const r = await fn(); if (okMsg) toast(okMsg); return r; }
  catch (e) { toast(e.message, true); return null; }
}
const send = (cmd) => act(() => POST("/api/send", { cmd }));

/* ------------------------------------------------------------------ state */
const S = {
  recs: [], maxRecs: 20000, snap: null, signals: [], messages: [], hw: null,
  paused: false, mode: "stream", tagSel: new Set(LS.get("tagSel", [])),
  search: "", ids: null, filters: [], cfOn: new Set(LS.get("cfOn", [])),
  plotSel: LS.get("plotSel", []), winS: 30, dirtyLog: true, gpioDraft: {},
  modPend: {}, dbcSel: null, sigBus: 0, sigSearch: "",
};

/* ------------------------------------------------------------------ nav + theme */
const TITLES = { connect: "Connection", live: "Live data", system: "IMU and current sensing", vehicle: "Vehicle", updates: "Firmware update and CAN databases", device: "ESP32 and S32K144 control" };
function showWin(w) {
  $$(".nav").forEach((b) => b.classList.toggle("on", b.dataset.win === w));
  $$(".win").forEach((s) => s.classList.toggle("on", s.id === "win-" + w));
  $("#winTitle").textContent = TITLES[w];
  S.win = w; LS.set("win", w);
  S.dirtyLog = true;
}
$$(".nav").forEach((b) => b.addEventListener("click", () => showWin(b.dataset.win)));
document.addEventListener("click", (e) => {
  const g = e.target.closest("[data-goto]");
  if (g) { e.preventDefault(); showWin(g.dataset.goto); }
});
const theme = LS.get("theme", null);
if (theme) document.documentElement.dataset.theme = theme;
$("#themeBtn").addEventListener("click", () => {
  const dark = document.documentElement.dataset.theme === "dark" ||
    (!document.documentElement.dataset.theme && matchMedia("(prefers-color-scheme: dark)").matches);
  document.documentElement.dataset.theme = dark ? "light" : "dark";
  LS.set("theme", document.documentElement.dataset.theme);
  Object.values(charts).forEach((c) => (c.dirty = true));
});

/* ------------------------------------------------------------------ websocket */
function connectWs() {
  const ws = new WebSocket(`${location.protocol === "https:" ? "wss" : "ws"}://${location.host}/ws`);
  ws.onmessage = (ev) => {
    const m = JSON.parse(ev.data);
    if (m.t === "recs") onRecs(m.items, m.replay);
    else if (m.t === "state") onState(m.state);
    else if (m.t === "link") { if (S.snap) { S.snap.link = m.link; renderLink(); } }
    else if (m.t === "hist") onHist(m);
    else if (m.t === "signals") { S.signals = m.signals; S.messages = m.messages; renderVehicleTables(); }
  };
  ws.onclose = () => { setPill("Server offline", "err"); setTimeout(connectWs, 1500); };
  setInterval(() => { if (ws.readyState === 1) ws.send("k"); }, 20000);
}

/* ------------------------------------------------------------------ records */
function onRecs(items, replay) {
  if (replay) S.recs = [];
  for (const r of items) {
    r.ch = channelOf(r);
    S.recs.push(r);
    if (!replay) ribbon.hit(ribbonLane(r), r.t);
    feedCharts(r);
    feedConsole(r);
  }
  if (S.recs.length > S.maxRecs) S.recs.splice(0, S.recs.length - S.maxRecs);
  S.dirtyLog = true;
}
function ribbonLane(r) {
  if (r.tags.includes("CAN1")) return "CAN1";
  if (r.tags.includes("CAN2")) return "CAN2";
  if (r.tags.includes("IMU")) return "IMU";
  if (r.tags.includes("CSA")) return "CSA";
  return "SYS";
}

/* ------------------------------------------------------------------ summaries */
const hex2 = (n) => n.toString(16).toUpperCase().padStart(2, "0");
const hexId = (id, ext) => "0x" + id.toString(16).toUpperCase().padStart(ext ? 8 : 3, "0");
function kvs(pairs) { return pairs.map(([k, v]) => `<span class="k">${esc(k)}</span> <span class="v">${esc(v)}</span>`).join("  "); }
function summary(r) {
  const f = r.fields || {};
  switch (r.type) {
    case "IMU": return kvs([["a", `${f.ax_mg}, ${f.ay_mg}, ${f.az_mg} mg`], ["ω", `${(f.gx_mdps / 1000).toFixed(2)}, ${(f.gy_mdps / 1000).toFixed(2)}, ${(f.gz_mdps / 1000).toFixed(2)} dps`], ["T", `${f.temp_c} °C`]]);
    case "CSA": return kvs([["I", `${f.current_ma} mA`], ["V", `${f.voltage_mv} mV`], ["P", `${f.power_mw} mW`]]);
    case "CAN": return kvs([["id", `${hexId(f.id, f.ext)} ${f.ext ? "EXT" : "STD"}${f.rtr ? " RTR" : ""}`], ["dlc", f.dlc], ["", (f.data || []).map(hex2).join(" ")]]);
    case "CAN_STATUS": return kvs([["state", f.state_name], ["baud", `${f.baud} k`], ["ready", f.ready], ["bus_off", f.bus_off], ["rx", f.rx], ["err", f.err], ["tec/rec", `${f.tx_err}/${f.rx_err}`]]);
    case "STATUS": return kvs([["mod", `imu ${f.imu} csa ${f.csa} can1 ${f.can1} can2 ${f.can2} flm ${f.flm}`], ["up", dur(f.uptime)], ["reset", f.reset_name], ["hb", f.hb]]);
    case "HEARTBEAT": return kvs([["uptime", dur(f.uptime)]]);
    case "FLM": return kvs([["used", `${f.used}/${f.total}`], ["free", f.free], ["records", f.records]]);
    case "GPIO_STATUS": return (f.pins || []).map((p) => `${p.id}:${p.dir}=${p.state}`).join(" ");
    case "CMD_ACK": return kvs(Object.entries(f));
    case "LOG": case "CMD_SENT": case "CMD_ERR": case "INFO": case "PONG": return esc(f.text || r.raw);
    case "TX": return esc(f.cmd);
    default: return esc(r.raw.replace(/^seq=\d+\s+/, "").replace(r.type, "").trim());
  }
}
function decodeText(r) {
  if (!r.dbc) return "";
  if (r.dbc.error) return "decode error: " + r.dbc.error;
  const s = r.dbc.signals || {};
  return r.dbc.message + "  " + Object.entries(s).map(([k, v]) => `${k}=${v.label ?? num(v.v)}${v.unit ? " " + v.unit : ""}`).join("  ");
}
function num(v) {
  if (v === null || v === undefined) return "–";
  if (Number.isInteger(v)) return String(v);
  const a = Math.abs(v);
  return a >= 100 ? v.toFixed(1) : a >= 1 ? v.toFixed(2) : v.toFixed(3);
}
function dur(ms) {
  if (ms === undefined || ms === null) return "–";
  let s = Math.floor(ms / 1000);
  const d = Math.floor(s / 86400); s %= 86400;
  const h = Math.floor(s / 3600); s %= 3600;
  const m = Math.floor(s / 60); s %= 60;
  return (d ? d + "d " : "") + (h || d ? h + "h " : "") + m + "m " + s + "s";
}
function clock(t) {
  const d = new Date(t * 1000);
  return d.toTimeString().slice(0, 8) + "." + String(d.getMilliseconds()).padStart(3, "0");
}
function ago(t) {
  if (!t) return "–";
  const s = now() - t;
  return s < 1 ? "now" : s < 60 ? s.toFixed(0) + " s" : (s / 60).toFixed(0) + " min";
}

/* ------------------------------------------------------------------ live log */
function buildChips() {
  const box = $("#tagChips");
  box.innerHTML = `<button class="chip ${S.tagSel.size ? "" : "on"}" data-tag="*">All</button>` +
    ["IMU", "CSA", "FLASH", "CAN1", "CAN2", "SYSTEM", "GPIO", "OTA", "CMD", "ESP32", "TX", "LOG", "RAW", "OTHER"].map((t) =>
      `<button class="chip ${S.tagSel.has(t) ? "on" : ""}" data-tag="${t}" style="--cc:var(${CH[t].c})"><i></i>${CH[t].label} <b id="cnt-${t}">0</b></button>`).join("");
  $("#cfTag").innerHTML = `<option value="">Any channel</option>` + CH_ORDER.map((t) => `<option value="${t}">${CH[t].label}</option>`).join("");
}
$("#tagChips").addEventListener("click", (e) => {
  const c = e.target.closest(".chip"); if (!c) return;
  const t = c.dataset.tag;
  if (t === "*") S.tagSel.clear();
  else S.tagSel.has(t) ? S.tagSel.delete(t) : S.tagSel.add(t);
  LS.set("tagSel", [...S.tagSel]);
  $$(".chip", $("#tagChips")).forEach((b) => b.classList.toggle("on", b.dataset.tag === "*" ? !S.tagSel.size : S.tagSel.has(b.dataset.tag)));
  S.dirtyLog = true;
});
function compileSearch(q) {
  q = q.trim();
  if (!q) return null;
  const m = q.match(/^\/(.+)\/(i?)$/);
  try { return m ? new RegExp(m[1], m[2] || "i") : new RegExp(q.replace(/[.*+?^${}()|[\]\\]/g, "\\$&"), "i"); }
  catch { return null; }
}
$("#logSearch").addEventListener("input", (e) => { S.search = compileSearch(e.target.value); S.dirtyLog = true; });
$("#logIds").addEventListener("input", (e) => {
  const ids = e.target.value.split(/[\s,]+/).filter(Boolean).map((x) => parseInt(x, 16)).filter((x) => !Number.isNaN(x));
  S.ids = ids.length ? new Set(ids) : null; S.dirtyLog = true;
});
$("#logMode").addEventListener("click", (e) => {
  const b = e.target.closest("button"); if (!b) return;
  S.mode = b.dataset.m;
  $$("button", $("#logMode")).forEach((x) => x.classList.toggle("on", x === b));
  $("#logBody").classList.toggle("raw", S.mode === "raw");
  $("#logHead").hidden = S.mode === "raw";
  S.dirtyLog = true;
});
$("#pauseBtn").addEventListener("click", () => {
  S.paused = !S.paused;
  $("#pauseBtn").textContent = S.paused ? "Resume" : "Pause";
  $("#logPaused").hidden = !S.paused;
  S.dirtyLog = true;
});
$("#clearBtn").addEventListener("click", async () => {
  if (!confirm("Clear all collected data in this session? Recording files are kept.")) return;
  await act(() => POST("/api/clear"));
  S.recs = []; Object.values(charts).forEach((c) => c.clear()); S.dirtyLog = true;
});

function recText(r) { return r.raw + " " + decodeText(r); }
function cfMatch(r) {
  const active = S.filters.filter((f) => S.cfOn.has(f.name));
  if (!active.length) return true;
  return active.some((f) => {
    if (f.tag && !r.tags.includes(f.tag)) return false;
    if (!f._re) {
      try { f._re = f.regex ? new RegExp(f.expr, "i") : new RegExp(f.expr.replace(/[.*+?^${}()|[\]\\]/g, "\\$&"), "i"); }
      catch { f._re = /$^/; }
    }
    return f._re.test(recText(r));
  });
}
function passes(r) {
  if (S.tagSel.size && !r.tags.some((t) => S.tagSel.has(t))) return false;
  if (S.ids && !(r.type === "CAN" && S.ids.has(r.fields.id))) return false;
  if (S.search && !S.search.test(recText(r))) return false;
  return cfMatch(r);
}
function filtered() { return S.recs.filter(passes); }

function rowHtml(r) {
  const cc = `--cc:var(${CH[r.ch].c})`;
  const err = r.tags.includes("ERR") || r.tags.includes("PARSE_ERR") || /fail|bus_off=1|ERROR|CMD_ERR/.test(r.raw);
  if (S.mode === "raw") return `<div class="lr${err ? " err" : ""}" style="${cc}" data-id="${r.id}"><span>${clock(r.t)}</span><span>${esc(r.raw)}</span></div>`;
  const busTxt = r.type === "CAN" ? CH[r.ch].label : CH[r.ch].label;
  return `<div class="lr${err ? " err" : ""}" style="${cc}" data-id="${r.id}"><span>${clock(r.t)}</span><span>${r.seq ?? ""}</span><span class="ch">${busTxt}</span><span class="ty">${esc(r.type)}</span><span>${summary(r)}</span><span class="dc" title="${esc(decodeText(r))}">${esc(decodeText(r))}</span></div>`;
}
function renderLog() {
  if (S.win !== "live" || !S.dirtyLog || S.paused) return;
  S.dirtyLog = false;
  const body = $("#logBody");
  const atBottom = body.scrollTop + body.clientHeight >= body.scrollHeight - 40;
  let rows = filtered();
  const total = rows.length;
  if (S.mode === "latest") {
    const m = new Map();
    for (const r of rows) m.set(r.type === "CAN" ? `CAN ${r.fields.bus} ${r.fields.id}` : r.type === "CAN_STATUS" ? `CS ${r.fields.bus}` : r.type, r);
    rows = [...m.entries()].sort((a, b) => a[0].localeCompare(b[0], undefined, { numeric: true })).map((e) => e[1]);
    body.innerHTML = rows.map(rowHtml).join("");
    $("#logCount").textContent = `${rows.length} distinct message types from ${total} matching`;
    return;
  }
  rows = rows.slice(-700);
  body.innerHTML = rows.map(rowHtml).join("") || `<p class="empty">No messages match these filters. Clear a filter or wait for data.</p>`;
  $("#logCount").textContent = `Showing ${rows.length} of ${total} matching (${S.recs.length} collected)`;
  if (atBottom) body.scrollTop = body.scrollHeight;
}
$("#logBody").addEventListener("click", (e) => {
  const row = e.target.closest(".lr"); if (!row) return;
  const r = S.recs.find((x) => x.id === +row.dataset.id); if (!r) return;
  $("#detBody").textContent = JSON.stringify({ time: clock(r.t), seq: r.seq, type: r.type, tags: r.tags, raw: r.raw, fields: r.fields, dbc: r.dbc }, null, 2);
  $("#recDetail").hidden = false;
});
$("#detClose").addEventListener("click", () => ($("#recDetail").hidden = true));

function download(name, text, type) {
  const a = document.createElement("a");
  a.href = URL.createObjectURL(new Blob([text], { type }));
  a.download = name; a.click();
  setTimeout(() => URL.revokeObjectURL(a.href), 1000);
}
const stamp = () => new Date().toISOString().slice(0, 19).replace(/[:T]/g, "-");
$("#expCsv").addEventListener("click", () => {
  const q = (s) => `"${String(s ?? "").replace(/"/g, '""')}"`;
  const lines = ["pc_time,seq,channel,type,raw,decoded"].concat(filtered().map((r) =>
    [q(clock(r.t)), r.seq ?? "", r.ch, r.type, q(r.raw), q(decodeText(r))].join(",")));
  download(`vcu_master_${stamp()}.csv`, lines.join("\n"), "text/csv");
});
$("#expTxt").addEventListener("click", () => {
  download(`vcu_master_${stamp()}.txt`, filtered().map((r) => `${clock(r.t)} ${r.raw}${r.dbc ? "   | " + decodeText(r) : ""}`).join("\n"), "text/plain");
});

/* custom filters */
function renderFilters() {
  $("#cfCount").textContent = S.filters.length ? `(${S.cfOn.size} of ${S.filters.length} active)` : "";
  $("#cfList").innerHTML = S.filters.map((f, i) =>
    `<span class="cf ${S.cfOn.has(f.name) ? "on" : ""}" data-i="${i}"><label class="chk"><input type="checkbox" ${S.cfOn.has(f.name) ? "checked" : ""}> ${esc(f.name)}</label> <code>${esc(f.expr)}${f.tag ? " in " + f.tag : ""}</code><button title="Delete filter" data-del="${i}">×</button></span>`).join("") ||
    `<span class="muted">No custom filters yet. Add one below; active filters are combined with OR.</span>`;
}
$("#cfList").addEventListener("change", (e) => {
  const el = e.target.closest(".cf"); if (!el) return;
  const f = S.filters[+el.dataset.i];
  e.target.checked ? S.cfOn.add(f.name) : S.cfOn.delete(f.name);
  LS.set("cfOn", [...S.cfOn]); renderFilters(); S.dirtyLog = true;
});
$("#cfList").addEventListener("click", async (e) => {
  const d = e.target.closest("[data-del]"); if (!d) return;
  const f = S.filters.splice(+d.dataset.del, 1)[0];
  S.cfOn.delete(f.name); LS.set("cfOn", [...S.cfOn]);
  await saveFilters(); renderFilters(); S.dirtyLog = true;
});
$("#cfForm").addEventListener("submit", async (e) => {
  e.preventDefault();
  const f = { name: $("#cfName").value.trim(), expr: $("#cfExpr").value.trim(), regex: $("#cfRegex").checked, tag: $("#cfTag").value };
  if (f.regex) { try { new RegExp(f.expr); } catch (err) { toast("That regex is not valid: " + err.message, true); return; } }
  if (S.filters.some((x) => x.name === f.name)) { toast("A filter with that name exists. Pick another name.", true); return; }
  S.filters.push(f); S.cfOn.add(f.name); LS.set("cfOn", [...S.cfOn]);
  await saveFilters(); renderFilters(); S.dirtyLog = true;
  $("#cfName").value = ""; $("#cfExpr").value = "";
  toast(`Filter "${f.name}" added and active`);
});
async function saveFilters() {
  await act(() => POST("/api/filters", { filters: S.filters.map(({ _re, ...f }) => f) }));
}

/* ------------------------------------------------------------------ charts */
const XYZ = [{ key: "x", label: "X", color: "#cc3d33" }, { key: "y", label: "Y", color: "#1f8f5f" }, { key: "z", label: "Z", color: "#2a6fdb" }];
const charts = {
  accel: new StripChart($("#chAccel"), XYZ),
  gyro: new StripChart($("#chGyro"), XYZ),
  cur: new StripChart($("#chCur"), [{ key: "v", label: "I", color: "--c-csa" }]),
  vol: new StripChart($("#chVol"), [{ key: "v", label: "V", color: "--c-csa" }]),
  pow: new StripChart($("#chPow"), [{ key: "v", label: "P", color: "--c-csa" }]),
  sig: new StripChart($("#chSig"), [], { maxPts: 6000 }),
};
$$(".chart-title").forEach((el) => { if (el.textContent.includes("mdps")) el.textContent = "Angular rate (dps)"; });
const ribbon = new Ribbon($("#traceCanvas"), [
  { tag: "CAN1", color: "--c-can1" }, { tag: "CAN2", color: "--c-can2" },
  { tag: "IMU", color: "--c-imu" }, { tag: "CSA", color: "--c-csa" }, { tag: "SYS", color: "--c-sys" },
]);
const PLOT_COLORS = ["#2a6fdb", "#c9711a", "#1f8f5f", "#8a4fd3", "#cc3d33", "#2b8aa8"];

function feedCharts(r) {
  const f = r.fields;
  if (r.type === "IMU") {
    charts.accel.push("x", r.t, f.ax_mg); charts.accel.push("y", r.t, f.ay_mg); charts.accel.push("z", r.t, f.az_mg);
    charts.gyro.push("x", r.t, f.gx_mdps / 1000); charts.gyro.push("y", r.t, f.gy_mdps / 1000); charts.gyro.push("z", r.t, f.gz_mdps / 1000);
    S.imuT = r.t;
  } else if (r.type === "CSA") {
    charts.cur.push("v", r.t, f.current_ma); charts.vol.push("v", r.t, f.voltage_mv); charts.pow.push("v", r.t, f.power_mw);
    S.csaT = r.t;
  } else if (r.type === "CAN" && r.dbc && r.dbc.signals && S.plotSel.length) {
    for (const [name, v] of Object.entries(r.dbc.signals)) {
      const key = `${f.bus}:${r.dbc.message}.${name}`;
      if (S.plotSel.includes(key)) charts.sig.push(key, r.t, v.v);
    }
  }
}
function onHist(m) {
  (m.imu || []).forEach((f) => feedCharts({ type: "IMU", t: f.t, fields: f }));
  (m.csa || []).forEach((f) => feedCharts({ type: "CSA", t: f.t, fields: f }));
}
$("#winSec").addEventListener("click", (e) => {
  const b = e.target.closest("button"); if (!b) return;
  S.winS = +b.dataset.s;
  $$("button", $("#winSec")).forEach((x) => x.classList.toggle("on", x === b));
  Object.values(charts).forEach((c) => { c.windowS = S.winS; c.dirty = true; });
});
function tiles(el, list) {
  el.innerHTML = list.map(([l, v, u, stale]) => `<div class="tile${stale ? " stale" : ""}"><div class="l">${esc(l)}</div><div class="v">${esc(v)}${u ? `<small>${esc(u)}</small>` : ""}</div></div>`).join("");
}

/* ------------------------------------------------------------------ state render */
function setPill(text, cls) {
  const p = $("#linkPill");
  p.className = "pill " + (cls || ""); $("b", p).textContent = text;
}
function onState(st) {
  S.snap = st;
  $("#rateVal").textContent = st.rate;
  $("#crcVal").textContent = st.latest.BRIDGE?.crc_errors ?? 0;
  const rec = $("#recBtn");
  rec.classList.toggle("on", !!st.recording);
  $("span", rec).textContent = st.recording ? `Recording ${st.recording.rows}` : "Record";
  rec.title = st.recording ? `Writing data/logs/${st.recording.name}` : "Record session to CSV";
  for (const t of CH_ORDER) { const el = $("#cnt-" + t); if (el) el.textContent = st.counts["#" + t] || 0; }
  renderLink(); renderSystem(); renderVehicle(); renderUpdates(); renderDevice();
}
function renderLink() {
  const L = S.snap?.link || {};
  const on = !!L.connected;
  setPill(on ? (L.kind === "sim" ? "Simulator" : "Connected") : (L.error ? "Link error" : "Offline"), on ? (L.kind === "sim" ? "sim" : "ok") : (L.error ? "err" : ""));
  $("#navLinkDot").classList.toggle("ok", on);
  $("#railLink").textContent = on ? `${L.name || ""} ${L.address || ""}` : "Not connected";
  $("#discBtn").disabled = !on;
  const kv = [
    ["State", on ? "Connected" : "Disconnected"], ["Device", L.name || "–"], ["Address", L.address || "–"],
    ["Transport", L.kind === "sim" ? "Simulator" : L.kind === "ble" ? "Bluetooth LE" : "–"],
    ["ATT MTU", L.mtu ? `${L.mtu} (max write ${L.mtu - 3} B)` : "–"], ["Connected for", L.since && on ? dur((now() - L.since) * 1000) : "–"],
    ["Lines received", L.rx_lines ?? 0], ["Commands sent", `${L.tx_cmds ?? 0}${L.tx_errors ? `, ${L.tx_errors} failed` : ""}`],
    ["Last data", ago(L.last_rx)], ["Reconnects", L.reconnects ?? 0],
  ];
  if (L.error) kv.push(["Last error", L.error]);
  $("#linkKv").innerHTML = kv.map(([k, v]) => `<dt>${k}</dt><dd>${esc(v)}</dd>`).join("");
  if (on && !S.gattLoaded) { S.gattLoaded = true; loadGatt(); }
  if (!on) S.gattLoaded = false;
}
async function loadGatt() {
  const r = await act(() => GET("/api/services"));
  if (!r) return;
  $("#gattList").innerHTML = r.services.map((s) => `<div class="svc">${esc(s.uuid)} <span class="muted">${esc(s.description)}</span>${s.characteristics.map((c) => `<div class="chr">${esc(c.uuid)} [${esc(c.properties.join(", "))}] ${esc(c.description)}</div>`).join("")}</div>`).join("") || `<p class="muted">No services reported.</p>`;
}

/* ------------------------------------------------------------------ connection */
$("#scanBtn").addEventListener("click", async () => {
  const b = $("#scanBtn"); b.disabled = true; b.textContent = "Scanning…";
  $("#devList").innerHTML = `<p class="empty">Scanning for ${$("#scanTime").value} seconds…</p>`;
  try {
    const r = await POST("/api/scan", { timeout: +$("#scanTime").value, name_filter: $("#scanName").value, only_bridge: $("#scanBridge").checked });
    renderDevices(r.devices);
  } catch (e) {
    toast(e.message, true);
    renderDevices([{ address: "SIM", name: "Zitto_MB_V1_Bridge (simulator)", rssi: -42, bridge: true }]);
  } finally { b.disabled = false; b.textContent = "Scan"; }
});
function rssiBars(r) {
  const n = r >= -60 ? 4 : r >= -70 ? 3 : r >= -80 ? 2 : r >= -90 ? 1 : 0;
  return `<span class="rssi" title="${r} dBm">${[5, 8, 11, 14].map((h, i) => `<i class="${i < n ? "on" : ""}" style="height:${h}px"></i>`).join("")}</span><span class="rssi-v">${r ?? "?"} dBm</span>`;
}
function renderDevices(list) {
  $("#devList").innerHTML = list.length ? list.map((d) => `<div class="dev ${d.bridge ? "bridge" : ""}"><div><div class="nm">${esc(d.name)}</div><div class="ad">${esc(d.address)}</div></div><div>${rssiBars(d.rssi)}</div><button class="${d.bridge ? "primary" : ""}" data-addr="${esc(d.address)}" data-name="${esc(d.name)}">Connect</button></div>`).join("")
    : `<p class="empty">No devices found. Check the ESP32 is powered and advertising as Zitto_MB_V1_Bridge, then scan again.</p>`;
}
$("#devList").addEventListener("click", async (e) => {
  const b = e.target.closest("[data-addr]"); if (!b) return;
  b.disabled = true; b.textContent = "Connecting…";
  const r = await act(() => POST("/api/connect", { address: b.dataset.addr, name: b.dataset.name, auto_reconnect: $("#autoRe").checked }));
  b.disabled = false; b.textContent = "Connect";
  if (r) { toast(`Connected to ${b.dataset.name}`); S.gattLoaded = false; }
});
$("#discBtn").addEventListener("click", () => act(() => POST("/api/disconnect"), "Disconnected"));
$$(".qc").forEach((b) => b.addEventListener("click", () => send(b.dataset.cmd)));

const hist = LS.get("cmdHist", []); let hi = hist.length;
$("#consoleForm").addEventListener("submit", async (e) => {
  e.preventDefault();
  const v = $("#consoleIn").value.trim(); if (!v) return;
  if (hist[hist.length - 1] !== v) { hist.push(v); if (hist.length > 50) hist.shift(); LS.set("cmdHist", hist); }
  hi = hist.length;
  const r = await send(v);
  if (r) $("#consoleIn").value = "";
});
$("#consoleIn").addEventListener("keydown", (e) => {
  if (e.key === "ArrowUp" && hi > 0) { hi--; e.target.value = hist[hi]; e.preventDefault(); }
  if (e.key === "ArrowDown") { hi = Math.min(hist.length, hi + 1); e.target.value = hist[hi] || ""; e.preventDefault(); }
});
function feedConsole(r) {
  const out = $("#console");
  const isCon = r.tags.some((t) => ["ESP32", "BRIDGE", "CMD", "TX", "RAW", "OTHER", "OTA"].includes(t)) || (r.type === "LOG" && !r.tags.includes("CAN1") && !r.tags.includes("CAN2"));
  if (isCon) {
    const cls = r.type === "TX" ? "tx" : (r.tags.includes("ERR") || /fail|ERR/.test(r.raw)) ? "er" : "rx";
    const d = document.createElement("div");
    d.className = cls; d.textContent = `${clock(r.t)}  ${r.type === "TX" ? r.fields.cmd : r.raw}`;
    out.appendChild(d);
    while (out.childElementCount > 400) out.firstChild.remove();
    out.scrollTop = out.scrollHeight;
  }
  if (r.tags.includes("FLASH") && r.type === "LOG" || (r.type === "LOG" && S.flashWait && now() - S.flashWait < 3)) {
    const fo = $("#flOut"), d = document.createElement("div");
    d.textContent = `${clock(r.t)}  ${r.fields.text}`;
    fo.appendChild(d); fo.scrollTop = fo.scrollHeight;
    if (!r.tags.includes("FLASH")) S.flashWait = 0;
  }
}

/* ------------------------------------------------------------------ system (IMU / CSA) */
function renderSystem() {
  const L = S.snap.latest;
  const imu = L.IMU, csa = L.CSA;
  const staleI = !S.imuT || now() - S.imuT > 3, staleC = !S.csaT || now() - S.csaT > 3;
  $("#imuAge").textContent = imu ? (staleI ? `Last sample ${ago(S.imuT)} ago` : "Live, every 500 ms") : "No IMU data yet. Enable the IMU module on the ESP32 and S32K page.";
  $("#csaAge").textContent = csa ? (staleC ? `Last sample ${ago(S.csaT)} ago` : "Live, every 200 ms") : "No CSA data yet";
  if (imu) {
    tiles($("#imuTiles"), [
      ["Accel X", imu.ax_mg, "mg", staleI], ["Accel Y", imu.ay_mg, "mg", staleI], ["Accel Z", imu.az_mg, "mg", staleI],
      ["|a|", imu.accel_g.toFixed(3), "g", staleI],
      ["Gyro X", (imu.gx_mdps / 1000).toFixed(2), "dps", staleI], ["Gyro Y", (imu.gy_mdps / 1000).toFixed(2), "dps", staleI],
      ["Gyro Z", (imu.gz_mdps / 1000).toFixed(2), "dps", staleI], ["Die temp", imu.temp_c, "°C", staleI],
    ]);
    $("#hzGroup").setAttribute("transform", `rotate(${-imu.roll_deg}) translate(0 ${Math.max(-80, Math.min(80, imu.pitch_deg * 2))})`);
    $("#attKv").innerHTML = `<dt>Roll</dt><dd>${imu.roll_deg.toFixed(1)}°</dd><dt>Pitch</dt><dd>${imu.pitch_deg.toFixed(1)}°</dd><dt>MCU timestamp</dt><dd>${dur(imu.ts_ms)}</dd>`;
  }
  if (csa) {
    const pts = (charts.cur.data.v || []).filter(([t]) => t > now() - 10).map((p) => p[1]);
    const avg = pts.length ? pts.reduce((a, b) => a + b, 0) / pts.length : null;
    tiles($("#csaTiles"), [
      ["Current", csa.current_ma, "mA", staleC], ["Voltage", (csa.voltage_mv / 1000).toFixed(3), "V", staleC],
      ["Power", (csa.power_mw / 1000).toFixed(3), "W", staleC], ["Avg current 10 s", avg === null ? "–" : avg.toFixed(0), "mA", staleC],
      ["Peak 10 s", pts.length ? Math.max(...pts) : "–", "mA", staleC], ["MCU timestamp", dur(csa.ts_ms), "", staleC],
    ]);
  }
}

/* ------------------------------------------------------------------ vehicle */
function gaugeSvg(id, cls) {
  return `<div class="gauge ${cls}" id="${id}"><svg viewBox="0 0 200 150"><path class="trk" d="${arc(100, 100, 78, -120, 120)}"/><path class="val" d="${arc(100, 100, 78, -120, 120)}"/><text class="n" x="100" y="104">–</text><text class="u" x="100" y="126"></text></svg><div class="lbl"></div><div class="src"></div></div>`;
}
function arc(cx, cy, r, a0, a1) {
  const p = (a) => [cx + r * Math.sin((a * Math.PI) / 180), cy - r * Math.cos((a * Math.PI) / 180)];
  const [x0, y0] = p(a0), [x1, y1] = p(a1);
  return `M${x0.toFixed(1)} ${y0.toFixed(1)} A${r} ${r} 0 ${a1 - a0 > 180 ? 1 : 0} 1 ${x1.toFixed(1)} ${y1.toFixed(1)}`;
}
$("#gauges").innerHTML = gaugeSvg("gSpeed", "spd") + gaugeSvg("gRpm", "rpm") + gaugeSvg("gSoc", "soc");
$$(".gauge .val").forEach((p) => { const L = p.getTotalLength(); p.style.strokeDasharray = L; p.style.strokeDashoffset = L; p.dataset.len = L; });
function setGauge(id, e) {
  const g = $("#" + id);
  const v = e?.value, max = e?.max || 100;
  $("text.n", g).textContent = v === null || v === undefined ? "–" : (Math.abs(v) >= 100 ? v.toFixed(0) : v.toFixed(1));
  $("text.u", g).textContent = e?.unit || "";
  $(".lbl", g).textContent = e?.label || "";
  $(".src", g).textContent = e?.signal || "Not mapped";
  const p = $(".val", g), L = +p.dataset.len;
  const frac = v === null || v === undefined ? 0 : Math.max(0, Math.min(1, v / max));
  p.style.strokeDashoffset = L * (1 - frac);
  if (id === "gSoc") g.classList.toggle("low", v !== null && v !== undefined && v < 20);
}
function renderVehicle() {
  const V = S.snap.vehicle, hasDbc = S.snap.dbc.length > 0;
  $("#vehEmpty").hidden = hasDbc;
  setGauge("gSpeed", V.speed); setGauge("gRpm", V.motor_rpm); setGauge("gSoc", V.soc);
  const t = (e, digits = 1) => e.text ?? (e.value === null || e.value === undefined ? "–" : e.value.toFixed(digits));
  const st = (e) => e.value === null || e.age_s > 3;
  const pw = V.pack_voltage.value !== null && V.pack_current.value !== null ? (V.pack_voltage.value * V.pack_current.value / 1000) : null;
  tiles($("#vehTiles"), [
    ["Pack voltage", t(V.pack_voltage, 2), V.pack_voltage.unit, st(V.pack_voltage)],
    ["Pack current", t(V.pack_current), V.pack_current.unit, st(V.pack_current)],
    ["Pack power", pw === null ? "–" : pw.toFixed(2), "kW", st(V.pack_current)],
    ["Battery temp", t(V.batt_temp, 0), "°C", st(V.batt_temp)],
    ["Motor temp", t(V.motor_temp, 0), "°C", st(V.motor_temp)],
    ["Controller temp", t(V.ctrl_temp, 0), "°C", st(V.ctrl_temp)],
    ["Throttle", t(V.throttle, 0), "%", st(V.throttle)],
    ["Brake", t(V.brake, 0), "", st(V.brake)],
    ["Gear / mode", t(V.gear, 0), "", st(V.gear)],
    ["Odometer", t(V.odometer, 1), V.odometer.unit, st(V.odometer)],
    ["Fault", t(V.fault, 0), "", st(V.fault)],
  ]);
}
$("#mapBtn").addEventListener("click", async () => {
  const p = $("#mapPanel"); p.hidden = !p.hidden;
  if (p.hidden) return;
  const r = await act(() => GET("/api/vehicle/map")); if (!r) return;
  const roles = S.hw.roles;
  $("#mapGrid").innerHTML = Object.keys(roles).map((k) => `<label>${esc(roles[k].label)}<select data-role="${k}"><option value="">Not mapped</option>${r.signals.map((s) => `<option ${r.map[k] === s ? "selected" : ""}>${esc(s)}</option>`).join("")}</select></label>`).join("");
});
$("#mapGrid").addEventListener("change", (e) => {
  const s = e.target.closest("select"); if (!s) return;
  act(() => POST("/api/vehicle/map", { role: s.dataset.role, signal: s.value || null }), "Mapping saved");
});
$("#sigSearch").addEventListener("input", (e) => { S.sigSearch = e.target.value.toLowerCase(); renderVehicleTables(); });
$("#sigBus").addEventListener("click", (e) => {
  const b = e.target.closest("button"); if (!b) return;
  S.sigBus = +b.dataset.b; $$("button", $("#sigBus")).forEach((x) => x.classList.toggle("on", x === b)); renderVehicleTables();
});
function renderVehicleTables() {
  if (S.win !== "vehicle") return;
  let sigs = S.signals.slice().sort((a, b) => a.key.localeCompare(b.key));
  if (S.sigBus) sigs = sigs.filter((s) => s.bus === S.sigBus);
  if (S.sigSearch) sigs = sigs.filter((s) => s.key.toLowerCase().includes(S.sigSearch));
  $("#sigTable tbody").innerHTML = sigs.map((s) => {
    const on = S.plotSel.includes(s.key);
    return `<tr class="${on ? "sel" : ""}"><td><input type="checkbox" data-key="${esc(s.key)}" ${on ? "checked" : ""} aria-label="Plot ${esc(s.signal)}"></td><td class="bus${s.bus}">CAN${s.bus}</td><td>${esc(s.message)}</td><td>${esc(s.signal)}</td><td class="num">${esc(s.label ?? num(s.value))}</td><td>${esc(s.unit)}</td><td class="num">${num(s.min_seen)}</td><td class="num">${num(s.max_seen)}</td><td class="num">${s.count}</td><td class="num">${ago(s.t)}</td></tr>`;
  }).join("") || `<tr><td colspan="10" class="unk">No decoded signals yet. Frames appear here once a DBC matches incoming CAN IDs.</td></tr>`;
  const msgs = S.messages.slice().sort((a, b) => a.bus - b.bus || a.id - b.id);
  $("#traceTable tbody").innerHTML = msgs.map((m) => `<tr><td class="bus${m.bus}">CAN${m.bus}</td><td>${hexId(m.id, m.ext)}</td><td class="${m.name ? "" : "unk"}">${esc(m.name || "not in DBC")}</td><td>${(m.data || []).map(hex2).join(" ")}</td><td class="num">${m.count}</td><td class="num">${m.rate_hz.toFixed(1)} Hz</td><td class="num">${ago(m.last)}</td></tr>`).join("") || `<tr><td colspan="7" class="unk">No CAN frames received yet.</td></tr>`;
}
$("#sigTable").addEventListener("change", (e) => {
  const c = e.target.closest("[data-key]"); if (!c) return;
  const k = c.dataset.key;
  if (c.checked) {
    if (S.plotSel.length >= 6) { c.checked = false; toast("Up to 6 signals can be plotted at once. Clear one first.", true); return; }
    S.plotSel.push(k);
  } else S.plotSel = S.plotSel.filter((x) => x !== k);
  LS.set("plotSel", S.plotSel); setPlotSeries(); renderVehicleTables();
});
function setPlotSeries() {
  charts.sig.setSeries(S.plotSel.map((k, i) => ({ key: k, label: "", color: PLOT_COLORS[i % 6] })));
  $("#sigLegend").innerHTML = S.plotSel.map((k, i) => `<span><i style="background:${PLOT_COLORS[i % 6]}"></i>${esc(k.replace(":", " "))}</span>`).join("");
  $("#plotHint").textContent = S.plotSel.length ? `${S.plotSel.length} of 6 signals` : "Select up to 6 signals in the table below";
}
async function loadDemoDbc() { const r = await act(() => POST("/api/dbc/sample"), "Demo DBC loaded on CAN1 and CAN2"); if (r) refreshDbcPick(); }
$("#loadDemoDbc").addEventListener("click", loadDemoDbc);
$("#demoDbc2").addEventListener("click", loadDemoDbc);

/* ------------------------------------------------------------------ updates: OTA */
function b64(buf) {
  let s = ""; const b = new Uint8Array(buf);
  for (let i = 0; i < b.length; i += 0x8000) s += String.fromCharCode.apply(null, b.subarray(i, i + 0x8000));
  return btoa(s);
}
function hookDrop(zone, input, handler) {
  input.addEventListener("change", () => { if (input.files.length) handler(input.files); input.value = ""; });
  zone.addEventListener("dragover", (e) => { e.preventDefault(); zone.classList.add("over"); });
  zone.addEventListener("dragleave", () => zone.classList.remove("over"));
  zone.addEventListener("drop", (e) => { e.preventDefault(); zone.classList.remove("over"); if (e.dataTransfer.files.length) handler(e.dataTransfer.files); });
}
hookDrop($("#otaDrop"), $("#otaFile"), async (files) => {
  const f = files[0];
  if (!/\.bin$/i.test(f.name) && !confirm(`${f.name} is not a .bin file. The MCU expects a raw binary image. Load it anyway?`)) return;
  const buf = await f.arrayBuffer();
  await act(() => POST("/api/ota/load", { name: f.name, b64: b64(buf) }), `${f.name} loaded`);
});
$("#otaStart").addEventListener("click", () => {
  const o = S.snap.ota;
  if (!confirm(`Send ${o.name} (${o.size} bytes, CRC32 ${o.crc}) to the S32K144?\n\nKeep the bench powered and in BLE range until it completes.`)) return;
  act(() => POST("/api/ota/start", { chunk: +$("#otaChunk").value, delay_ms: +$("#otaDelay").value }), "Update started");
});
$("#otaAbort").addEventListener("click", () => act(() => POST("/api/ota/abort"), "Abort sent"));
function renderUpdates() {
  const o = S.snap.ota;
  const run = ["STARTING", "SENDING", "FINISHING"].includes(o.state);
  const b = $("#otaState");
  b.textContent = { IDLE: "Idle", LOADED: "Ready", STARTING: "Starting", SENDING: "Sending", FINISHING: "Verifying", COMPLETE: "Complete", FAILED: "Failed", ABORTED: "Aborted" }[o.state] || o.state;
  b.className = "badge " + (o.state === "COMPLETE" ? "ok" : o.state === "FAILED" ? "err" : run ? "run" : "");
  const kv = o.name ? [["Image", o.name], ["Size", `${o.size.toLocaleString()} bytes`], ["CRC32", o.crc], ["Sent", `${o.sent.toLocaleString()} bytes`], ["Rate", o.rate_bps ? `${o.rate_bps} B/s` : "–"], ["Time left", o.eta_s !== null && run ? dur(o.eta_s * 1000) : "–"]] : [["Image", "None loaded"]];
  if (o.error) kv.push(["Error", o.error]);
  $("#otaKv").innerHTML = kv.map(([k, v]) => `<dt>${k}</dt><dd>${esc(v)}</dd>`).join("");
  $("#otaBar").style.width = o.pct + "%"; $("#otaPct").textContent = o.pct + "%";
  $("#otaStart").disabled = !o.name || run || !S.snap.link.connected;
  $("#otaAbort").disabled = !run;
  if (o.size) {
    const ch = +$("#otaChunk").value || 96, gap = +$("#otaDelay").value || 0;
    const est = Math.ceil(o.size / ch) * (gap + 18) / 1000;
    $("#otaEst").textContent = `About ${dur(est * 1000)} at these settings`;
  }
  $("#otaEvents").textContent = o.events.join("\n") || "Transfer events appear here.";

  // DBC list
  $("#dbcList").innerHTML = S.snap.dbc.map((d) => `<div class="dbc"><div><b>${esc(d.name)}</b><div class="muted">${d.messages} messages, ${d.signals} signals</div></div><div class="row"><label class="chk"><input type="checkbox" data-dbc="${esc(d.name)}" data-bus="1" ${d.buses.includes(1) ? "checked" : ""}> CAN1</label><label class="chk"><input type="checkbox" data-dbc="${esc(d.name)}" data-bus="2" ${d.buses.includes(2) ? "checked" : ""}> CAN2</label></div><button class="ghost small danger" data-rm="${esc(d.name)}">Remove</button></div>`).join("") || `<p class="muted">No DBC loaded. Frames still show in the CAN trace as raw bytes.</p>`;
  if (S.dbcNames !== S.snap.dbc.map((d) => d.name).join("|")) refreshDbcPick();
}
hookDrop($("#dbcDrop"), $("#dbcFile"), async (files) => {
  const buses = [1, 2].filter((b) => $("#dbcB" + b).checked);
  if (!buses.length) { toast("Pick CAN1, CAN2 or both before uploading.", true); return; }
  for (const f of files) {
    const text = await f.text();
    const r = await act(() => POST("/api/dbc", { name: f.name, text, buses }));
    if (r) toast(`${f.name}: ${r.dbc.messages.length} messages loaded`);
  }
  refreshDbcPick();
});
$("#dbcList").addEventListener("change", (e) => {
  const c = e.target.closest("[data-dbc]"); if (!c) return;
  const name = c.dataset.dbc;
  const buses = $$(`[data-dbc="${CSS.escape(name)}"]`).filter((x) => x.checked).map((x) => +x.dataset.bus);
  act(() => POST(`/api/dbc/${encodeURIComponent(name)}/buses`, { buses }), "Bus assignment saved");
});
$("#dbcList").addEventListener("click", async (e) => {
  const b = e.target.closest("[data-rm]"); if (!b) return;
  if (!confirm(`Remove ${b.dataset.rm}? Its decoded values are cleared.`)) return;
  await act(() => DEL(`/api/dbc/${encodeURIComponent(b.dataset.rm)}`), "DBC removed");
  refreshDbcPick();
});
async function refreshDbcPick() {
  const r = await act(() => GET("/api/dbc")); if (!r) return;
  S.dbcNames = r.dbc.map((d) => d.name).join("|");
  const sel = $("#dbcPick");
  sel.innerHTML = r.dbc.map((d) => `<option>${esc(d.name)}</option>`).join("") || `<option value="">No DBC loaded</option>`;
  if (r.dbc.length) loadDbcBrowser(sel.value);
  else { $("#dbcMsgs").innerHTML = ""; $("#dbcSigs tbody").innerHTML = ""; }
}
$("#dbcPick").addEventListener("change", (e) => loadDbcBrowser(e.target.value));
async function loadDbcBrowser(name) {
  if (!name) return;
  const d = await act(() => GET(`/api/dbc/${encodeURIComponent(name)}`)); if (!d) return;
  S.dbcDesc = d;
  $("#dbcMsgs").innerHTML = d.messages.map((m, i) => `<li data-i="${i}" class="${i === 0 ? "on" : ""}">${esc(m.name)}<small>${hexId(m.id, m.ext)} dlc ${m.dlc}${m.cycle_ms ? ", " + m.cycle_ms + " ms" : ""}${m.senders.length ? ", from " + esc(m.senders.join(", ")) : ""}</small></li>`).join("");
  showDbcMsg(0);
}
$("#dbcMsgs").addEventListener("click", (e) => {
  const li = e.target.closest("li"); if (!li) return;
  $$("li", $("#dbcMsgs")).forEach((x) => x.classList.toggle("on", x === li)); showDbcMsg(+li.dataset.i);
});
function showDbcMsg(i) {
  const m = S.dbcDesc?.messages[i]; if (!m) return;
  $("#dbcSigs tbody").innerHTML = m.signals.map((s) => `<tr title="${esc(s.comment)}"><td>${esc(s.name)}</td><td class="num">${s.start}</td><td class="num">${s.length}</td><td>${s.byte_order === "little_endian" ? "Intel" : "Motorola"}${s.signed ? ", signed" : ""}</td><td class="num">${s.scale}</td><td class="num">${s.offset}</td><td>${s.min ?? ""} to ${s.max ?? ""}</td><td>${esc(s.unit)}</td><td>${esc(Object.entries(s.choices).map(([k, v]) => `${k}=${v}`).join(", "))}</td></tr>`).join("");
}

/* ------------------------------------------------------------------ device */
const MOD_META = { IMU: ["ICM-42670-P over I2C", "--c-imu", "imu"], CSA: ["Current sense", "--c-csa", "csa"], CAN1: ["FlexCAN1, TCAN334", "--c-can1", "can1"], CAN2: ["FlexCAN2, PTC16/PTB13", "--c-can2", "can2"], FLM: ["Flash log manager", "--c-flash", "flm"] };
function buildDevice() {
  $("#modList").innerHTML = Object.entries(MOD_META).map(([m, [d, c]]) => `<div class="mod" style="--cc:var(${c})"><div><b>${m}</b><small>${d}</small></div><button class="sw" role="switch" aria-checked="false" aria-label="${m} module" data-mod="${m}"></button></div>`).join("");
  $("#gpioTable tbody").innerHTML = S.hw.s32_gpio.map((g) => `<tr data-g="${g.id}"><td class="num">${g.id}</td><td class="mono">${g.port}</td><td class="num">${g.pkg_pin}</td><td><select data-f="dir"><option value="0">Input</option><option value="1">Output</option></select></td><td><button class="lvl" data-f="st" disabled>LOW</button></td><td class="rep mono">–</td><td><button class="small" data-f="apply">Apply</button></td></tr>`).join("");
  S.hw.s32_gpio.forEach((g) => (S.gpioDraft[g.id] = { dir: 0, state: 0 }));
  $("#espGrid").innerHTML = S.hw.esp_pins.map((p) => `<div class="esp"><b>GPIO${p.pin}</b><small>${esc(p.note)}</small><div class="row"><button class="lvl small" data-esp="${p.pin}" data-v="0">LOW</button><button class="lvl small" data-esp="${p.pin}" data-v="1">HIGH</button></div></div>`).join("");
  $("#refBox").textContent = REF;
}
$("#modList").addEventListener("click", (e) => {
  const b = e.target.closest("[data-mod]"); if (!b) return;
  const want = !b.classList.contains("on");
  S.modPend[b.dataset.mod] = now();
  b.classList.add("pend");
  act(() => POST("/api/cmd/module", { name: b.dataset.mod, state: want }), `${b.dataset.mod} ${want ? "enable" : "disable"} sent`);
});
$("#stReq").addEventListener("click", () => act(() => POST("/api/cmd/status"), "Status requested"));
$("#mcuReset").addEventListener("click", () => {
  if (confirm("Reset the S32K144? CAN, IMU and CSA streams stop for a moment while it reboots.")) act(() => POST("/api/cmd/reset"), "Reset command sent");
});
$("#gpioTable").addEventListener("change", (e) => {
  const tr = e.target.closest("tr"); if (!tr || e.target.dataset.f !== "dir") return;
  S.gpioDraft[+tr.dataset.g].dir = +e.target.value;
  $("[data-f=st]", tr).disabled = e.target.value === "0";
});
$("#gpioTable").addEventListener("click", (e) => {
  const tr = e.target.closest("tr"); if (!tr) return;
  const id = +tr.dataset.g, d = S.gpioDraft[id];
  if (e.target.dataset.f === "st") {
    d.state ^= 1; e.target.textContent = d.state ? "HIGH" : "LOW"; e.target.classList.toggle("hi", !!d.state);
  } else if (e.target.dataset.f === "apply") {
    const via = $("#gpioVia").value;
    if (via === "native" && id === 13) { toast("The stock bridge rejects ID 13. Switch to RAW frame or flash the patched bridge.", true); return; }
    act(() => POST("/api/cmd/s32gpio", { id, dir: d.dir, state: d.dir ? d.state : 0, via }), `GPIO ${id} command sent`);
  }
});
$("#espGrid").addEventListener("click", (e) => {
  const b = e.target.closest("[data-esp]"); if (!b) return;
  act(() => POST("/api/cmd/espgpio", { pin: +b.dataset.esp, state: +b.dataset.v }));
});
$("#ledD").addEventListener("input", (e) => ($("#ledDv").textContent = e.target.value + "%"));
$("#ledApply").addEventListener("click", () => act(() => POST("/api/cmd/led", { period_ms: +$("#ledP").value, duty_pct: +$("#ledD").value }), "LED settings sent"));
$("#flRead").addEventListener("click", () => { S.flashWait = now(); act(() => POST("/api/cmd/flash", { op: "read" })); });
$("#flWrite").addEventListener("click", () => {
  const v = $("#flData").value; if (!v.trim()) { toast("Enter something to write first.", true); return; }
  act(() => POST("/api/cmd/flash", $("#flHex").checked ? { op: "write", hex: v } : { op: "write", text: v }), "Write sent");
});
$("#flDel").addEventListener("click", () => { if (confirm("Delete the latest flash record?")) act(() => POST("/api/cmd/flash", { op: "delete" }), "Delete sent"); });

function renderDevice() {
  if (!S.hw) return;
  const L = S.snap.latest, st = L.STATUS, hb = L.HEARTBEAT;
  const kv = [
    ["Uptime", st ? dur(st.uptime) : hb ? dur(hb.uptime) : "–"], ["Heartbeats", st?.hb ?? "–"],
    ["Reset cause", st ? `${st.reset_name} (0x${(st.reset || 0).toString(16).toUpperCase()})` : "–"], ["OTA pending", st ? (st.ota ? "Yes" : "No") : "–"],
    ["CAN1 baud", st ? `${st.can1_baud} kbps` : "–"], ["CAN2 baud", st ? `${st.can2_baud} kbps` : "–"],
    ["Flash free", st ? `${st.flash_free} pages` : "–"], ["Last status", st ? ago(st.t) : "–"],
  ];
  $("#sysKv").innerHTML = kv.map(([k, v]) => `<dt>${k}</dt><dd>${esc(v)}</dd>`).join("");
  $$("[data-mod]").forEach((b) => {
    const m = b.dataset.mod, key = MOD_META[m][2];
    const on = st ? !!st[key] : false;
    b.classList.toggle("on", on); b.setAttribute("aria-checked", on);
    if (S.modPend[m] && now() - S.modPend[m] > 1.5) delete S.modPend[m];
    b.classList.toggle("pend", !!S.modPend[m]);
  });
  $("#canCards").innerHTML = [1, 2].map((bus) => {
    const c = L["CAN_STATUS_" + bus];
    const cc = bus === 1 ? "--c-can1" : "--c-can2";
    if (!c) return `<div class="cancard" style="--cc:var(${cc})"><h4>CAN${bus}<span class="muted">No status yet</span></h4></div>`;
    const bad = c.bus_off || c.state_name === "ERROR";
    return `<div class="cancard" style="--cc:var(${cc})"><h4>CAN${bus}<span class="badge ${bad ? "err" : c.ready ? "ok" : "run"}">${esc(c.state_name)}</span></h4><dl><dt>Baud</dt><dd>${c.baud} kbps</dd><dt>Frames</dt><dd>${c.rx}</dd><dt>Errors</dt><dd>${c.err}${bus === 1 ? `, TEC ${c.tx_err}, REC ${c.rx_err}` : ""}</dd><dt>Bus off</dt><dd>${c.bus_off ? "Yes" : "No"}</dd><dt>IRQ</dt><dd>${c.irq} (err ${c.err_irq}, mb ${c.mb_irq})</dd></dl></div>`;
  }).join("");
  $$("#gpioTable tbody tr").forEach((tr) => {
    const p = L.GPIO[tr.dataset.g];
    const d = S.gpioDraft[+tr.dataset.g];
    const cell = $(".rep", tr);
    if (p) {
      cell.innerHTML = `${p.dir} <span class="lvl ${p.state ? "hi" : ""}">${p.state ? "HIGH" : "LOW"}</span>`;
      const mismatch = p.dir === "OUT" && d.dir === 1 && p.state !== d.state;
      $(".lvl", cell).classList.toggle("mismatch", mismatch);
    }
  });
  const B = L.BRIDGE || {}, Lk = S.snap.link;
  const bk = [["Info", L.INFO || "–"], ["UART frames", B.uart_frames ?? B.frames ?? "–"], ["CRC errors", B.crc_errors ?? "–"], ["Bad length", B.bad_len ?? "–"], ["UART bytes", B.uart_bytes ?? "–"], ["Ping", L.PING_MS !== null && L.PING_MS !== undefined ? `${L.PING_MS} ms` : "–"], ["BLE MTU", Lk.mtu ?? "–"], ["Updated", ago(B.t)]];
  $("#bridgeKv").innerHTML = bk.map(([k, v]) => `<dt>${k}</dt><dd>${esc(v)}</dd>`).join("");
  $$("[data-esp]").forEach((b) => b.classList.toggle("hi", L.ESP_GPIO[b.dataset.esp] === +b.dataset.v && +b.dataset.v === 1));
  const F = L.FLM;
  $("#flmKv").innerHTML = F ? [["Pages used", `${F.used} of ${F.total}`], ["Free", F.free], ["Records", F.records], ["Next page", F.next]].map(([k, v]) => `<dt>${k}</dt><dd>${esc(v)}</dd>`).join("") : `<dt>Status</dt><dd>No FLM report yet</dd>`;
}

const REF = `UART frame (S32K144 <-> ESP32, 115200 8N1)
  AA 55 | VER 01 | TYPE | LEN_L LEN_H | SEQ | PAYLOAD | CRC16_L CRC16_H
  CRC16 Modbus (poly 0xA001, init 0xFFFF) over VER..PAYLOAD

Commands to S32K144 (via bridge RAW:<TT><hex>)
  01 MODULE_EN   id, state    (IMU 0, CSA 1, CAN1 2, CAN2 3, FLM 4)
  02 GPIO_SET    id, dir, state   (IDs 1..13)
  03 STATUS_REQ
  04 MCU_RESET   (AIRCR 0x05FA0004 after 100 ms)
  05 LED_CTRL    period u16 LE, duty %, 0
  06 FLASH_RD   07 FLASH_WR data   08 FLASH_DEL
  10 OTA_START   size u32 BE, crc32 u32 BE
  11 OTA_DATA    chunk   12 OTA_FINISH   13 OTA_ABORT

Messages from S32K144
  80 LOG  81 STATUS  82 HEARTBEAT  83 IMU  84 CSA
  85 CAN  86 GPIO_STATUS  87 CMD_ACK  88 CAN_STATUS  89 FLM

Bridge text commands (BLE RX 6e400002)
  PING  INFO  GPIO  STATS
  ESP:<pin>:<0|1>
  S32:<id>:<dir>:<state>
  RAW:<TT><payload hex>      (VCU Master bridge patch)`;

/* ------------------------------------------------------------------ record */
$("#recBtn").addEventListener("click", () => {
  const on = !S.snap?.recording;
  act(() => POST("/api/record", { on }), on ? "Recording to data/logs" : "Recording saved in data/logs");
});

/* ------------------------------------------------------------------ loop */
function frame() {
  const t = now();
  ribbon.draw(t);
  if (S.win === "system") ["accel", "gyro", "cur", "vol", "pow"].forEach((k) => charts[k].draw(t));
  if (S.win === "vehicle") charts.sig.draw(t);
  renderLog();
  setTimeout(() => requestAnimationFrame(frame), 50);
}

/* ------------------------------------------------------------------ boot */
(async function boot() {
  buildChips();
  try { S.hw = await GET("/api/hw"); buildDevice(); } catch (e) { toast("Backend not reachable: " + e.message, true); }
  const f = await act(() => GET("/api/filters"));
  S.filters = f ? f.filters : [];
  renderFilters();
  setPlotSeries();
  showWin(LS.get("win", "connect"));
  refreshDbcPick();
  connectWs();
  requestAnimationFrame(frame);
})();
})();
