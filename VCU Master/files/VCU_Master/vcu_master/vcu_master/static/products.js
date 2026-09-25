/* VCU Master V0.0073 additions
 *   - Battery (BMS) and Motor (MCU) windows, laid out from the DBC by the same
 *     analyzer as CAN_DBC_Simulator (read-only: values come from decoded CAN).
 *   - Command feedback: a module / GPIO box glows while a command is pending and
 *     turns green only when the S32K's CMD_ACK (with read-back state) arrives.
 *   - Board movement (IMU displacement since power-on) and a data-integrity card.
 * Talks to app.js through window.VCU (helpers) and window.VCUProducts (hooks).
 */
(() => {
"use strict";
const V = window.VCU;
const { $, $$, esc, POST, act, toast } = V;
const lsGet = (k, d) => { try { return localStorage.getItem(k) || d; } catch { return d; } };
const lsSet = (k, v) => { try { localStorage.setItem(k, v); } catch { /* storage blocked */ } };
const P = {
  meta: null, values: {}, age: {}, sig: {}, layout: lsGet("vcum.bmsLayout", "auto"), view: lsGet("vcum.bmsView", "grid"),
  pend: {}, cmdLog: [], track: [],
};
const unit = (u) => (u === "degC" ? "°C" : u || "");
const fmt = (v, d = 2) => (v === undefined || v === null || Number.isNaN(v) ? "–" : Number(v).toFixed(d));
const MODS = { 0: "IMU", 1: "CSA", 2: "CAN1", 3: "CAN2", 4: "FLM" };
const MOD_ID = { IMU: 0, CSA: 1, CAN1: 2, CAN2: 3, FLM: 4 };

/* ================================================================ roles
 * The Battery / Motor windows are BUILT once per layout change and then only
 * PATCHED at 5 Hz.  Rebuilding innerHTML on every update destroyed the open
 * Layout dropdown and the keyboard focus (V0.0073 bug: "layout not accessible").
 */
function roleOf(key) { return P.meta?.roles?.[key]; }
function val(key) { return P.values[key]; }
function stale(key) { const a = P.age[key]; return a === undefined || a > 5; }
function has(panel) { return !!P.meta?.panels?.some((p) => p.key === panel); }
function roleText(r, v) {
  if (v === undefined || v === null) return "–";
  if (r.kind === "bool") return v ? "ON" : "OFF";
  if (r.kind === "enum" && r.choices) return r.choices[Math.round(v)] ?? fmt(v, 0);
  return fmt(v, r.step >= 1 || r.section === "count" ? 0 : (r.unit === "V" ? 3 : 1));
}
const srcOf = (r) => r.bindings.map((b) => b.msg + "." + b.sig).join(", ");

function tile(key, label) {
  const r = roleOf(key); if (!r) return "";
  return `<div class="ptile" data-role="${key}" role="group" aria-label="${esc(label || r.label)}"><small>${esc(label || r.label)}</small><b>–</b><i>${esc(r.kind === "number" ? unit(r.unit) : "")}</i><span class="src" title="${esc(srcOf(r))}">${esc(srcOf(r))}</span></div>`;
}
function ring(key, label) {
  const r = roleOf(key); if (!r) return "";
  const C = 2 * Math.PI * 52;
  return `<div class="pring" data-ring="${key}" role="meter" aria-label="${esc(label || r.label)}" aria-valuemin="${r.min ?? 0}" aria-valuemax="${r.max ?? 100}"><svg viewBox="0 0 130 130" aria-hidden="true"><circle cx="65" cy="65" r="52" class="trk"/><circle cx="65" cy="65" r="52" class="val" style="stroke-dasharray:${C};stroke-dashoffset:${C}"/><text x="65" y="68">–</text><text x="65" y="88" class="u">${esc(unit(r.unit))}</text></svg><div>${esc(label || r.label)}</div></div>`;
}
function patchRoles(root) {
  $$("[data-role]", root).forEach((el) => {
    const k = el.dataset.role, r = roleOf(k); if (!r) return;
    const t = roleText(r, val(k));
    const b = $("b", el); if (b.textContent !== t) b.textContent = t;
    el.classList.toggle("stale", stale(k));
    el.classList.toggle("on", r.kind === "bool" && !!val(k));
  });
  $$("[data-ring]", root).forEach((el) => {
    const k = el.dataset.ring, r = roleOf(k); if (!r) return;
    const v = val(k), lo = r.min ?? 0, hi = r.max ?? 100, C = 2 * Math.PI * 52;
    const f = v === undefined ? 0 : Math.max(0, Math.min(1, (v - lo) / ((hi - lo) || 1)));
    $(".val", el).style.strokeDashoffset = C * (1 - f);
    $("text", el).textContent = fmt(v, r.step >= 1 ? 0 : 1);
    el.setAttribute("aria-valuenow", v ?? 0);
    el.setAttribute("aria-valuetext", v === undefined ? "no data" : `${fmt(v, 1)} ${unit(r.unit)}`);
    el.classList.toggle("stale", stale(k));
  });
}

function flagsHtml(panel) {
  const list = P.meta?.flags?.[panel] || [];
  if (!list.length) return "";
  return `<section class="panel span2" aria-labelledby="fl-${panel}"><div class="ph"><h2 id="fl-${panel}">Faults and status</h2><span class="muted">${list.length} from the DBC · filled dot = active, red = active fault</span></div><ul class="pflags">${
    list.map((f, i) => `<li class="pflag" data-flag="${i}" title="${esc(f.msg)}"><i aria-hidden="true"></i><span>${esc(f.sig)}</span><b>–</b></li>`).join("")}</ul></section>`;
}
function patchFlags(root, panel) {
  const list = P.meta?.flags?.[panel] || [];
  $$("[data-flag]", root).forEach((el) => {
    const f = list[+el.dataset.flag]; if (!f) return;
    const v = P.sig[f.msg + "." + f.sig]?.value;
    const on = v !== undefined && v !== null && v !== 0;
    const fault = /fault|err|fail|alarm|warn|protect|over|under|short|high|low/i.test(f.sig);
    const txt = v === undefined || v === null ? "–" : f.kind === "enum" ? (f.choices[Math.round(v)] ?? v) : v;
    el.className = "pflag" + (on ? (fault ? " on fault" : " on") : "");
    $("b", el).textContent = txt;
  });
}

/* ---------------------------------------------------------------- battery */
const LAYOUTS = ["auto", 4, 7, 8, 10, 12, 13, 14, 15, 16, 17, 20, 24, 28, 32, 48, 64, 96, 128, 144, 160, 192];
function countOf(key, max) {
  const v = val(key);
  return v !== undefined && v >= 1 && v <= 512 && !stale(key) ? Math.min(Math.round(v), Math.max(max, 1) * 4) : 0;
}
function bmsGeometry() {
  const cells = P.meta.cells || [], temps = P.meta.temps || [];
  const repC = countOf("bms.cell_count", cells.length), repT = countOf("bms.temp_count", temps.length);
  let seen = 0; cells.forEach((k, i) => { if (val(k) > 0) seen = i + 1; });       // highest cell with data
  let seenT = 0; temps.forEach((k, i) => { if (val(k) !== undefined && !stale(k)) seenT = i + 1; });
  const autoN = repC || seen || cells.length;
  const slots = P.layout === "auto" ? autoN : +P.layout;
  const tslots = repT || (temps.length > 8 && seenT ? seenT : temps.length);
  return { cells, temps, repC, repT, autoN, slots, tslots };
}
function renderBms() {
  const root = $("#bmsBody"); if (!root) return;
  if (!has("battery")) {
    if (root.dataset.sig !== "none") {
      root.dataset.sig = "none";
      root.innerHTML = `<div class="panel callout"><p>No battery signals yet. VCU Master loads a matching DBC from its library automatically when BMS frames arrive, or pick one in <a href="#" data-goto="updates">OTA and DBC</a>.</p></div>`;
    }
    return;
  }
  const G = bmsGeometry();
  const sig = [P.meta.version, G.slots, G.tslots, G.repC, G.autoN, P.view].join("|");
  if (root.dataset.sig !== sig) {
    const focusId = document.activeElement?.id, focusView = document.activeElement?.dataset?.view;
    root.dataset.sig = sig; buildBms(root, G);
    if (focusId && $("#" + focusId, root)) $("#" + focusId, root).focus();
    else if (focusView) $(`[data-view="${focusView}"]`, root)?.focus();
  }
  patchBms(root, G);
}
function buildBms(root, G) {
  const kpi = ["bms.pack_voltage", "bms.current", "bms.power", "bms.soh", "bms.remaining_cap", "bms.full_cap", "bms.cycles", "bms.chg_limit", "bms.dsg_limit", "bms.cell_count", "bms.temp_count"];
  const stats = ["bms.max_cell_v", "bms.min_cell_v", "bms.avg_cell_v", "bms.delta_cell_v", "bms.max_cell_v_id", "bms.min_cell_v_id", "bms.max_temp", "bms.min_temp", "bms.avg_temp", "bms.delta_temp"];
  const sw = ["bms.charge_mos", "bms.discharge_mos"];
  const opts = LAYOUTS.map((L) => `<option value="${L}" ${String(L) === String(P.layout) ? "selected" : ""}>${L === "auto" ? `Auto (${G.autoN}${G.repC ? ", reported by BMS" : ""})` : L + " cells"}</option>`).join("");
  let cellHtml = "", rows = "";
  for (let i = 0; i < G.slots; i++) {
    const k = G.cells[i];
    const state = !k ? "nodbc" : G.repC && i >= G.repC ? "unfitted" : "";
    const note = state === "nodbc" ? "not in DBC" : state === "unfitted" ? "not fitted" : "mV";
    const lbl = k ? roleOf(k).label : `C${i + 1}`;
    cellHtml += `<li class="pcell ${state ? "off" : ""}" data-cell="${i}"><small><span>${esc(lbl)}</span><u hidden></u><s hidden></s></small><div class="bar" aria-hidden="true"><i></i></div><b>${state ? "N/A" : "–"}</b><em>${note}</em></li>`;
    if (!state) rows += `<tr data-cell="${i}"><th scope="row">${esc(lbl)}</th><td class="num">–</td><td></td><td></td><td class="mono">${esc(roleOf(k).source)}</td></tr>`;
  }
  const tempHtml = G.temps.slice(0, G.tslots).map((k, i) => `<li class="pcell temp" data-temp="${i}"><small><span>${esc(roleOf(k).label)}</span></small><div class="bar" aria-hidden="true"><i></i></div><b>–</b><em>°C</em></li>`).join("");
  const grid = P.view !== "table";
  const seg = `<div class="seg" role="group" aria-label="Cell view"><button type="button" data-view="grid" aria-pressed="${grid}" class="${grid ? "on" : ""}">Grid</button><button type="button" data-view="table" aria-pressed="${!grid}" class="${grid ? "" : "on"}">Table</button></div>`;
  root.innerHTML = `
    <div class="grid g-bms">
      <section class="panel span2" aria-labelledby="bms-pack"><div class="ph"><h2 id="bms-pack">Pack</h2><span class="muted">DBC ${esc(P.meta.dbcs.join(", "))}</span></div>
        <div class="packrow">${ring("bms.soc", "State of charge")}<div class="ptiles">${kpi.map((k) => tile(k)).join("")}${sw.map((k) => tile(k)).join("")}</div></div></section>
      <section class="panel span2" aria-labelledby="bms-cells"><div class="ph"><h2 id="bms-cells">Cell voltages</h2>
          <div class="row wrap"><label class="inl" for="bmsLayout">Layout</label><select id="bmsLayout">${opts}</select>${seg}</div></div>
        <p class="muted pad slim" id="bmsCellInfo"></p>
        <ul class="pcells" aria-label="Cell voltages" ${grid ? "" : "hidden"}>${cellHtml}</ul>
        <div class="tablewrap" ${grid ? "hidden" : ""}><table class="tbl"><caption class="sr-only">Cell voltages</caption><thead><tr><th scope="col">Cell</th><th scope="col" class="r">Voltage (mV)</th><th scope="col">Mark</th><th scope="col">Balancing</th><th scope="col">Signal</th></tr></thead><tbody>${rows}</tbody></table></div>
      </section>
      ${G.tslots ? `<section class="panel span2" aria-labelledby="bms-temps"><div class="ph"><h2 id="bms-temps">Temperatures</h2><span class="muted">${G.tslots}${G.repT ? " reported by the BMS" : ""} of ${G.temps.length} in the DBC</span></div><ul class="pcells" aria-label="Temperatures">${tempHtml}</ul></section>` : ""}
      ${stats.some((k) => roleOf(k)) ? `<section class="panel span2" aria-labelledby="bms-stats"><div class="ph"><h2 id="bms-stats">Cell statistics</h2><span class="muted">as reported by the BMS</span></div><div class="ptiles">${stats.map((k) => tile(k)).join("")}</div></section>` : ""}
      ${flagsHtml("battery")}
    </div>`;
}
function patchBms(root, G) {
  patchRoles(root);
  patchFlags(root, "battery");
  const vs = [];
  for (let i = 0; i < Math.min(G.slots, G.cells.length); i++) {
    if (G.repC && i >= G.repC) continue;
    const v = val(G.cells[i]); if (v > 0 && !stale(G.cells[i])) vs.push(v);
  }
  const vmax = vs.length ? Math.max(...vs) : null, vmin = vs.length ? Math.min(...vs) : null;
  const avg = vs.length ? vs.reduce((a, b) => a + b, 0) / vs.length : null;
  $("#bmsCellInfo").textContent = vs.length
    ? `${vs.length} live of ${G.slots} shown · max ${Math.round(vmax * 1000)} mV · min ${Math.round(vmin * 1000)} mV · spread ${Math.round((vmax - vmin) * 1000)} mV · average ${Math.round(avg * 1000)} mV · faded = no update for 5 s`
    : `${G.slots} slot(s) · waiting for cell frames`;
  $$("[data-cell]", root).forEach((el) => {
    const i = +el.dataset.cell, k = G.cells[i];
    if (!k || (G.repC && i >= G.repC)) return;
    const v = val(k), st = stale(k) || !(v > 0);
    const mark = !st && vs.length > 1 && vmax !== vmin ? (v === vmax ? "MAX" : v === vmin ? "MIN" : "") : "";
    const bal = P.meta.balance?.[String(i + 1)], balOn = bal ? !!val(bal) : null;
    const mv = v === undefined ? "–" : String(Math.round(v * 1000));
    if (el.tagName === "TR") {
      el.children[1].textContent = mv; el.children[2].textContent = mark;
      el.children[3].textContent = balOn === null ? "" : balOn ? "active" : "off";
      el.classList.toggle("stale", st); return;
    }
    el.className = `pcell ${mark.toLowerCase()} ${st ? "stale" : ""}`;
    $("b", el).textContent = mv;
    const u = $("u", el); u.hidden = !mark; u.textContent = mark;
    const s = $("s", el); s.hidden = balOn === null; s.classList.toggle("on", !!balOn);
    if (balOn !== null) s.textContent = balOn ? "balancing" : "";
    $(".bar i", el).style.height = (v > 0 ? Math.max(2, Math.min(100, ((v - 2.5) / (4.3 - 2.5)) * 100)) : 0) + "%";
    el.title = `${roleOf(k).label}: ${mv} mV${mark ? " (" + mark + ")" : ""}${balOn ? ", balancing" : ""}\n${roleOf(k).source}`;
  });
  $$("[data-temp]", root).forEach((el) => {
    const k = G.temps[+el.dataset.temp], v = val(k);
    $("b", el).textContent = fmt(v, 1);
    $(".bar i", el).style.height = (v === undefined ? 0 : Math.max(2, Math.min(100, ((v + 20) / 110) * 100))) + "%";
    el.classList.toggle("hot", v > 50); el.classList.toggle("stale", stale(k));
    el.title = `${roleOf(k).label}: ${fmt(v, 1)} °C\n${roleOf(k).source}`;
  });
}

/* ---------------------------------------------------------------- motor */
function renderMcu() {
  const root = $("#mcuBody"); if (!root) return;
  if (!has("motor")) {
    if (root.dataset.sig !== "none") {
      root.dataset.sig = "none";
      root.innerHTML = `<div class="panel callout"><p>No motor-controller signals yet. A matching MCU DBC is loaded automatically when its frames arrive, or pick one in <a href="#" data-goto="updates">OTA and DBC</a>.</p></div>`;
    }
    return;
  }
  const sig = String(P.meta.version);
  if (root.dataset.sig !== sig) {
    root.dataset.sig = sig;
    const gauges = ["mcu.rpm", "mcu.torque", "veh.speed"].filter((k) => roleOf(k));
    const rest = ["mcu.throttle", "mcu.brake", "mcu.gear", "mcu.mode", "mcu.dc_voltage", "mcu.dc_current", "mcu.phase_current", "mcu.motor_temp", "mcu.ctrl_temp", "veh.odometer", "veh.trip"];
    root.innerHTML = `
      <div class="grid g-bms">
        <section class="panel span2" aria-labelledby="mcu-h"><div class="ph"><h2 id="mcu-h">Motor</h2><span class="muted">DBC ${esc(P.meta.dbcs.join(", "))}</span></div><div class="packrow">${gauges.map((k) => ring(k)).join("")}</div></section>
        <section class="panel span2" aria-labelledby="mcu-p"><div class="ph"><h2 id="mcu-p">Drive, power and thermal</h2></div><div class="ptiles">${rest.map((k) => tile(k)).join("")}</div></section>
        ${flagsHtml("motor")}
      </div>`;
  }
  patchRoles(root);
  patchFlags(root, "motor");
}

/* ================================================================ command feedback */
function pending(kind, id, want) {
  P.pend[kind + ":" + id] = { t: Date.now(), want, state: "pending" };
  addCmdLog(`sent ${kind === "mod" ? "MODULE " + id + " " + (want ? "ENABLE" : "DISABLE") : "GPIO " + id + " " + (want.dir ? "OUT " + (want.state ? "HIGH" : "LOW") : "IN")} - waiting for S32K`, "pend");
  paintPending();
}
function onAck(f) {
  const cmd = f.cmd, ok = f.result === 0;
  let key = null, label = "";
  if (cmd === 1) { key = "mod:" + MODS[f.gpio_id]; label = `MODULE ${MODS[f.gpio_id]} -> ${f.state ? "ENABLED" : "DISABLED"}`; }
  else if (cmd === 2) { key = "gpio:" + f.gpio_id; label = `GPIO ${f.gpio_id} read-back ${f.state ? "HIGH" : "LOW"}`; }
  else { label = `command 0x${(cmd || 0).toString(16)}`; }
  addCmdLog(`S32K ACK ${label} ${ok ? "OK" : "FAILED (code " + f.result + ")"}`, ok ? "ok" : "err");
  if (key && P.pend[key]) {
    P.pend[key].state = ok ? "confirmed" : "failed";
    P.pend[key].t = Date.now();
    paintPending();
  }
}
function paintPending() {
  const t = Date.now();
  for (const [key, p] of Object.entries(P.pend)) {
    if (p.state === "pending" && t - p.t > 2000) { p.state = "timeout"; p.t = t; addCmdLog(`no ACK from S32K for ${key.replace(":", " ")} within 2 s`, "err"); toast(`No confirmation from the S32K for ${key.replace(":", " ")}`, true); }
    if ((p.state === "confirmed" && t - p.t > 4000) || ((p.state === "failed" || p.state === "timeout") && t - p.t > 8000)) { delete P.pend[key]; }
  }
  $$("#modList .mod").forEach((box) => {
    const m = $("[data-mod]", box)?.dataset.mod;
    const p = P.pend["mod:" + m];
    box.classList.remove("c-pending", "c-confirmed", "c-failed", "c-timeout");
    if (p) box.classList.add("c-" + p.state);
  });
  $$("#gpioTable tbody tr").forEach((tr) => {
    const p = P.pend["gpio:" + tr.dataset.g];
    tr.classList.remove("c-pending", "c-confirmed", "c-failed", "c-timeout");
    if (p) tr.classList.add("c-" + p.state);
  });
}
function addCmdLog(text, cls) {
  const d = new Date();
  P.cmdLog.unshift({ ts: d.toTimeString().slice(0, 8) + "." + String(d.getMilliseconds()).padStart(3, "0"), text, cls });
  P.cmdLog.length = Math.min(P.cmdLog.length, 40);
  const box = $("#cmdLog");
  if (box) box.innerHTML = P.cmdLog.map((e) => `<li class="${e.cls || ""}"><time>${e.ts}</time>${esc(e.text)}</li>`).join("");
}

/* ================================================================ IMU movement + integrity */
function renderMove(st) {
  const imu = st?.latest?.IMU, box = $("#moveTiles"); if (!box) return;
  if (!imu || imu.pos_x_mm === undefined) {
    box.innerHTML = `<p class="muted">Needs S32K firmware V0.0073 or newer (IMU displacement fields).</p>`;
    return;
  }
  const t = [["X", imu.pos_x_mm, "mm"], ["Y", imu.pos_y_mm, "mm"], ["Z (up)", imu.pos_z_mm, "mm"],
    ["Distance travelled", imu.dist_mm, "mm"], ["Speed", imu.speed_mms, "mm/s"],
    ["Roll", imu.roll_fw, "°"], ["Pitch", imu.pitch_fw, "°"], ["Yaw", imu.yaw_fw, "°"]];
  box.innerHTML = t.map(([k, v, u]) => `<div class="ptile"><small>${k}</small><b>${fmt(v, 1)}</b><i>${u}</i></div>`).join("");
  $("#moveState").innerHTML = `<span class="badge ${imu.moving ? "run" : "ok"}">${imu.moving ? "MOVING" : "STILL"}</span> tracking for ${fmtDur(imu.imu_up_ms)}`;
  const last = P.track[P.track.length - 1];
  if (!last || Math.hypot(imu.pos_x_mm - last[0], imu.pos_y_mm - last[1]) > 0.5) P.track.push([imu.pos_x_mm, imu.pos_y_mm]);
  if (P.track.length > 600) P.track.shift();
  const xs = P.track.map((p) => p[0]), ys = P.track.map((p) => p[1]);
  const span = Math.max(50, ...xs.map(Math.abs), ...ys.map(Math.abs)) * 1.15;
  const pts = P.track.map(([x, y]) => `${(x / span) * 90},${(-y / span) * 90}`).join(" ");
  $("#moveTrack").innerHTML = `<line x1="-95" x2="95" y1="0" y2="0" class="ax"/><line y1="-95" y2="95" x1="0" x2="0" class="ax"/><text x="92" y="-4" class="lb">+X</text><text x="4" y="-86" class="lb">+Y</text><text x="-94" y="92" class="lb">scale ±${Math.round(span)} mm</text><polyline points="${pts}" class="path"/>${P.track.length ? `<circle cx="${(xs[xs.length - 1] / span) * 90}" cy="${(-ys[ys.length - 1] / span) * 90}" r="3.5" class="now"/>` : ""}`;
}
function fmtDur(ms) {
  if (!ms) return "0 s";
  const s = Math.floor(ms / 1000), h = Math.floor(s / 3600), m = Math.floor((s % 3600) / 60);
  return h ? `${h} h ${m} min` : m ? `${m} min ${s % 60} s` : `${s} s`;
}
function renderIntegrity(st) {
  const I = st?.integrity, box = $("#integKv"); if (!I || !box) return;
  const rows = [
    ["S32K frames received", I.s32_frames], ["S32K frames lost (sequence gaps)", `${I.s32_lost} (${I.s32_loss_pct} %)`],
    ["CAN1 received / S32K counted", `${I.can["1"].received} / ${I.can["1"].s32_counted}${I.can["1"].missing ? ` (missing ${I.can["1"].missing})` : ""}`],
    ["CAN2 received / S32K counted", `${I.can["2"].received} / ${I.can["2"].s32_counted}${I.can["2"].missing ? ` (missing ${I.can["2"].missing})` : ""}`],
  ];
  for (const [k, v] of Object.entries(I.bridge || {})) rows.push(["ESP32 " + k.replace(/_/g, " "), v]);
  box.innerHTML = rows.map(([k, v]) => `<dt>${esc(k)}</dt><dd>${esc(v)}</dd>`).join("");
  const bad = I.s32_lost > 0 || I.can["1"].missing > 0 || I.can["2"].missing > 0;
  $("#integBadge").className = "badge " + (I.s32_frames ? (bad ? "err" : "ok") : "");
  $("#integBadge").textContent = I.s32_frames ? (bad ? "LOSS DETECTED" : "NO LOSS") : "NO DATA";
}

/* ================================================================ hooks */
window.VCUProducts = {
  onMeta(m) { P.meta = m; renderBms(); renderMcu(); },
  onNotice(text) { toast(text); addCmdLog(text, "ok"); },
  refresh() { renderBms(); renderMcu(); },
  onRoles(r) { P.values = r.values || {}; P.age = r.age || {}; if (V.S.win === "bms") renderBms(); if (V.S.win === "mcu") renderMcu(); },
  onSignals(list) { const o = {}; for (const s of list || []) { const k = s.message + "." + s.signal; if (!o[k] || (s.t || 0) > (o[k].t || 0)) o[k] = s; } P.sig = o; },
  onRec(r) {
    if (r.type === "CMD_ACK" && !(r.raw || "").includes("ESP")) onAck(r.fields);
    else if (r.type === "LOG" && /^\[(CMD|GPIO|IMU)\]/.test(r.fields?.text || "")) addCmdLog("S32K " + r.fields.text, "s32");
  },
  onState(st) { renderMove(st); renderIntegrity(st); paintPending(); },
  pending,
};

document.addEventListener("change", (e) => {
  if (e.target.id === "bmsLayout") { P.layout = e.target.value; lsSet("vcum.bmsLayout", P.layout); renderBms(); $("#bmsLayout")?.focus(); }
});
document.addEventListener("click", (e) => {
  const vb = e.target.closest("[data-view]");
  if (vb) { P.view = vb.dataset.view; lsSet("vcum.bmsView", P.view); renderBms(); $(`[data-view="${P.view}"]`)?.focus(); return; }
  if (e.target.id === "imuZero") act(() => POST("/api/cmd/imu_zero"), "IMU zero sent").then(() => { P.track = []; });
});
setInterval(paintPending, 500);
})();
