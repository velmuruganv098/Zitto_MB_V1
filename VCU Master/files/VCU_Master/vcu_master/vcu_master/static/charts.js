/* charts.js - dependency-free canvas strip charts (works offline on the bench) */
(function () {
  const css = (n) => getComputedStyle(document.documentElement).getPropertyValue(n).trim();

  function niceNum(range, round) {
    const exp = Math.floor(Math.log10(range || 1));
    const f = range / Math.pow(10, exp);
    let nf;
    if (round) nf = f < 1.5 ? 1 : f < 3 ? 2 : f < 7 ? 5 : 10;
    else nf = f <= 1 ? 1 : f <= 2 ? 2 : f <= 5 ? 5 : 10;
    return nf * Math.pow(10, exp);
  }

  function fmt(v) {
    const a = Math.abs(v);
    if (a >= 1e5) return (v / 1000).toFixed(0) + "k";
    if (Number.isInteger(v)) return String(v);
    if (a >= 100) return v.toFixed(0);
    if (a >= 10) return v.toFixed(1);
    return v.toFixed(2);
  }

  function tick(v, step) {
    if (Math.abs(v) >= 1e5) return (v / 1000).toFixed(Math.max(0, -Math.floor(Math.log10(step / 1000)))) + "k";
    const d = Math.max(0, -Math.floor(Math.log10(step)));
    return v.toFixed(Math.min(d, 4));
  }

  class StripChart {
    /* series: [{key, label, color: "--css-var" or "#hex"}] */
    constructor(canvas, series, opts = {}) {
      this.c = canvas;
      this.ctx = canvas.getContext("2d");
      this.series = series;
      this.data = {};           // key -> [[t, v], ...]
      this.windowS = opts.windowS || 30;
      this.min = opts.min; this.max = opts.max;
      this.maxPts = opts.maxPts || 4000;
      series.forEach((s) => (this.data[s.key] = []));
      this.dirty = true;
      new ResizeObserver(() => { this.dirty = true; }).observe(canvas);
    }
    setSeries(series) {
      this.series = series;
      series.forEach((s) => { if (!this.data[s.key]) this.data[s.key] = []; });
      this.dirty = true;
    }
    push(key, t, v) {
      if (v === null || v === undefined || Number.isNaN(v)) return;
      const arr = this.data[key] || (this.data[key] = []);
      arr.push([t, v]);
      if (arr.length > this.maxPts) arr.splice(0, arr.length - this.maxPts);
      this.dirty = true;
    }
    clear() { Object.keys(this.data).forEach((k) => (this.data[k] = [])); this.dirty = true; }
    color(s) { return s.color.startsWith("--") ? css(s.color) : s.color; }
    draw(now) {
      const c = this.c, ctx = this.ctx;
      const dpr = window.devicePixelRatio || 1;
      const w = c.clientWidth, h = c.clientHeight;
      if (!w || !h) return;
      if (c.width !== Math.round(w * dpr) || c.height !== Math.round(h * dpr)) {
        c.width = Math.round(w * dpr); c.height = Math.round(h * dpr);
      }
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      ctx.clearRect(0, 0, w, h);
      const t1 = now, t0 = now - this.windowS;
      let lo = Infinity, hi = -Infinity;
      for (const s of this.series) {
        for (const [t, v] of this.data[s.key] || []) {
          if (t < t0) continue;
          if (v < lo) lo = v; if (v > hi) hi = v;
        }
      }
      if (this.min !== undefined) lo = Math.min(lo, this.min);
      if (this.max !== undefined) hi = Math.max(hi, this.max);
      const empty = !isFinite(lo);
      if (empty) { lo = 0; hi = 1; }
      if (hi - lo < 1e-9) { hi += 1; lo -= 1; }
      const pad = (hi - lo) * 0.08; lo -= pad; hi += pad;
      const step = niceNum(niceNum(hi - lo, false) / 4, true);
      const gl = Math.floor(lo / step) * step;

      const L = 46, R = 6, T = 6, B = 18;
      const pw = w - L - R, ph = h - T - B;
      const X = (t) => L + ((t - t0) / (t1 - t0)) * pw;
      const Y = (v) => T + (1 - (v - lo) / (hi - lo)) * ph;

      ctx.font = `11px ${css("--mono") || "monospace"}`;
      ctx.fillStyle = css("--ink-3");
      ctx.strokeStyle = css("--rule-2");
      ctx.lineWidth = 1;
      ctx.textAlign = "right"; ctx.textBaseline = "middle";
      for (let v = gl; v <= hi; v += step) {
        if (v < lo) continue;
        const y = Math.round(Y(v)) + 0.5;
        ctx.beginPath(); ctx.moveTo(L, y); ctx.lineTo(w - R, y); ctx.stroke();
        ctx.fillText(tick(v, step), L - 6, y);
      }
      if (lo < 0 && hi > 0) {
        ctx.strokeStyle = css("--rule");
        const y = Math.round(Y(0)) + 0.5;
        ctx.beginPath(); ctx.moveTo(L, y); ctx.lineTo(w - R, y); ctx.stroke();
      }
      ctx.textAlign = "center"; ctx.textBaseline = "top";
      const tstep = this.windowS <= 30 ? 5 : this.windowS <= 60 ? 10 : 30;
      for (let s = 0; s <= this.windowS; s += tstep) {
        ctx.fillText(s === 0 ? "now" : `-${s}s`, X(t1 - s), h - B + 4);
      }
      if (empty) {
        ctx.textAlign = "center"; ctx.textBaseline = "middle";
        ctx.fillText("Waiting for data", L + pw / 2, T + ph / 2);
        return;
      }
      ctx.save();
      ctx.beginPath(); ctx.rect(L, T, pw, ph); ctx.clip();
      ctx.lineWidth = 1.6; ctx.lineJoin = "round";
      for (const s of this.series) {
        const arr = this.data[s.key] || [];
        ctx.strokeStyle = this.color(s);
        ctx.beginPath();
        let started = false;
        for (let i = 0; i < arr.length; i++) {
          const [t, v] = arr[i];
          if (t < t0 - 1) continue;
          const x = X(t), y = Y(v);
          if (!started) { ctx.moveTo(x, y); started = true; } else ctx.lineTo(x, y);
        }
        ctx.stroke();
      }
      ctx.restore();
      // legend top-left
      let lx = L + 8;
      ctx.textAlign = "left"; ctx.textBaseline = "top";
      for (const s of this.series) {
        if (!s.label) continue;
        const arr = this.data[s.key] || [];
        const last = arr.length ? arr[arr.length - 1][1] : null;
        const txt = `${s.label} ${last === null ? "–" : fmt(last)}`;
        ctx.fillStyle = this.color(s);
        ctx.fillRect(lx, T + 7, 10, 2);
        ctx.fillStyle = css("--ink-2");
        ctx.fillText(txt, lx + 14, T + 2);
        lx += ctx.measureText(txt).width + 30;
      }
      this.dirty = false;
    }
  }

  /* Trace ribbon: one lane per channel, a tick per message - shows at a
     glance which data sources are alive. */
  class Ribbon {
    constructor(canvas, lanes) {
      this.c = canvas; this.ctx = canvas.getContext("2d");
      this.lanes = lanes; // [{tag, color}]
      this.ticks = [];    // [t, laneIndex]
      this.span = 20;
    }
    hit(tag, t) {
      const i = this.lanes.findIndex((l) => l.tag === tag);
      if (i >= 0) this.ticks.push([t, i]);
      if (this.ticks.length > 3000) this.ticks.splice(0, 1000);
    }
    draw(now) {
      const c = this.c, ctx = this.ctx, dpr = window.devicePixelRatio || 1;
      const w = c.clientWidth, h = c.clientHeight;
      if (!w) return;
      if (c.width !== Math.round(w * dpr)) { c.width = Math.round(w * dpr); c.height = Math.round(h * dpr); }
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      ctx.clearRect(0, 0, w, h);
      const n = this.lanes.length, lh = h / n, labW = 40;
      ctx.font = `10px ${css("--mono") || "monospace"}`;
      ctx.textBaseline = "middle";
      this.lanes.forEach((l, i) => {
        ctx.fillStyle = css("--ink-3");
        ctx.fillText(l.tag, 0, i * lh + lh / 2);
        ctx.fillStyle = css("--rule-2");
        ctx.fillRect(labW, i * lh + lh / 2, w - labW, 1);
      });
      const t0 = now - this.span;
      this.ticks = this.ticks.filter((k) => k[0] >= t0);
      for (const [t, i] of this.ticks) {
        const x = labW + ((t - t0) / this.span) * (w - labW);
        ctx.fillStyle = css(this.lanes[i].color);
        ctx.fillRect(Math.round(x), i * lh + 1, 2, lh - 2);
      }
    }
  }

  window.VCUCharts = { StripChart, Ribbon, fmt };
})();
