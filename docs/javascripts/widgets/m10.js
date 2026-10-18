/* M10 · Robot control widgets: m10-pid (one joint with gravity, PID + feedforward) and
   m10-impedance (virtual spring–damper pushing a wall). RK4 integration, every number live. */
(function () {
  "use strict";

  // ---------- one joint: I θ̈ = τ − b θ̇ − mgl·sin θ   (θ = 0 hangs straight down) ----------
  const P0 = { I: 0.05, b: 0.02, mgl: 0.6 }; // example values: ~0.2 kg at 0.3 m
  function simPID(o) {
    const { Kp, Ki, Kd, thd, grav, ff, tmax, aw, T = 4, dt = 0.001 } = o;
    const I = P0.I, b = P0.b, G = grav ? P0.mgl : 0;
    let th = 0, w = 0, z = 0; // z = ∫e dt
    const ts = [], ths = [], taus = [];
    const ctrl = (th, w, z) => {
      const e = thd - th;
      let u = Kp * e + Ki * z - Kd * w + (ff ? G * Math.sin(thd) : 0); // D on measurement: no derivative kick
      const us = Math.max(-tmax, Math.min(tmax, u));
      return [us, u !== us];
    };
    const f = (th, w, tau) => (tau - b * w - G * Math.sin(th)) / I;
    const n = Math.round(T / dt);
    for (let i = 0; i <= n; i++) {
      const [tau, sat] = ctrl(th, w, z);
      if (i % 5 === 0) { ts.push(i * dt); ths.push(th); taus.push(tau); }
      // RK4 on (θ, ω); torque held over the step (zero-order hold, like a digital controller)
      const k1t = w, k1w = f(th, w, tau);
      const k2t = w + 0.5 * dt * k1w, k2w = f(th + 0.5 * dt * k1t, w + 0.5 * dt * k1w, tau);
      const k3t = w + 0.5 * dt * k2w, k3w = f(th + 0.5 * dt * k2t, w + 0.5 * dt * k2w, tau);
      const k4t = w + dt * k3w, k4w = f(th + dt * k3t, w + dt * k3w, tau);
      th += (dt / 6) * (k1t + 2 * k2t + 2 * k3t + k4t);
      w += (dt / 6) * (k1w + 2 * k2w + 2 * k3w + k4w);
      if (!(aw && sat)) z += (thd - th) * dt; // clamping anti-windup: freeze ∫e while saturated
    }
    // metrics
    let peak = 0; for (const v of ths) peak = Math.max(peak, v);
    const fin = ths[ths.length - 1];
    const os = thd !== 0 ? Math.max(0, (100 * (peak - thd)) / Math.abs(thd)) : 0;
    let tset = null; const band = 0.02 * Math.abs(thd);
    for (let i = ths.length - 1; i >= 0; i--) if (Math.abs(ths[i] - thd) > band) { tset = i + 1 < ts.length ? ts[i + 1] : null; break; }
    if (tset === null && Math.abs(ths[0] - thd) <= band) tset = 0;
    let tmaxUsed = 0; for (const v of taus) tmaxUsed = Math.max(tmaxUsed, Math.abs(v));
    return { ts, ths, taus, os, tset, ess: thd - fin, tmaxUsed };
  }
  // linearised PD theory about θd: I ë + (b+Kd) ė + (Kp + mgl·cos θd) e = 0 (gravity not fed forward)
  function theory(Kp, Kd, thd, grav, ff) {
    // gravity's slope about θd adds stiffness mgl·cos θd (feedforward cancels the offset, not the slope)
    const k = Kp + (grav ? P0.mgl * Math.cos(thd) : 0);
    const c = P0.b + Kd, wn = Math.sqrt(Math.max(k, 0) / P0.I), z = c / (2 * Math.sqrt(Math.max(k, 1e-12) * P0.I));
    const Mp = z < 1 ? 100 * Math.exp((-Math.PI * z) / Math.sqrt(1 - z * z)) : 0;
    const ts = z * wn > 0 ? 4 / (z * wn) : Infinity;
    const ess = grav && !ff ? null : 0;
    return { k, wn, z, Mp, ts, ess };
  }

  // ---------- 1D impedance: m ẍ = K(xd − x) − B ẋ − F_wall,  F_wall = kw·(x − xw)⁺ + bw·ẋ when in contact ----------
  function simImp(o) {
    const { K, B, xd, m = 1, xw = 0.1, kw = 5000, bw = 20, T = 2, dt = 0.0002 } = o;
    let x = 0, v = 0; const ts = [], xs = [], Fs = [];
    const Fw = (x, v) => (x > xw ? Math.max(0, kw * (x - xw) + bw * v) : 0);
    const acc = (x, v) => (K * (xd - x) - B * v - Fw(x, v)) / m;
    const n = Math.round(T / dt);
    for (let i = 0; i <= n; i++) {
      if (i % 25 === 0) { ts.push(i * dt); xs.push(x); Fs.push(Fw(x, v)); }
      const a1 = acc(x, v), v1 = v;
      const a2 = acc(x + 0.5 * dt * v1, v + 0.5 * dt * a1), v2 = v + 0.5 * dt * a1;
      const a3 = acc(x + 0.5 * dt * v2, v + 0.5 * dt * a2), v3 = v + 0.5 * dt * a2;
      const a4 = acc(x + dt * v3, v + dt * a3), v4 = v + dt * a3;
      x += (dt / 6) * (v1 + 2 * v2 + 2 * v3 + v4); v += (dt / 6) * (a1 + 2 * a2 + 2 * a3 + a4);
    }
    let Fpk = 0; for (const F of Fs) Fpk = Math.max(Fpk, F);
    return { ts, xs, Fs, Fpk, Fss: Fs[Fs.length - 1], xss: xs[xs.length - 1], xw, kw, m };
  }

  const API = { simPID, theory, simImp, P0 };
  if (typeof module !== "undefined" && module.exports) module.exports = API;

  const L = typeof window !== "undefined" && window.Lab; if (!L) return;
  const { colors, fmt, el, slider, canvas, deg } = L;

  // simple time plot shared by both widgets
  function plot(ctx, w, h, ts, series, opt) {
    const k = colors(), x0 = 40, y0 = 8, pw = w - x0 - 8, ph = h - y0 - 22;
    let lo = opt.lo, hi = opt.hi;
    if (lo === undefined) { lo = Infinity; hi = -Infinity; for (const s of series) for (const v of s.y) { lo = Math.min(lo, v); hi = Math.max(hi, v); } for (const r of opt.refs || []) { lo = Math.min(lo, r.v); hi = Math.max(hi, r.v); } const pad = 0.08 * (hi - lo || 1); lo -= pad; hi += pad; }
    const T = ts[ts.length - 1], X = (t) => x0 + (pw * t) / T, Y = (v) => y0 + ph - (ph * (v - lo)) / (hi - lo || 1);
    ctx.clearRect(0, 0, w, h);
    ctx.strokeStyle = k.light; ctx.lineWidth = 1; ctx.strokeRect(x0 + 0.5, y0 + 0.5, pw, ph);
    if (lo < 0 && hi > 0) { ctx.beginPath(); ctx.moveTo(x0, Y(0)); ctx.lineTo(x0 + pw, Y(0)); ctx.stroke(); }
    for (const r of opt.refs || []) { ctx.strokeStyle = r.c; ctx.setLineDash([4, 3]); ctx.beginPath(); ctx.moveTo(x0, Y(r.v)); ctx.lineTo(x0 + pw, Y(r.v)); ctx.stroke(); ctx.setLineDash([]); }
    for (const b of opt.bands || []) { ctx.fillStyle = b.c; ctx.fillRect(x0, Y(b.hi), pw, Y(b.lo) - Y(b.hi)); }
    for (const s of series) { ctx.strokeStyle = s.c; ctx.lineWidth = 2; ctx.beginPath(); s.y.forEach((v, i) => (i ? ctx.lineTo(X(ts[i]), Y(v)) : ctx.moveTo(X(ts[i]), Y(v)))); ctx.stroke(); }
    if (opt.vline != null) { ctx.strokeStyle = k.green; ctx.setLineDash([2, 3]); ctx.beginPath(); ctx.moveTo(X(opt.vline), y0); ctx.lineTo(X(opt.vline), y0 + ph); ctx.stroke(); ctx.setLineDash([]); }
    ctx.fillStyle = k.fg; ctx.font = "12px sans-serif";
    ctx.fillText(fmt(hi, opt.d ?? 2), 2, y0 + 10); ctx.fillText(fmt(lo, opt.d ?? 2), 2, y0 + ph);
    ctx.fillText(opt.title, x0 + 6, y0 + 14); ctx.fillText("0", x0 - 3, h - 6); ctx.fillText(fmt(T, 0) + " s", x0 + pw - 22, h - 6);
  }

  L.register("m10-pid", function (root) {
    const ui = el("div", { class: "lab-controls" });
    const pics = el("div", { class: "lab-pics" });
    root.append(ui, pics);
    const A = canvas(pics, 200, 200);
    const plots = el("div"); plots.style.flex = "1 1 300px"; plots.style.maxWidth = "420px"; pics.appendChild(plots);
    const P1 = canvas(plots, 420, 150), P2 = canvas(plots, 420, 110);
    const out = el("div", { class: "lab-readout" }); root.appendChild(out);
    const kp = slider(ui, "Kp", 0, 20, 0.1, 4, draw, " N·m/rad");
    const kd = slider(ui, "Kd", 0, 2, 0.01, 0.3, draw, " N·m·s");
    const ki = slider(ui, "Ki", 0, 20, 0.1, 0, draw, " N·m/(rad·s)");
    const thd = slider(ui, "target θd", -120, 120, 1, 60, draw, "°");
    const tm = slider(ui, "torque limit", 0.2, 5, 0.1, 5, draw, " N·m");
    const row = el("div", { class: "lab-row" });
    const mk = (txt, on) => { const c = el("input", { type: "checkbox" }); c.checked = on; const l = el("label", {}); l.style.marginRight = "0.8rem"; l.append(c, document.createTextNode(" " + txt)); row.appendChild(l); c.addEventListener("change", draw); return c; };
    row.appendChild(el("span", { class: "lab-label" }, "Switches"));
    const cg = mk("gravity", true), cf = mk("gravity feedforward", false), ca = mk("anti-windup", false);
    ui.appendChild(row);

    function draw() {
      const k = colors();
      const o = { Kp: +kp.value, Ki: +ki.value, Kd: +kd.value, thd: deg(+thd.value), grav: cg.checked, ff: cf.checked, tmax: +tm.value, aw: ca.checked };
      const r = simPID(o), th = theory(o.Kp, o.Kd, o.thd, o.grav, o.ff);
      // arm picture at the final state, ghost at the target
      const c = A.ctx, cx = 100, cy = 85, Lp = 70; c.clearRect(0, 0, 200, 200);
      const tip = (a) => [cx + Lp * Math.sin(a), cy + Lp * Math.cos(a)];
      c.strokeStyle = k.light; c.setLineDash([4, 3]); c.lineWidth = 3; c.beginPath(); c.moveTo(cx, cy); c.lineTo(...tip(o.thd)); c.stroke(); c.setLineDash([]);
      // trace of the tip over time
      c.strokeStyle = "rgba(255,179,0,0.5)"; c.lineWidth = 1.2; c.beginPath(); r.ths.forEach((a, i) => (i ? c.lineTo(...tip(a)) : c.moveTo(...tip(a)))); c.stroke();
      const thf = r.ths[r.ths.length - 1];
      c.strokeStyle = k.primary; c.lineWidth = 6; c.lineCap = "round"; c.beginPath(); c.moveTo(cx, cy); c.lineTo(...tip(thf)); c.stroke();
      c.fillStyle = k.fg; c.beginPath(); c.arc(cx, cy, 5, 0, 7); c.fill();
      c.fillStyle = k.accent; c.beginPath(); c.arc(...tip(thf), 7, 0, 7); c.fill();
      c.fillStyle = k.fg; c.font = "12px sans-serif"; c.fillText("dashed = target", 6, 16); c.fillText("θ = 0 hangs down", 6, 192);
      if (o.grav) { c.strokeStyle = k.red; c.lineWidth = 1.5; c.beginPath(); c.moveTo(170, 120); c.lineTo(170, 150); c.lineTo(165, 143); c.moveTo(170, 150); c.lineTo(175, 143); c.stroke(); c.fillStyle = k.red; c.fillText("g", 178, 140); }
      // plots
      const D = (v) => (v * 180) / Math.PI;
      plot(P1.ctx, 420, 150, r.ts, [{ y: r.ths.map(D), c: k.primary }], { title: "θ(t) [°]", refs: [{ v: +thd.value, c: k.accent }], bands: [{ lo: +thd.value - 0.02 * Math.abs(+thd.value), hi: +thd.value + 0.02 * Math.abs(+thd.value), c: "rgba(67,160,71,0.15)" }], vline: r.tset, d: 0 });
      plot(P2.ctx, 420, 110, r.ts, [{ y: r.taus, c: k.red }], { title: "τ(t) [N·m]", refs: [{ v: o.tmax, c: k.light }, { v: -o.tmax, c: k.light }], d: 1 });
      const sat = r.tmaxUsed >= o.tmax - 1e-9;
      const gNote = o.grav && !o.ff ? (o.Ki > 0 ? "integral removes the gravity offset" : `PD alone sags: predicted e<sub>ss</sub> ≈ mgl·sin θ<sub>ss</sub>/Kp`) : "no steady gravity load on the error";
      out.innerHTML =
        `<b>Theory (linearised PD about θd):</b> k = Kp${o.grav ? " + mgl·cos θd" : ""} = ${fmt(th.k, 2)} N·m/rad, ω<sub>n</sub> = √(k/I) = <b>${fmt(th.wn, 2)}</b> rad/s, ζ = (b+Kd)/(2√(kI)) = <b>${fmt(th.z, 3)}</b> → ` +
        `overshoot ${th.z < 1 ? fmt(th.Mp, 1) + "%" : "0% (no oscillation)"}, t<sub>s</sub> ≈ 4/(ζω<sub>n</sub>) = ${isFinite(th.ts) ? fmt(th.ts, 2) + " s" : "∞"}.<br>` +
        `<b>Simulation (RK4, 1 ms):</b> overshoot <b>${fmt(r.os, 1)}%</b>, 2% settling <b>${r.tset == null ? "not within 4 s" : fmt(r.tset, 2) + " s"}</b>, final error <b>${fmt(D(r.ess), 2)}°</b> (${gNote}).` +
        `${sat ? ` <b style="color:${k.red}">Torque saturated at ±${fmt(o.tmax, 1)} N·m</b>${o.Ki > 0 && !o.aw ? " — watch the integral wind up." : "."}` : ""}<br>` +
        `<i>Example values:</i> I = ${P0.I} kg·m², b = ${P0.b} N·m·s, mgl = ${P0.mgl} N·m. Theory ignores Ki and saturation; the simulation does not.`;
    }
    draw();
  });

  L.register("m10-impedance", function (root) {
    const ui = el("div", { class: "lab-controls" });
    const pics = el("div", { class: "lab-pics" });
    root.append(ui, pics);
    const A = canvas(pics, 420, 110), P1 = canvas(pics, 420, 130);
    const out = el("div", { class: "lab-readout" }); root.appendChild(out);
    const K = slider(ui, "stiffness K", 10, 2000, 10, 300, draw, " N/m");
    const B = slider(ui, "damping B", 0, 100, 1, 20, draw, " N·s/m");
    const xd = slider(ui, "target past wall", 0, 0.1, 0.005, 0.05, draw, " m");
    function draw() {
      const k = colors(), xw = 0.1;
      const r = simImp({ K: +K.value, B: +B.value, xd: xw + +xd.value, xw });
      const c = A.ctx; c.clearRect(0, 0, 420, 110);
      const S = 2400, X = (x) => 40 + S * x, wx = X(xw);
      // wall
      c.fillStyle = k.light; c.fillRect(wx, 10, 14, 90); c.fillStyle = k.fg; c.font = "12px sans-serif"; c.fillText("wall (5000 N/m)", wx + 18, 24);
      // target ghost
      const gx = X(xw + +xd.value); c.strokeStyle = k.accent; c.setLineDash([4, 3]); c.beginPath(); c.moveTo(gx, 15); c.lineTo(gx, 95); c.stroke(); c.setLineDash([]); c.fillStyle = k.accent; c.fillText("x_d", gx + 3, 108);
      // spring from target anchor to block (virtual), drawn above
      const bx = X(r.xss) - 26; // block right face = x
      const zig = (x0, x1, y) => { c.beginPath(); c.moveTo(x0, y); const n = 10; for (let i = 1; i < n; i++) c.lineTo(x0 + ((x1 - x0) * i) / n, y + (i % 2 ? -6 : 6)); c.lineTo(x1, y); c.stroke(); };
      c.strokeStyle = k.primary; c.lineWidth = 1.5; zig(10, bx, 45);
      c.fillStyle = k.fg; c.fillText("virtual spring K + damper B", 12, 30);
      c.fillStyle = k.primary; c.fillRect(bx, 35, 26, 40);
      c.fillStyle = k.fg; c.fillText("x = " + fmt(r.xss * 1000, 1) + " mm", bx - 10, 92);
      // plots
      plot(P1.ctx, 420, 130, r.ts, [{ y: r.Fs, c: k.red }], { title: "contact force F(t) [N]", refs: [{ v: (+K.value) * (+xd.value), c: k.accent }], lo: 0, hi: Math.max(1, r.Fpk * 1.1), d: 1 });
      const Kc = +K.value + r.kw, zf = +B.value / (2 * Math.sqrt(+K.value * r.m)), zc = (+B.value + 20) / (2 * Math.sqrt(Kc * r.m));
      out.innerHTML =
        `Steady force <b>${fmt(r.Fss, 2)} N</b> ≈ K·Δx = ${fmt(+K.value * +xd.value, 2)} N (exact series springs: K·kw/(K+kw)·Δx = ${fmt((+K.value * r.kw) / (+K.value + r.kw) * +xd.value, 2)} N). Peak impact force <b>${fmt(r.Fpk, 1)} N</b>.<br>` +
        `Free-space ζ = B/(2√(Km)) = ${fmt(zf, 2)}; in contact the wall adds stiffness, so ζ drops to ${fmt(zc, 3)} — that is why contact rings. Mass m = 1 kg (example).`;
    }
    draw();
  });
})();
