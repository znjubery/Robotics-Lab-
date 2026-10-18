/* Track B widgets: tb-quad — planar quadrotor with cascaded PD control, RK4 integration. */
(function () {
  "use strict";

  // ---------- pure simulation core (testable in node) ----------
  // Example values, roughly Crazyflie-sized (not an official spec):
  // mass 33 g, half-span 4.6 cm, roll inertia 1.4e-5 kg·m², each planar "rotor" = two real motors.
  const P = { m: 0.033, l: 0.046, I: 1.4e-5, g: 9.81, fmax: 0.30 };

  // state s = [y, z, phi, vy, vz, w]; inputs f1 (left), f2 (right)
  // m ÿ = −(f1+f2) sinφ,  m z̈ = (f1+f2) cosφ − m g,  I φ̈ = l (f2 − f1)
  function deriv(s, f1, f2, p) {
    const F = f1 + f2;
    return [s[3], s[4], s[5], (-F * Math.sin(s[2])) / p.m, (F * Math.cos(s[2])) / p.m - p.g, (p.l * (f2 - f1)) / p.I];
  }
  function rk4(s, f1, f2, dt, p) {
    const add = (a, b, h) => a.map((v, i) => v + h * b[i]);
    const k1 = deriv(s, f1, f2, p);
    const k2 = deriv(add(s, k1, dt / 2), f1, f2, p);
    const k3 = deriv(add(s, k2, dt / 2), f1, f2, p);
    const k4 = deriv(add(s, k3, dt), f1, f2, p);
    return s.map((v, i) => v + (dt / 6) * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]));
  }
  // cascade: position PD → desired acceleration → (thrust F, tilt φd) → attitude PD → torque → mixer
  function control(s, tgt, k, p) {
    const ay = k.kpp * (tgt[0] - s[0]) - k.kdp * s[3];
    const az = k.kpp * (tgt[1] - s[1]) - k.kdp * s[4];
    const tmax = (k.tilt * Math.PI) / 180;
    let phid = -Math.atan2(ay, p.g + az);            // flatness: thrust vector ∥ (ÿ, z̈ + g)
    phid = Math.max(-tmax, Math.min(tmax, phid));
    let F = (p.m * (p.g + az)) / Math.max(0.2, Math.cos(s[2]));
    F = Math.max(0, Math.min(2 * p.fmax, F));
    const tau = p.I * (k.kpa * (phid - s[2]) - k.kda * s[5]);
    let f1 = F / 2 - tau / (2 * p.l), f2 = F / 2 + tau / (2 * p.l);   // mixer (inverse of [1 1; −l l])
    const sat = f1 < 0 || f2 < 0 || f1 > p.fmax || f2 > p.fmax;
    f1 = Math.max(0, Math.min(p.fmax, f1)); f2 = Math.max(0, Math.min(p.fmax, f2));
    return { f1, f2, F, tau, phid, sat };
  }

  if (typeof window === "undefined") { module.exports = { P, deriv, rk4, control }; return; }
  const L = window.Lab; if (!L) return;
  const { colors, fmt, el, slider, canvas } = L;

  L.register("tb-quad", function (root) {
    const ui = el("div", { class: "lab-controls" });
    const pics = el("div", { class: "lab-pics" });
    root.append(ui, pics);
    const W = canvas(pics, 420, 300);
    const S = canvas(pics, 420, 120);
    const out = el("div", { class: "lab-readout" }); root.appendChild(out);
    const kpp = slider(ui, "Kp position", 0, 20, 0.5, 4, () => {}, " 1/s²");
    const kdp = slider(ui, "Kd position", 0, 10, 0.1, 2.8, () => {}, " 1/s");
    const kpa = slider(ui, "Kp attitude", 0, 1500, 10, 400, () => {}, " 1/s²");
    const kda = slider(ui, "Kd attitude", 0, 80, 1, 28, () => {}, " 1/s");
    const tilt = slider(ui, "max tilt", 5, 60, 1, 30, () => {}, "°");
    const btns = el("div", { class: "lab-row" });
    const bReset = el("button", { class: "md-button" }, "Reset");
    const bGust = el("button", { class: "md-button" }, "Gust (spin kick)");
    const bStep = el("button", { class: "md-button" }, "Target +1 m right");
    for (const b of [bReset, bGust, bStep]) { b.style.padding = "0.2em 0.8em"; b.style.fontSize = "0.7rem"; btns.appendChild(b); }
    ui.appendChild(btns);

    const X0 = 210, Z0 = 280, SC = 85;                      // pixels per metre; world y ∈ [−2.4, 2.4], z ∈ [0, 3.2]
    const toPx = (y, z) => [X0 + SC * y, Z0 - SC * z];
    let s, tgt, hist, trail, t, last;
    function reset() { s = [-1, 1, 0, 0, 0, 0]; tgt = [1, 1.8]; hist = []; trail = []; t = 0; }
    reset();
    bReset.onclick = reset;
    bGust.onclick = () => { s[5] += 12; };                   // +12 rad/s roll-rate kick
    bStep.onclick = () => { tgt = [Math.min(2.3, tgt[0] + 1), tgt[1]]; };
    W.c.addEventListener("click", (ev) => {
      const r = W.c.getBoundingClientRect();
      const px = ((ev.clientX - r.left) / r.width) * 420, py = ((ev.clientY - r.top) / r.height) * 300;
      tgt = [Math.max(-2.3, Math.min(2.3, (px - X0) / SC)), Math.max(0.2, Math.min(3.1, (Z0 - py) / SC))];
    });

    function step() {
      const k = { kpp: +kpp.value, kdp: +kdp.value, kpa: +kpa.value, kda: +kda.value, tilt: +tilt.value };
      const dt = 0.001; let u;
      for (let i = 0; i < 16; i++) {                         // 16 ms of physics per frame (≈ real time at 60 fps)
        u = control(s, tgt, k, P);                           // controller at 1 kHz, zero-order hold
        s = rk4(s, u.f1, u.f2, dt, P);
        t += dt;
        if (s[1] < 0) { s[1] = 0; s[4] = Math.max(0, s[4]); s[3] *= 0.5; }       // floor
        if (!s.every(Number.isFinite) || Math.abs(s[0]) > 50 || Math.abs(s[1]) > 50) { s = [-1, 1, 0, 0, 0, 0]; }
      }
      last = u;
      hist.push([t, s[0], s[2], u.phid]); if (hist.length > 300) hist.shift();
      trail.push([s[0], s[1]]); if (trail.length > 160) trail.shift();
    }
    function arrow(ctx, x, y, dx, dy, col) {
      ctx.strokeStyle = col; ctx.fillStyle = col; ctx.lineWidth = 2;
      ctx.beginPath(); ctx.moveTo(x, y); ctx.lineTo(x + dx, y + dy); ctx.stroke();
      const a = Math.atan2(dy, dx), h = 5;
      ctx.beginPath(); ctx.moveTo(x + dx, y + dy);
      ctx.lineTo(x + dx - h * Math.cos(a - 0.4), y + dy - h * Math.sin(a - 0.4));
      ctx.lineTo(x + dx - h * Math.cos(a + 0.4), y + dy - h * Math.sin(a + 0.4)); ctx.fill();
    }
    function draw() {
      const k = colors(), ctx = W.ctx;
      ctx.clearRect(0, 0, 420, 300); ctx.font = "12px sans-serif";
      // ground + grid
      ctx.strokeStyle = k.light; ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(0, Z0); ctx.lineTo(420, Z0); ctx.stroke();
      ctx.setLineDash([2, 4]);
      for (let y = -2; y <= 2; y++) { const [px] = toPx(y, 0); ctx.beginPath(); ctx.moveTo(px, 0); ctx.lineTo(px, Z0); ctx.stroke(); }
      ctx.setLineDash([]);
      // target
      const [tx, tz] = toPx(tgt[0], tgt[1]);
      ctx.strokeStyle = k.accent; ctx.lineWidth = 2;
      ctx.beginPath(); ctx.moveTo(tx - 8, tz); ctx.lineTo(tx + 8, tz); ctx.moveTo(tx, tz - 8); ctx.lineTo(tx, tz + 8); ctx.stroke();
      // trail
      ctx.strokeStyle = k.primary; ctx.globalAlpha = 0.5; ctx.lineWidth = 1.5; ctx.beginPath();
      trail.forEach(([y, z], i) => { const [px, pz] = toPx(y, z); i ? ctx.lineTo(px, pz) : ctx.moveTo(px, pz); });
      ctx.stroke(); ctx.globalAlpha = 1;
      // body: draw arm 6× larger than real so it is visible
      const [cx, cz] = toPx(s[0], s[1]), c = Math.cos(s[2]), sn = Math.sin(s[2]), A = 28;
      const lx = cx - A * c, lz = cz + A * sn, rx = cx + A * c, rz = cz - A * sn;   // canvas y is down
      ctx.strokeStyle = k.fg; ctx.lineWidth = 4; ctx.lineCap = "round";
      ctx.beginPath(); ctx.moveTo(lx, lz); ctx.lineTo(rx, rz); ctx.stroke();
      ctx.fillStyle = k.fg; ctx.beginPath(); ctx.arc(cx, cz, 4, 0, 7); ctx.fill();
      // thrust arrows along body z (−sinφ, cosφ) → canvas (−sinφ, −cosφ)
      const u = last || { f1: 0, f2: 0, sat: false, phid: 0, F: 0 }, Sa = 140;
      arrow(ctx, lx, lz, -sn * Sa * u.f1, -c * Sa * u.f1, u.f1 >= P.fmax - 1e-9 || u.f1 <= 1e-9 ? k.red : k.green);
      arrow(ctx, rx, rz, -sn * Sa * u.f2, -c * Sa * u.f2, u.f2 >= P.fmax - 1e-9 || u.f2 <= 1e-9 ? k.red : k.green);
      // gravity arrow
      arrow(ctx, cx, cz, 0, 0.5 * Sa * P.m * P.g / 2, k.light);
      ctx.fillStyle = k.fg;
      ctx.fillText("click to move the target ✚   (arm drawn 6× real size)", 8, 16);
      ctx.fillText("y →", 395, Z0 - 6); ctx.fillText("z ↑", 6, 34);
      // strip chart: y(t) and φ, φd
      const g = S.ctx; g.clearRect(0, 0, 420, 120); g.font = "12px sans-serif";
      g.strokeStyle = k.light; g.lineWidth = 1; g.beginPath(); g.moveTo(0, 60); g.lineTo(420, 60); g.stroke();
      if (hist.length > 1) {
        const t0 = hist[0][0], span = Math.max(4.8, hist[hist.length - 1][0] - t0);
        const line = (idx, sc, col, dash) => {
          g.strokeStyle = col; g.lineWidth = 1.8; g.setLineDash(dash || []); g.beginPath();
          hist.forEach((h, i) => { const x = ((h[0] - t0) / span) * 420, yy = 60 - sc * h[idx]; i ? g.lineTo(x, yy) : g.moveTo(x, yy); });
          g.stroke(); g.setLineDash([]);
        };
        line(1, 22, k.primary); line(3, 90, k.accent, [4, 3]); line(2, 90, k.red);
      }
      g.fillStyle = k.primary; g.fillText("y (m)", 8, 14);
      g.fillStyle = k.red; g.fillText("φ tilt", 58, 14);
      g.fillStyle = k.accent; g.fillText("φ desired (dashed)", 108, 14);
      g.fillStyle = k.fg; g.fillText("last ≈5 s", 360, 14);
      const hover = (P.m * P.g) / 2;
      out.innerHTML =
        `State: y = ${fmt(s[0], 2)} m, z = ${fmt(s[1], 2)} m, φ = ${fmt((s[2] * 180) / Math.PI, 1)}° (desired ${fmt((u.phid * 180) / Math.PI, 1)}°).<br>` +
        `Thrusts: f₁ = ${fmt(u.f1, 3)} N, f₂ = ${fmt(u.f2, 3)} N (hover each ${fmt(hover, 3)} N, max ${fmt(P.fmax, 2)} N) → ` +
        `total ${fmt(u.f1 + u.f2, 3)} N = ${fmt((u.f1 + u.f2) / (P.m * P.g), 2)} × weight. ` +
        `<b style="color:${u.sat ? k.red : k.green}">${u.sat ? "SATURATED — mixer asked for more than a motor can give" : "within limits"}</b><br>` +
        `Attitude loop ωₙ = √Kp = ${fmt(Math.sqrt(+kpa.value), 1)} rad/s vs position loop ωₙ = ${fmt(Math.sqrt(+kpp.value), 1)} rad/s → ratio ${fmt(Math.sqrt(+kpa.value) / Math.max(1e-6, Math.sqrt(+kpp.value)), 1)} (want ≳ 5).`;
    }
    function loop() {
      if (!root.isConnected) return;
      step(); draw(); requestAnimationFrame(loop);
    }
    loop();
  });
})();
