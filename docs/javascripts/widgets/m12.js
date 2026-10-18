/* M12 widgets: differential-drive odometry (exact SE(2) vs Euler) and Lie-bracket wiggle */
(function () {
  "use strict";
  const L = window.Lab; if (!L) return;
  const { colors, fmt, el, slider, canvas } = L;

  // ---------- pure math (also tested with node) ----------
  const WR = 0.05, WD = 0.15; // example values: wheel radius r [m], half axle length d [m]
  const twist = (uL, uR) => ({ w: (WR * (uR - uL)) / (2 * WD), v: (WR * (uR + uL)) / 2 });
  // exact exponential of a constant planar twist (ω, v along body x), applied in body frame
  function expStep(q, w, v, t) {
    const [x, y, p] = q;
    if (Math.abs(w) < 1e-12) return [x + v * t * Math.cos(p), y + v * t * Math.sin(p), p];
    const R = v / w; // turning radius
    return [x + R * (Math.sin(p + w * t) - Math.sin(p)), y - R * (Math.cos(p + w * t) - Math.cos(p)), p + w * t];
  }
  const eulerStep = (q, w, v, h) => [q[0] + v * Math.cos(q[2]) * h, q[1] + v * Math.sin(q[2]) * h, q[2] + w * h];
  function eulerPath(w, v, T, h) {
    const n = Math.max(1, Math.round(T / h)), hh = T / n, P = [[0, 0, 0]];
    for (let i = 0; i < n; i++) P.push(eulerStep(P[i], w, v, hh));
    return P;
  }
  // wiggle: drive ε, spin ε, drive −ε, spin −ε  (flows of g1 = drive, g2 = spin), repeated N times
  function wigglePath(eps, N, sub = 20) {
    let q = [0, 0, 0]; const P = [q];
    const legs = [[0, 1], [1, 0], [0, -1], [-1, 0]]; // [ω, v] directions
    for (let c = 0; c < N; c++) for (const [w, v] of legs)
      for (let s = 1; s <= sub; s++) { q = expStep(P[P.length - 1], w, v, eps / sub); P.push(q); }
    return P;
  }
  if (typeof module !== "undefined") module.exports = { twist, expStep, eulerPath, wigglePath };

  function drawRover(ctx, X, Y, q, S, fill, stroke) {
    const c = Math.cos(q[2]), s = Math.sin(q[2]);
    const body = [[-0.12, -WD], [0.16, -WD], [0.22, 0], [0.16, WD], [-0.12, WD]];
    ctx.beginPath();
    body.forEach(([a, b], i) => { const x = q[0] + c * a - s * b, y = q[1] + s * a + c * b; i ? ctx.lineTo(X(x), Y(y)) : ctx.moveTo(X(x), Y(y)); });
    ctx.closePath(); ctx.fillStyle = fill; ctx.fill(); ctx.strokeStyle = stroke; ctx.lineWidth = 1; ctx.stroke();
    for (const sd of [-1, 1]) { // wheels
      const a0 = [q[0] - 0.06 * c - s * sd * WD, q[1] - 0.06 * s + c * sd * WD], a1 = [q[0] + 0.06 * c - s * sd * WD, q[1] + 0.06 * s + c * sd * WD];
      ctx.lineWidth = 4; ctx.beginPath(); ctx.moveTo(X(a0[0]), Y(a0[1])); ctx.lineTo(X(a1[0]), Y(a1[1])); ctx.stroke();
    }
    ctx.lineWidth = 1;
  }
  function fitView(pts, w, h, pad, minSpan) {
    let x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity;
    for (const p of pts) { x0 = Math.min(x0, p[0]); x1 = Math.max(x1, p[0]); y0 = Math.min(y0, p[1]); y1 = Math.max(y1, p[1]); }
    const S = Math.min((w - 2 * pad) / Math.max(x1 - x0, minSpan), (h - 2 * pad - 20) / Math.max(y1 - y0, minSpan));
    const mx = (x0 + x1) / 2, my = (y0 + y1) / 2;
    return { S, X: (x) => w / 2 + S * (x - mx), Y: (y) => h / 2 + 10 - S * (y - my) };
  }

  L.register("m12-diffdrive", function (root) {
    const ui = el("div", { class: "lab-controls" });
    const pics = el("div", { class: "lab-pics" });
    root.append(ui, pics);
    const A = canvas(pics, 400, 300), E = canvas(pics, 240, 300), B = canvas(pics, 300, 260);
    const out = el("div", { class: "lab-readout" }); root.appendChild(out);
    const uL = slider(ui, "left wheel u_L", -10, 10, 0.1, 4, draw, " rad/s");
    const uR = slider(ui, "right wheel u_R", -10, 10, 0.1, 6, draw, " rad/s");
    const T = slider(ui, "time T", 0.5, 20, 0.1, 8, draw, " s");
    const hS = slider(ui, "Euler step h", 0.01, 1, 0.01, 0.4, draw, " s");
    const slip = slider(ui, "left-wheel slip (mud)", 0, 50, 1, 0, draw, "%");
    const ep = slider(ui, "wiggle size ε", 0.05, 0.6, 0.01, 0.3, draw, "");
    const NC = slider(ui, "wiggle cycles N", 1, 12, 1, 4, draw, "");
    const brow = el("div", { class: "lab-row" });
    const btn = el("button", { class: "md-button", type: "button" }, "▶ Wiggle");
    brow.append(el("span", { class: "lab-label" }, "parallel park"), btn); ui.appendChild(brow);
    let anim = 1, raf = 0;
    btn.addEventListener("click", () => {
      cancelAnimationFrame(raf); const t0 = performance.now();
      const step = (t) => { anim = Math.min(1, (t - t0) / 3000); draw(); if (anim < 1) raf = requestAnimationFrame(step); };
      raf = requestAnimationFrame(step);
    });

    function draw() {
      const k = colors();
      const Tt = +T.value, h = +hS.value, sl = +slip.value / 100;
      const { w, v } = twist(+uL.value, +uR.value);
      const tr = twist(+uL.value * (1 - sl), +uR.value);           // what the ground actually does
      const N = 200, ex = [], tru = [];
      for (let i = 0; i <= N; i++) { ex.push(expStep([0, 0, 0], w, v, (Tt * i) / N)); tru.push(expStep([0, 0, 0], tr.w, tr.v, (Tt * i) / N)); }
      const eu = eulerPath(w, v, Tt, h);
      const all = ex.concat(eu, sl > 0 ? tru : [], [[0, 0]]);
      // --- panel A: paths ---
      let ctx = A.ctx; ctx.clearRect(0, 0, 400, 300);
      const V = fitView(all, 400, 300, 28, 0.6), X = V.X, Y = V.Y;
      ctx.strokeStyle = k.light; ctx.setLineDash([2, 4]);
      ctx.beginPath(); ctx.moveTo(0, Y(0)); ctx.lineTo(400, Y(0)); ctx.moveTo(X(0), 20); ctx.lineTo(X(0), 300); ctx.stroke(); ctx.setLineDash([]);
      const line = (P, col, wd, dash) => { ctx.strokeStyle = col; ctx.lineWidth = wd; ctx.setLineDash(dash || []); ctx.beginPath(); P.forEach((p, i) => (i ? ctx.lineTo(X(p[0]), Y(p[1])) : ctx.moveTo(X(p[0]), Y(p[1])))); ctx.stroke(); ctx.setLineDash([]); ctx.lineWidth = 1; };
      if (sl > 0) line(tru, k.green, 2, [6, 3]);
      line(ex, k.primary, 2.5);
      line(eu, k.red, 1.5);
      ctx.fillStyle = k.red; eu.forEach((p) => { ctx.beginPath(); ctx.arc(X(p[0]), Y(p[1]), 2.2, 0, 7); ctx.fill(); });
      drawRover(ctx, X, Y, [0, 0, 0], V.S, k.light + "80", k.fg);
      drawRover(ctx, X, Y, ex[N], V.S, k.accent, k.fg);
      ctx.font = "12px sans-serif"; ctx.fillStyle = k.primary; ctx.fillText("exact arc e^[𝒱]t", 6, 14);
      ctx.fillStyle = k.red; ctx.fillText("Euler steps", 120, 14);
      if (sl > 0) { ctx.fillStyle = k.green; ctx.fillText("true path with slip", 210, 14); }
      // --- panel E: log–log error vs h ---
      ctx = E.ctx; ctx.clearRect(0, 0, 240, 300);
      const hs = [], errs = [];
      for (let i = 0; i <= 24; i++) { const hh = Math.pow(10, -2 + (2 * i) / 24), P = eulerPath(w, v, Tt, hh), q = P[P.length - 1]; hs.push(hh); errs.push(Math.hypot(q[0] - ex[N][0], q[1] - ex[N][1])); }
      const qe = eu[eu.length - 1], errNow = Math.hypot(qe[0] - ex[N][0], qe[1] - ex[N][1]);
      const lx = (hh) => 40 + 185 * (Math.log10(hh) + 2) / 2;
      const emax = Math.max(...errs, 1e-6), lmax = Math.ceil(Math.log10(emax)), lmin = lmax - 4;
      const ly = (e) => 30 + 230 * (lmax - Math.log10(Math.max(e, 1e-12))) / (lmax - lmin);
      ctx.strokeStyle = k.light; ctx.strokeRect(40, 30, 185, 230);
      ctx.fillStyle = k.fg; ctx.font = "12px sans-serif";
      ctx.fillText("end error vs h (log–log)", 40, 18);
      ctx.fillText("0.01", 34, 276); ctx.fillText("0.1", 124, 276); ctx.fillText("1", 218, 276); ctx.fillText("h [s]", 120, 292);
      for (let l = lmin; l <= lmax; l += 2) ctx.fillText("1e" + l, 4, ly(Math.pow(10, l)) + 4);
      ctx.save(); ctx.beginPath(); ctx.rect(40, 30, 185, 230); ctx.clip();
      ctx.strokeStyle = k.red; ctx.lineWidth = 2; ctx.beginPath();
      hs.forEach((hh, i) => (i ? ctx.lineTo(lx(hh), ly(errs[i])) : ctx.moveTo(lx(hh), ly(errs[i])))); ctx.stroke();
      // slope-1 guide through the h = 0.01 point
      ctx.strokeStyle = k.light; ctx.setLineDash([4, 3]); ctx.beginPath(); ctx.moveTo(lx(0.01), ly(errs[0])); ctx.lineTo(lx(1), ly(errs[0] * 100)); ctx.stroke(); ctx.setLineDash([]);
      ctx.fillStyle = k.accent; ctx.beginPath(); ctx.arc(lx(h), ly(errNow), 5, 0, 7); ctx.fill(); ctx.restore();
      ctx.fillStyle = k.light; ctx.fillText("dashed = slope 1", 120, 252);
      // --- panel B: wiggle (Lie bracket) ---
      const eps = +ep.value, nc = +NC.value, WP = wigglePath(eps, nc), M = Math.max(1, Math.round(anim * (WP.length - 1)));
      ctx = B.ctx; ctx.clearRect(0, 0, 300, 260);
      const VB = fitView(WP.concat([[0, 0.3], [0, -0.3]]), 300, 260, 30, 0.6), XB = VB.X, YB = VB.Y;
      ctx.strokeStyle = k.light; ctx.setLineDash([2, 4]); ctx.beginPath(); ctx.moveTo(XB(0), 20); ctx.lineTo(XB(0), 260); ctx.stroke(); ctx.setLineDash([]);
      ctx.strokeStyle = k.primary; ctx.lineWidth = 1.5; ctx.beginPath();
      for (let i = 0; i <= M; i++) { const p = WP[i]; i ? ctx.lineTo(XB(p[0]), YB(p[1])) : ctx.moveTo(XB(p[0]), YB(p[1])); }
      ctx.stroke(); ctx.lineWidth = 1;
      drawRover(ctx, XB, YB, [0, 0, 0], VB.S, k.light + "60", k.light);
      drawRover(ctx, XB, YB, WP[M], VB.S, k.accent, k.fg);
      ctx.fillStyle = k.fg; ctx.font = "12px sans-serif"; ctx.fillText("Wiggle: drive ε, spin ε, back ε, spin −ε", 6, 14);
      const qf = WP[WP.length - 1];
      // --- readout ---
      const dx = qe[0] - ex[N][0], dy = qe[1] - ex[N][1];
      out.innerHTML =
        `Twist from wheels (r = ${WR} m, d = ${WD} m, example values): ω = r(u_R − u_L)/(2d) = <b>${fmt(w, 3)}</b> rad/s, v = r(u_R + u_L)/2 = <b>${fmt(v, 3)}</b> m/s` +
        (Math.abs(w) > 1e-9 ? `, turning radius v/ω = ${fmt(v / w, 2)} m.` : ` — straight line.`) + `<br>` +
        `After T = ${fmt(Tt, 1)} s exact pose (φ, x, y) = (${fmt((ex[N][2] * 180) / Math.PI, 1)}°, ${fmt(ex[N][0], 3)}, ${fmt(ex[N][1], 3)}). ` +
        `Euler with h = ${fmt(h, 2)} s (${eu.length - 1} steps) ends <b style="color:${k.red}">${fmt(Math.hypot(dx, dy), 3)} m</b> away. Halve h → error about halves (first order).` +
        (sl > 0 ? `<br>Slip ${slip.value}%: true pose is <b style="color:${k.green}">${fmt(Math.hypot(tru[N][0] - ex[N][0], tru[N][1] - ex[N][1]), 3)} m</b> from even the exact odometry — no integrator fixes wrong inputs.` : "") +
        `<br>Wiggle net motion after ${nc} cycles: Δφ = ${fmt((qf[2] * 180) / Math.PI, 2)}°, Δx = ${fmt(qf[0], 4)}, Δy = <b>${fmt(qf[1], 4)}</b>. ` +
        `Lie-bracket prediction: Δy ≈ −Nε² = ${fmt(-nc * eps * eps, 4)} (sideways, a direction no wheel can push).`;
    }
    draw();
  });
})();
