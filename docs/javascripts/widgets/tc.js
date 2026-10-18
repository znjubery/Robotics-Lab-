/* Track C widgets: tc-ekf — unicycle rover, dead reckoning vs EKF with GPS or landmark fixes. */
(function () {
  "use strict";

  // ---------- pure core (testable in node) ----------
  function rng(seed) {                         // mulberry32 + Box–Muller → reproducible noise
    let a = seed >>> 0;
    const u = () => { a = (a + 0x6d2b79f5) >>> 0; let t = a; t = Math.imul(t ^ (t >>> 15), t | 1); t ^= t + Math.imul(t ^ (t >>> 7), t | 61); return ((t ^ (t >>> 14)) >>> 0) / 4294967296; };
    return { u, n: () => Math.sqrt(-2 * Math.log(u() + 1e-12)) * Math.cos(2 * Math.PI * u()) };
  }
  const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));
  // 3×3 helpers
  const mm = (A, B) => A.map((r) => B[0].map((_, j) => r.reduce((s, v, k) => s + v * B[k][j], 0)));
  const tr = (A) => A[0].map((_, j) => A.map((r) => r[j]));
  const add = (A, B) => A.map((r, i) => r.map((v, j) => v + B[i][j]));
  function inv2(M) { const d = M[0][0] * M[1][1] - M[0][1] * M[1][0]; return [[M[1][1] / d, -M[0][1] / d], [-M[1][0] / d, M[0][0] / d]]; }

  // motion model with midpoint heading; returns new pose
  function move(x, v, w, dt) { const h = x[2] + (w * dt) / 2; return [x[0] + v * dt * Math.cos(h), x[1] + v * dt * Math.sin(h), wrap(x[2] + w * dt)]; }
  // commanded racetrack: up a row, headland U-turn, down the next row, U-turn …
  function command(t) {
    const v = 0.5, Ts = 16, Tt = 6, lap = 2 * (Ts + Tt), tt = t % lap;
    const turning = (tt > Ts && tt <= Ts + Tt) || tt > 2 * Ts + Tt;
    return [v, turning ? Math.PI / Tt : 0];
  }
  const LM = [[-1.5, -1.5], [9.5, -1.5], [9.5, 3.4], [-1.5, 3.4], [4, 0.95]];   // known landmarks (example layout)

  function simulate(o) {
    const R = rng(o.seed), Rz = rng(o.seed * 7919 + 13), dt = 0.05, T = o.T || 88, N = Math.round(T / dt);
    const sv = o.sv, sw = o.sw;                     // odometry noise std: m/s and rad/s
    const Q = [[sv * sv, 0], [0, sw * sw]];
    let xt = [0, 0, 0], xd = [0, 0, 0], xe = [0, 0, 0];
    let P = [[1e-4, 0, 0], [0, 1e-4, 0], [0, 0, 1e-4]];
    const every = Math.max(1, Math.round(o.fixEvery / dt));
    const out = { truth: [], dr: [], ekf: [], P: [], fixes: [], errDR: [], errEKF: [], sig: [] };
    let se = 0, sd = 0, nis = 0, nfix = 0;
    for (let k = 1; k <= N; k++) {
      const t = k * dt, [v, w] = command(t);
      xt = move(xt, v, w, dt);
      const vm = v + sv * R.n(), wm = w + sw * R.n();          // noisy wheel odometry
      xd = move(xd, vm, wm, dt);                               // dead reckoning
      // EKF predict
      const h = xe[2] + (wm * dt) / 2, c = Math.cos(h), s = Math.sin(h);
      const F = [[1, 0, -vm * dt * s], [0, 1, vm * dt * c], [0, 0, 1]];
      const G = [[dt * c, (-vm * dt * dt * s) / 2], [dt * s, (vm * dt * dt * c) / 2], [0, dt]];
      xe = move(xe, vm, wm, dt);
      P = add(mm(mm(F, P), tr(F)), mm(mm(G, Q), tr(G)));
      // EKF update
      if (o.fixOn && k % every === 0) {
        let z, zhat, H, Rm;
        if (o.mode === "gps") {
          z = [xt[0] + o.sm * Rz.n(), xt[1] + o.sm * Rz.n()];
          zhat = [xe[0], xe[1]]; H = [[1, 0, 0], [0, 1, 0]]; Rm = [[o.sm * o.sm, 0], [0, o.sm * o.sm]];
          out.fixes.push(z);
        } else {
          // nearest landmark (true distance), range–bearing
          let best = 0, bd = 1e9;
          LM.forEach((l, i) => { const d = Math.hypot(l[0] - xt[0], l[1] - xt[1]); if (d < bd) { bd = d; best = i; } });
          const l = LM[best], sb = o.sb;
          z = [Math.hypot(l[0] - xt[0], l[1] - xt[1]) + o.sm * Rz.n(), wrap(Math.atan2(l[1] - xt[1], l[0] - xt[0]) - xt[2] + sb * Rz.n())];
          const dx = l[0] - xe[0], dy = l[1] - xe[1], q = dx * dx + dy * dy, r = Math.sqrt(q);
          zhat = [r, wrap(Math.atan2(dy, dx) - xe[2])];
          H = [[-dx / r, -dy / r, 0], [dy / q, -dx / q, -1]];
          Rm = [[o.sm * o.sm, 0], [0, sb * sb]];
          out.fixes.push(l);
        }
        const y = [z[0] - zhat[0], o.mode === "gps" ? z[1] - zhat[1] : wrap(z[1] - zhat[1])];
        const S = add(mm(mm(H, P), tr(H)), Rm), Si = inv2(S);
        const K = mm(mm(P, tr(H)), Si);                        // 3×2 Kalman gain
        nis += y[0] * (Si[0][0] * y[0] + Si[0][1] * y[1]) + y[1] * (Si[1][0] * y[0] + Si[1][1] * y[1]); nfix++;
        xe = [xe[0] + K[0][0] * y[0] + K[0][1] * y[1], xe[1] + K[1][0] * y[0] + K[1][1] * y[1], wrap(xe[2] + K[2][0] * y[0] + K[2][1] * y[1])];
        // Joseph form keeps P symmetric positive definite
        const IKH = [[1, 0, 0], [0, 1, 0], [0, 0, 1]].map((r, i) => r.map((v, j) => v - (K[i][0] * H[0][j] + K[i][1] * H[1][j])));
        P = add(mm(mm(IKH, P), tr(IKH)), mm(mm(K, Rm), tr(K)));
      }
      const ed = Math.hypot(xd[0] - xt[0], xd[1] - xt[1]), ee = Math.hypot(xe[0] - xt[0], xe[1] - xt[1]);
      sd += ed * ed; se += ee * ee;
      if (k % 4 === 0) {
        out.truth.push(xt.slice()); out.dr.push(xd.slice()); out.ekf.push(xe.slice());
        out.P.push([[P[0][0], P[0][1]], [P[1][0], P[1][1]]]);
        out.errDR.push(ed); out.errEKF.push(ee); out.sig.push(2 * Math.sqrt(P[0][0] + P[1][1]));
      }
    }
    out.rmseDR = Math.sqrt(sd / N); out.rmseEKF = Math.sqrt(se / N);
    out.nis = nfix ? nis / nfix : NaN; out.nfix = nfix; out.dt = dt * 4;
    return out;
  }
  // 95% ellipse of a 2×2 covariance: semi-axes √(5.991 λᵢ)
  function ellipse(C) {
    const a = C[0][0], b = C[0][1], d = C[1][1], m = (a + d) / 2, r = Math.sqrt(((a - d) / 2) ** 2 + b * b);
    const l1 = m + r, l2 = Math.max(0, m - r), ang = 0.5 * Math.atan2(2 * b, a - d);
    return { a: Math.sqrt(5.991 * l1), b: Math.sqrt(5.991 * l2), ang };
  }

  if (typeof window === "undefined") { module.exports = { simulate, ellipse, rng, LM }; return; }
  const L = window.Lab; if (!L) return;
  const { colors, fmt, el, slider, canvas } = L;

  L.register("tc-ekf", function (root) {
    const ui = el("div", { class: "lab-controls" });
    const pics = el("div", { class: "lab-pics" });
    root.append(ui, pics);
    const M = canvas(pics, 420, 260), E = canvas(pics, 420, 130);
    const out = el("div", { class: "lab-readout" }); root.appendChild(out);
    const row = el("div", { class: "lab-row" });
    const sel = el("select");
    sel.append(el("option", { value: "gps" }, "GPS fix (x, y)"), el("option", { value: "lm" }, "Landmark range + bearing"), el("option", { value: "none" }, "No fixes (odometry only)"));
    row.append(el("span", { class: "lab-label" }, "Correction"), sel); ui.appendChild(row);
    sel.addEventListener("change", run);
    const sv = slider(ui, "odom σ speed", 0, 0.3, 0.01, 0.08, run, " m/s");
    const sw = slider(ui, "odom σ turn", 0, 20, 0.5, 6, run, "°/s");
    const sm = slider(ui, "meas. σ", 0.02, 1.5, 0.02, 0.3, run, " m");
    const fe = slider(ui, "fix every", 0.5, 20, 0.5, 4, run, " s");
    const sd = slider(ui, "seed", 1, 50, 1, 7, run, "");
    const tt = slider(ui, "show time", 1, 88, 1, 88, draw, " s");
    let R = null;
    function run() {
      R = simulate({ seed: +sd.value, sv: +sv.value, sw: (+sw.value * Math.PI) / 180, sm: +sm.value, sb: (3 * Math.PI) / 180,
        fixEvery: +fe.value, fixOn: sel.value !== "none", mode: sel.value === "lm" ? "lm" : "gps" });
      draw();
    }
    function draw() {
      if (!R) return;
      const k = colors(), ctx = M.ctx; ctx.clearRect(0, 0, 420, 260); ctx.font = "12px sans-serif";
      const S = 30, ox = 92, oy = 175, X = (x) => ox + S * x, Y = (y) => oy - S * y;
      const n = Math.min(R.truth.length, Math.max(2, Math.round(+tt.value / R.dt)));
      // crop rows (decoration): rows at y = 0 and y = 1.9 are the two passes
      ctx.strokeStyle = k.green; ctx.globalAlpha = 0.25; ctx.lineWidth = 6;
      for (const yy of [-0.5, 0.95, 2.4]) { ctx.beginPath(); ctx.moveTo(X(0), Y(yy)); ctx.lineTo(X(8), Y(yy)); ctx.stroke(); }
      ctx.globalAlpha = 1;
      // landmarks
      if (sel.value === "lm") for (const l of LM) {
        ctx.fillStyle = k.accent; ctx.beginPath(); ctx.moveTo(X(l[0]), Y(l[1]) - 6); ctx.lineTo(X(l[0]) + 5, Y(l[1]) + 4); ctx.lineTo(X(l[0]) - 5, Y(l[1]) + 4); ctx.fill();
      }
      const path = (arr, col, w, dash) => {
        ctx.strokeStyle = col; ctx.lineWidth = w; ctx.setLineDash(dash || []); ctx.beginPath();
        for (let i = 0; i < n; i++) { const p = arr[i]; i ? ctx.lineTo(X(p[0]), Y(p[1])) : ctx.moveTo(X(p[0]), Y(p[1])); }
        ctx.stroke(); ctx.setLineDash([]);
      };
      path(R.truth, k.fg, 2.5); path(R.dr, k.red, 1.5, [5, 3]); path(R.ekf, k.blue, 1.8);
      // GPS fixes
      if (sel.value === "gps") { ctx.fillStyle = k.accent; const nf = Math.floor(R.fixes.length * n / R.truth.length); for (let i = 0; i < nf; i++) { const f = R.fixes[i]; ctx.fillRect(X(f[0]) - 2, Y(f[1]) - 2, 4, 4); } }
      // 95% ellipses every ~4 s
      ctx.strokeStyle = k.blue; ctx.lineWidth = 1;
      for (let i = 20; i < n; i += 20) {
        const e = ellipse(R.P[i]), p = R.ekf[i];
        ctx.beginPath(); ctx.ellipse(X(p[0]), Y(p[1]), Math.min(400, S * e.a), Math.min(400, S * e.b), -e.ang, 0, 2 * Math.PI); ctx.stroke();
      }
      // rover at current time
      const p = R.truth[n - 1];
      ctx.fillStyle = k.fg; ctx.save(); ctx.translate(X(p[0]), Y(p[1])); ctx.rotate(-p[2]);
      ctx.beginPath(); ctx.moveTo(9, 0); ctx.lineTo(-6, 5); ctx.lineTo(-6, -5); ctx.closePath(); ctx.fill(); ctx.restore();
      ctx.fillStyle = k.fg; ctx.fillText("— truth", 8, 16);
      ctx.fillStyle = k.red; ctx.fillText("- - dead reckoning", 62, 16);
      ctx.fillStyle = k.blue; ctx.fillText("— EKF + 95% ellipses", 180, 16);
      ctx.fillStyle = k.accent; ctx.fillText(sel.value === "lm" ? "▲ landmarks" : sel.value === "gps" ? "■ fixes" : "", 320, 16);
      ctx.fillStyle = k.light; ctx.fillText("grid: 1 m", 8, 252);
      ctx.strokeStyle = k.light; ctx.beginPath(); ctx.moveTo(70, 247); ctx.lineTo(70 + S, 247); ctx.stroke();
      // error vs time with EKF 2σ envelope
      const g = E.ctx; g.clearRect(0, 0, 420, 130); g.font = "12px sans-serif";
      const ymax = Math.max(0.5, ...R.errDR.slice(0, n).concat(R.sig.slice(0, n)).map((v) => Math.min(v, 30)));
      const tx = (i) => 30 + (380 * i) / R.truth.length, ty = (v) => 120 - (100 * Math.min(v, ymax)) / ymax;
      g.strokeStyle = k.light; g.lineWidth = 1; g.beginPath(); g.moveTo(30, 20); g.lineTo(30, 120); g.lineTo(410, 120); g.stroke();
      g.fillStyle = "rgba(30,136,229,0.15)"; g.beginPath(); g.moveTo(tx(0), ty(0));
      for (let i = 0; i < n; i++) g.lineTo(tx(i), ty(R.sig[i])); g.lineTo(tx(n - 1), ty(0)); g.fill();
      const ln = (arr, col) => { g.strokeStyle = col; g.lineWidth = 1.6; g.beginPath(); for (let i = 0; i < n; i++) i ? g.lineTo(tx(i), ty(arr[i])) : g.moveTo(tx(i), ty(arr[i])); g.stroke(); };
      ln(R.errDR, k.red); ln(R.errEKF, k.blue);
      g.fillStyle = k.fg; g.fillText(`position error (m), top = ${fmt(ymax, 1)} m; shaded = EKF 2σ`, 34, 14);
      g.fillText("time →", 370, 114);
      const nfix = sel.value === "none" ? 0 : R.nfix;
      out.innerHTML =
        `RMSE over 88 s: dead reckoning <b style="color:${k.red}">${fmt(R.rmseDR, 2)} m</b>, EKF <b style="color:${k.blue}">${fmt(R.rmseEKF, 2)} m</b>` +
        ` (${R.rmseEKF > 0 ? fmt(R.rmseDR / R.rmseEKF, 1) : "—"}× better). Fixes used: ${nfix}.<br>` +
        `Final error: DR ${fmt(R.errDR[R.errDR.length - 1], 2)} m, EKF ${fmt(R.errEKF[R.errEKF.length - 1], 2)} m; EKF says 2σ ≈ ${fmt(R.sig[R.sig.length - 1], 2)} m.` +
        (nfix ? ` Mean NIS = ${fmt(R.nis, 2)} (a consistent filter gives ≈ 2).` : "");
    }
    run();
  });
})();
