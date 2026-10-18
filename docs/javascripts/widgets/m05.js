/* m05 widgets: inverse kinematics — analytic 2R, Newton (pseudoinverse) vs damped least squares */
(function () {
  "use strict";
  const L = window.Lab; if (!L) return;
  const { colors, fmt, el, slider, canvas, deg } = L;

  // ---------- pure math (also exported for node tests) ----------
  function fk(Ls, th) {
    let x = 0, y = 0, a = 0; const pts = [[0, 0]];
    for (let i = 0; i < Ls.length; i++) { a += th[i]; x += Ls[i] * Math.cos(a); y += Ls[i] * Math.sin(a); pts.push([x, y]); }
    return pts;
  }
  // 2 x n position Jacobian of the planar chain
  function jac(Ls, th) {
    const n = Ls.length, J = [new Array(n).fill(0), new Array(n).fill(0)];
    for (let i = 0; i < n; i++) {
      let a = 0;
      for (let j = 0; j < n; j++) { a += th[j]; if (j >= i) { J[0][i] -= Ls[j] * Math.sin(a); J[1][i] += Ls[j] * Math.cos(a); } }
    }
    return J;
  }
  // eigen-decomposition of symmetric 2x2 A = V diag(s2) V^T  (s2 = sigma^2 of J)
  function eig2(a, b, d) {
    const t = (a + d) / 2, r = Math.hypot((a - d) / 2, b);
    const l1 = t + r, l2 = Math.max(0, t - r);
    let v1;
    if (Math.abs(b) > 1e-14) v1 = [l1 - d, b]; else v1 = a >= d ? [1, 0] : [0, 1];
    const nv = Math.hypot(v1[0], v1[1]); v1 = [v1[0] / nv, v1[1] / nv];
    return { s2: [l1, l2], V: [v1, [-v1[1], v1[0]]] };
  }
  // dtheta = J^T (J J^T + lam^2 I)^-1 e   (lam = 0 -> pseudoinverse, singular directions dropped)
  function step(J, e, lam) {
    const n = J[0].length;
    const a = J[0].reduce((s, v) => s + v * v, 0), d = J[1].reduce((s, v) => s + v * v, 0);
    const b = J[0].reduce((s, v, i) => s + v * J[1][i], 0);
    const { s2, V } = eig2(a, b, d);
    let w = [0, 0]; // w = (J J^T + lam^2 I)^-1 e
    for (let k = 0; k < 2; k++) {
      const den = s2[k] + lam * lam;
      if (den < 1e-12) continue; // exactly singular, pseudoinverse drops it
      const c = (V[k][0] * e[0] + V[k][1] * e[1]) / den;
      w[0] += c * V[k][0]; w[1] += c * V[k][1];
    }
    const dth = []; for (let i = 0; i < n; i++) dth.push(J[0][i] * w[0] + J[1][i] * w[1]);
    return { dth, sig: s2.map(Math.sqrt) };
  }
  function pinvNull(J, z) { // (I - J+ J) z
    const n = J[0].length, Jz = [0, 1].map((r) => J[r].reduce((s, v, i) => s + v * z[i], 0));
    const p = step(J, Jz, 0).dth; return z.map((v, i) => v - p[i]);
  }
  function solve(Ls, th0, target, lam, iters, nullK) {
    let th = th0.slice(); const err = [], path = [th.slice()]; let maxStep = 0, sigmin = 0;
    for (let k = 0; k <= iters; k++) {
      const tip = fk(Ls, th).at(-1), e = [target[0] - tip[0], target[1] - tip[1]];
      err.push(Math.hypot(e[0], e[1]));
      if (k === iters) break;
      const J = jac(Ls, th), s = step(J, e, lam); sigmin = s.sig[1];
      let dth = s.dth;
      if (nullK) { const nz = pinvNull(J, th.map((t) => -nullK * t)); dth = dth.map((v, i) => v + nz[i]); }
      maxStep = Math.max(maxStep, Math.hypot(...dth));
      th = th.map((t, i) => Math.atan2(Math.sin(t + dth[i]), Math.cos(t + dth[i])));
      path.push(th.slice());
    }
    return { th, err, path, maxStep, sigmin };
  }
  function analytic2R(L1, L2, x, y) {
    const D = (x * x + y * y - L1 * L1 - L2 * L2) / (2 * L1 * L2);
    if (Math.abs(D) > 1) return { D, sols: [] };
    return { D, sols: [1, -1].map((sg) => { const t2 = sg * Math.acos(D); return [Math.atan2(y, x) - Math.atan2(L2 * Math.sin(t2), L1 + L2 * Math.cos(t2)), t2]; }) };
  }
  L.m05 = { fk, jac, step, solve, analytic2R };

  // ---------- widget ----------
  L.register("m05-ik", function (root) {
    const ui = el("div", { class: "lab-controls" });
    const pics = el("div", { class: "lab-pics" });
    root.append(ui, pics);
    const A = canvas(pics, 360, 320), P = canvas(pics, 320, 320);
    const out = el("div", { class: "lab-readout" }); root.appendChild(out);
    const sel = el("select"); ["2R arm (L = 1.0, 0.8)", "3R arm (L = 0.8, 0.6, 0.4)"].forEach((t, i) => sel.appendChild(el("option", { value: i }, t)));
    const row = el("div", { class: "lab-row" }); row.append(el("span", { class: "lab-label" }, "Arm"), sel); ui.appendChild(row);
    const lg = slider(ui, "log₁₀ λ (damping)", -3, 0, 0.05, -1, draw, "");
    const it = slider(ui, "iterations", 1, 60, 1, 25, draw, "");
    const g0 = slider(ui, "start guess θ₀ (all joints)", -180, 180, 1, 30, draw, "°");
    const nsRow = el("div", { class: "lab-row" }); const ns = el("input", { type: "checkbox" });
    nsRow.append(el("span", { class: "lab-label" }, "3R: null-space pull θ → 0"), ns); ui.appendChild(nsRow);
    sel.addEventListener("change", draw); ns.addEventListener("change", draw);
    let target = [1.2, 0.6];
    const S = 80, cx = 180, cy = 165;
    A.c.addEventListener("click", (ev) => {
      const r = A.c.getBoundingClientRect();
      const px = ((ev.clientX - r.left) / r.width) * 360, py = ((ev.clientY - r.top) / r.height) * 320;
      target = [(px - cx) / S, (cy - py) / S]; draw();
    });
    function arm(ctx, pts, color, width, alpha) {
      ctx.globalAlpha = alpha; ctx.lineWidth = width; ctx.lineCap = "round"; ctx.lineJoin = "round"; ctx.strokeStyle = color;
      ctx.beginPath(); pts.forEach(([x, y], i) => (i ? ctx.lineTo(cx + S * x, cy - S * y) : ctx.moveTo(cx + S * x, cy - S * y))); ctx.stroke();
      ctx.fillStyle = colors().fg; pts.slice(0, -1).forEach(([x, y]) => { ctx.beginPath(); ctx.arc(cx + S * x, cy - S * y, width > 3 ? 4 : 2, 0, 7); ctx.fill(); });
      ctx.globalAlpha = 1;
    }
    function draw() {
      const k = colors(), three = sel.value === "1";
      const Ls = three ? [0.8, 0.6, 0.4] : [1.0, 0.8];
      const lam = Math.pow(10, +lg.value), n = +it.value, th0 = Ls.map(() => deg(+g0.value));
      const nk = three && ns.checked ? 0.3 : 0;
      const N = solve(Ls, th0, target, 0, n, nk), D = solve(Ls, th0, target, lam, n, nk);
      const reach = Ls.reduce((a, b) => a + b, 0), inner = Math.max(0, 2 * Math.max(...Ls) - reach);
      // ---- arm view ----
      let ctx = A.ctx; ctx.clearRect(0, 0, 360, 320);
      ctx.strokeStyle = k.light; ctx.lineWidth = 1; ctx.setLineDash([3, 3]);
      ctx.beginPath(); ctx.arc(cx, cy, S * reach, 0, 7); ctx.stroke();
      if (inner > 0) { ctx.beginPath(); ctx.arc(cx, cy, S * inner, 0, 7); ctx.stroke(); }
      ctx.setLineDash([]);
      let txtA = "";
      if (!three) {
        const an = analytic2R(1.0, 0.8, target[0], target[1]);
        an.sols.forEach((s) => arm(ctx, fk(Ls, s), k.green, 2, 0.6));
        txtA = an.sols.length
          ? `Analytic (green): cos θ₂ = ${fmt(an.D, 3)} → θ₂ = ±${fmt(Math.acos(an.D) * 180 / Math.PI, 1)}°; solutions (θ₁, θ₂) = ` +
            an.sols.map((s) => `(${fmt(s[0] * 180 / Math.PI, 1)}°, ${fmt(s[1] * 180 / Math.PI, 1)}°)`).join(" and ") + ". "
          : `Analytic: cos θ₂ = ${fmt(an.D, 3)} — |cos θ₂| &gt; 1, so <b>no solution</b> (target outside the reachable ring). `;
      }
      // Newton tip trail
      ctx.strokeStyle = k.red; ctx.lineWidth = 1; ctx.globalAlpha = 0.5; ctx.beginPath();
      N.path.forEach((th, i) => { const p = fk(Ls, th).at(-1); i ? ctx.lineTo(cx + S * p[0], cy - S * p[1]) : ctx.moveTo(cx + S * p[0], cy - S * p[1]); }); ctx.stroke(); ctx.globalAlpha = 1;
      arm(ctx, fk(Ls, th0), k.light, 2, 0.5);
      arm(ctx, fk(Ls, N.th), k.red, 4, 0.75);
      arm(ctx, fk(Ls, D.th), k.primary, 6, 1);
      ctx.strokeStyle = k.accent; ctx.lineWidth = 2.5;
      const tx = cx + S * target[0], ty = cy - S * target[1];
      ctx.beginPath(); ctx.moveTo(tx - 7, ty); ctx.lineTo(tx + 7, ty); ctx.moveTo(tx, ty - 7); ctx.lineTo(tx, ty + 7); ctx.stroke();
      ctx.fillStyle = k.fg; ctx.font = "12px sans-serif";
      ctx.fillText("click to move target ✚", 8, 16);
      ctx.fillStyle = k.red; ctx.fillText("Newton (J⁺)", 8, 300); ctx.fillStyle = k.primary; ctx.fillText("damped LS", 92, 300);
      ctx.fillStyle = k.light; ctx.fillText("start", 170, 300); if (!three) { ctx.fillStyle = k.green; ctx.fillText("analytic", 210, 300); }
      // ---- error plot (log10) ----
      ctx = P.ctx; ctx.clearRect(0, 0, 320, 320);
      const x0 = 42, x1 = 310, y0 = 28, y1 = 285, lo = -12, hi = 1;
      const X = (i) => x0 + ((x1 - x0) * i) / n, Y = (e) => y0 + ((y1 - y0) * (hi - Math.max(lo, Math.min(hi, Math.log10(Math.max(e, 1e-16)))))) / (hi - lo);
      ctx.strokeStyle = k.light; ctx.lineWidth = 1; ctx.fillStyle = k.fg; ctx.font = "12px sans-serif";
      for (let p = lo; p <= hi; p += 2) { ctx.globalAlpha = 0.35; ctx.beginPath(); ctx.moveTo(x0, Y(10 ** p)); ctx.lineTo(x1, Y(10 ** p)); ctx.stroke(); ctx.globalAlpha = 1; ctx.fillText("1e" + p, 4, Y(10 ** p) + 4); }
      ctx.strokeRect(x0, y0, x1 - x0, y1 - y0);
      ctx.fillText("tip error ‖e‖ (log) vs iteration", x0, 18); ctx.fillText("0", x0 - 3, y1 + 14); ctx.fillText(String(n), x1 - 12, y1 + 14);
      for (const [R, c] of [[N, k.red], [D, k.primary]]) {
        ctx.strokeStyle = c; ctx.lineWidth = 2; ctx.beginPath();
        R.err.forEach((e, i) => (i ? ctx.lineTo(X(i), Y(e)) : ctx.moveTo(X(i), Y(e)))); ctx.stroke();
        ctx.fillStyle = c; R.err.forEach((e, i) => { ctx.beginPath(); ctx.arc(X(i), Y(e), 2, 0, 7); ctx.fill(); });
      }
      const dist = Math.hypot(...target), gap = Math.max(0, dist - reach);
      const firstBelow = (R) => { const j = R.err.findIndex((e) => e < 1e-6); return j < 0 ? "not within " + n : j; };
      out.innerHTML =
        `Target (${fmt(target[0], 2)}, ${fmt(target[1], 2)}), distance ${fmt(dist, 2)} vs reach ${fmt(reach, 2)}` +
        (gap > 0 ? ` → <b style="color:${k.red}">unreachable</b>; best possible error = ${fmt(gap, 3)}.` : ".") + "<br>" + txtA + "<br>" +
        `<span style="color:${k.red}">Newton</span>: final ‖e‖ = ${N.err.at(-1).toExponential(1)}, iterations to 1e-6: ${firstBelow(N)}, largest step ‖Δθ‖ = ${fmt(N.maxStep, 2)} rad. ` +
        `<span style="color:${k.primary}">Damped LS (λ = ${fmt(lam, 3)})</span>: final ‖e‖ = ${D.err.at(-1).toExponential(1)}, to 1e-6: ${firstBelow(D)}, largest step ${fmt(D.maxStep, 2)} rad.<br>` +
        `Smallest singular value of J at the last DLS step σ_min = ${fmt(D.sigmin, 3)}. Newton gain on that direction 1/σ = ${D.sigmin > 1e-9 ? fmt(1 / D.sigmin, 1) : "∞"}; DLS gain σ/(σ²+λ²) = ${fmt(D.sigmin / (D.sigmin ** 2 + lam * lam), 2)} (never above 1/(2λ) = ${fmt(1 / (2 * lam), 1)}).`;
    }
    draw();
  });
})();
