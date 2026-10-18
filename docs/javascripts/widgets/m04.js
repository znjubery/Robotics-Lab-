/* M4 · Velocity kinematics & statics widgets: m04-ellipse (manipulability / force ellipse), m04-statics (τ = Jᵀ F). */
(function () {
  "use strict";

  // ---------- pure math (testable in node) ----------
  // planar n-R arm, all joints about z. Returns joint points, tip, space Jacobian (rows ω, vx, vy) and tip Jacobian (2×n).
  function arm(Ls, th) {
    const P = [[0, 0]]; let a = 0;
    for (let i = 0; i < Ls.length; i++) { a += th[i]; const [x, y] = P[i]; P.push([x + Ls[i] * Math.cos(a), y + Ls[i] * Math.sin(a)]); }
    const p = P[Ls.length], n = Ls.length;
    // space Jacobian column i = Ad_{e^{[S1]θ1}⋯e^{[S(i−1)]θ(i−1)}} S_i = (1, q_y, −q_x) at the joint's CURRENT location q
    const Js = [Array(n).fill(1), P.slice(0, n).map((q) => q[1]), P.slice(0, n).map((q) => -q[0])];
    // tip linear velocity: v_tip = v_s + ω × p  →  column = (−(p_y − q_y), p_x − q_x)
    const Jv = [P.slice(0, n).map((q) => -(p[1] - q[1])), P.slice(0, n).map((q) => p[0] - q[0])];
    return { P, p, phi: a, Js, Jv };
  }
  // eigen of symmetric 2×2 [[a,b],[b,c]] → values (desc) and unit vector of the largest
  function eig2(a, b, c) {
    const m = (a + c) / 2, d = Math.sqrt(((a - c) / 2) ** 2 + b * b);
    const l1 = m + d, l2 = m - d;
    let v = Math.abs(b) > 1e-12 ? [l1 - c, b] : (a >= c ? [1, 0] : [0, 1]);
    const nv = Math.hypot(v[0], v[1]); v = [v[0] / nv, v[1] / nv];
    return { l1, l2: Math.max(l2, 0), v };
  }
  function ellipse(Jv) {
    const n = Jv[0].length; let a = 0, b = 0, c = 0;
    for (let i = 0; i < n; i++) { a += Jv[0][i] ** 2; b += Jv[0][i] * Jv[1][i]; c += Jv[1][i] ** 2; }
    const e = eig2(a, b, c);
    return { A: [[a, b], [b, c]], s1: Math.sqrt(e.l1), s2: Math.sqrt(e.l2), v: e.v };
  }
  // one null-space direction of a 2×3 J: cross product of its two rows
  const null3 = (J) => {
    const r = J[0], s = J[1];
    const z = [r[1] * s[2] - r[2] * s[1], r[2] * s[0] - r[0] * s[2], r[0] * s[1] - r[1] * s[0]];
    const nz = Math.hypot(...z); return nz < 1e-12 ? [0, 0, 0] : z.map((x) => x / nz);
  };
  const tauOf = (Jv, f) => Jv[0].map((_, i) => Jv[0][i] * f[0] + Jv[1][i] * f[1]); // τ = J_vᵀ f
  const MATH = { arm, eig2, ellipse, null3, tauOf };
  if (typeof module !== "undefined" && module.exports) module.exports = MATH;

  const L = typeof window !== "undefined" && window.Lab; if (!L) return;
  const { colors, fmt, el, slider, canvas, deg, matrixHTML } = L;

  function select(parent, label, opts, onChange) {
    const row = el("div", { class: "lab-row" });
    const s = el("select"); opts.forEach((o) => s.appendChild(el("option", { value: o }, o)));
    s.addEventListener("change", onChange);
    row.append(el("span", { class: "lab-label" }, label), s); parent.appendChild(row); return s;
  }
  function drawArm(ctx, k, P, X, Y) {
    ctx.strokeStyle = k.primary; ctx.lineWidth = 6; ctx.lineCap = "round";
    ctx.beginPath(); P.forEach(([x, y], i) => (i ? ctx.lineTo(X(x), Y(y)) : ctx.moveTo(X(x), Y(y)))); ctx.stroke();
    ctx.fillStyle = k.fg; for (let i = 0; i < P.length - 1; i++) { ctx.beginPath(); ctx.arc(X(P[i][0]), Y(P[i][1]), 4, 0, 7); ctx.fill(); }
  }
  function arrow(ctx, x0, y0, x1, y1, col, w = 2.5) {
    const a = Math.atan2(y1 - y0, x1 - x0), h = 8;
    ctx.strokeStyle = col; ctx.fillStyle = col; ctx.lineWidth = w;
    ctx.beginPath(); ctx.moveTo(x0, y0); ctx.lineTo(x1, y1); ctx.stroke();
    if (Math.hypot(x1 - x0, y1 - y0) < 3) return;
    ctx.beginPath(); ctx.moveTo(x1, y1); ctx.lineTo(x1 - h * Math.cos(a - 0.4), y1 - h * Math.sin(a - 0.4));
    ctx.lineTo(x1 - h * Math.cos(a + 0.4), y1 - h * Math.sin(a + 0.4)); ctx.closePath(); ctx.fill();
  }

  // Widget 1: manipulability (velocity) ellipse and its inverse, the force ellipse
  L.register("m04-ellipse", function (root) {
    const ui = el("div", { class: "lab-controls" });
    const pics = el("div", { class: "lab-pics" });
    root.append(ui, pics);
    const C = canvas(pics, 380, 320);
    const side = el("div", { class: "lab-side" }); pics.appendChild(side);
    const mode = select(ui, "Arm", ["2R (L = 1.0, 0.8)", "3R (L = 1.0, 0.8, 0.5)"], draw);
    const show = select(ui, "Ellipse", ["velocity  (J Jᵀ)", "force  ((J Jᵀ)⁻¹)", "both"], draw);
    const t1 = slider(ui, "θ₁", -180, 180, 1, 20, draw, "°");
    const t2 = slider(ui, "θ₂", -180, 180, 1, 70, draw, "°");
    const t3 = slider(ui, "θ₃", -180, 180, 1, -40, draw, "°");
    function draw() {
      const k = colors(), ctx = C.ctx, s = 70, cx = 170, cy = 175;
      const three = mode.value.startsWith("3R"), Ls = three ? [1.0, 0.8, 0.5] : [1.0, 0.8];
      t3.parentElement.style.opacity = three ? 1 : 0.35; t3.disabled = !three;
      const th = [deg(+t1.value), deg(+t2.value), deg(+t3.value)].slice(0, Ls.length);
      const { P, p, Js, Jv } = arm(Ls, th), E = ellipse(Jv);
      const X = (x) => cx + s * x, Y = (y) => cy - s * y;
      ctx.clearRect(0, 0, 380, 320);
      const R = Ls.reduce((a, b) => a + b, 0);
      ctx.strokeStyle = k.light; ctx.lineWidth = 1; ctx.setLineDash([3, 3]);
      ctx.beginPath(); ctx.arc(cx, cy, s * R, 0, 7); ctx.stroke(); ctx.setLineDash([]);
      drawArm(ctx, k, P, X, Y);
      const ang = Math.atan2(E.v[1], E.v[0]), sc = 0.45; // ellipse drawing scale (m per unit)
      const drawE = (a1, a2, col, label) => {
        ctx.save(); ctx.translate(X(p[0]), Y(p[1])); ctx.rotate(-ang);
        ctx.strokeStyle = col; ctx.lineWidth = 2.5; ctx.beginPath();
        ctx.ellipse(0, 0, Math.max(s * a1, 0.8), Math.max(s * a2, 0.8), 0, 0, 7); ctx.stroke(); ctx.restore();
        ctx.fillStyle = col; ctx.font = "12px sans-serif"; ctx.fillText(label, 8, label.startsWith("v") ? 34 : 52);
      };
      const sing = E.s2 < 1e-3;
      if (!show.value.startsWith("force")) drawE(sc * E.s1, sc * E.s2, k.accent, "velocity ellipse: axes ∝ σ₁, σ₂ (unit joint speed)");
      if (!show.value.startsWith("velocity")) {
        const f1 = 1 / Math.max(E.s2, 1e-3), f2 = 1 / E.s1, cap = 3.5; // force axes ∝ 1/σ: long where velocity is short
        drawE(Math.min(sc * f2 * 0.5, cap), Math.min(sc * f1 * 0.5, cap), k.red, "force ellipse: axes ∝ 1/σ₂, 1/σ₁ (τ on unit circle)");
      }
      ctx.fillStyle = k.accent; ctx.beginPath(); ctx.arc(X(p[0]), Y(p[1]), 4, 0, 7); ctx.fill();
      ctx.fillStyle = k.fg; ctx.font = "12px sans-serif"; ctx.fillText("dashed = reach limit", 8, 16);
      let extra = "";
      if (three) {
        const nz = null3(Jv);
        // draw null-space self-motion: arm moved a little along the null direction (tip should stay put)
        const ghost = arm(Ls, th.map((t, i) => t + 0.35 * nz[i]));
        ctx.globalAlpha = 0.35; drawArm(ctx, k, ghost.P, X, Y); ctx.globalAlpha = 1;
        extra = `<br>Null space (3 joints, 2 tip numbers): dθ/dt ∝ (${nz.map((v) => fmt(v, 2)).join(", ")}) moves the elbow, not the tip (faint arm).`;
      }
      const cond = sing ? "∞" : fmt(E.s1 / E.s2, 2);
      side.innerHTML =
        matrixHTML(Js, "J<sub>s</sub> (rows ω, vₓ, v_y)") +
        matrixHTML(Jv, "J<sub>v</sub> (tip)") +
        `<div class="lab-checks">σ₁ = ${fmt(E.s1, 3)}, σ₂ = ${fmt(E.s2, 3)} (m/rad)<br>` +
        `condition number σ₁/σ₂ = <b>${cond}</b>, area measure σ₁σ₂ = ${fmt(E.s1 * E.s2, 3)}<br>` +
        `<b style="color:${sing ? k.red : k.green}">${sing ? "SINGULAR: rank drops, the ellipse is a line" : "full rank"}</b>` +
        `${extra}</div>`;
    }
    draw();
  });

  // Widget 2: statics τ = Jᵀ f for a tip force on a 2R arm
  L.register("m04-statics", function (root) {
    const Ls = [1.0, 0.8];
    const ui = el("div", { class: "lab-controls" });
    const pics = el("div", { class: "lab-pics" });
    root.append(ui, pics);
    const C = canvas(pics, 380, 300);
    const side = el("div", { class: "lab-side" }); pics.appendChild(side);
    const t1 = slider(ui, "θ₁", -180, 180, 1, 0, draw, "°");
    const t2 = slider(ui, "θ₂", -180, 180, 1, 90, draw, "°");
    const fa = slider(ui, "force direction", -180, 180, 1, -135, draw, "°");
    const fm = slider(ui, "force size", 0, 20, 0.5, 10, draw, " N");
    function draw() {
      const k = colors(), ctx = C.ctx, s = 75, cx = 110, cy = 170;
      const th = [deg(+t1.value), deg(+t2.value)];
      const { P, p, Jv } = arm(Ls, th);
      const f = [+fm.value * Math.cos(deg(+fa.value)), +fm.value * Math.sin(deg(+fa.value))];
      const tau = tauOf(Jv, f);
      const X = (x) => cx + s * x, Y = (y) => cy - s * y;
      ctx.clearRect(0, 0, 380, 300);
      drawArm(ctx, k, P, X, Y);
      const fs = 0.06; // m per N for drawing
      arrow(ctx, X(p[0]), Y(p[1]), X(p[0] + fs * f[0]), Y(p[1] + fs * f[1]), k.red);
      ctx.fillStyle = k.red; ctx.font = "12px sans-serif"; ctx.fillText("f", X(p[0] + fs * f[0]) + 6, Y(p[1] + fs * f[1]) + 4);
      // torque bars
      const bx = 300, maxT = 20 * 1.8;
      ctx.strokeStyle = k.light; ctx.lineWidth = 1; ctx.beginPath(); ctx.moveTo(bx - 10, 150); ctx.lineTo(370, 150); ctx.stroke();
      tau.forEach((t, i) => {
        const h = (t / maxT) * 120;
        ctx.fillStyle = i ? k.blue : k.green; ctx.fillRect(bx + i * 34, 150 - Math.max(h, 0), 22, Math.abs(h));
        ctx.fillStyle = k.fg; ctx.fillText("τ" + (i ? "₂" : "₁"), bx + 6 + i * 34, 150 + (tau[i] > 0 ? 16 : -6));
      });
      ctx.fillStyle = k.fg; ctx.fillText("joint torques", bx - 10, 16);
      // power check with a random joint velocity
      const qd = [0.7, -0.4], v = [Jv[0][0] * qd[0] + Jv[0][1] * qd[1], Jv[1][0] * qd[0] + Jv[1][1] * qd[1]];
      const pTip = v[0] * f[0] + v[1] * f[1], pJ = tau[0] * qd[0] + tau[1] * qd[1];
      side.innerHTML =
        matrixHTML(Jv, "J<sub>v</sub>") +
        `<div class="lab-checks">f = (${fmt(f[0], 2)}, ${fmt(f[1], 2)}) N<br>` +
        `τ = Jᵀ f = (<b>${fmt(tau[0], 2)}</b>, <b>${fmt(tau[1], 2)}</b>) N·m = motor torques that make the tip push with f<br>` +
        `power check with dθ/dt = (0.7, −0.4) rad/s: fᵀv = ${fmt(pTip, 4)} W, τᵀ dθ/dt = ${fmt(pJ, 4)} W<br>` +
        `aim f along link 2 and τ₂ → 0: the elbow carries none of it.</div>`;
    }
    draw();
  });
})();
