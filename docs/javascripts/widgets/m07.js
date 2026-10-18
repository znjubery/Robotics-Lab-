/* M7 widgets — dynamics of open chains.
   m07-pendulum2: 2R arm / double pendulum under gravity (point masses at link tips,
   Lynch & Park Example 8.1). Three integrators: explicit Euler, symplectic Euler
   (momentum form, fixed-point solve), RK4. Live energy-drift plot. */
(function () {
  "use strict";

  // ---------------- physics (pure functions, also testable in node) ----------------
  const P = { m1: 1, m2: 1, L1: 1, L2: 0.8, g: 9.81 };

  function massMatrix(th, p = P) {
    const c2 = Math.cos(th[1]);
    const M11 = p.m1 * p.L1 ** 2 + p.m2 * (p.L1 ** 2 + 2 * p.L1 * p.L2 * c2 + p.L2 ** 2);
    const M12 = p.m2 * (p.L1 * p.L2 * c2 + p.L2 ** 2);
    const M22 = p.m2 * p.L2 ** 2;
    return [[M11, M12], [M12, M22]];
  }
  // Coriolis/centripetal vector c(θ, θ̇)
  function coriolis(th, w, p = P) {
    const h = p.m2 * p.L1 * p.L2 * Math.sin(th[1]);
    return [-h * (2 * w[0] * w[1] + w[1] * w[1]), h * w[0] * w[0]];
  }
  // C matrix with c = C θ̇ (Christoffel choice)
  function Cmat(th, w, p = P) {
    const h = p.m2 * p.L1 * p.L2 * Math.sin(th[1]);
    return [[-h * w[1], -h * (w[0] + w[1])], [h * w[0], 0]];
  }
  function Mdot(th, w, p = P) {
    const h = p.m2 * p.L1 * p.L2 * Math.sin(th[1]);
    return [[-2 * h * w[1], -h * w[1]], [-h * w[1], 0]];
  }
  // gravity vector g(θ); θ measured from +x, gravity along −y
  function gravity(th, p = P) {
    const c1 = Math.cos(th[0]), c12 = Math.cos(th[0] + th[1]);
    return [(p.m1 + p.m2) * p.L1 * p.g * c1 + p.m2 * p.g * p.L2 * c12, p.m2 * p.g * p.L2 * c12];
  }
  const solve2 = (A, b) => {
    const d = A[0][0] * A[1][1] - A[0][1] * A[1][0];
    return [(A[1][1] * b[0] - A[0][1] * b[1]) / d, (-A[1][0] * b[0] + A[0][0] * b[1]) / d];
  };
  // forward dynamics: θ̈ = M⁻¹(τ − c − g − bθ̇)
  function accel(th, w, damp, p = P) {
    const M = massMatrix(th, p), c = coriolis(th, w, p), g = gravity(th, p);
    return solve2(M, [-c[0] - g[0] - damp * w[0], -c[1] - g[1] - damp * w[1]]);
  }
  function energy(th, w, p = P) {
    const M = massMatrix(th, p);
    const K = 0.5 * (w[0] * (M[0][0] * w[0] + M[0][1] * w[1]) + w[1] * (M[1][0] * w[0] + M[1][1] * w[1]));
    const y1 = p.L1 * Math.sin(th[0]), y2 = y1 + p.L2 * Math.sin(th[0] + th[1]);
    const V = p.m1 * p.g * y1 + p.m2 * p.g * y2;
    return { K, V, E: K + V };
  }
  // ---- steppers: state x = [θ1, θ2, θ̇1, θ̇2] ----
  function stepEuler(x, h, damp, p = P) {
    const a = accel([x[0], x[1]], [x[2], x[3]], damp, p);
    return [x[0] + h * x[2], x[1] + h * x[3], x[2] + h * a[0], x[3] + h * a[1]];
  }
  function stepRK4(x, h, damp, p = P) {
    const f = (s) => { const a = accel([s[0], s[1]], [s[2], s[3]], damp, p); return [s[2], s[3], a[0], a[1]]; };
    const add = (s, k, c) => s.map((v, i) => v + c * k[i]);
    const k1 = f(x), k2 = f(add(x, k1, h / 2)), k3 = f(add(x, k2, h / 2)), k4 = f(add(x, k3, h));
    return x.map((v, i) => v + (h / 6) * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]));
  }
  // Symplectic Euler on H(θ,p) = ½ pᵀM(θ)⁻¹p + V(θ):
  //   p⁺ = p − h ∂H/∂θ(θ, p⁺)   (implicit in p⁺ → fixed-point iterations)
  //   θ⁺ = θ + h M(θ)⁻¹ p⁺
  // ∂H/∂θ_k = −½ θ̇ᵀ (∂M/∂θ_k) θ̇ + g_k, with θ̇ = M⁻¹p. Damping added as −h·b·θ̇.
  function stepSymplectic(x, h, damp, pp = P) {
    const th = [x[0], x[1]], M = massMatrix(th, pp), g = gravity(th, pp);
    const mom = [M[0][0] * x[2] + M[0][1] * x[3], M[1][0] * x[2] + M[1][1] * x[3]];
    const k = pp.m2 * pp.L1 * pp.L2 * Math.sin(th[1]); // ∂M/∂θ2 = −k [[2,1],[1,0]]
    let pn = mom.slice();
    for (let it = 0; it < 8; it++) {
      const w = solve2(M, pn);
      const dH2 = 0.5 * k * (2 * w[0] * w[0] + 2 * w[0] * w[1]) + g[1];
      const dH1 = g[0];
      pn = [mom[0] - h * (dH1 + damp * w[0]), mom[1] - h * (dH2 + damp * w[1])];
    }
    const w = solve2(M, pn);
    const th2 = [th[0] + h * w[0], th[1] + h * w[1]];
    const w2 = solve2(massMatrix(th2, pp), pn);
    return [th2[0], th2[1], w2[0], w2[1]];
  }
  const phys = { P, massMatrix, coriolis, Cmat, Mdot, gravity, accel, energy, stepEuler, stepRK4, stepSymplectic };
  if (typeof module !== "undefined" && module.exports) module.exports = phys;
  if (typeof window === "undefined") return;

  // ---------------- widget ----------------
  const L = window.Lab; if (!L) return;
  const { colors, fmt, el, slider, canvas, deg } = L;

  L.register("m07-pendulum2", function (root) {
    const ui = el("div", { class: "lab-controls" });
    const pics = el("div", { class: "lab-pics" });
    root.append(ui, pics);
    const A = canvas(pics, 320, 300), G = canvas(pics, 400, 300);
    const out = el("div", { class: "lab-readout" }); root.appendChild(out);

    const row = el("div", { class: "lab-row" });
    const sel = el("select");
    for (const [v, t] of [["rk4", "RK4 (4th order)"], ["sym", "Symplectic Euler (1st order)"], ["eul", "Explicit Euler (1st order)"]])
      sel.appendChild(el("option", { value: v }, t));
    const btn = el("button", { class: "md-button" }, "Pause");
    const rst = el("button", { class: "md-button" }, "Reset");
    for (const b of [btn, rst]) { b.style.padding = "0.15em 0.9em"; b.style.fontSize = "0.7rem"; b.style.margin = "0"; }
    row.append(el("span", { class: "lab-label" }, "Integrator"), sel); ui.appendChild(row);
    const row2 = el("div", { class: "lab-row" });
    row2.append(el("span", { class: "lab-label" }, "Run"), btn, rst); ui.appendChild(row2);

    const a0 = slider(ui, "start θ₁", -180, 180, 1, 30, reset, "°");
    const b0 = slider(ui, "start θ₂", -180, 180, 1, 0, reset, "°");
    const hs = slider(ui, "time step h", 0.001, 0.03, 0.001, 0.01, reset, " s");
    const bs = slider(ui, "damping b (N·m·s)", 0, 1, 0.01, 0, reset, "");
    sel.addEventListener("change", reset);

    let x, t, E0, hist, trail, running = true, last = null;
    function reset() {
      x = [deg(+a0.value), deg(+b0.value), 0, 0]; t = 0;
      E0 = phys.energy([x[0], x[1]], [x[2], x[3]]).E; hist = [[0, 0]]; trail = [];
      draw();
    }
    btn.onclick = () => { running = !running; btn.textContent = running ? "Pause" : "Play"; };
    rst.onclick = reset;
    const Escale = (P.m1 + P.m2) * P.g * (P.L1 + P.L2); // ≈ 35.3 J, for relative drift

    function advance(simTime) {
      const h = +hs.value, b = +bs.value, n = Math.max(1, Math.round(simTime / h));
      const f = sel.value === "rk4" ? phys.stepRK4 : sel.value === "sym" ? phys.stepSymplectic : phys.stepEuler;
      for (let i = 0; i < n; i++) {
        x = f(x, h, b); t += h;
        if (!isFinite(x[0]) || Math.abs(x[2]) > 1e4) { running = false; btn.textContent = "Play"; break; }
      }
      const e = phys.energy([x[0], x[1]], [x[2], x[3]]).E;
      hist.push([t, (e - E0) / Escale]); if (hist.length > 3000) hist.splice(0, hist.length - 3000);
    }
    function loop(ts) {
      if (!document.body.contains(root)) return;
      if (running && last !== null) {
        const dtw = Math.min(0.05, (ts - last) / 1000);
        advance(dtw); draw();
      }
      last = ts; requestAnimationFrame(loop);
    }

    function draw() {
      const k = colors();
      // ---- arm ----
      let ctx = A.ctx, s = 76, cx = 160, cy = 146;
      ctx.clearRect(0, 0, 320, 300);
      ctx.strokeStyle = k.light; ctx.setLineDash([3, 3]);
      ctx.beginPath(); ctx.arc(cx, cy, s * (P.L1 + P.L2), 0, 7); ctx.stroke(); ctx.setLineDash([]);
      const x1 = cx + s * P.L1 * Math.cos(x[0]), y1 = cy - s * P.L1 * Math.sin(x[0]);
      const x2 = x1 + s * P.L2 * Math.cos(x[0] + x[1]), y2 = y1 - s * P.L2 * Math.sin(x[0] + x[1]);
      trail.push([x2, y2]); if (trail.length > 400) trail.shift();
      ctx.strokeStyle = k.accent; ctx.lineWidth = 1; ctx.globalAlpha = 0.6; ctx.beginPath();
      trail.forEach(([u, v], i) => (i ? ctx.lineTo(u, v) : ctx.moveTo(u, v))); ctx.stroke(); ctx.globalAlpha = 1;
      ctx.lineWidth = 5; ctx.lineCap = "round"; ctx.strokeStyle = k.primary;
      ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(x1, y1); ctx.lineTo(x2, y2); ctx.stroke();
      ctx.fillStyle = k.fg; ctx.beginPath(); ctx.arc(cx, cy, 4, 0, 7); ctx.fill();
      ctx.fillStyle = k.blue; ctx.beginPath(); ctx.arc(x1, y1, 7, 0, 7); ctx.fill();
      ctx.fillStyle = k.red; ctx.beginPath(); ctx.arc(x2, y2, 7, 0, 7); ctx.fill();
      ctx.fillStyle = k.fg; ctx.font = "12px sans-serif";
      ctx.fillText("t = " + fmt(t, 1) + " s   gravity ↓, τ = 0", 8, 16);
      ctx.fillText("example values: m₁ = m₂ = 1 kg, L₁ = 1, L₂ = 0.8 m", 8, 292);

      // ---- energy drift plot ----
      ctx = G.ctx; ctx.clearRect(0, 0, 400, 300);
      const l = 52, r = 390, top = 28, bot = 262;
      const tmax = Math.max(10, t);
      let ymax = 1e-6; for (const [, d] of hist) ymax = Math.max(ymax, Math.abs(d));
      const ex = Math.ceil(Math.log10(ymax)); ymax = Math.pow(10, ex) * (ymax > 0.5 * Math.pow(10, ex) ? 1 : 0.5);
      const X = (tt) => l + ((r - l) * tt) / tmax, Y = (d) => (top + bot) / 2 - ((bot - top) / 2) * (d / ymax);
      ctx.strokeStyle = k.light; ctx.lineWidth = 1; ctx.strokeRect(l, top, r - l, bot - top);
      ctx.beginPath(); ctx.moveTo(l, Y(0)); ctx.lineTo(r, Y(0)); ctx.stroke();
      ctx.fillStyle = k.fg; ctx.font = "12px sans-serif";
      ctx.fillText("energy drift (E − E₀) / " + fmt(Escale, 1) + " J", l, 18);
      ctx.fillText("+" + ymax.toExponential(0), 4, top + 10); ctx.fillText("−" + ymax.toExponential(0), 4, bot);
      ctx.fillText("0", 36, Y(0) + 4);
      ctx.fillText("0", l - 3, bot + 14); ctx.fillText(fmt(tmax, 0) + " s", r - 24, bot + 14);
      ctx.fillText("time →", (l + r) / 2 - 18, bot + 14);
      ctx.strokeStyle = sel.value === "eul" ? k.red : sel.value === "sym" ? k.green : k.blue; ctx.lineWidth = 1.8;
      ctx.beginPath(); hist.forEach(([tt, d], i) => (i ? ctx.lineTo(X(tt), Y(d)) : ctx.moveTo(X(tt), Y(d)))); ctx.stroke();

      // ---- readouts ----
      const th = [x[0], x[1]], w = [x[2], x[3]];
      const M = phys.massMatrix(th), en = phys.energy(th, w);
      const tr = M[0][0] + M[1][1], dt = M[0][0] * M[1][1] - M[0][1] ** 2, disc = Math.sqrt(Math.max(0, tr * tr / 4 - dt));
      const C = phys.Cmat(th, w), Md = phys.Mdot(th, w);
      const N12 = Md[0][1] - 2 * C[0][1], N21 = Md[1][0] - 2 * C[1][0], N11 = Md[0][0] - 2 * C[0][0];
      const drift = hist[hist.length - 1][1];
      out.innerHTML =
        `M(θ) = [[${fmt(M[0][0], 2)}, ${fmt(M[0][1], 2)}], [${fmt(M[1][0], 2)}, ${fmt(M[1][1], 2)}]] &nbsp; eigenvalues ${fmt(tr / 2 + disc, 2)}, ${fmt(tr / 2 - disc, 2)} &gt; 0 → positive definite.<br>` +
        `Ṁ − 2C = [[${fmt(N11, 2)}, ${fmt(N12, 2)}], [${fmt(N21, 2)}, ${fmt(Md[1][1] - 2 * C[1][1], 2)}]] → skew (off-diagonals opposite, diagonal 0).<br>` +
        `Energy: K = ${fmt(en.K, 3)} J, V = ${fmt(en.V, 3)} J, E = ${fmt(en.E, 4)} J (start ${fmt(E0, 4)} J). ` +
        `Relative drift <b>${drift.toExponential(2)}</b>` +
        (+bs.value > 0 ? ` — damping b > 0 removes energy on purpose, so E must fall.` : ` — with τ = 0 and b = 0 the exact answer is 0.`);
    }
    reset();
    requestAnimationFrame(loop);
  });
})();
