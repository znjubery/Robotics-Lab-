/* m06 widgets: closed chains — four-bar loop closure, assembly modes, Grashof, coupler curve */
(function () {
  "use strict";
  const L = window.Lab; if (!L) return;
  const { colors, fmt, el, slider, canvas, deg } = L;

  // Pivots A=(0,0), D=(g,0). Crank a at angle t2. Coupler b (B→C). Rocker c (D→C).
  // Loop closure: a e^{i t2} + b e^{i t3} − c e^{i t4} − g = 0
  function solve(g, a, b, c, t2, mode) {
    const Bx = a * Math.cos(t2), By = a * Math.sin(t2);
    const dx = g - Bx, dy = -By, d = Math.hypot(dx, dy);
    if (d > b + c || d < Math.abs(b - c) || d < 1e-12) return null;
    const m = (b * b - c * c + d * d) / (2 * d), h = Math.sqrt(Math.max(0, b * b - m * m));
    const ux = dx / d, uy = dy / d;
    const Cx = Bx + m * ux - mode * h * uy, Cy = By + m * uy + mode * h * ux;
    const t3 = Math.atan2(Cy - By, Cx - Bx), t4 = Math.atan2(Cy, Cx - g);
    return { B: [Bx, By], C: [Cx, Cy], t3, t4 };
  }
  function grashof(g, a, b, c) {
    const Ls = [["ground", g], ["crank", a], ["coupler", b], ["rocker", c]].sort((x, y) => x[1] - y[1]);
    const s = Ls[0][1], l = Ls[3][1], pq = Ls[1][1] + Ls[2][1];
    if (Math.abs(s + l - pq) < 1e-9) return { ok: true, s, l, pq, type: "change-point (s + l = p + q): passes through a flat, folded pose" };
    if (s + l > pq) return { ok: false, s, l, pq, type: "non-Grashof triple-rocker: no link turns fully" };
    const t = { ground: "double-crank (drag link): crank and rocker both turn fully", crank: "crank-rocker: crank turns 360°, rocker swings", coupler: "double-rocker: coupler turns fully, crank and rocker swing", rocker: "rocker-crank: the output link turns fully, input only swings" }[Ls[0][0]];
    return { ok: true, s, l, pq, type: t };
  }
  // Passive rates: J_p [θ̇3, θ̇4]ᵀ = −J_a θ̇2 with θ̇2 = 1 (Cramer's rule)
  // J_p = [[−b s3, c s4], [b c3, −c c4]], J_a = [−a s2, a c2]ᵀ, det J_p = −bc sin(θ4 − θ3)
  function rates(a, b, c, t2, t3, t4) {
    const r0 = a * Math.sin(t2), r1 = -a * Math.cos(t2);
    const p00 = -b * Math.sin(t3), p01 = c * Math.sin(t4), p10 = b * Math.cos(t3), p11 = -c * Math.cos(t4);
    const det = p00 * p11 - p01 * p10;
    if (Math.abs(det) < 1e-9) return { w3: NaN, w4: NaN, det };
    return { w3: (r0 * p11 - p01 * r1) / det, w4: (p00 * r1 - p10 * r0) / det, det };
  }
  L.m06 = { solve, grashof, rates };

  L.register("m06-fourbar", function (root) {
    const ui = el("div", { class: "lab-controls" });
    const pics = el("div", { class: "lab-pics" });
    root.append(ui, pics);
    const C = canvas(pics, 420, 320);
    const out = el("div", { class: "lab-readout" }); root.appendChild(out);
    const th = slider(ui, "crank θ₂ (active)", -180, 180, 1, 60, draw, "°");
    const sg = slider(ui, "ground g", 0.5, 2.5, 0.05, 2.0, draw, "");
    const sa = slider(ui, "crank a", 0.2, 2.0, 0.05, 0.6, draw, "");
    const sb = slider(ui, "coupler b", 0.2, 2.5, 0.05, 1.8, draw, "");
    const sc = slider(ui, "rocker c", 0.2, 2.0, 0.05, 1.2, draw, "");
    const su = slider(ui, "coupler point (along b)", -0.5, 1.5, 0.05, 0.5, draw, "");
    const sv = slider(ui, "coupler point (sideways)", -1, 1, 0.05, 0.5, draw, "");
    const rowA = el("div", { class: "lab-row" }), anim = el("input", { type: "checkbox" });
    rowA.append(el("span", { class: "lab-label" }, "drive the crank (animate)"), anim); ui.appendChild(rowA);
    let raf = 0;
    anim.addEventListener("change", () => { if (anim.checked) tick(); else cancelAnimationFrame(raf); });
    function tick() {
      if (!anim.checked || !root.isConnected) return;
      let v = +th.value + 2; if (v > 180) v -= 360; th.value = v; th.dispatchEvent(new Event("input"));
      raf = requestAnimationFrame(tick);
    }
    const cpt = (s, u, v, b) => {
      const ex = (s.C[0] - s.B[0]) / b, ey = (s.C[1] - s.B[1]) / b;
      return [s.B[0] + u * b * ex - v * ey, s.B[1] + u * b * ey + v * ex];
    };
    function draw() {
      const k = colors(), ctx = C.ctx;
      const g = +sg.value, a = +sa.value, b = +sb.value, c = +sc.value, u = +su.value, v = +sv.value, t2 = deg(+th.value);
      const span = Math.max(g + c, a + b, g) + 0.6, S = Math.min(380 / (span + Math.max(a, c, 0.8)), 260 / (2 * Math.max(a, c, b) + 0.4), 120);
      const cx = Math.max(20 + S * a, 210 - S * g / 2), cy = 170;
      const X = (p) => cx + S * p[0], Y = (p) => cy - S * p[1];
      ctx.clearRect(0, 0, 420, 320);
      // coupler curves, both assembly modes
      const modeCol = [k.primary, k.accent];
      [1, -1].forEach((mode, mi) => {
        ctx.strokeStyle = modeCol[mi]; ctx.globalAlpha = 0.55; ctx.lineWidth = 1.5; ctx.setLineDash(mi ? [4, 3] : []);
        let pen = false; ctx.beginPath();
        for (let i = 0; i <= 360; i++) {
          const s = solve(g, a, b, c, deg(-180 + i), mode);
          if (!s) { pen = false; continue; }
          const p = cpt(s, u, v, b);
          pen ? ctx.lineTo(X(p), Y(p)) : ctx.moveTo(X(p), Y(p)); pen = true;
        }
        ctx.stroke(); ctx.setLineDash([]); ctx.globalAlpha = 1;
      });
      // ground
      const A0 = [0, 0], D0 = [g, 0];
      ctx.strokeStyle = k.light; ctx.lineWidth = 3; ctx.setLineDash([6, 4]);
      ctx.beginPath(); ctx.moveTo(X(A0), Y(A0)); ctx.lineTo(X(D0), Y(D0)); ctx.stroke(); ctx.setLineDash([]);
      for (const p of [A0, D0]) { ctx.fillStyle = k.light; ctx.beginPath(); ctx.moveTo(X(p), Y(p)); ctx.lineTo(X(p) - 9, Y(p) + 13); ctx.lineTo(X(p) + 9, Y(p) + 13); ctx.closePath(); ctx.fill(); }
      const sols = [solve(g, a, b, c, t2, 1), solve(g, a, b, c, t2, -1)];
      const drawLinks = (s, color, alpha, w) => {
        ctx.globalAlpha = alpha; ctx.lineWidth = w; ctx.lineCap = "round"; ctx.lineJoin = "round"; ctx.strokeStyle = color;
        const P = cpt(s, u, v, b);
        ctx.beginPath(); ctx.moveTo(X(s.B), Y(s.B)); ctx.lineTo(X(s.C), Y(s.C)); ctx.lineTo(X(P), Y(P)); ctx.closePath();
        ctx.fillStyle = color; ctx.globalAlpha = alpha * 0.18; ctx.fill(); ctx.globalAlpha = alpha; ctx.stroke();
        ctx.beginPath(); ctx.moveTo(X(D0), Y(D0)); ctx.lineTo(X(s.C), Y(s.C)); ctx.stroke();
        ctx.fillStyle = k.fg; for (const p of [s.B, s.C]) { ctx.beginPath(); ctx.arc(X(p), Y(p), 4, 0, 7); ctx.fill(); }
        ctx.fillStyle = color; ctx.beginPath(); ctx.arc(X(P), Y(P), 5, 0, 7); ctx.fill();
        ctx.globalAlpha = 1;
      };
      if (sols[1]) drawLinks(sols[1], k.accent, 0.45, 4);
      if (sols[0]) drawLinks(sols[0], k.primary, 1, 6);
      // crank (active) — always drawn
      const B = [a * Math.cos(t2), a * Math.sin(t2)];
      ctx.strokeStyle = sols[0] ? k.red : k.light; ctx.lineWidth = 6; ctx.lineCap = "round";
      ctx.beginPath(); ctx.moveTo(X(A0), Y(A0)); ctx.lineTo(X(B), Y(B)); ctx.stroke();
      ctx.fillStyle = k.fg; for (const p of [A0, D0, B]) { ctx.beginPath(); ctx.arc(X(p), Y(p), 4, 0, 7); ctx.fill(); }
      ctx.font = "12px sans-serif"; ctx.fillStyle = k.fg;
      ctx.fillText("red = active crank · solid = mode +1 · faint/dashed = mode −1", 8, 16);
      ctx.fillText("A", X(A0) - 14, Y(A0) + 4); ctx.fillText("D", X(D0) + 8, Y(D0) + 4);
      // readouts
      const G = grashof(g, a, b, c);
      let txt = `Grashof: s + l = ${fmt(G.s, 2)} + ${fmt(G.l, 2)} = ${fmt(G.s + G.l, 2)} ${G.s + G.l <= G.pq + 1e-9 ? "≤" : "&gt;"} p + q = ${fmt(G.pq, 2)} → <b>${G.type}</b>.<br>`;
      if (!sols[0]) {
        const d = Math.hypot(g - B[0], B[1]);
        txt += `<b style="color:${k.red}">Loop cannot close</b> at θ₂ = ${th.value}°: distance B→D = ${fmt(d, 3)} is outside [|b − c|, b + c] = [${fmt(Math.abs(b - c), 2)}, ${fmt(b + c, 2)}]. The crank cannot get here.`;
      } else {
        const s = sols[0];
        const res = Math.hypot(a * Math.cos(t2) + b * Math.cos(s.t3) - c * Math.cos(s.t4) - g, a * Math.sin(t2) + b * Math.sin(s.t3) - c * Math.sin(s.t4));
        const detJ = -b * c * Math.sin(s.t4 - s.t3);
        let mu = Math.abs(((s.t4 - s.t3) * 180) / Math.PI) % 360; if (mu > 180) mu = 360 - mu; const muT = mu > 90 ? 180 - mu : mu;
        const { w3: w3c, w4 } = rates(a, b, c, t2, s.t3, s.t4);
        txt += `Mode +1: θ₃ = ${fmt((s.t3 * 180) / Math.PI, 1)}°, θ₄ = ${fmt((s.t4 * 180) / Math.PI, 1)}°. ` +
          (sols[1] ? `Mode −1: θ₃ = ${fmt((sols[1].t3 * 180) / Math.PI, 1)}°, θ₄ = ${fmt((sols[1].t4 * 180) / Math.PI, 1)}°.` : "") +
          `<br>Loop-closure residual ‖g(θ)‖ = ${res.toExponential(1)}. ` +
          `det of passive-joint constraint Jacobian = −bc·sin(θ₄ − θ₃) = <b>${fmt(detJ, 3)}</b>` +
          (Math.abs(detJ) < 0.05 * b * c ? ` <b style="color:${k.red}">≈ 0: singularity — the two modes merge</b>` : "") +
          `. Transmission angle μ = ${fmt(muT, 1)}° (keep above ~40°).<br>` +
          `Velocity ratio from the Jacobian: θ̇₃/θ̇₂ = ${isFinite(w3c) ? fmt(w3c, 3) : "∞"}, θ̇₄/θ̇₂ = ${isFinite(w4) ? fmt(w4, 3) : "∞"}.`;
      }
      out.innerHTML = txt;
    }
    draw();
  });
})();
