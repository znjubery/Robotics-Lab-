/* M3 · Forward kinematics widgets: m03-poe (planar 3R PoE), m03-so101 (5-axis spatial PoE, example values). */
(function () {
  "use strict";

  // ---------- pure math (also testable in node) ----------
  const mm = (A, B) => A.map((r, i) => B[0].map((_, j) => r.reduce((s, _, k) => s + A[i][k] * B[k][j], 0)));
  // planar revolute screw S = (ω=1, v) with v = (q_y, −q_x): e^{[S]θ} = [R  (I−R)q; 0 1]
  function exp2(q, th) {
    const c = Math.cos(th), s = Math.sin(th);
    const px = (1 - c) * q[0] + s * q[1], py = -s * q[0] + (1 - c) * q[1];
    return [[c, -s, px], [s, c, py], [0, 0, 1]];
  }
  // same thing written the textbook way: p = [ s  −(1−c) ; 1−c  s ] v   (ω = 1)
  function exp2v(v, th) {
    const c = Math.cos(th), s = Math.sin(th);
    return [[c, -s, s * v[0] - (1 - c) * v[1]], [s, c, (1 - c) * v[0] + s * v[1]], [0, 0, 1]];
  }
  function fk3rPoE(Ls, th) {
    // space form: T = e^{[S1]θ1} e^{[S2]θ2} e^{[S3]θ3} M
    const x = [0, Ls[0], Ls[0] + Ls[1]], Lt = Ls[0] + Ls[1] + Ls[2];
    const S = x.map((xi) => [1, 0, -xi]); // (ω, v_x, v_y), v = −ω̂ × q = (q_y, −q_x)
    const M = [[1, 0, Lt], [0, 1, 0], [0, 0, 1]];
    let T = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
    for (let i = 0; i < 3; i++) T = mm(T, exp2v([S[i][1], S[i][2]], th[i]));
    const Ts = mm(T, M);
    // body form: T = M e^{[B1]θ1} e^{[B2]θ2} e^{[B3]θ3},  B_i = (1, 0, Lt − x_i)
    let Tb = M;
    for (let i = 0; i < 3; i++) Tb = mm(Tb, exp2v([0, Lt - x[i]], th[i]));
    return { S, M, Ts, Tb };
  }
  function fk3rTrig(Ls, th) {
    const a1 = th[0], a2 = a1 + th[1], a3 = a2 + th[2];
    const x = Ls[0] * Math.cos(a1) + Ls[1] * Math.cos(a2) + Ls[2] * Math.cos(a3);
    const y = Ls[0] * Math.sin(a1) + Ls[1] * Math.sin(a2) + Ls[2] * Math.sin(a3);
    return [[Math.cos(a3), -Math.sin(a3), x], [Math.sin(a3), Math.cos(a3), y], [0, 0, 1]];
  }
  // spatial zero-pitch screw: e^{[S]θ} = [R (I−R)q; 0 1], R = Rodrigues(ω̂, θ)
  function rod(w, t) {
    const K = [[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]], K2 = mm(K, K);
    return [0, 1, 2].map((i) => [0, 1, 2].map((j) => (i === j ? 1 : 0) + Math.sin(t) * K[i][j] + (1 - Math.cos(t)) * K2[i][j]));
  }
  function exp3(w, q, t) {
    const R = rod(w, t);
    const p = [0, 1, 2].map((i) => q[i] - (R[i][0] * q[0] + R[i][1] * q[1] + R[i][2] * q[2]));
    return [[...R[0], p[0]], [...R[1], p[1]], [...R[2], p[2]], [0, 0, 0, 1]];
  }
  const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
  // SO-ARM101-like structure: base yaw, 3 parallel pitch joints, wrist roll. Lengths are EXAMPLE values (m).
  const ARM = { d1: 0.10, L2: 0.11, L3: 0.13, L4: 0.10 };
  function armAxes(A = ARM) {
    const w = [[0, 0, 1], [0, 1, 0], [0, 1, 0], [0, 1, 0], [1, 0, 0]];
    const q = [[0, 0, 0], [0, 0, A.d1], [A.L2, 0, A.d1], [A.L2 + A.L3, 0, A.d1], [A.L2 + A.L3, 0, A.d1]];
    const S = w.map((wi, i) => wi.concat(cross(wi, q[i]).map((x) => -x)));
    const M = [[1, 0, 0, A.L2 + A.L3 + A.L4], [0, 1, 0, 0], [0, 0, 1, A.d1], [0, 0, 0, 1]];
    return { w, q, S, M };
  }
  function armFK(th, A = ARM) {
    const { w, q, M } = armAxes(A);
    let T = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]];
    const pts = [[0, 0, 0]];
    for (let i = 0; i < 5; i++) {
      // where joint i sits now = home point q_i carried by joints 1..i−1
      const h = mm(T, [[q[i][0]], [q[i][1]], [q[i][2]], [1]]);
      pts.push([h[0][0], h[1][0], h[2][0]]);
      T = mm(T, exp3(w[i], q[i], th[i]));
    }
    const Tt = mm(T, M);
    pts.push([Tt[0][3], Tt[1][3], Tt[2][3]]);
    return { T: Tt, pts };
  }
  const MATH = { mm, exp2, exp2v, fk3rPoE, fk3rTrig, exp3, armAxes, armFK, ARM };
  if (typeof module !== "undefined" && module.exports) module.exports = MATH;

  // ---------- widgets ----------
  const L = typeof window !== "undefined" && window.Lab; if (!L) return;
  const { colors, fmt, el, slider, canvas, deg, project, drawFrame, matrixHTML } = L;
  const maxDiff = (A, B) => Math.max(...A.flat().map((v, i) => Math.abs(v - B.flat()[i])));

  // Widget 1: planar 3R arm, screw axes on the straight "ruler", live PoE vs trig
  L.register("m03-poe", function (root) {
    const Ls = [1.0, 0.8, 0.5];
    const ui = el("div", { class: "lab-controls" });
    const pics = el("div", { class: "lab-pics" });
    root.append(ui, pics);
    const C = canvas(pics, 380, 320);
    const side = el("div", { class: "lab-side" }); pics.appendChild(side);
    const t1 = slider(ui, "θ₁", -180, 180, 1, 30, draw, "°");
    const t2 = slider(ui, "θ₂", -180, 180, 1, 60, draw, "°");
    const t3 = slider(ui, "θ₃", -180, 180, 1, -90, draw, "°");
    function draw() {
      const k = colors(), ctx = C.ctx, s = 72, cx = 120, cy = 210;
      const th = [deg(+t1.value), deg(+t2.value), deg(+t3.value)];
      const { S, M, Ts, Tb } = fk3rPoE(Ls, th), Tg = fk3rTrig(Ls, th);
      const X = (x) => cx + s * x, Y = (y) => cy - s * y;
      ctx.clearRect(0, 0, 380, 320);
      ctx.strokeStyle = k.light; ctx.lineWidth = 1; ctx.setLineDash([2, 4]);
      ctx.beginPath(); ctx.moveTo(0, cy); ctx.lineTo(380, cy); ctx.moveTo(cx, 0); ctx.lineTo(cx, 320); ctx.stroke(); ctx.setLineDash([]);
      // home configuration = the straight ruler, with hinge marks (space-frame screw axes)
      ctx.globalAlpha = 0.35; ctx.strokeStyle = k.light; ctx.lineWidth = 6; ctx.lineCap = "round";
      ctx.beginPath(); ctx.moveTo(X(0), Y(0)); ctx.lineTo(X(M[0][2]), Y(0)); ctx.stroke(); ctx.globalAlpha = 1;
      ctx.font = "12px sans-serif";
      S.forEach((Si, i) => {
        const qx = -Si[2];
        ctx.fillStyle = k.red; ctx.beginPath(); ctx.arc(X(qx), Y(0), 4.5, 0, 7); ctx.fill();
        ctx.fillText("S" + (i + 1), X(qx) - 6, Y(0) + 18);
      });
      ctx.fillStyle = k.light; ctx.fillText("M (home)", X(M[0][2]) - 20, Y(0) + 18);
      // current arm from trig joint positions
      const a1 = th[0], a2 = a1 + th[1];
      const P = [[0, 0], [Ls[0] * Math.cos(a1), Ls[0] * Math.sin(a1)]];
      P.push([P[1][0] + Ls[1] * Math.cos(a2), P[1][1] + Ls[1] * Math.sin(a2)]);
      P.push([Tg[0][2], Tg[1][2]]);
      ctx.strokeStyle = k.primary; ctx.lineWidth = 6;
      ctx.beginPath(); P.forEach(([x, y], i) => (i ? ctx.lineTo(X(x), Y(y)) : ctx.moveTo(X(x), Y(y)))); ctx.stroke();
      ctx.fillStyle = k.fg; for (let i = 0; i < 3; i++) { ctx.beginPath(); ctx.arc(X(P[i][0]), Y(P[i][1]), 4, 0, 7); ctx.fill(); }
      // tip frame from the PoE result
      const px = Ts[0][2], py = Ts[1][2], ax = 0.35;
      ctx.lineWidth = 2.5;
      ctx.strokeStyle = k.red; ctx.beginPath(); ctx.moveTo(X(px), Y(py)); ctx.lineTo(X(px + ax * Ts[0][0]), Y(py + ax * Ts[1][0])); ctx.stroke();
      ctx.strokeStyle = k.green; ctx.beginPath(); ctx.moveTo(X(px), Y(py)); ctx.lineTo(X(px + ax * Ts[0][1]), Y(py + ax * Ts[1][1])); ctx.stroke();
      ctx.fillStyle = k.accent; ctx.beginPath(); ctx.arc(X(px), Y(py), 5, 0, 7); ctx.fill();
      ctx.fillStyle = k.fg; ctx.fillText("{b}", X(px) + 8, Y(py) - 8);
      ctx.fillText("faint = home M with hinge marks S₁ S₂ S₃", 6, 16);
      const e1 = maxDiff(Ts, Tg), e2 = maxDiff(Tb, Tg);
      side.innerHTML =
        `Screw axes (ω, vₓ, v_y), read off the straight arm:<br>` +
        S.map((Si, i) => `S${i + 1} = (${fmt(Si[0], 0)}, ${fmt(Si[1], 1)}, ${fmt(Si[2], 1)})`).join("<br>") +
        matrixHTML(Ts, "T = e<sup>[S₁]θ₁</sup>e<sup>[S₂]θ₂</sup>e<sup>[S₃]θ₃</sup>M") +
        `<div class="lab-checks">plain trig: x = ${fmt(Tg[0][2], 3)}, y = ${fmt(Tg[1][2], 3)}, φ = ${fmt(+t1.value + +t2.value + +t3.value, 0)}°<br>` +
        `space form vs trig: max error ${e1.toExponential(1)}<br>body form M e<sup>[B₁]θ₁</sup>⋯ vs trig: ${e2.toExponential(1)}</div>`;
    }
    draw();
  });

  // Widget 2: SO-ARM101-like 5R spatial arm (example lengths), 4×4 PoE
  L.register("m03-so101", function (root) {
    const ui = el("div", { class: "lab-controls" });
    const pics = el("div", { class: "lab-pics" });
    root.append(ui, pics);
    const C = canvas(pics, 380, 320);
    const side = el("div", { class: "lab-side" }); pics.appendChild(side);
    const names = ["θ₁ base yaw", "θ₂ shoulder", "θ₃ elbow", "θ₄ wrist pitch", "θ₅ wrist roll"];
    const init = [30, -40, 60, 20, 0];
    const sl = names.map((n, i) => slider(ui, n, -180, 180, 1, init[i], draw, "°"));
    const { w, q, S } = armAxes();
    function draw() {
      const k = colors(), ctx = C.ctx, s = 700, cx = 120, cy = 250;
      const th = sl.map((x) => deg(+x.value));
      const { T, pts } = armFK(th);
      const P = (p) => project(p, cx, cy, s);
      ctx.clearRect(0, 0, 380, 320);
      // floor axes of {s}
      drawFrame(ctx, [[1, 0, 0], [0, 1, 0], [0, 0, 1]], cx, cy, 70, 0.3, 1.2);
      // home arm and space-frame screw axes (fixed lines drawn at home)
      const home = armFK([0, 0, 0, 0, 0]).pts;
      ctx.globalAlpha = 0.3; ctx.strokeStyle = k.light; ctx.lineWidth = 5; ctx.lineCap = "round";
      ctx.beginPath(); home.forEach((p, i) => { const [x, y] = P(p); i ? ctx.lineTo(x, y) : ctx.moveTo(x, y); }); ctx.stroke();
      ctx.globalAlpha = 1;
      ctx.font = "12px sans-serif"; ctx.setLineDash([4, 3]); ctx.lineWidth = 1.4;
      for (let i = 0; i < 5; i++) {
        const a = q[i].map((v, j) => v - 0.05 * w[i][j]), b = q[i].map((v, j) => v + 0.05 * w[i][j]);
        const [x0, y0] = P(a), [x1, y1] = P(b);
        ctx.strokeStyle = k.red; ctx.beginPath(); ctx.moveTo(x0, y0); ctx.lineTo(x1, y1); ctx.stroke();
        ctx.fillStyle = k.red; ctx.fillText("S" + (i + 1), x1 + 3, y1 + [-2, -6, 14, -6, 14][i]);
      }
      ctx.setLineDash([]);
      // current arm
      ctx.strokeStyle = k.primary; ctx.lineWidth = 6;
      ctx.beginPath(); pts.forEach((p, i) => { const [x, y] = P(p); i ? ctx.lineTo(x, y) : ctx.moveTo(x, y); }); ctx.stroke();
      ctx.fillStyle = k.fg; pts.slice(0, -1).forEach((p) => { const [x, y] = P(p); ctx.beginPath(); ctx.arc(x, y, 3.5, 0, 7); ctx.fill(); });
      const R = [0, 1, 2].map((i) => T[i].slice(0, 3)), [tx, ty] = P(pts[pts.length - 1]);
      drawFrame(ctx, R, tx, ty, 40, 1, 2.5);
      ctx.fillStyle = k.fg; ctx.fillText("faint = home M, red dashes = screw axes S₁…S₅", 6, 16);
      side.innerHTML =
        `S<sub>i</sub> = (ω̂<sub>i</sub>, −ω̂<sub>i</sub>×q<sub>i</sub>), example lengths in m:<br>` +
        S.map((Si, i) => `S${i + 1} = (${Si.map((v) => fmt(v, 2)).join(", ")})`).join("<br>") +
        matrixHTML(T, "T<sub>sb</sub>") +
        `<div class="lab-checks">tip p = (${fmt(T[0][3], 3)}, ${fmt(T[1][3], 3)}, ${fmt(T[2][3], 3)}) m<br>` +
        `5 joints, 6 pose numbers: the tool axis always stays in the arm's vertical plane (one direction is missing).</div>`;
    }
    draw();
  });
})();
