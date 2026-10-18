/* Track E widgets: te-ridge — one regularized least-squares equation, three skins */
(function () {
  "use strict";

  // ---------------- numerics core (testable in node) ----------------
  function rng(seed) { let a = seed >>> 0; return function () { a = (a + 0x6D2B79F5) >>> 0; let t = a; t = Math.imul(t ^ (t >>> 15), t | 1); t ^= t + Math.imul(t ^ (t >>> 7), t | 61); return ((t ^ (t >>> 14)) >>> 0) / 4294967296; }; }
  function gauss(r) { const u = Math.max(r(), 1e-12), v = r(); return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v); }
  const zeros = (m, n) => Array.from({ length: m }, () => new Array(n).fill(0));
  const dot = (a, b) => a.reduce((s, v, i) => s + v * b[i], 0);
  const norm = (a) => Math.sqrt(dot(a, a));
  const mv = (A, x) => A.map((r) => dot(r, x));
  const mtv = (A, y) => { const n = A[0].length, o = new Array(n).fill(0); A.forEach((r, i) => r.forEach((v, j) => (o[j] += v * y[i]))); return o; };

  // cyclic Jacobi eigen-decomposition of a symmetric matrix: S = V diag(d) Vᵀ
  function jacobiEig(S0) {
    const n = S0.length, S = S0.map((r) => r.slice()), V = zeros(n, n);
    for (let i = 0; i < n; i++) V[i][i] = 1;
    for (let sweep = 0; sweep < 60; sweep++) {
      let off = 0; for (let i = 0; i < n; i++) for (let j = i + 1; j < n; j++) off += S[i][j] * S[i][j];
      if (off < 1e-22) break;
      for (let p = 0; p < n; p++) for (let q = p + 1; q < n; q++) {
        if (Math.abs(S[p][q]) < 1e-300) continue;
        const th = (S[q][q] - S[p][p]) / (2 * S[p][q]);
        const t = Math.sign(th || 1) / (Math.abs(th) + Math.sqrt(th * th + 1)), c = 1 / Math.sqrt(t * t + 1), s = t * c;
        for (let k = 0; k < n; k++) { const a = S[k][p], b = S[k][q]; S[k][p] = c * a - s * b; S[k][q] = s * a + c * b; }
        for (let k = 0; k < n; k++) { const a = S[p][k], b = S[q][k]; S[p][k] = c * a - s * b; S[q][k] = s * a + c * b; }
        for (let k = 0; k < n; k++) { const a = V[k][p], b = V[k][q]; V[k][p] = c * a - s * b; V[k][q] = s * a + c * b; }
      }
    }
    const idx = [...Array(n).keys()].sort((a, b) => S[b][b] - S[a][a]);
    return { d: idx.map((i) => S[i][i]), V: idx.map((i) => V.map((r) => r[i])) }; // V[k] = k-th eigenvector
  }
  // thin SVD via the smaller Gram matrix. Returns sigma, U (list of m-vectors), Vt (list of n-vectors)
  function svd(A) {
    const m = A.length, n = A[0].length, sig = [], U = [], Vt = [];
    if (m <= n) {
      const G = A.map((ri) => A.map((rj) => dot(ri, rj))), e = jacobiEig(G);
      e.d.forEach((d, k) => { const s = Math.sqrt(Math.max(d, 0)); if (s < 1e-10) return; sig.push(s); U.push(e.V[k]); Vt.push(mtv(A, e.V[k]).map((v) => v / s)); });
    } else {
      const G = zeros(n, n); for (const r of A) for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) G[i][j] += r[i] * r[j];
      const e = jacobiEig(G);
      e.d.forEach((d, k) => { const s = Math.sqrt(Math.max(d, 0)); if (s < 1e-10) return; sig.push(s); Vt.push(e.V[k]); U.push(mv(A, e.V[k]).map((v) => v / s)); });
    }
    return { sig, U, Vt, m, n };
  }
  // ridge / Tikhonov / LM step through the SVD: x(λ) = Σ σᵢ/(σᵢ²+λ²) (uᵢᵀb) vᵢ
  function ridgeSVD(F, b, lam) {
    const beta = F.U.map((u) => dot(u, b)), x = new Array(F.n).fill(0), l2 = lam * lam;
    let res2 = Math.max(dot(b, b) - beta.reduce((s, v) => s + v * v, 0), 0), dof = 0;
    F.sig.forEach((s, i) => {
      const c = (s / (s * s + l2)) * beta[i];
      for (let j = 0; j < F.n; j++) x[j] += c * F.Vt[i][j];
      const r = (l2 / (s * s + l2)) * beta[i]; res2 += r * r; dof += (s * s) / (s * s + l2);
    });
    const smin = F.sig.length < F.n ? 0 : F.sig[F.sig.length - 1];
    return { x, xn: norm(x), rn: Math.sqrt(res2), dof, kappa: (F.sig[0] ** 2 + l2) / (smin * smin + l2) };
  }
  // direct check: solve (AᵀA + λ²I) x = Aᵀb by Gaussian elimination
  function ridgeDirect(A, b, lam) {
    const n = A[0].length, M = zeros(n, n + 1);
    for (const [k, r] of A.entries()) for (let i = 0; i < n; i++) { for (let j = 0; j < n; j++) M[i][j] += r[i] * r[j]; M[i][n] += r[i] * b[k]; }
    for (let i = 0; i < n; i++) M[i][i] += lam * lam;
    for (let i = 0; i < n; i++) { let p = i; for (let k = i + 1; k < n; k++) if (Math.abs(M[k][i]) > Math.abs(M[p][i])) p = k; [M[i], M[p]] = [M[p], M[i]];
      for (let k = i + 1; k < n; k++) { const f = M[k][i] / M[i][i]; for (let j = i; j <= n; j++) M[k][j] -= f * M[i][j]; } }
    const x = new Array(n).fill(0); for (let i = n - 1; i >= 0; i--) { let s = M[i][n]; for (let j = i + 1; j < n; j++) s -= M[i][j] * x[j]; x[i] = s / M[i][i]; }
    return x;
  }

  // --- skin 1: IK step on a 3R planar arm (M5) ---
  const ARM = [1.0, 0.8, 0.6]; // example link lengths
  function fk3(t) { let x = 0, y = 0, a = 0; const pts = [[0, 0]]; for (let i = 0; i < 3; i++) { a += t[i]; x += ARM[i] * Math.cos(a); y += ARM[i] * Math.sin(a); pts.push([x, y]); } return pts; }
  function jac3(t) { const P = fk3(t), e = P[3], J = [[0, 0, 0], [0, 0, 0]]; for (let i = 0; i < 3; i++) { J[0][i] = -(e[1] - P[i][1]); J[1][i] = e[0] - P[i][0]; } return J; }
  function ikProblem() {
    const th = [0.35, 0.12, 0.08], P = fk3(th), goal = [1.55, 1.75]; // nearly stretched arm, target to the upper left of the reach
    const e = [goal[0] - P[3][0], goal[1] - P[3][1]];
    return { th, goal, A: jac3(th), b: e };
  }
  function ikIterate(th0, goal, lam, iters) {
    let th = th0.slice();
    for (let k = 0; k < iters; k++) { const P = fk3(th), e = [goal[0] - P[3][0], goal[1] - P[3][1]]; const d = ridgeDirect(jac3(th), e, lam); th = th.map((v, i) => v + d[i]); }
    const P = fk3(th); return { th, err: Math.hypot(goal[0] - P[3][0], goal[1] - P[3][1]) };
  }

  // --- skin 2: GBLUP / ridge on synthetic markers ---
  function gblupProblem() {
    const r = rng(2026), p = 200, nTr = 40, nTe = 60, n = nTr + nTe;
    const freq = Array.from({ length: p }, () => 0.15 + 0.7 * r());
    const Z = Array.from({ length: n }, () => freq.map((f) => (r() < f ? 1 : 0) + (r() < f ? 1 : 0)));
    // centre and scale each marker, then divide by √p so that G = XXᵀ has diagonal ≈ 1 (VanRaden-style)
    for (let j = 0; j < p; j++) { const mu = Z.reduce((s, z) => s + z[j], 0) / n; const sd = Math.sqrt(Z.reduce((s, z) => s + (z[j] - mu) ** 2, 0) / n) || 1; Z.forEach((z) => (z[j] = (z[j] - mu) / sd / Math.sqrt(p))); }
    const beta = Array.from({ length: p }, () => gauss(r)); // σ_b² = 1 → var(g) ≈ 1
    const g = Z.map((z) => dot(z, beta)), y = g.map((v) => v + gauss(r)); // σ_e² = 1 → h² ≈ 0.5
    return { A: Z.slice(0, nTr), b: y.slice(0, nTr), Xte: Z.slice(nTr), yte: y.slice(nTr), gte: g.slice(nTr), beta };
  }
  // --- skin 3: Levenberg–Marquardt fit of y = a·e^(−b·x) + c ---
  const fexp = (p, x) => p[0] * Math.exp(-p[1] * x) + p[2];
  function lmData() { const r = rng(99), xs = [], ys = []; for (let i = 0; i < 20; i++) { const x = (4 * i) / 19; xs.push(x); ys.push(fexp([2, 1.3, 0.5], x) + 0.06 * gauss(r)); } return { xs, ys }; }
  function lmLin(p, d) { const A = d.xs.map((x) => [Math.exp(-p[1] * x), -p[0] * x * Math.exp(-p[1] * x), 1]); const b = d.xs.map((x, i) => d.ys[i] - fexp(p, x)); return { A, b }; }
  function lmIterate(p0, d, lam, iters) {
    let p = p0.slice(); const cost = [0.5 * norm(lmLin(p, d).b) ** 2];
    for (let k = 0; k < iters; k++) { const { A, b } = lmLin(p, d); const dp = ridgeDirect(A, b, lam); p = p.map((v, i) => v + dp[i]); cost.push(0.5 * norm(lmLin(p, d).b) ** 2); }
    return { p, cost };
  }
  const core = { svd, ridgeSVD, ridgeDirect, ikProblem, ikIterate, fk3, gblupProblem, lmData, lmLin, lmIterate, dot, norm, mv };
  if (typeof module !== "undefined" && module.exports) module.exports = core;
  if (typeof window === "undefined") return;

  // ---------------- widget ----------------
  const L = window.Lab; if (!L) return;
  const { colors, fmt, el, slider, canvas } = L;

  L.register("te-ridge", function (root) {
    const ui = el("div", { class: "lab-controls" });
    const pics = el("div", { class: "lab-pics" });
    root.append(ui, pics);
    const sel = el("select");
    for (const [v, t] of [["ik", "IK step (M5)"], ["gb", "GBLUP / ridge"], ["lm", "Levenberg–Marquardt curve fit"]]) sel.appendChild(el("option", { value: v }, t));
    const row = el("div", { class: "lab-row" }); row.append(el("span", { class: "lab-label" }, "Skin"), sel); ui.appendChild(row);
    const lg = slider(ui, "log₁₀ λ", -3, 1.5, 0.05, -0.5, draw, "");
    sel.addEventListener("change", draw);
    const P = canvas(pics, 360, 280), Lc = canvas(pics, 300, 280), Fc = canvas(pics, 360, 120);
    const out = el("div", { class: "lab-readout" }); root.appendChild(out);

    const ik = ikProblem(), gb = gblupProblem(), lmd = lmData(), p0 = [1, 0.3, 0];
    const lm = lmLin(p0, lmd);
    const F = { ik: svd(ik.A), gb: svd(gb.A), lm: svd(lm.A) };
    const B = { ik: ik.b, gb: gb.b, lm: lm.b };
    const lams = Array.from({ length: 91 }, (_, i) => Math.pow(10, -3 + (4.5 * i) / 90));
    const sweep = {};
    for (const k of ["ik", "gb", "lm"]) sweep[k] = lams.map((l) => ridgeSVD(F[k], B[k], l));
    const gbTest = lams.map((l, i) => { const x = sweep.gb[i].x, pr = gb.Xte.map((z) => dot(z, x)); return pr.reduce((s, v, j) => s + (v - gb.yte[j]) ** 2, 0) / pr.length; });
    const gbTrain = lams.map((l, i) => (sweep.gb[i].rn ** 2) / gb.b.length);
    function corr(a, b) { const n = a.length, ma = a.reduce((s, v) => s + v, 0) / n, mb = b.reduce((s, v) => s + v, 0) / n; let sab = 0, saa = 0, sbb = 0; for (let i = 0; i < n; i++) { sab += (a[i] - ma) * (b[i] - mb); saa += (a[i] - ma) ** 2; sbb += (b[i] - mb) ** 2; } return sab / Math.sqrt(saa * sbb); }

    function axes(ctx, x0, y0, x1, y1, c) { ctx.strokeStyle = c.light; ctx.lineWidth = 1; ctx.beginPath(); ctx.moveTo(x0, y0); ctx.lineTo(x0, y1); ctx.lineTo(x1, y1); ctx.stroke(); }

    function draw() {
      const c = colors(), skin = sel.value, lam = Math.pow(10, +lg.value);
      const sol = ridgeSVD(F[skin], B[skin], lam), Fk = F[skin];
      // ---- picture panel ----
      const g = P.ctx; g.clearRect(0, 0, 360, 280); g.font = "12px sans-serif";
      let extra = "";
      if (skin === "ik") {
        const S = 80, cx = 70, cy = 230, X = (p) => cx + S * p[0], Y = (p) => cy - S * p[1];
        g.strokeStyle = c.light; g.setLineDash([3, 3]); g.beginPath(); g.arc(cx, cy, S * 2.4, -Math.PI / 2, 0); g.stroke(); g.setLineDash([]);
        const arm = (th, col, w, a) => { const pts = fk3(th); g.globalAlpha = a; g.strokeStyle = col; g.lineWidth = w; g.lineCap = "round"; g.beginPath(); pts.forEach((p, i) => (i ? g.lineTo(X(p), Y(p)) : g.moveTo(X(p), Y(p)))); g.stroke(); g.fillStyle = c.fg; pts.slice(0, 3).forEach((p) => { g.beginPath(); g.arc(X(p), Y(p), 3, 0, 7); g.fill(); }); g.globalAlpha = 1; return pts[3]; };
        arm(ik.th, c.light, 5, 0.8);
        const th1 = ik.th.map((v, i) => v + sol.x[i]), tip1 = arm(th1, c.accent, 4, 1);
        const it = ikIterate(ik.th, ik.goal, lam, 15); arm(it.th, c.green, 2, 0.8);
        g.fillStyle = c.red; g.beginPath(); g.arc(X(ik.goal), Y(ik.goal), 5, 0, 7); g.fill();
        g.fillStyle = c.fg; g.fillText("grey = start, amber = one damped step, green = 15 steps", 8, 16); g.fillText("target", X(ik.goal) + 8, Y(ik.goal) + 4);
        const e1 = Math.hypot(ik.goal[0] - tip1[0], ik.goal[1] - tip1[1]);
        extra = `A = J(θ) (2×3), x = Δθ, b = e (tip error, ‖e‖ = ${fmt(norm(ik.b), 3)}). After one step the true tip error is <b>${fmt(e1, 3)}</b>; after 15 steps <b>${fmt(it.err, 4)}</b>. |Δθ| = ${fmt((sol.xn * 180) / Math.PI, 1)}°.`;
      } else if (skin === "gb") {
        // top: marker effects (first 50), bottom: train/test error vs λ
        const m = 50, bw = 320 / m, top = 24, h = 90, mid = top + h / 2;
        g.strokeStyle = c.light; g.beginPath(); g.moveTo(30, mid); g.lineTo(350, mid); g.stroke();
        const sc = 18;
        for (let j = 0; j < m; j++) {
          const v = sol.x[j]; // estimate and truth share units
          g.fillStyle = c.accent; const hh = Math.max(-h / 2, Math.min(h / 2, v * sc)); g.fillRect(30 + j * bw, mid - Math.max(hh, 0), bw - 1, Math.abs(hh));
          g.fillStyle = c.primary; g.beginPath(); g.arc(30 + j * bw + bw / 2, mid - Math.max(-h / 2, Math.min(h / 2, gb.beta[j] * sc)), 1.8, 0, 7); g.fill();
        }
        g.fillStyle = c.fg; g.fillText("marker effects 1–50: bars = estimate, dots = truth", 8, 16);
        const x0 = 40, x1 = 350, y0 = 150, y1 = 260, lx = (l) => x0 + ((Math.log10(l) + 3) / 4.5) * (x1 - x0), ly = (v) => y1 - (Math.min(v, 3) / 3) * (y1 - y0);
        axes(g, x0, y0, x1, y1, c);
        const curve = (arr, col) => { g.strokeStyle = col; g.lineWidth = 2; g.beginPath(); arr.forEach((v, i) => (i ? g.lineTo(lx(lams[i]), ly(v)) : g.moveTo(lx(lams[i]), ly(v)))); g.stroke(); };
        curve(gbTrain, c.light); curve(gbTest, c.primary);
        g.strokeStyle = c.green; g.setLineDash([3, 3]); g.beginPath(); g.moveTo(lx(1), y0); g.lineTo(lx(1), y1); g.stroke(); g.setLineDash([]);
        g.strokeStyle = c.fg; g.beginPath(); g.moveTo(lx(lam), y0); g.lineTo(lx(lam), y1); g.stroke();
        g.fillStyle = c.fg; g.fillText("MSE vs λ: grey = train, blue = test", x0, y0 - 6); g.fillStyle = c.green; g.fillText("BLUP λ = σₑ/σ_b = 1", lx(1) + 3, y0 + 12);
        g.fillStyle = c.fg; g.fillText("0", x0 - 12, y1 + 4); g.fillText("3", x0 - 12, y0 + 4); g.fillText("log λ →", x1 - 44, y1 + 14);
        const pr = gb.Xte.map((z) => dot(z, sol.x)), i = Math.round(((Math.log10(lam) + 3) / 4.5) * 90);
        extra = `A = X (40 lines × 200 markers), x = marker effects β, b = phenotypes y. Ridge on markers = GBLUP on lines. Test MSE <b>${fmt(gbTest[i], 3)}</b>, train MSE ${fmt(gbTrain[i], 3)}; accuracy r(ŷ, true g) on 60 new lines = <b>${fmt(corr(pr, gb.gte), 2)}</b>.`;
      } else {
        const x0 = 40, x1 = 350, y0 = 26, y1 = 250, X = (x) => x0 + (x / 4) * (x1 - x0), Y = (y) => y1 - ((y + 0.2) / 2.9) * (y1 - y0);
        axes(g, x0, y0, x1, y1, c);
        g.fillStyle = c.fg; lmd.xs.forEach((x, i) => { g.beginPath(); g.arc(X(x), Y(lmd.ys[i]), 3, 0, 7); g.fill(); });
        const curve = (p, col, w, dash) => { g.strokeStyle = col; g.lineWidth = w; g.setLineDash(dash); g.beginPath(); for (let i = 0; i <= 80; i++) { const x = (4 * i) / 80, y = Math.max(-0.2, Math.min(2.7, fexp(p, x))); i ? g.lineTo(X(x), Y(y)) : g.moveTo(X(x), Y(y)); } g.stroke(); g.setLineDash([]); };
        const p1 = p0.map((v, i) => v + sol.x[i]), it = lmIterate(p0, lmd, lam, 15);
        curve(p0, c.light, 2, []); curve(p1, c.accent, 2.5, []); curve(it.p, c.green, 2, [5, 4]);
        g.fillStyle = c.fg; g.fillText("fit y = a·e^(−b·x) + c   grey = start, amber = 1 step, green = 15 steps", 8, 16);
        // mini cost history
        g.fillText("cost per iteration", 230, 70);
        const cm = Math.max(...it.cost.filter(isFinite)), hx = (k) => 230 + k * 7, hy = (v) => 130 - (Math.log10(Math.max(v, 1e-4)) + 4) / (Math.log10(cm) + 4) * 50;
        g.strokeStyle = c.primary; g.lineWidth = 1.5; g.beginPath(); it.cost.forEach((v, k) => (k ? g.lineTo(hx(k), hy(v)) : g.moveTo(hx(k), hy(v)))); g.stroke();
        extra = `A = ∂f/∂p (20 points × 3 parameters), x = Δp, b = residual r (cost ½‖r‖² = ${fmt(it.cost[0], 3)}). After one step: (a, b, c) = (${p1.map((v) => fmt(v, 2)).join(", ")}); after 15 steps cost <b>${fmt(it.cost[15], 4)}</b>, (a, b, c) = (${it.p.map((v) => fmt(v, 2)).join(", ")}). True (2, 1.3, 0.5).`;
      }
      // ---- L-curve ----
      const h = Lc.ctx; h.clearRect(0, 0, 300, 280); h.font = "12px sans-serif";
      const sw = sweep[skin], lr = sw.map((s) => Math.log10(Math.max(s.rn, 1e-6))), lx_ = sw.map((s) => Math.log10(Math.max(s.xn, 1e-6)));
      const rmin = Math.min(...lr), rmax = Math.max(...lr), xmin = Math.min(...lx_), xmax = Math.max(...lx_);
      const ax = (v) => 50 + ((v - rmin) / (rmax - rmin || 1)) * 230, ay = (v) => 240 - ((v - xmin) / (xmax - xmin || 1)) * 200;
      axes(h, 50, 30, 285, 240, c);
      h.strokeStyle = c.primary; h.lineWidth = 2; h.beginPath(); lr.forEach((v, i) => (i ? h.lineTo(ax(v), ay(lx_[i])) : h.moveTo(ax(v), ay(lx_[i])))); h.stroke();
      h.fillStyle = c.accent; h.strokeStyle = c.fg; h.lineWidth = 1; h.beginPath(); h.arc(ax(Math.log10(Math.max(sol.rn, 1e-6))), ay(Math.log10(Math.max(sol.xn, 1e-6))), 6, 0, 7); h.fill(); h.stroke();
      h.fillStyle = c.fg; h.fillText("L-curve (log–log)", 50, 18);
      h.fillText("residual ‖Ax − b‖ →", 150, 262); h.save(); h.translate(16, 200); h.rotate(-Math.PI / 2); h.fillText("solution size ‖x‖ →", 0, 0); h.restore();
      h.fillStyle = c.light; h.fillText("small λ", ax(lr[0]) + 6, ay(lx_[0]) + 14); h.fillText("big λ", ax(lr[90]) - 34, ay(lx_[90]) - 8);
      // ---- filter factors ----
      const f = Fc.ctx; f.clearRect(0, 0, 360, 120); f.font = "12px sans-serif";
      const ns = Fk.sig.length, bw = Math.min(40, 300 / ns);
      axes(f, 40, 24, 350, 100, c);
      Fk.sig.forEach((s, i) => { const ff = (s * s) / (s * s + lam * lam); f.fillStyle = c.accent; f.fillRect(42 + i * bw, 100 - 76 * ff, Math.max(bw - 1, 1), 76 * ff); });
      f.fillStyle = c.fg; f.fillText("filter factors fᵢ = σᵢ²/(σᵢ² + λ²), one bar per singular value σᵢ (largest left)", 4, 14);
      f.fillText("1", 28, 30); f.fillText("0", 28, 102); f.fillText(`σ: ${fmt(Fk.sig[0], 2)} … ${fmt(Fk.sig[ns - 1], 3)}`, 44, 116);
      out.innerHTML = `<b>Same equation:</b> x̂ = (AᵀA + λ²I)⁻¹ Aᵀb, λ = ${fmt(lam, 3)}. ${extra}<br>` +
        `‖x̂‖ = ${fmt(sol.xn, 3)}, ‖Ax̂ − b‖ = ${fmt(sol.rn, 3)}, effective parameters Σfᵢ = <b>${fmt(sol.dof, 2)}</b> of ${Fk.n}, condition number of AᵀA + λ²I = ${sol.kappa > 1e6 ? sol.kappa.toExponential(1) : fmt(sol.kappa, 1)}.`;
    }
    draw();
  });
})();
