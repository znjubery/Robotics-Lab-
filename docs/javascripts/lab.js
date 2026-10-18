/* Robotics Learning Lab — widgets and self-checks.
   Every number is computed live in the browser. No external libraries. */
(function () {
  "use strict";

  // ---------- helpers ----------
  const css = (name, fallback) => {
    const v = getComputedStyle(document.body).getPropertyValue(name).trim();
    return v || fallback;
  };
  const colors = () => ({
    fg: css("--md-default-fg-color", "#222"),
    light: css("--md-default-fg-color--lighter", "#aaa"),
    bg: css("--md-default-bg-color", "#fff"),
    accent: css("--md-accent-fg-color", "#ffb300"),
    primary: css("--md-primary-fg-color", "#3f51b5"),
    red: "#e53935", green: "#43a047", blue: "#1e88e5",
  });
  const fmt = (x, d = 3) => {
    const v = Math.abs(x) < 5e-10 ? 0 : x;
    return v.toFixed(d);
  };
  const el = (tag, attrs = {}, html = "") => {
    const e = document.createElement(tag);
    for (const [k, v] of Object.entries(attrs)) e.setAttribute(k, v);
    if (html) e.innerHTML = html;
    return e;
  };
  function slider(parent, label, min, max, step, value, onInput, unit = "") {
    const row = el("div", { class: "lab-row" });
    const span = el("span", { class: "lab-label" }, label);
    const inp = el("input", { type: "range", min, max, step, value });
    const out = el("output", { class: "lab-val" });
    const show = () => (out.textContent = Number(inp.value).toFixed(step < 1 ? 2 : 0) + unit);
    inp.addEventListener("input", () => { show(); onInput(); });
    row.append(span, inp, out); parent.appendChild(row); show();
    return inp;
  }
  function canvas(parent, w, h) {
    const c = el("canvas", { width: w * 2, height: h * 2, class: "lab-canvas" });
    c.style.width = "100%"; c.style.maxWidth = w + "px"; c.style.aspectRatio = w + " / " + h;
    parent.appendChild(c);
    const ctx = c.getContext("2d"); ctx.scale(2, 2);
    return { c, ctx, w, h };
  }
  const deg = (d) => (d * Math.PI) / 180;

  // ---------- 3x3 linear algebra ----------
  const matmul = (A, B) => A.map((r, i) => B[0].map((_, j) => r.reduce((s, _, k) => s + A[i][k] * B[k][j], 0)));
  const transpose = (A) => A[0].map((_, j) => A.map((r) => r[j]));
  const det3 = (A) =>
    A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) -
    A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
    A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
  const skew = (w) => [[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]];
  // Rodrigues: R = I + sin(t)[w] + (1-cos t)[w]^2
  function rodrigues(w, t) {
    const K = skew(w), K2 = matmul(K, K);
    const I = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
    return I.map((r, i) => r.map((v, j) => v + Math.sin(t) * K[i][j] + (1 - Math.cos(t)) * K2[i][j]));
  }
  const apply = (R, p) => R.map((r) => r[0] * p[0] + r[1] * p[1] + r[2] * p[2]);
  // fixed oblique view for drawing 3D on a 2D canvas
  function project(p, cx, cy, s) {
    const az = deg(35), el_ = deg(25);
    const x1 = Math.cos(az) * p[0] - Math.sin(az) * p[1];
    const y1 = Math.sin(az) * p[0] + Math.cos(az) * p[1];
    const z2 = Math.cos(el_) * p[2] - Math.sin(el_) * y1;
    return [cx + s * x1, cy - s * z2];
  }
  function drawFrame(ctx, R, cx, cy, s, alpha = 1, width = 2.5) {
    const cols = ["#e53935", "#43a047", "#1e88e5"], names = ["x", "y", "z"];
    ctx.globalAlpha = alpha;
    for (let k = 0; k < 3; k++) {
      const e = [0, 0, 0]; e[k] = 1;
      const q = apply(R, e);
      const [x0, y0] = project([0, 0, 0], cx, cy, s), [x1, y1] = project(q, cx, cy, s);
      ctx.strokeStyle = cols[k]; ctx.lineWidth = width;
      ctx.beginPath(); ctx.moveTo(x0, y0); ctx.lineTo(x1, y1); ctx.stroke();
      ctx.fillStyle = cols[k]; ctx.font = "12px sans-serif"; ctx.fillText(names[k], x1 + 3, y1 - 3);
    }
    ctx.globalAlpha = 1;
  }
  function drawBox(ctx, R, cx, cy, s, color) {
    const a = 0.45, b = 0.25, c = 0.12; // a flat "phone/leaf" box makes orientation obvious
    const V = [];
    for (const x of [-a, a]) for (const y of [-b, b]) for (const z of [-c, c]) V.push(project(apply(R, [x, y, z]), cx, cy, s));
    const E = [[0,1],[0,2],[0,4],[1,3],[1,5],[2,3],[2,6],[3,7],[4,5],[4,6],[5,7],[6,7]];
    ctx.strokeStyle = color; ctx.lineWidth = 1.5;
    for (const [i, j] of E) { ctx.beginPath(); ctx.moveTo(...V[i]); ctx.lineTo(...V[j]); ctx.stroke(); }
  }
  function matrixHTML(R, label) {
    return `<div class="lab-matrix"><span>${label} =</span><table>${R.map((r) => "<tr>" + r.map((v) => `<td>${fmt(v)}</td>`).join("") + "</tr>").join("")}</table></div>`;
  }

  // ---------- Widget 1: 2R arm and its configuration space (torus unrolled) ----------
  function cspace(root) {
    const L1 = 1.0, L2 = 0.8;
    const ui = el("div", { class: "lab-controls" });
    const pics = el("div", { class: "lab-pics" });
    root.append(ui, pics);
    const W = canvas(pics, 300, 300), C = canvas(pics, 300, 300);
    const readout = el("div", { class: "lab-readout" }); root.appendChild(readout);
    const t1 = slider(ui, "θ₁ (shoulder)", -180, 180, 1, 40, draw, "°");
    const t2 = slider(ui, "θ₂ (elbow)", -180, 180, 1, 60, draw, "°");
    const ox = slider(ui, "obstacle x", -1.6, 1.6, 0.05, 0.9, recompute, "");
    const oy = slider(ui, "obstacle y", -1.6, 1.6, 0.05, -0.6, recompute, "");
    const orr = slider(ui, "obstacle radius", 0, 0.6, 0.05, 0.3, recompute, "");
    const N = 120; let grid = null;

    function segHitsCircle(ax, ay, bx, by, cx, cy, r) {
      const dx = bx - ax, dy = by - ay, L = dx * dx + dy * dy;
      let t = ((cx - ax) * dx + (cy - ay) * dy) / L; t = Math.max(0, Math.min(1, t));
      const px = ax + t * dx - cx, py = ay + t * dy - cy;
      return px * px + py * py <= r * r;
    }
    function collides(a, b) {
      const x1 = L1 * Math.cos(a), y1 = L1 * Math.sin(a);
      const x2 = x1 + L2 * Math.cos(a + b), y2 = y1 + L2 * Math.sin(a + b);
      const cx = +ox.value, cy = +oy.value, r = +orr.value;
      return r > 0 && (segHitsCircle(0, 0, x1, y1, cx, cy, r) || segHitsCircle(x1, y1, x2, y2, cx, cy, r));
    }
    function recompute() {
      grid = new Uint8Array(N * N); let bad = 0;
      for (let i = 0; i < N; i++) for (let j = 0; j < N; j++) {
        const a = -Math.PI + (2 * Math.PI * (i + 0.5)) / N, b = -Math.PI + (2 * Math.PI * (j + 0.5)) / N;
        if (collides(a, b)) { grid[i * N + j] = 1; bad++; }
      }
      grid.frac = bad / (N * N); draw();
    }
    C.c.addEventListener("click", (ev) => {
      const r = C.c.getBoundingClientRect();
      const u = (ev.clientX - r.left) / r.width, v = (ev.clientY - r.top) / r.height;
      t1.value = Math.round(-180 + 360 * u); t2.value = Math.round(180 - 360 * v);
      t1.dispatchEvent(new Event("input")); t2.dispatchEvent(new Event("input"));
    });
    function draw() {
      const k = colors(), a = deg(+t1.value), b = deg(+t2.value);
      // workspace
      let ctx = W.ctx, s = 75, cx = 150, cy = 150;
      ctx.clearRect(0, 0, 300, 300);
      ctx.strokeStyle = k.light; ctx.setLineDash([3, 3]);
      ctx.beginPath(); ctx.arc(cx, cy, s * (L1 + L2), 0, 2 * Math.PI); ctx.stroke();
      ctx.beginPath(); ctx.arc(cx, cy, s * (L1 - L2), 0, 2 * Math.PI); ctx.stroke(); ctx.setLineDash([]);
      ctx.fillStyle = "rgba(229,57,53,0.35)";
      ctx.beginPath(); ctx.arc(cx + s * ox.value, cy - s * oy.value, s * orr.value, 0, 2 * Math.PI); ctx.fill();
      const x1 = cx + s * L1 * Math.cos(a), y1 = cy - s * L1 * Math.sin(a);
      const x2 = x1 + s * L2 * Math.cos(a + b), y2 = y1 - s * L2 * Math.sin(a + b);
      const hit = collides(a, b);
      ctx.lineWidth = 6; ctx.lineCap = "round"; ctx.strokeStyle = hit ? k.red : k.primary;
      ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(x1, y1); ctx.lineTo(x2, y2); ctx.stroke();
      ctx.fillStyle = k.fg; for (const [x, y] of [[cx, cy], [x1, y1]]) { ctx.beginPath(); ctx.arc(x, y, 4, 0, 7); ctx.fill(); }
      ctx.fillStyle = k.accent; ctx.beginPath(); ctx.arc(x2, y2, 5, 0, 7); ctx.fill();
      ctx.fillStyle = k.fg; ctx.font = "12px sans-serif"; ctx.fillText("Workspace (task space)", 8, 16);
      // C-space
      ctx = C.ctx; ctx.clearRect(0, 0, 300, 300);
      if (grid) {
        const cell = 300 / N; ctx.fillStyle = "rgba(229,57,53,0.55)";
        for (let i = 0; i < N; i++) for (let j = 0; j < N; j++) if (grid[i * N + j]) ctx.fillRect(i * cell, 300 - (j + 1) * cell, cell + 0.5, cell + 0.5);
      }
      ctx.strokeStyle = k.light; ctx.strokeRect(0.5, 0.5, 299, 299);
      ctx.setLineDash([2, 4]); ctx.beginPath(); ctx.moveTo(150, 0); ctx.lineTo(150, 300); ctx.moveTo(0, 150); ctx.lineTo(300, 150); ctx.stroke(); ctx.setLineDash([]);
      const u = ((+t1.value + 180) / 360) * 300, v = 300 - ((+t2.value + 180) / 360) * 300;
      ctx.fillStyle = hit ? k.red : k.accent; ctx.strokeStyle = k.fg;
      ctx.beginPath(); ctx.arc(u, v, 6, 0, 7); ctx.fill(); ctx.stroke();
      ctx.fillStyle = k.fg; ctx.fillText("C-space: θ₁ →, θ₂ ↑ (edges wrap = torus)", 8, 16);
      const ex = L1 * Math.cos(a) + L2 * Math.cos(a + b), ey = L1 * Math.sin(a) + L2 * Math.sin(a + b);
      readout.innerHTML =
        `Configuration q = (θ₁, θ₂) = (${t1.value}°, ${t2.value}°) &nbsp;→&nbsp; tip (x, y) = (${fmt(ex, 2)}, ${fmt(ey, 2)}).<br>` +
        `Status: <b style="color:${hit ? k.red : k.green}">${hit ? "COLLISION — this point is inside a C-obstacle" : "free"}</b>. ` +
        `C-obstacle covers <b>${grid ? (100 * grid.frac).toFixed(1) : 0}%</b> of C-space. Click the right panel to jump there.`;
    }
    recompute();
  }

  // ---------- Widget 2: Grübler calculator ----------
  function grubler(root) {
    const presets = {
      "Four-bar linkage (planar)": { m: 3, N: 4, joints: [1, 1, 1, 1] },
      "Slider-crank (planar)": { m: 3, N: 4, joints: [1, 1, 1, 1] },
      "2R planar arm": { m: 3, N: 3, joints: [1, 1] },
      "SO-ARM101 arm, 5 revolute (spatial)": { m: 6, N: 6, joints: [1, 1, 1, 1, 1] },
      "Generic 6R industrial arm": { m: 6, N: 7, joints: [1, 1, 1, 1, 1, 1] },
      "Stewart platform 6-UPS": { m: 6, N: 14, joints: Array(6).fill(2).concat(Array(6).fill(1), Array(6).fill(3)) },
      "Crazyflie (free flyer, no joints)": { m: 6, N: 2, joints: [6] },
      "Differential-drive rover on floor (planar)": { m: 3, N: 2, joints: [3] },
    };
    const ui = el("div", { class: "lab-controls" }); root.appendChild(ui);
    const sel = el("select"); for (const k of Object.keys(presets)) sel.appendChild(el("option", { value: k }, k));
    const row = el("div", { class: "lab-row" }); row.append(el("span", { class: "lab-label" }, "Mechanism"), sel); ui.appendChild(row);
    const out = el("div", { class: "lab-readout" }); root.appendChild(out);
    function run() {
      const P = presets[sel.value], J = P.joints.length, sumf = P.joints.reduce((a, b) => a + b, 0);
      const dof = P.m * (P.N - 1 - J) + sumf;
      out.innerHTML =
        `m = ${P.m} (${P.m === 3 ? "planar" : "spatial"} body), N = ${P.N} links incl. ground, J = ${J} joints, Σfᵢ = ${sumf}<br>` +
        `dof = m(N − 1 − J) + Σfᵢ = ${P.m}(${P.N} − 1 − ${J}) + ${sumf} = <b style="font-size:1.2em">${dof}</b>` +
        (sel.value.includes("rover") ? `<br><i>Careful:</i> the rover has 3 C-space DOF (x, y, φ) but only 2 velocity inputs — a nonholonomic constraint (M12). Grübler counts positions, not allowed velocities.` : "") +
        (sel.value.includes("Crazyflie") ? `<br>Modelled as one "joint" with 6 freedoms between drone and world: 3 position + 3 orientation = SE(3).` : "") +
        (sel.value.includes("SO-ARM101") ? `<br>Plus the gripper jaw = 6 actuators total. The arm alone cannot reach every orientation at every point: 5 &lt; 6.` : "");
    }
    sel.addEventListener("change", run); run();
  }

  // ---------- Widget 3: rotation matrix from axis-angle (Rodrigues) ----------
  function rotation(root) {
    const ui = el("div", { class: "lab-controls" });
    const pics = el("div", { class: "lab-pics" });
    root.append(ui, pics);
    const D = canvas(pics, 300, 280);
    const right = el("div", { class: "lab-side" }); pics.appendChild(right);
    const az = slider(ui, "axis azimuth", -180, 180, 1, 0, draw, "°");
    const elv = slider(ui, "axis elevation", -90, 90, 1, 90, draw, "°");
    const th = slider(ui, "angle θ", -180, 180, 1, 45, draw, "°");
    function draw() {
      const k = colors();
      const a = deg(+az.value), e = deg(+elv.value), t = deg(+th.value);
      const w = [Math.cos(e) * Math.cos(a), Math.cos(e) * Math.sin(a), Math.sin(e)];
      const R = rodrigues(w, t), RtR = matmul(transpose(R), R), d = det3(R);
      const tr = R[0][0] + R[1][1] + R[2][2];
      const back = Math.acos(Math.max(-1, Math.min(1, (tr - 1) / 2)));
      const ctx = D.ctx; ctx.clearRect(0, 0, 300, 280);
      drawFrame(ctx, [[1,0,0],[0,1,0],[0,0,1]], 150, 150, 95, 0.25, 1.5);
      drawBox(ctx, R, 150, 150, 95, k.light);
      drawFrame(ctx, R, 150, 150, 95, 1, 3);
      const [ax0, ay0] = project([-1.2 * w[0], -1.2 * w[1], -1.2 * w[2]], 150, 150, 95);
      const [ax1, ay1] = project([1.2 * w[0], 1.2 * w[1], 1.2 * w[2]], 150, 150, 95);
      ctx.strokeStyle = k.accent; ctx.lineWidth = 2; ctx.setLineDash([6, 4]);
      ctx.beginPath(); ctx.moveTo(ax0, ay0); ctx.lineTo(ax1, ay1); ctx.stroke(); ctx.setLineDash([]);
      ctx.fillStyle = k.accent; ctx.fillText("ω̂", ax1 + 4, ay1);
      ctx.fillStyle = k.fg; ctx.font = "12px sans-serif"; ctx.fillText("faint = space frame {s}, bold = body frame {b}", 6, 272);
      right.innerHTML =
        `ω̂ = (${fmt(w[0], 2)}, ${fmt(w[1], 2)}, ${fmt(w[2], 2)}), θ = ${th.value}°` +
        matrixHTML(R, "R = e<sup>[ω̂]θ</sup>") +
        `<div class="lab-checks">RᵀR = I? max error ${(Math.max(...RtR.flat().map((v, i) => Math.abs(v - (i % 4 === 0 ? 1 : 0))))).toExponential(1)}<br>` +
        `det R = ${fmt(d, 6)}<br>` +
        `log check: θ = arccos((tr R − 1)/2) = <b>${fmt((back * 180) / Math.PI, 1)}°</b>` +
        `${Math.abs(+th.value) > 179 ? " (θ ≈ 180°: axis sign becomes ambiguous)" : ""}<br>` +
        `columns of R = where {b}'s x, y, z axes point in {s}</div>`;
    }
    draw();
  }

  // ---------- Widget 4: rotations do not commute ----------
  function noncommute(root) {
    const ui = el("div", { class: "lab-controls" });
    const pics = el("div", { class: "lab-pics" });
    root.append(ui, pics);
    const A = canvas(pics, 250, 230), B = canvas(pics, 250, 230);
    const out = el("div", { class: "lab-readout" }); root.appendChild(out);
    const ta = slider(ui, "θ about x", -180, 180, 1, 90, draw, "°");
    const tb = slider(ui, "θ about z", -180, 180, 1, 90, draw, "°");
    function draw() {
      const k = colors();
      const Rx = rodrigues([1, 0, 0], deg(+ta.value)), Rz = rodrigues([0, 0, 1], deg(+tb.value));
      const XZ = matmul(Rx, Rz), ZX = matmul(Rz, Rx);
      for (const [cv, R, title] of [[A, XZ, "R = Rx · Rz"], [B, ZX, "R = Rz · Rx"]]) {
        const ctx = cv.ctx; ctx.clearRect(0, 0, 250, 230);
        drawFrame(ctx, [[1,0,0],[0,1,0],[0,0,1]], 125, 125, 75, 0.2, 1.5);
        drawBox(ctx, R, 125, 125, 75, k.light); drawFrame(ctx, R, 125, 125, 75);
        ctx.fillStyle = k.fg; ctx.font = "13px sans-serif"; ctx.fillText(title, 8, 18);
      }
      let diff = 0; for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) diff = Math.max(diff, Math.abs(XZ[i][j] - ZX[i][j]));
      out.innerHTML = `Largest entry difference between the two orders: <b>${fmt(diff, 3)}</b>. ` +
        (diff < 1e-9 ? "Same result — one angle is 0 (or both are 180° about orthogonal axes is a special case to try!)." : "Different poses: order matters. That is why we multiply matrices and never add angles.");
    }
    draw();
  }

  // ---------- Widget 5: planar twist → screw motion ----------
  function screw(root) {
    const ui = el("div", { class: "lab-controls" });
    const pics = el("div", { class: "lab-pics" });
    root.append(ui, pics);
    const D = canvas(pics, 420, 320);
    const out = el("div", { class: "lab-readout" }); root.appendChild(out);
    const w = slider(ui, "ω (spin rate)", -2, 2, 0.05, 1, draw, " rad/s");
    const vx = slider(ui, "vₓ", -2, 2, 0.05, 0.5, draw, " m/s");
    const vy = slider(ui, "v_y", -2, 2, 0.05, -0.5, draw, " m/s");
    const tt = slider(ui, "time t", 0, 6.28, 0.02, 2, draw, " s");
    // exact exponential of a planar twist (body starts at origin, aligned)
    function pose(om, ux, uy, t) {
      if (Math.abs(om) < 1e-9) return [ux * t, uy * t, 0];
      const s = Math.sin(om * t), c = Math.cos(om * t);
      // p(t) = (1/ω)[ s  -(1-c) ; (1-c)  s ] v
      return [(s * ux - (1 - c) * uy) / om, ((1 - c) * ux + s * uy) / om, om * t];
    }
    function draw() {
      const k = colors(), ctx = D.ctx, S = 95, cx = 190, cy = 190;
      const om = +w.value, ux = +vx.value, uy = +vy.value, t = +tt.value;
      ctx.clearRect(0, 0, 420, 320);
      ctx.strokeStyle = k.light; ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(0, cy); ctx.lineTo(420, cy); ctx.moveTo(cx, 0); ctx.lineTo(cx, 320); ctx.stroke();
      const X = (x) => cx + S * x, Y = (y) => cy - S * y;
      // path of the body origin
      ctx.strokeStyle = k.primary; ctx.lineWidth = 2; ctx.beginPath();
      for (let i = 0; i <= 200; i++) { const p = pose(om, ux, uy, (t * i) / 200); i ? ctx.lineTo(X(p[0]), Y(p[1])) : ctx.moveTo(X(p[0]), Y(p[1])); }
      ctx.stroke();
      // ghost bodies along the way
      const body = [[-0.35, -0.15], [0.35, -0.15], [0.5, 0], [0.35, 0.15], [-0.35, 0.15]];
      for (let g = 0; g <= 6; g++) {
        const p = pose(om, ux, uy, (t * g) / 6), c = Math.cos(p[2]), s = Math.sin(p[2]);
        ctx.globalAlpha = g === 6 ? 1 : 0.18;
        ctx.fillStyle = g === 6 ? k.accent : k.light; ctx.strokeStyle = k.fg; ctx.beginPath();
        body.forEach(([a, b], i) => { const x = p[0] + c * a - s * b, y = p[1] + s * a + c * b; i ? ctx.lineTo(X(x), Y(y)) : ctx.moveTo(X(x), Y(y)); });
        ctx.closePath(); ctx.fill(); if (g === 6) ctx.stroke();
      }
      ctx.globalAlpha = 1;
      let txt;
      if (Math.abs(om) > 1e-6) {
        // fixed point of the motion: v + ω × r = 0  →  r = (-v_y/ω, v_x/ω)
        const qx = -uy / om, qy = ux / om;
        ctx.fillStyle = k.red; ctx.beginPath(); ctx.arc(X(qx), Y(qy), 5, 0, 7); ctx.fill();
        ctx.font = "12px sans-serif"; ctx.fillText("pole (screw axis)", X(qx) - 40, Y(qy) - 10);
        ctx.setLineDash([3, 3]); ctx.strokeStyle = k.red; ctx.beginPath();
        ctx.arc(X(qx), Y(qy), S * Math.hypot(qx, qy), 0, 7); ctx.stroke(); ctx.setLineDash([]);
        txt = `Pole q = (−v_y/ω, vₓ/ω) = (${fmt(qx, 2)}, ${fmt(qy, 2)}). Every point circles this spot: pure rotation, pitch h = 0 in the plane.`;
      } else {
        txt = `ω = 0: the pole is at infinity, so the motion is pure translation (an infinite-pitch screw).`;
      }
      const p = pose(om, ux, uy, t);
      out.innerHTML = `Twist 𝒱 = (ω, vₓ, v_y) = (${fmt(om, 2)}, ${fmt(ux, 2)}, ${fmt(uy, 2)}). ` +
        `After t = ${fmt(t, 2)} s: body at (${fmt(p[0], 2)}, ${fmt(p[1], 2)}), heading ${fmt((p[2] * 180) / Math.PI, 1)}°.<br>${txt}<br>` +
        `<i>This exact curved path is T(t) = e<sup>[𝒱]t</sup>. Adding v·t would draw a straight line — wrong whenever ω ≠ 0.</i>`;
    }
    draw();
  }

  // ---------- Self-check questions ----------
  const KEY = "robotics-lab-answers";
  const load = () => { try { return JSON.parse(localStorage.getItem(KEY) || "{}"); } catch (e) { return {}; } };
  const save = (o) => { try { localStorage.setItem(KEY, JSON.stringify(o)); } catch (e) {} };
  function quiz(root) {
    const id = root.dataset.q, ans = root.dataset.answer, why = root.dataset.why || "";
    const opts = root.querySelectorAll("li");
    const fb = el("div", { class: "quiz-fb" }); root.appendChild(fb);
    function mark(choice) {
      opts.forEach((li) => li.classList.remove("right", "wrong"));
      const li = root.querySelector(`li[data-k="${choice}"]`);
      if (!li) return;
      const ok = choice === ans;
      li.classList.add(ok ? "right" : "wrong");
      fb.innerHTML = ok ? "✅ Right. " + why : "❌ Not quite — try another.";
    }
    opts.forEach((li) => li.addEventListener("click", () => {
      const s = load(); s[id] = li.dataset.k; save(s); mark(li.dataset.k); progress();
    }));
    const prev = load()[id]; if (prev) mark(prev);
  }
  function progress() {
    const total = document.querySelectorAll(".quiz").length;
    const s = load(); let right = 0;
    document.querySelectorAll(".quiz").forEach((q) => { if (s[q.dataset.q] === q.dataset.answer) right++; });
    document.querySelectorAll("[data-lab-progress]").forEach((p) => {
      const all = Object.keys(s).length;
      p.innerHTML = total
        ? `Your path on this page: <b>${right} / ${total}</b> checks right.`
        : `Checks answered across the lab: <b>${all}</b>. <button class="md-button lab-reset">Clear my answers</button>`;
      const b = p.querySelector(".lab-reset"); if (b) b.onclick = () => { save({}); progress(); };
    });
  }

  const WIDGETS = { cspace, grubler, rotation, noncommute, screw };
  function init() {
    document.querySelectorAll("[data-widget]").forEach((n) => {
      if (n.dataset.ready) return;
      const f = WIDGETS[n.dataset.widget];
      if (!f) return; // its module file may register later
      n.dataset.ready = "1";
      try { f(n); } catch (e) { n.textContent = "Widget failed: " + e.message; }
    });
    document.querySelectorAll(".quiz").forEach((q) => { if (!q.dataset.ready) { q.dataset.ready = "1"; quiz(q); } });
    progress();
  }
  // Public API for per-module widget files (docs/javascripts/widgets/*.js):
  //   Lab.register("name", function (root) { ... })
  window.Lab = {
    css, colors, fmt, el, slider, canvas, deg,
    matmul, transpose, det3, skew, rodrigues, apply, project, drawFrame, drawBox, matrixHTML,
    register(name, fn) { WIDGETS[name] = fn; init(); },
  };
  if (window.document$) window.document$.subscribe(init); else document.addEventListener("DOMContentLoaded", init);
  // redraw in new colours when the theme toggles
  new MutationObserver(() => document.querySelectorAll("[data-widget] input").forEach((i) => i.dispatchEvent(new Event("input"))))
    .observe(document.body, { attributes: true, attributeFilter: ["data-md-color-scheme"] });
})();
