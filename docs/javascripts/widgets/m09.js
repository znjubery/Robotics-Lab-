/* M9 · Motion planning widgets: m09-rrt (A* on a grid, RRT, RRT*, potential field).
   Everything is computed live; the random sampler is seeded so results are reproducible. */
(function () {
  "use strict";

  // ---------- seeded RNG (mulberry32) ----------
  function rng(seed) {
    let a = seed >>> 0;
    return function () {
      a = (a + 0x6d2b79f5) | 0;
      let t = Math.imul(a ^ (a >>> 15), 1 | a);
      t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
      return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
  }
  const dist = (ax, ay, bx, by) => Math.hypot(ax - bx, ay - by);

  // ---------- collision checks for a disc robot of radius w.rob ----------
  // C-obstacle of a disc robot around a disc obstacle = disc grown by the robot radius (M1).
  function freePt(w, x, y) {
    if (x < 0 || y < 0 || x > w.W || y > w.H) return false;
    for (const o of w.obs) { const R = o.r + w.rob; if ((x - o.x) ** 2 + (y - o.y) ** 2 <= R * R) return false; }
    return true;
  }
  function segDist2(px, py, ax, ay, bx, by) {
    const dx = bx - ax, dy = by - ay, L = dx * dx + dy * dy;
    let t = L > 0 ? ((px - ax) * dx + (py - ay) * dy) / L : 0; t = Math.max(0, Math.min(1, t));
    const qx = ax + t * dx - px, qy = ay + t * dy - py;
    return qx * qx + qy * qy;
  }
  function freeSeg(w, ax, ay, bx, by) {
    if (!freePt(w, ax, ay) || !freePt(w, bx, by)) return false;
    for (const o of w.obs) { const R = o.r + w.rob; if (segDist2(o.x, o.y, ax, ay, bx, by) <= R * R) return false; }
    return true;
  }
  function pathLen(P) { let s = 0; for (let i = 1; i < P.length; i++) s += dist(P[i - 1][0], P[i - 1][1], P[i][0], P[i][1]); return s; }

  // ---------- A* on an 8-connected grid (wt = 0 → Dijkstra, wt > 1 → inadmissible) ----------
  function astar(w, h, wt) {
    const nx = Math.max(2, Math.round(w.W / h)), ny = Math.max(2, Math.round(w.H / h));
    const hx = w.W / nx, hy = w.H / ny, n = nx * ny;
    const cx = (k) => ((k % nx) + 0.5) * hx, cy = (k) => (Math.floor(k / nx) + 0.5) * hy;
    const free = new Uint8Array(n);
    for (let k = 0; k < n; k++) free[k] = freePt(w, cx(k), cy(k)) ? 1 : 0;
    const cell = (x, y) => Math.min(nx - 1, Math.floor(x / hx)) + nx * Math.min(ny - 1, Math.floor(y / hy));
    const s = cell(w.start[0], w.start[1]), g = cell(w.goal[0], w.goal[1]);
    const res = { path: null, nx, ny, hx, hy, free, closed: new Uint8Array(n), nExp: 0, nFree: free.reduce((a, b) => a + b, 0) };
    if (!free[s] || !free[g] || !freeSeg(w, w.start[0], w.start[1], cx(s), cy(s)) || !freeSeg(w, cx(g), cy(g), w.goal[0], w.goal[1])) return res;
    const G = new Float64Array(n).fill(Infinity), par = new Int32Array(n).fill(-1);
    const gx = cx(g), gy = cy(g), H = (k) => wt * dist(cx(k), cy(k), gx, gy);
    // binary min-heap of [f, k]
    const heap = [];
    const push = (f, k) => { heap.push([f, k]); let i = heap.length - 1; while (i > 0) { const p = (i - 1) >> 1; if (heap[p][0] <= heap[i][0]) break; [heap[p], heap[i]] = [heap[i], heap[p]]; i = p; } };
    const pop = () => { const top = heap[0], last = heap.pop(); if (heap.length) { heap[0] = last; let i = 0; for (;;) { const l = 2 * i + 1, r = l + 1; let m = i; if (l < heap.length && heap[l][0] < heap[m][0]) m = l; if (r < heap.length && heap[r][0] < heap[m][0]) m = r; if (m === i) break; [heap[m], heap[i]] = [heap[i], heap[m]]; i = m; } } return top; };
    G[s] = 0; push(H(s), s);
    const D = [[1, 0], [-1, 0], [0, 1], [0, -1], [1, 1], [1, -1], [-1, 1], [-1, -1]];
    while (heap.length) {
      const [, k] = pop();
      if (res.closed[k]) continue;
      res.closed[k] = 1; res.nExp++;
      if (k === g) break;
      const i = k % nx, j = Math.floor(k / nx);
      for (const [di, dj] of D) {
        const a = i + di, b = j + dj; if (a < 0 || b < 0 || a >= nx || b >= ny) continue;
        const m = a + nx * b; if (!free[m] || res.closed[m]) continue;
        if (di && dj && !freeSeg(w, cx(k), cy(k), cx(m), cy(m))) continue; // no corner cutting
        if (!(di && dj) && !freeSeg(w, cx(k), cy(k), cx(m), cy(m))) continue;
        const c = G[k] + Math.hypot(di * hx, dj * hy);
        if (c < G[m]) { G[m] = c; par[m] = k; push(c + H(m), m); }
      }
    }
    if (!res.closed[g]) return res;
    const P = [[w.goal[0], w.goal[1]]];
    for (let k = g; k !== -1; k = par[k]) P.push([cx(k), cy(k)]);
    P.push([w.start[0], w.start[1]]); P.reverse();
    res.path = P; return res;
  }

  // ---------- RRT and RRT* ----------
  function rrt(w, eta, N, seed, star) {
    const R = rng(seed), X = [w.start[0]], Y = [w.start[1]], par = [-1], cost = [0], kids = [[]];
    const goalNodes = [], hist = []; let firstIt = -1, rEnd = 0;
    const gamma = 12, gx = w.goal[0], gy = w.goal[1];
    const best = () => { let b = Infinity, bi = -1; for (const i of goalNodes) { const c = cost[i] + dist(X[i], Y[i], gx, gy); if (c < b) { b = c; bi = i; } } return [b, bi]; };
    for (let it = 0; it < N; it++) {
      let sx, sy; if (R() < 0.05) { sx = gx; sy = gy; } else { sx = R() * w.W; sy = R() * w.H; }
      let ni = 0, nd = Infinity;
      for (let i = 0; i < X.length; i++) { const d = (X[i] - sx) ** 2 + (Y[i] - sy) ** 2; if (d < nd) { nd = d; ni = i; } }
      nd = Math.sqrt(nd); if (nd < 1e-9) continue;
      const f = Math.min(1, eta / nd), qx = X[ni] + f * (sx - X[ni]), qy = Y[ni] + f * (sy - Y[ni]);
      if (!freeSeg(w, X[ni], Y[ni], qx, qy)) continue;
      let p = ni, pc = cost[ni] + dist(X[ni], Y[ni], qx, qy), near = [];
      if (star) {
        const n = X.length + 1; rEnd = Math.min(gamma * Math.sqrt(Math.log(n) / n), 2 * eta);
        for (let i = 0; i < X.length; i++) if (dist(X[i], Y[i], qx, qy) <= rEnd) near.push(i);
        for (const i of near) { const c = cost[i] + dist(X[i], Y[i], qx, qy); if (c < pc - 1e-12 && freeSeg(w, X[i], Y[i], qx, qy)) { p = i; pc = c; } }
      }
      const q = X.length; X.push(qx); Y.push(qy); par.push(p); cost.push(pc); kids.push([]); kids[p].push(q);
      if (star) {
        for (const i of near) {
          if (i === p) continue;
          const c = pc + dist(qx, qy, X[i], Y[i]);
          if (c < cost[i] - 1e-12 && freeSeg(w, qx, qy, X[i], Y[i])) {
            const old = par[i]; kids[old].splice(kids[old].indexOf(i), 1); par[i] = q; kids[q].push(i);
            const delta = c - cost[i], stack = [i];
            while (stack.length) { const u = stack.pop(); cost[u] += delta; for (const v of kids[u]) stack.push(v); }
          }
        }
      }
      if (dist(qx, qy, gx, gy) <= eta && freeSeg(w, qx, qy, gx, gy)) { goalNodes.push(q); if (firstIt < 0) firstIt = it; }
      if (goalNodes.length) hist.push([it, best()[0]]);
      if (!star && goalNodes.length) break; // plain RRT stops at the first solution
    }
    const [bc, bi] = best();
    let path = null;
    if (bi >= 0) { path = [[gx, gy]]; for (let k = bi; k !== -1; k = par[k]) path.push([X[k], Y[k]]); path.reverse(); }
    return { X, Y, par, path, cost: bc, firstIt, hist, rEnd };
  }

  // ---------- shortcut smoothing (seeded) ----------
  function smooth(w, P, seed, tries = 150) {
    if (!P || P.length < 3) return P;
    const R = rng(seed + 7919); P = P.slice();
    for (let t = 0; t < tries && P.length > 2; t++) {
      let i = Math.floor(R() * P.length), j = Math.floor(R() * P.length);
      if (i > j) [i, j] = [j, i];
      if (j - i < 2) continue;
      if (freeSeg(w, P[i][0], P[i][1], P[j][0], P[j][1])) P.splice(i + 1, j - i - 1);
    }
    return P;
  }

  // ---------- artificial potential field, gradient descent ----------
  const PF = { katt: 1, dstar: 2, krep: 0.6, d0: 1.0 };
  function potU(w, x, y) {
    const d = dist(x, y, w.goal[0], w.goal[1]);
    let U = d <= PF.dstar ? 0.5 * PF.katt * d * d : PF.katt * PF.dstar * (d - 0.5 * PF.dstar);
    for (const o of w.obs) { const s = dist(x, y, o.x, o.y) - o.r - w.rob; if (s <= 0) return Infinity; if (s < PF.d0) U += 0.5 * PF.krep * (1 / s - 1 / PF.d0) ** 2; }
    return U;
  }
  function potGrad(w, x, y) {
    const gx0 = x - w.goal[0], gy0 = y - w.goal[1], d = Math.hypot(gx0, gy0);
    const a = d <= PF.dstar ? PF.katt : (PF.katt * PF.dstar) / d;
    let gx = a * gx0, gy = a * gy0;
    for (const o of w.obs) {
      const ex = x - o.x, ey = y - o.y, c = Math.hypot(ex, ey), s = c - o.r - w.rob;
      if (s > 0 && s < PF.d0) { const k = -PF.krep * (1 / s - 1 / PF.d0) / (s * s); gx += (k * ex) / c; gy += (k * ey) / c; }
    }
    return [gx, gy];
  }
  function potential(w, eta, N) {
    let x = w.start[0], y = w.start[1]; const P = [[x, y]], dmax = Math.min(0.05, eta / 4);
    let status = "iteration limit";
    if (!freePt(w, x, y)) return { path: P, status: "start in collision" };
    for (let it = 0; it < N; it++) {
      if (dist(x, y, w.goal[0], w.goal[1]) < 0.08) { status = "reached goal"; break; }
      const [gx, gy] = potGrad(w, x, y), gn = Math.hypot(gx, gy);
      if (gn < 1e-4) { status = "stuck: ∇U = 0 (local minimum)"; break; }
      const step = Math.min(dmax, 0.05 * gn);
      x -= (step * gx) / gn; y -= (step * gy) / gn; P.push([x, y]);
      if (it > 120) { const q = P[P.length - 100]; if (dist(x, y, q[0], q[1]) < 2 * dmax) { status = "stuck: local minimum (oscillating)"; break; } }
    }
    if (status === "iteration limit" && dist(x, y, w.goal[0], w.goal[1]) < 0.08) status = "reached goal";
    return { path: P, status };
  }

  const API = { rng, freePt, freeSeg, astar, rrt, smooth, potential, potU, pathLen };
  if (typeof module !== "undefined" && module.exports) module.exports = API;

  const L = typeof window !== "undefined" && window.Lab; if (!L) return;
  const { colors, fmt, el, slider, canvas } = L;

  L.register("m09-rrt", function (root) {
    const CW = 420, CH = 300, S = 42, W = CW / S, H = CH / S;
    const DEF = () => ({
      W, H, rob: 0.15, start: [1.0, 3.6], goal: [9.0, 3.6],
      obs: [{ x: 5, y: 3.6, r: 1.1 }, { x: 3.0, y: 1.3, r: 0.8 }, { x: 7.2, y: 5.8, r: 0.9 }, { x: 3.2, y: 5.9, r: 0.7 }, { x: 7.3, y: 1.5, r: 0.7 }],
    });
    let world = DEF();
    const ui = el("div", { class: "lab-controls" });
    const pics = el("div", { class: "lab-pics" });
    root.append(ui, pics);
    const C = canvas(pics, CW, CH), G = canvas(pics, CW, 110);
    C.c.style.touchAction = "none"; C.c.style.cursor = "crosshair";
    const out = el("div", { class: "lab-readout" }); root.appendChild(out);

    const row = el("div", { class: "lab-row" });
    const sel = el("select");
    for (const [v, t] of [["astar", "A* on a grid"], ["rrt", "RRT"], ["rrtstar", "RRT*"], ["pf", "Potential field"]]) sel.appendChild(el("option", { value: v }, t));
    sel.value = "rrtstar";
    row.append(el("span", { class: "lab-label" }, "Planner"), sel); ui.appendChild(row);
    const eta = slider(ui, "step η / cell", 0.1, 1.5, 0.05, 0.4, schedule, "");
    const iters = slider(ui, "iterations N", 100, 4000, 50, 1500, schedule, "");
    const seed = slider(ui, "seed", 1, 50, 1, 7, schedule, "");
    const rob = slider(ui, "robot radius", 0, 0.5, 0.05, 0.15, schedule, "");
    const wt = slider(ui, "A* weight w", 0, 3, 0.1, 1, schedule, "");
    const row2 = el("div", { class: "lab-row" });
    const sm = el("input", { type: "checkbox" }); const rst = el("button", { class: "md-button" }, "Reset scene");
    rst.style.padding = "0.1rem 0.6rem"; rst.style.fontSize = "0.7rem";
    const lab = el("label", {}, ""); lab.append(sm, document.createTextNode(" shortcut-smooth the path"));
    row2.append(el("span", { class: "lab-label" }, "Extras"), lab, rst); ui.appendChild(row2);
    sel.addEventListener("change", schedule); sm.addEventListener("change", schedule);
    rst.addEventListener("click", () => { world = DEF(); rob.value = 0.15; rob.dispatchEvent(new Event("input")); schedule(); });

    // ---- mouse / touch: drag start, goal, obstacles; tap empty space to add; double-click obstacle to delete
    const toW = (ev) => { const r = C.c.getBoundingClientRect(); return [((ev.clientX - r.left) / r.width) * W, (1 - (ev.clientY - r.top) / r.height) * H]; };
    let drag = null, moved = false;
    function hit(x, y) {
      if (dist(x, y, ...world.start) < 0.35) return { k: "start" };
      if (dist(x, y, ...world.goal) < 0.35) return { k: "goal" };
      for (let i = world.obs.length - 1; i >= 0; i--) { const o = world.obs[i]; if (dist(x, y, o.x, o.y) < o.r) return { k: "obs", i, dx: o.x - x, dy: o.y - y }; }
      return null;
    }
    C.c.addEventListener("pointerdown", (ev) => { const [x, y] = toW(ev); drag = hit(x, y); moved = false; C.c.setPointerCapture(ev.pointerId); });
    C.c.addEventListener("pointermove", (ev) => {
      if (!drag) return; const [x0, y0] = toW(ev); const x = Math.max(0.05, Math.min(W - 0.05, x0)), y = Math.max(0.05, Math.min(H - 0.05, y0)); moved = true;
      if (drag.k === "start") world.start = [x, y]; else if (drag.k === "goal") world.goal = [x, y];
      else { world.obs[drag.i].x = x + drag.dx; world.obs[drag.i].y = y + drag.dy; }
      schedule();
    });
    C.c.addEventListener("pointerup", (ev) => {
      const [x, y] = toW(ev);
      if (!drag && !moved && world.obs.length < 14) { world.obs.push({ x, y, r: 0.6 }); schedule(); }
      drag = null;
    });
    C.c.addEventListener("dblclick", (ev) => { const [x, y] = toW(ev); const h = hit(x, y); if (h && h.k === "obs") { world.obs.splice(h.i, 1); schedule(); } });

    let pending = false, refLen = null, refKey = "";
    function schedule() { if (pending) return; pending = true; requestAnimationFrame(() => { pending = false; draw(); }); }

    function draw() {
      const k = colors(), ctx = C.ctx, alg = sel.value;
      world.rob = +rob.value;
      const X = (x) => x * S, Y = (y) => CH - y * S;
      const t0 = performance.now();
      let res, P = null, info = "";
      if (alg === "astar") { res = astar(world, +eta.value, +wt.value); P = res.path; }
      else if (alg === "pf") { res = potential(world, +eta.value, +iters.value); P = res.path; }
      else { res = rrt(world, +eta.value, +iters.value, +seed.value, alg === "rrtstar"); P = res.path; }
      let Ps = P;
      if (sm.checked && P && alg !== "pf") Ps = smooth(world, P, +seed.value);
      const ms = performance.now() - t0;
      // fine-grid A* reference length (recomputed only when the scene changes)
      const key = JSON.stringify([world.obs, world.start, world.goal, world.rob]);
      if (key !== refKey) { refKey = key; const r = astar(world, 0.1, 1); refLen = r.path ? pathLen(smooth(world, r.path, 1, 400)) : null; }

      ctx.clearRect(0, 0, CW, CH);
      ctx.strokeStyle = k.light; ctx.lineWidth = 1; ctx.strokeRect(0.5, 0.5, CW - 1, CH - 1);
      // potential-field shading
      if (alg === "pf") {
        const n = 6; let Umax = 0; const vals = [];
        for (let y = 0; y < CH; y += n) for (let x = 0; x < CW; x += n) { const u = potU(world, (x + n / 2) / S, (CH - y - n / 2) / S); vals.push(u); if (isFinite(u)) Umax = Math.max(Umax, Math.min(u, 25)); }
        let q = 0;
        for (let y = 0; y < CH; y += n) for (let x = 0; x < CW; x += n) { const u = vals[q++]; if (!isFinite(u)) continue; ctx.fillStyle = `rgba(30,136,229,${(0.45 * Math.min(u, 25)) / Umax})`; ctx.fillRect(x, y, n, n); }
      }
      // A* explored cells
      if (alg === "astar") {
        ctx.fillStyle = "rgba(30,136,229,0.22)";
        for (let c = 0; c < res.nx * res.ny; c++) if (res.closed[c]) ctx.fillRect(X((c % res.nx) * res.hx), Y((Math.floor(c / res.nx) + 1) * res.hy), res.hx * S - 0.5, res.hy * S - 0.5);
      }
      // obstacles + grown C-obstacles
      for (const o of world.obs) {
        ctx.fillStyle = "rgba(229,57,53,0.45)"; ctx.beginPath(); ctx.arc(X(o.x), Y(o.y), o.r * S, 0, 7); ctx.fill();
        if (world.rob > 0) { ctx.strokeStyle = k.red; ctx.setLineDash([3, 3]); ctx.beginPath(); ctx.arc(X(o.x), Y(o.y), (o.r + world.rob) * S, 0, 7); ctx.stroke(); ctx.setLineDash([]); }
      }
      // RRT tree
      if (res.X) {
        ctx.strokeStyle = k.light; ctx.lineWidth = 0.8; ctx.beginPath();
        for (let i = 1; i < res.X.length; i++) { const p = res.par[i]; ctx.moveTo(X(res.X[p]), Y(res.Y[p])); ctx.lineTo(X(res.X[i]), Y(res.Y[i])); }
        ctx.stroke();
      }
      // path
      const drawPath = (Q, col, wdt, dash) => { if (!Q) return; ctx.strokeStyle = col; ctx.lineWidth = wdt; ctx.setLineDash(dash); ctx.beginPath(); Q.forEach((p, i) => (i ? ctx.lineTo(X(p[0]), Y(p[1])) : ctx.moveTo(X(p[0]), Y(p[1])))); ctx.stroke(); ctx.setLineDash([]); };
      if (Ps !== P) drawPath(P, k.primary, 1.5, [4, 3]);
      drawPath(Ps, alg === "pf" && res.status !== "reached goal" ? k.red : k.primary, 3, []);
      // start / goal
      ctx.fillStyle = k.green; ctx.beginPath(); ctx.arc(X(world.start[0]), Y(world.start[1]), 7, 0, 7); ctx.fill();
      ctx.fillStyle = k.accent; ctx.beginPath(); ctx.arc(X(world.goal[0]), Y(world.goal[1]), 7, 0, 7); ctx.fill();
      ctx.strokeStyle = k.fg; ctx.lineWidth = 1; ctx.stroke();
      ctx.fillStyle = k.fg; ctx.font = "12px sans-serif";
      ctx.fillText("start", X(world.start[0]) - 14, Y(world.start[1]) - 11); ctx.fillText("goal", X(world.goal[0]) - 12, Y(world.goal[1]) - 11);
      ctx.fillText("drag dots/discs · tap empty space to add · double-click a disc to delete", 6, 14);

      // ---- convergence panel: best path length vs iteration
      const g = G.ctx; g.clearRect(0, 0, CW, 110);
      g.strokeStyle = k.light; g.strokeRect(0.5, 0.5, CW - 1, 109);
      g.fillStyle = k.fg; g.font = "12px sans-serif";
      const Lstraight = dist(...world.start, ...world.goal);
      const len = Ps ? pathLen(Ps) : null;
      if ((alg === "rrt" || alg === "rrtstar") && res.hist.length) {
        const N = +iters.value, ys = res.hist.map((h) => h[1]).concat(refLen ? [refLen] : []);
        const lo = Math.min(...ys) * 0.97, hi = Math.max(...ys) * 1.03;
        const px = (it) => 34 + ((CW - 44) * it) / N, py = (v) => 100 - (80 * (v - lo)) / Math.max(1e-9, hi - lo);
        g.strokeStyle = k.primary; g.lineWidth = 2; g.beginPath();
        res.hist.forEach(([it, v], i) => (i ? g.lineTo(px(it), py(v)) : g.moveTo(px(it), py(v))));
        if (alg === "rrtstar") g.lineTo(px(N), py(res.hist[res.hist.length - 1][1]));
        g.stroke(); g.fillStyle = k.primary; g.beginPath(); g.arc(px(res.hist[0][0]), py(res.hist[0][1]), 3, 0, 7); g.fill();
        if (refLen) { g.strokeStyle = k.green; g.setLineDash([4, 3]); g.lineWidth = 1; g.beginPath(); g.moveTo(34, py(refLen)); g.lineTo(CW - 10, py(refLen)); g.stroke(); g.setLineDash([]); }
        g.fillStyle = k.fg; g.fillText("best path length vs iteration (green dashed = fine-grid reference)", 34, 14);
        g.fillText(fmt(hi, 1), 2, 24); g.fillText(fmt(lo, 1), 2, 100);
      } else {
        g.fillText(alg === "astar" ? "A* is one-shot: no anytime improvement curve." : alg === "pf" ? "Potential field: follows −∇U; no search, so no guarantee." : "No solution yet — raise N or η.", 34, 58);
      }

      // ---- readout
      if (alg === "astar") info = `Grid ${res.nx}×${res.ny}, cells expanded <b>${res.nExp}</b> of ${res.nFree} free. f = g + w·h with w = ${(+wt.value).toFixed(1)} ${+wt.value === 0 ? "(Dijkstra: no heuristic)" : +wt.value <= 1 ? "(admissible → optimal on this grid)" : "(inadmissible → faster, may be longer)"}.`;
      else if (alg === "pf") info = `Status: <b style="color:${res.status === "reached goal" ? k.green : k.red}">${res.status}</b>, ${res.path.length} gradient steps.`;
      else info = `Tree nodes <b>${res.X.length}</b>${res.firstIt >= 0 ? `, first solution at iteration ${res.firstIt}` : ""}${alg === "rrtstar" ? `, final rewire radius r = ${fmt(res.rEnd, 2)}` : ""}.`;
      out.innerHTML =
        `Path length <b>${len ? fmt(len, 2) : "—"}</b>${Ps !== P && P ? ` (raw ${fmt(pathLen(P), 2)})` : ""} &nbsp;·&nbsp; straight line ${fmt(Lstraight, 2)} &nbsp;·&nbsp; fine-grid reference ${refLen ? fmt(refLen, 2) : "—"}` +
        `${len && refLen ? ` &nbsp;·&nbsp; ratio to reference <b>${fmt(len / refLen, 3)}</b>` : ""}<br>${info} Compute ${fmt(ms, 1)} ms.` +
        `${!P && alg !== "pf" ? ` <b style="color:${k.red}">No path found</b>${alg === "astar" ? " (start/goal cell blocked or no free route at this resolution)" : ""}.` : ""}`;
    }
    draw();
  });
})();
