/* Track D widgets: td-bc (behavior cloning, compounding error, action chunking, DAgger) */
(function () {
  "use strict";

  // ---------------- pure simulation core (testable in node) ----------------
  function rng(seed) { // mulberry32
    let a = seed >>> 0;
    return function () {
      a = (a + 0x6D2B79F5) >>> 0; let t = a;
      t = Math.imul(t ^ (t >>> 15), t | 1);
      t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
      return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
  }
  function gauss(r) { const u = Math.max(r(), 1e-12), v = r(); return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v); }

  const W = { OX: 0.5, OY: 0.5, OR: 0.15, GX: 0.9, GY: 0.5, STEP: 0.025, T: 90, KMAX: 12, GOAL_R: 0.04, NB: 3, PAUSE: 6, PAUSE_P: 0.7 };

  // the expert: go to a waypoint above or below the obstacle, then to the goal
  function expert(x, y) {
    let tx, ty;
    if (x < W.OX - 0.02) { tx = W.OX; ty = y >= W.OY ? W.OY + 0.25 : W.OY - 0.25; }
    else { tx = W.GX; ty = W.GY; }
    const dx = tx - x, dy = ty - y, d = Math.hypot(dx, dy) || 1, s = Math.min(W.STEP, d);
    return [s * dx / d, s * dy / d];
  }
  function expertChunk(x, y) { // noise-free expert rollout for KMAX steps = the label for a chunked policy
    const ch = [];
    for (let i = 0; i < W.KMAX; i++) { const a = expert(x, y); ch.push(a); x += a[0]; y += a[1]; }
    return ch;
  }
  function status(x, y) {
    if (Math.hypot(x - W.OX, y - W.OY) < W.OR) return "hit";
    if (x < 0 || x > 1 || y < 0 || y > 1) return "out";
    if (Math.hypot(x - W.GX, y - W.GY) < W.GOAL_R) return "goal";
    return "";
  }

  // N human-like demos: starts near (0.1, 0.5), small hand jitter
  function makeDemos(N, seed) {
    const r = rng(seed), demos = [];
    for (let n = 0; n < N; n++) {
      let x = 0.1 + 0.02 * (r() - 0.5), y = 0.5 + 0.24 * (r() - 0.5);
      const path = [[x, y]], acts = [];
      let paused = r() > W.PAUSE_P; // humans hesitate: most demos stop for a moment at the obstacle
      for (let t = 0; t < W.T; t++) {
        if (!paused && x >= W.OX - 0.02) { // pause: several "do nothing" actions at the same state
          paused = true;
          for (let i = 0; i < W.PAUSE; i++) { acts.push([0, 0]); path.push([x, y]); }
        }
        const a = expert(x, y); a[0] += 0.003 * gauss(r); a[1] += 0.003 * gauss(r);
        acts.push(a); x += a[0]; y += a[1]; path.push([x, y]);
        if (status(x, y)) break;
      }
      demos.push({ path, acts });
    }
    return demos;
  }
  // dataset = (state, chunk of next KMAX actions)
  function demoData(demos) {
    const D = [];
    for (const d of demos) for (let t = 0; t < d.acts.length; t++) {
      const ch = [];
      for (let i = 0; i < W.KMAX; i++) ch.push(d.acts[Math.min(t + i, d.acts.length - 1)].slice());
      D.push({ s: d.path[t], ch });
    }
    return D;
  }
  // k-nearest-neighbour policy: average the chunks of the NB closest stored states
  function policy(D, x, y) {
    const best = [], bd = [];
    for (let i = 0; i < D.length; i++) {
      const dx = D[i].s[0] - x, dy = D[i].s[1] - y, d = dx * dx + dy * dy;
      if (best.length < W.NB) { best.push(i); bd.push(d); continue; }
      let w = 0; for (let j = 1; j < W.NB; j++) if (bd[j] > bd[w]) w = j;
      if (d < bd[w]) { best[w] = i; bd[w] = d; }
    }
    const ch = [];
    for (let i = 0; i < W.KMAX; i++) {
      let ax = 0, ay = 0; for (const j of best) { ax += D[j].ch[i][0]; ay += D[j].ch[i][1]; }
      ch.push([ax / best.length, ay / best.length]);
    }
    return ch;
  }
  // one rollout: query the policy, execute k actions open loop, add disturbance each step
  function rollout(D, k, start, noise, seed, record) {
    const r = rng(seed); let [x, y] = start; const path = [[x, y]], visited = [];
    let t = 0, st = "";
    while (t < W.T && !st) {
      const ch = policy(D, x, y); visited.push([x, y]);
      for (let i = 0; i < k && t < W.T && !st; i++, t++) {
        x += ch[i][0] + noise * gauss(r); y += ch[i][1] + noise * gauss(r);
        path.push([x, y]); st = status(x, y);
        if (record && i > 0 && !st) visited.push([x, y]);
      }
    }
    return { path, st: st || "time", visited };
  }
  function evalStarts(M, spread, seed) {
    const r = rng(seed), S = [];
    for (let m = 0; m < M; m++) S.push([0.1 + spread * (r() - 0.5) * 2, 0.5 + (0.12 + spread) * (r() - 0.5) * 2]);
    return S;
  }
  // DAgger: roll out the learner, ask the expert what it would have done at each visited state, aggregate
  function dagger(D0, rounds, k, spread, noise, seed) {
    let D = D0.slice(); const added = [];
    for (let rd = 0; rd < rounds; rd++) {
      const S = evalStarts(10, spread, seed + 101 * rd);
      for (let m = 0; m < S.length; m++) {
        const ro = rollout(D, k, S[m], noise, seed + 7 * rd + 1000 * m, true);
        ro.visited.forEach((s, i) => { if (i % 2 === 0 && !status(s[0], s[1])) { const e = { s, ch: expertChunk(s[0], s[1]) }; added.push(e); } });
      }
      D = D0.concat(added);
    }
    return { D, added };
  }
  function evaluate(D, k, spread, noise, M, seed) {
    const S = evalStarts(M, spread, seed), res = [];
    for (let m = 0; m < M; m++) res.push(rollout(D, k, S[m], noise, seed * 31 + m, false));
    const c = { goal: 0, hit: 0, out: 0, time: 0 };
    res.forEach((q) => c[q.st]++);
    return { res, c, rate: c.goal / M };
  }
  const core = { W, rng, expert, makeDemos, demoData, policy, rollout, dagger, evaluate };
  if (typeof module !== "undefined" && module.exports) module.exports = core;
  if (typeof window === "undefined") return;

  // ---------------- widget ----------------
  const L = window.Lab; if (!L) return;
  const { colors, fmt, el, slider, canvas } = L;

  L.register("td-bc", function (root) {
    const ui = el("div", { class: "lab-controls" });
    const pics = el("div", { class: "lab-pics" });
    root.append(ui, pics);
    const A = canvas(pics, 400, 320), B = canvas(pics, 300, 320);
    const out = el("div", { class: "lab-readout" }); root.appendChild(out);
    const nDemo = slider(ui, "demos N", 2, 40, 1, 12, schedule, "");
    const spread = slider(ui, "start shift", 0, 0.15, 0.01, 0.08, schedule, "");
    const noise = slider(ui, "disturbance σ", 0, 12, 1, 3, schedule, " ×10⁻³");
    const kS = slider(ui, "chunk k", 2, 12, 1, 6, schedule, " steps");
    const dR = slider(ui, "DAgger rounds", 1, 5, 1, 2, schedule, "");
    function check(label, on) {
      const row = el("div", { class: "lab-row" });
      const cb = el("input", { type: "checkbox" }); cb.checked = on;
      cb.addEventListener("change", schedule);
      row.append(el("span", { class: "lab-label" }, label), cb); ui.appendChild(row); return cb;
    }
    const chunkOn = check("action chunking", false), dagOn = check("DAgger corrections", false);
    const M = 50, SEED = 7;
    let timer = null;
    function schedule() { clearTimeout(timer); timer = setTimeout(run, 30); }

    function run() {
      const N = +nDemo.value, sp = +spread.value, sg = +noise.value / 1000;
      const k = chunkOn.checked ? +kS.value : 1, R = dagOn.checked ? +dR.value : 0;
      const demos = makeDemos(N, 11), D0 = demoData(demos);
      const dg = dagger(D0, R, k, sp, sg, 500);
      const ev = evaluate(dg.D, k, sp, sg, M, SEED);
      // curve: success vs DAgger rounds, without and with chunking (same seeds)
      const curve = { k1: [], kk: [] };
      for (let r = 0; r <= 5; r++) {
        curve.k1.push(evaluate(dagger(D0, r, 1, sp, sg, 500).D, 1, sp, sg, M, SEED).rate);
        curve.kk.push(evaluate(dagger(D0, r, +kS.value, sp, sg, 500).D, +kS.value, sp, sg, M, SEED).rate);
      }
      draw(demos, dg.added, ev, curve, k, R);
    }

    function draw(demos, added, ev, curve, k, R) {
      const c = colors(), ctx = A.ctx, S = 300, ox = 50, oy = 10;
      const X = (x) => ox + S * x, Y = (y) => oy + S * (1 - y);
      ctx.clearRect(0, 0, 400, 320);
      ctx.strokeStyle = c.light; ctx.lineWidth = 1; ctx.strokeRect(X(0), Y(1), S, S);
      ctx.fillStyle = "rgba(229,57,53,0.35)"; ctx.beginPath(); ctx.arc(X(W.OX), Y(W.OY), S * W.OR, 0, 7); ctx.fill();
      ctx.fillStyle = c.green; ctx.beginPath(); ctx.arc(X(W.GX), Y(W.GY), S * W.GOAL_R, 0, 7); ctx.fill();
      // rollouts
      for (const q of ev.res) {
        ctx.strokeStyle = q.st === "goal" ? "rgba(67,160,71,0.55)" : "rgba(229,57,53,0.75)"; ctx.lineWidth = 1.2;
        ctx.beginPath(); q.path.forEach((p, i) => (i ? ctx.lineTo(X(p[0]), Y(p[1])) : ctx.moveTo(X(p[0]), Y(p[1])))); ctx.stroke();
        if (q.st !== "goal") { const p = q.path[q.path.length - 1]; ctx.fillStyle = c.red; ctx.fillRect(X(p[0]) - 2, Y(p[1]) - 2, 4, 4); }
      }
      // demos on top
      ctx.strokeStyle = c.primary; ctx.lineWidth = 2.2;
      for (const d of demos) { ctx.beginPath(); d.path.forEach((p, i) => (i ? ctx.lineTo(X(p[0]), Y(p[1])) : ctx.moveTo(X(p[0]), Y(p[1])))); ctx.stroke(); }
      ctx.fillStyle = c.accent;
      for (const e of added) { ctx.beginPath(); ctx.arc(X(e.s[0]), Y(e.s[1]), 1.6, 0, 7); ctx.fill(); }
      ctx.fillStyle = c.fg; ctx.font = "12px sans-serif";
      ctx.fillText("blue = expert demos, green/red = 50 learner rollouts", X(0), Y(0) + 0);
      ctx.fillText("goal", X(W.GX) - 12, Y(W.GY) - 16);
      ctx.fillText("obstacle", X(W.OX) - 22, Y(W.OY) + 4);
      if (added.length) { ctx.fillStyle = c.accent; ctx.fillText("amber dots = DAgger expert labels", X(0) + 4, Y(1) + 14); }

      // success curve
      const g = B.ctx; g.clearRect(0, 0, 300, 320);
      const gx = (r) => 45 + (r / 5) * 230, gy = (v) => 280 - v * 240;
      g.strokeStyle = c.light; g.lineWidth = 1; g.beginPath(); g.moveTo(45, 40); g.lineTo(45, 280); g.lineTo(275, 280); g.stroke();
      g.fillStyle = c.fg; g.font = "12px sans-serif";
      for (const v of [0, 0.5, 1]) { g.fillText((v * 100) + "%", 8, gy(v) + 4); g.strokeStyle = c.light; g.setLineDash([2, 4]); g.beginPath(); g.moveTo(45, gy(v)); g.lineTo(275, gy(v)); g.stroke(); g.setLineDash([]); }
      for (let r = 0; r <= 5; r++) g.fillText(r, gx(r) - 3, 296);
      g.fillText("DAgger rounds", 120, 314);
      g.fillText("success rate over 50 seeded rollouts", 45, 16);
      const line = (arr, col, lab, yoff) => {
        g.strokeStyle = col; g.lineWidth = 2; g.beginPath();
        arr.forEach((v, r) => (r ? g.lineTo(gx(r), gy(v)) : g.moveTo(gx(r), gy(v)))); g.stroke();
        g.fillStyle = col; arr.forEach((v, r) => { g.beginPath(); g.arc(gx(r), gy(v), 3, 0, 7); g.fill(); });
        g.fillText(lab, 55, yoff);
      };
      line(curve.k1, c.primary, "no chunking (k = 1)", 34);
      line(curve.kk, c.accent, "chunk k = " + kS.value, 48);
      g.strokeStyle = c.fg; g.lineWidth = 2; g.beginPath(); g.arc(gx(R), gy(ev.rate), 7, 0, 7); g.stroke();

      const n = ev.c;
      out.innerHTML =
        `Policy: ${W.NB}-nearest-neighbour regression on ${demos.length} demos (${demos.reduce((s, d) => s + d.acts.length, 0)} state–action pairs)` +
        `${added.length ? ` + <b>${added.length}</b> DAgger labels` : ""}; ` +
        `${k > 1 ? `executes <b>${k}</b> actions open loop per query` : "re-queries every step"}.<br>` +
        `Success <b>${(100 * ev.rate).toFixed(0)}%</b> — hit obstacle ${n.hit}, left the box ${n.out}, ran out of time ${n.time} (of 50). ` +
        `<i>Circle on the chart = current setting.</i>`;
    }
    run();
  });
})();
