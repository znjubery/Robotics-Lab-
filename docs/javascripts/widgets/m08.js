/* M8 widgets — trajectory generation.
   m08-timescale: cubic vs quintic vs trapezoidal time scaling s(t) of a straight
   joint-space path for a 2R arm. Plots s, ṡ, s̈ live; animates the arm. */
(function () {
  "use strict";

  // ---------------- time scalings (s: 0 → 1) ----------------
  // each returns [s, sdot, sddot] at time t for duration T
  const cubic = (t, T) => { const u = t / T; return [3 * u * u - 2 * u ** 3, (6 * u - 6 * u * u) / T, (6 - 12 * u) / (T * T)]; };
  const quintic = (t, T) => {
    const u = t / T;
    return [10 * u ** 3 - 15 * u ** 4 + 6 * u ** 5, (30 * u * u - 60 * u ** 3 + 30 * u ** 4) / T, (60 * u - 180 * u * u + 120 * u ** 3) / (T * T)];
  };
  // trapezoid from (v, a): returns {T, v, a, f(t)}; switches to triangle (bang-bang) if v²/a > 1
  function trapezoid(v, a) {
    if (v * v / a > 1) v = Math.sqrt(a);
    const T = (a + v * v) / (v * a), ta = v / a;
    const f = (t) => {
      if (t <= ta) return [0.5 * a * t * t, a * t, a];
      if (t <= T - ta) return [v * t - v * v / (2 * a), v, 0];
      const r = T - t; return [1 - 0.5 * a * r * r, a * r, -a];
    };
    return { T, v, a, ta, f };
  }
  // trapezoid with given T and a (L&P option): v = (aT − √(a²T² − 4a))/2, needs aT² ≥ 4
  function trapezoidTA(T, a) {
    a = Math.max(a, 4 / (T * T));
    const v = (a * T - Math.sqrt(Math.max(0, a * a * T * T - 4 * a))) / 2;
    return trapezoid(v, a);
  }
  const peaks = { cubic: [1.5, 6], quintic: [1.875, 10 / Math.sqrt(3)] }; // ṡmax·T, s̈max·T²
  const fastestT = (kind, v, a) => Math.max(peaks[kind][0] / v, Math.sqrt(peaks[kind][1] / a));
  const api = { cubic, quintic, trapezoid, trapezoidTA, peaks, fastestT };
  if (typeof module !== "undefined" && module.exports) module.exports = api;
  if (typeof window === "undefined") return;

  // ---------------- widget ----------------
  const L = window.Lab; if (!L) return;
  const { colors, fmt, el, slider, canvas, deg } = L;

  L.register("m08-timescale", function (root) {
    const ui = el("div", { class: "lab-controls" });
    const pics = el("div", { class: "lab-pics" });
    root.append(ui, pics);
    const G = canvas(pics, 420, 340), A = canvas(pics, 300, 300);
    const out = el("div", { class: "lab-readout" }); root.appendChild(out);

    const row = el("div", { class: "lab-row" });
    const mode = el("select");
    mode.appendChild(el("option", { value: "lim" }, "Fastest motion within ṡmax, s̈max"));
    mode.appendChild(el("option", { value: "T" }, "All take the same time T"));
    row.append(el("span", { class: "lab-label" }, "Compare"), mode); ui.appendChild(row);
    const row2 = el("div", { class: "lab-row" });
    const drive = el("select");
    for (const [v, t] of [["quintic", "quintic"], ["cubic", "cubic"], ["trap", "trapezoid"]]) drive.appendChild(el("option", { value: v }, t));
    const play = el("button", { class: "md-button" }, "Play");
    play.style.padding = "0.15em 0.9em"; play.style.fontSize = "0.7rem"; play.style.margin = "0";
    row2.append(el("span", { class: "lab-label" }, "Arm follows"), drive, play); ui.appendChild(row2);

    const Ts = slider(ui, "duration T", 0.5, 4, 0.05, 2, draw, " s");
    const vs = slider(ui, "ṡmax", 0.3, 2, 0.05, 0.8, draw, " /s");
    const as = slider(ui, "s̈max", 0.5, 6, 0.1, 2, draw, " /s²");
    mode.addEventListener("change", draw); drive.addEventListener("change", draw);

    // example joint-space path for the arm (straight line in θ)
    const L1 = 1, L2 = 0.8, thA = [deg(-20), deg(100)], thB = [deg(110), deg(-40)];
    let tNow = 0, anim = false, last = null;
    play.onclick = () => { anim = true; tNow = 0; last = null; requestAnimationFrame(loop); };
    function loop(ts) {
      if (!anim || !document.body.contains(root)) return;
      if (last !== null) tNow += (ts - last) / 1000;
      last = ts; draw();
      if (tNow > profiles().Tmax + 0.4) { anim = false; tNow = profiles().Tmax; draw(); return; }
      requestAnimationFrame(loop);
    }

    function profiles() {
      const v = +vs.value, a = +as.value, T = +Ts.value;
      let P;
      if (mode.value === "lim") {
        const Tc = fastestT("cubic", v, a), Tq = fastestT("quintic", v, a), tr = trapezoid(v, a);
        P = { cubic: { T: Tc, f: (t) => cubic(t, Tc) }, quintic: { T: Tq, f: (t) => quintic(t, Tq) }, trap: { T: tr.T, f: tr.f, tr } };
      } else {
        const tr = trapezoidTA(T, a);
        P = { cubic: { T, f: (t) => cubic(t, T) }, quintic: { T, f: (t) => quintic(t, T) }, trap: { T: tr.T, f: tr.f, tr } };
      }
      P.Tmax = Math.max(P.cubic.T, P.quintic.T, P.trap.T);
      return P;
    }
    const evalP = (p, t) => (t <= 0 ? [0, 0, 0] : t >= p.T ? [1, 0, 0] : p.f(t));

    function draw() {
      const k = colors(), P = profiles(), v = +vs.value, a = +as.value;
      const col = { cubic: k.blue, quintic: k.green, trap: k.red };
      // ---- three stacked plots ----
      let ctx = G.ctx; ctx.clearRect(0, 0, 420, 340);
      const l = 40, r = 410, tEnd = P.Tmax * 1.08;
      const X = (t) => l + ((r - l) * t) / tEnd;
      const panels = [
        { name: "s(t)", i: 0, lo: 0, hi: 1.05, lim: null },
        { name: "ṡ(t)", i: 1, lo: 0, hi: Math.max(v, 1.5 / P.cubic.T,1.875 / P.quintic.T, P.trap.tr.v) * 1.15, lim: v },
        { name: "s̈(t)", i: 2, lo: null, hi: null, lim: a },
      ];
      const amax = Math.max(a, 6 / P.cubic.T ** 2, 5.78 / P.quintic.T ** 2, P.trap.tr.a) * 1.15;
      panels[2].lo = -amax; panels[2].hi = amax;
      panels.forEach((pn, j) => {
        const top = 8 + j * 108, bot = top + 92;
        const Y = (y) => bot - ((bot - top) * (y - pn.lo)) / (pn.hi - pn.lo);
        ctx.strokeStyle = k.light; ctx.lineWidth = 1; ctx.strokeRect(l, top, r - l, bot - top);
        if (pn.lo < 0) { ctx.beginPath(); ctx.moveTo(l, Y(0)); ctx.lineTo(r, Y(0)); ctx.stroke(); }
        if (pn.lim !== null && mode.value === "lim") {
          ctx.setLineDash([4, 3]); ctx.strokeStyle = k.fg; ctx.globalAlpha = 0.5;
          ctx.beginPath(); ctx.moveTo(l, Y(pn.lim)); ctx.lineTo(r, Y(pn.lim)); ctx.stroke();
          if (pn.lo < 0) { ctx.beginPath(); ctx.moveTo(l, Y(-pn.lim)); ctx.lineTo(r, Y(-pn.lim)); ctx.stroke(); }
          ctx.setLineDash([]); ctx.globalAlpha = 1;
        }
        for (const key of ["cubic", "quintic", "trap"]) {
          ctx.strokeStyle = col[key]; ctx.lineWidth = key === drive.value ? 2.6 : 1.4; ctx.beginPath();
          const N = 300;
          for (let q = 0; q <= N; q++) {
            const t = (tEnd * q) / N; let y = evalP(P[key], t)[pn.i];
            // draw the acceleration jumps as vertical lines (they are real jumps)
            const px = X(t), py = Y(y);
            q ? ctx.lineTo(px, py) : ctx.moveTo(px, py);
          }
          ctx.stroke();
        }
        ctx.fillStyle = k.fg; ctx.font = "12px sans-serif"; ctx.fillText(pn.name, 4, top + 14);
        ctx.fillText(fmt(pn.hi, 1), 4, top + 28);
        if (pn.lo < 0) ctx.fillText(fmt(pn.lo, 1), 4, bot);
        if (anim || tNow > 0) { ctx.strokeStyle = k.accent; ctx.beginPath(); ctx.moveTo(X(tNow), top); ctx.lineTo(X(tNow), bot); ctx.stroke(); }
      });
      ctx.fillStyle = k.fg; ctx.fillText("0", l - 3, 336); ctx.fillText(fmt(tEnd, 2) + " s", r - 36, 336);
      ctx.fillStyle = k.blue; ctx.fillText("cubic", 130, 336); ctx.fillStyle = k.green; ctx.fillText("quintic", 180, 336);
      ctx.fillStyle = k.red; ctx.fillText("trapezoid", 240, 336);

      // ---- arm ----
      ctx = A.ctx; ctx.clearRect(0, 0, 300, 300);
      const s = 72, cx = 150, cy = 170;
      const fk = (th) => { const x1 = L1 * Math.cos(th[0]), y1 = L1 * Math.sin(th[0]);
        return [[x1, y1], [x1 + L2 * Math.cos(th[0] + th[1]), y1 + L2 * Math.sin(th[0] + th[1])]]; };
      const thAt = (sv) => [thA[0] + sv * (thB[0] - thA[0]), thA[1] + sv * (thB[1] - thA[1])];
      // tip path (curved — it is straight in joint space, not task space)
      ctx.strokeStyle = k.light; ctx.setLineDash([3, 3]); ctx.lineWidth = 1.2; ctx.beginPath();
      for (let q = 0; q <= 60; q++) { const p = fk(thAt(q / 60))[1]; q ? ctx.lineTo(cx + s * p[0], cy - s * p[1]) : ctx.moveTo(cx + s * p[0], cy - s * p[1]); }
      ctx.stroke(); ctx.setLineDash([]);
      // dots at equal time steps: spacing shows the speed profile
      const pd = P[drive.value];
      ctx.fillStyle = col[drive.value];
      for (let q = 0; q <= 20; q++) { const p = fk(thAt(evalP(pd, (pd.T * q) / 20)[0]))[1]; ctx.beginPath(); ctx.arc(cx + s * p[0], cy - s * p[1], 2.5, 0, 7); ctx.fill(); }
      const sv = evalP(pd, tNow)[0], [e1, e2] = fk(thAt(sv));
      ctx.lineWidth = 5; ctx.lineCap = "round"; ctx.strokeStyle = k.primary; ctx.beginPath();
      ctx.moveTo(cx, cy); ctx.lineTo(cx + s * e1[0], cy - s * e1[1]); ctx.lineTo(cx + s * e2[0], cy - s * e2[1]); ctx.stroke();
      ctx.fillStyle = k.fg; ctx.beginPath(); ctx.arc(cx, cy, 4, 0, 7); ctx.fill();
      ctx.fillStyle = k.accent; ctx.beginPath(); ctx.arc(cx + s * e2[0], cy - s * e2[1], 5, 0, 7); ctx.fill();
      ctx.fillStyle = k.fg; ctx.font = "12px sans-serif";
      ctx.fillText("straight line in θ → curved tip path", 8, 16);
      ctx.fillText("dots = equal time steps (Δt = T/20)", 8, 292);

      // ---- readout ----
      const line = (key, name) => {
        const p = P[key];
        const pk = key === "trap" ? [p.tr.v, p.tr.a] : [peaks[key][0] / p.T, peaks[key][1] / p.T ** 2];
        const a0 = evalP(p, 1e-9)[2];
        return `<span style="color:${col[key]}">■</span> ${name}: T = <b>${fmt(p.T, 2)} s</b>, peak ṡ = ${fmt(pk[0], 2)}, peak s̈ = ${fmt(pk[1], 2)}, s̈ at start = ${fmt(a0, 2)}` +
          (key === "quintic" ? " (smooth start)" : " (jumps from 0: infinite jerk)");
      };
      out.innerHTML = line("cubic", "cubic") + "<br>" + line("quintic", "quintic") + "<br>" + line("trap", "trapezoid") +
        (P.trap.tr.v * P.trap.tr.v / P.trap.tr.a > 0.999 ? " — <i>triangle: never reaches cruise</i>" : ` — cruise for ${fmt(P.trap.T - 2 * P.trap.tr.ta, 2)} s`) +
        (mode.value === "T" && P.trap.tr.a > a * 1.0001
          ? `<br><b style="color:${k.red}">Note:</b> s̈max = ${fmt(a, 2)} is too small to finish in T = ${fmt(+Ts.value, 2)} s (needs a·T² ≥ 4), so the trapezoid was raised to s̈ = ${fmt(P.trap.tr.a, 2)}.`
          : "");
    }
    draw();
  });
})();
