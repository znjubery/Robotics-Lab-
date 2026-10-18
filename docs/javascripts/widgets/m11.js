/* M11 widgets: friction cones and planar force closure */
(function () {
  "use strict";
  const L = window.Lab; if (!L) return;
  const { colors, fmt, el, slider, canvas, deg, project } = L;

  // ---------- pure math (also tested with node) ----------
  const SH = {
    ellipse: { label: "ellipse (maize ear, side view)", rho: 1.3,
      pt(u) { const t = 2 * Math.PI * u; return [1.3 * Math.cos(t), 0.8 * Math.sin(t)]; },
      nrm(u) { const t = 2 * Math.PI * u, a = Math.cos(t) / 1.3, b = Math.sin(t) / 0.8, n = Math.hypot(a, b); return [a / n, b / n]; } },
    circle: { label: "circle (tomato, apple)", rho: 1.0,
      pt(u) { const t = 2 * Math.PI * u; return [Math.cos(t), Math.sin(t)]; },
      nrm(u) { const t = 2 * Math.PI * u; return [Math.cos(t), Math.sin(t)]; } },
    square: { label: "square (dry block)", rho: 0.9 * Math.SQRT2,
      pt(u) { const h = 0.9, v = ((u % 1) + 1) % 1, k = Math.min(3, Math.floor(4 * v)), s = (4 * v - k) * 2 * h - h;
        return [[h, s], [-s, h], [-h, -s], [s, -h]][k]; },
      nrm(u) { const v = ((u % 1) + 1) % 1, k = Math.min(3, Math.floor(4 * v)); return [[1, 0], [0, 1], [-1, 0], [0, -1]][k]; } },
  };
  const rot = (v, a) => [Math.cos(a) * v[0] - Math.sin(a) * v[1], Math.sin(a) * v[0] + Math.cos(a) * v[1]];
  const cross3 = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
  const sub3 = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
  const dot3 = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];

  // edge forces of each friction cone and their planar wrenches (m/ρ, fx, fy)
  function contactWrenches(shape, us, mu) {
    const S = SH[shape], al = Math.atan(mu), out = [];
    us.forEach((u, i) => {
      const p = S.pt(u), n = S.nrm(u), inw = [-n[0], -n[1]];
      for (const sgn of [-1, 1]) {
        const f = rot(inw, sgn * al);
        out.push({ i, p, f, w: [(p[0] * f[1] - p[1] * f[0]) / S.rho, f[0], f[1]] });
      }
    });
    return out;
  }
  // Brute-force convex hull facets (≤ 6 points). Origin strictly inside ⇔ force closure.
  // ε = distance from origin to the nearest facet (Ferrari–Canny with unit edge forces).
  function closure(W) {
    const P = W.map((o) => o.w), n = P.length, facets = [];
    let eps = Infinity;
    for (let i = 0; i < n; i++) for (let j = i + 1; j < n; j++) for (let k = j + 1; k < n; k++) {
      let N = cross3(sub3(P[j], P[i]), sub3(P[k], P[i]));
      const nn = Math.hypot(...N); if (nn < 1e-9) continue;
      N = N.map((v) => v / nn);
      const d = dot3(N, P[i]);
      let pos = false, neg = false;
      for (let l = 0; l < n; l++) { const s = dot3(N, P[l]) - d; if (s > 1e-9) pos = true; if (s < -1e-9) neg = true; }
      if (!pos) { facets.push([i, j, k]); eps = Math.min(eps, d); }      // outward normal N, origin side needs d > 0
      if (!neg) { facets.push([i, j, k]); eps = Math.min(eps, -d); }     // outward normal −N
    }
    if (!facets.length) eps = -Infinity;
    return { ok: eps > 1e-6, eps, facets };
  }
  // Nguyen's two-finger test: segment p1→p2 lies inside both cones
  function nguyen(shape, us, mu) {
    const S = SH[shape], p1 = S.pt(us[0]), p2 = S.pt(us[1]), n1 = S.nrm(us[0]), n2 = S.nrm(us[1]);
    const d = [p2[0] - p1[0], p2[1] - p1[1]], dl = Math.hypot(...d) || 1;
    const a1 = Math.acos(Math.max(-1, Math.min(1, -(n1[0] * d[0] + n1[1] * d[1]) / dl)));
    const a2 = Math.acos(Math.max(-1, Math.min(1, (n2[0] * d[0] + n2[1] * d[1]) / dl)));
    return { a1, a2, ok: Math.max(a1, a2) < Math.atan(mu) };
  }
  if (typeof module !== "undefined") module.exports = { contactWrenches, closure, nguyen, SH };

  L.register("m11-cone", function (root) {
    const ui = el("div", { class: "lab-controls" });
    const pics = el("div", { class: "lab-pics" });
    root.append(ui, pics);
    const O = canvas(pics, 360, 300), Wc = canvas(pics, 300, 300);
    const out = el("div", { class: "lab-readout" }); root.appendChild(out);
    const mkSel = (label, opts, val) => {
      const s = el("select"); for (const [k, v] of opts) s.appendChild(el("option", { value: k }, v)); s.value = val;
      const row = el("div", { class: "lab-row" }); row.append(el("span", { class: "lab-label" }, label), s); ui.appendChild(row);
      s.addEventListener("change", draw); return s;
    };
    const shp = mkSel("object", Object.entries(SH).map(([k, v]) => [k, v.label]), "ellipse");
    const nf = mkSel("fingers", [["2", "2 contacts (pinch)"], ["3", "3 contacts"]], "2");
    const mu = slider(ui, "friction μ", 0, 1.2, 0.01, 0.3, draw, "");
    const c1 = slider(ui, "contact 1 at", 0, 360, 1, 10, draw, "°");
    const c2 = slider(ui, "contact 2 at", 0, 360, 1, 180, draw, "°");
    const c3 = slider(ui, "contact 3 at", 0, 360, 1, 290, draw, "°");
    const cols = ["#e53935", "#1e88e5", "#43a047"];
    const S0 = 95, CX = 180, CY = 150;

    O.c.addEventListener("click", (ev) => {
      const r = O.c.getBoundingClientRect();
      const x = ((ev.clientX - r.left) / r.width * 360 - CX) / S0, y = -((ev.clientY - r.top) / r.height * 300 - CY) / S0;
      let best = 0, bd = Infinity;
      for (let i = 0; i < 720; i++) { const p = SH[shp.value].pt(i / 720), dd = (p[0] - x) ** 2 + (p[1] - y) ** 2; if (dd < bd) { bd = dd; best = i / 2; } }
      const sl = [c1, c2, c3].slice(0, +nf.value);
      let near = sl[0], nd = Infinity;
      for (const s of sl) { const p = SH[shp.value].pt(+s.value / 360), dd = (p[0] - x) ** 2 + (p[1] - y) ** 2; if (dd < nd) { nd = dd; near = s; } }
      near.value = Math.round(best); near.dispatchEvent(new Event("input"));
    });

    function draw() {
      const k = colors(), S = SH[shp.value], m = +mu.value, al = Math.atan(m);
      const us = [c1, c2, c3].slice(0, +nf.value).map((s) => +s.value / 360);
      c3.parentElement.style.opacity = nf.value === "3" ? 1 : 0.35;
      const W = contactWrenches(shp.value, us, m), R = closure(W);
      // --- object panel ---
      let ctx = O.ctx; ctx.clearRect(0, 0, 360, 300);
      const X = (x) => CX + S0 * x, Y = (y) => CY - S0 * y;
      ctx.beginPath();
      for (let i = 0; i <= 200; i++) { const p = S.pt(i / 200); i ? ctx.lineTo(X(p[0]), Y(p[1])) : ctx.moveTo(X(p[0]), Y(p[1])); }
      ctx.closePath(); ctx.fillStyle = R.ok ? "rgba(67,160,71,0.18)" : "rgba(229,57,53,0.12)"; ctx.fill();
      ctx.strokeStyle = k.fg; ctx.lineWidth = 1.5; ctx.stroke();
      ctx.fillStyle = k.fg; ctx.beginPath(); ctx.arc(X(0), Y(0), 3, 0, 7); ctx.fill();
      if (us.length === 2) {
        const p1 = S.pt(us[0]), p2 = S.pt(us[1]);
        ctx.setLineDash([4, 4]); ctx.strokeStyle = k.light; ctx.beginPath(); ctx.moveTo(X(p1[0]), Y(p1[1])); ctx.lineTo(X(p2[0]), Y(p2[1])); ctx.stroke(); ctx.setLineDash([]);
      }
      us.forEach((u, i) => {
        const p = S.pt(u), n = S.nrm(u), inw = [-n[0], -n[1]], Lc = 0.75;
        const e1 = rot(inw, -al), e2 = rot(inw, al);
        ctx.fillStyle = cols[i] + "40"; ctx.strokeStyle = cols[i]; ctx.lineWidth = 1.5;
        ctx.beginPath(); ctx.moveTo(X(p[0]), Y(p[1]));
        ctx.lineTo(X(p[0] + Lc * e1[0]), Y(p[1] + Lc * e1[1]));
        ctx.lineTo(X(p[0] + Lc * e2[0]), Y(p[1] + Lc * e2[1])); ctx.closePath(); ctx.fill(); ctx.stroke();
        ctx.setLineDash([2, 3]); ctx.beginPath(); ctx.moveTo(X(p[0]), Y(p[1])); ctx.lineTo(X(p[0] + 0.9 * inw[0]), Y(p[1] + 0.9 * inw[1])); ctx.stroke(); ctx.setLineDash([]);
        ctx.fillStyle = cols[i]; ctx.beginPath(); ctx.arc(X(p[0]), Y(p[1]), 5, 0, 7); ctx.fill();
        ctx.font = "12px sans-serif"; ctx.fillText(String(i + 1), X(p[0] + 0.18 * n[0]) - 3, Y(p[1] + 0.18 * n[1]) + 4);
      });
      ctx.fillStyle = k.fg; ctx.font = "12px sans-serif";
      ctx.fillText(`Object + friction cones (half-angle ${fmt((al * 180) / Math.PI, 1)}°)`, 6, 16);
      ctx.fillText("click near the edge to move the nearest finger", 6, 292);
      // --- wrench space panel (fx, fy, m/ρ) ---
      ctx = Wc.ctx; ctx.clearRect(0, 0, 300, 300);
      const cx = 150, cy = 165, s = 85, P = (w) => project([w[1], w[2], w[0]], cx, cy, s);
      ctx.strokeStyle = k.light; ctx.lineWidth = 1;
      for (const [a, lab] of [[[1.2, 0, 0], "fₓ"], [[0, 1.2, 0], "f_y"], [[0, 0, 1.2], "m/ρ"]]) {
        const [x0, y0] = project([0, 0, 0], cx, cy, s), [x1, y1] = project(a, cx, cy, s);
        ctx.beginPath(); ctx.moveTo(x0, y0); ctx.lineTo(x1, y1); ctx.stroke(); ctx.fillStyle = k.light; ctx.fillText(lab, x1 + 3, y1);
      }
      ctx.fillStyle = R.ok ? "rgba(67,160,71,0.13)" : "rgba(229,57,53,0.10)";
      ctx.strokeStyle = R.ok ? k.green : k.red;
      for (const [a, b, c] of R.facets) {
        const A = P(W[a].w), B = P(W[b].w), C = P(W[c].w);
        ctx.beginPath(); ctx.moveTo(...A); ctx.lineTo(...B); ctx.lineTo(...C); ctx.closePath(); ctx.fill(); ctx.stroke();
      }
      W.forEach((o) => { const q = P(o.w); ctx.fillStyle = cols[o.i]; ctx.beginPath(); ctx.arc(q[0], q[1], 4, 0, 7); ctx.fill(); });
      const z = project([0, 0, 0], cx, cy, s);
      ctx.fillStyle = k.fg; ctx.beginPath(); ctx.arc(z[0], z[1], 4, 0, 7); ctx.fill(); ctx.fillText("origin", z[0] + 6, z[1] + 12);
      ctx.fillText("Wrench space: hull of cone-edge wrenches", 6, 16);
      // --- readout ---
      let ng = "";
      if (us.length === 2) {
        const G = nguyen(shp.value, us, m);
        ng = `<br>Two-finger check: line 1→2 makes ${fmt((G.a1 * 180) / Math.PI, 1)}° and ${fmt((G.a2 * 180) / Math.PI, 1)}° with the inward normals; ` +
          `cone half-angle is ${fmt((al * 180) / Math.PI, 1)}° → ${G.ok ? "inside both cones" : "outside a cone"} (agrees with the hull test).`;
      }
      out.innerHTML =
        `α = atan μ = <b>${fmt((al * 180) / Math.PI, 1)}°</b>. ${W.length} edge wrenches in 3D wrench space (mᶻ/ρ, fₓ, f_y), ${R.facets.length} hull facets.<br>` +
        `Force closure: <b style="color:${R.ok ? k.green : k.red}">${R.ok ? "YES — the hull wraps the origin, so any push can be resisted" : "NO — some push makes it squirt out"}</b>. ` +
        `Margin ε = ${isFinite(R.eps) ? fmt(R.eps, 3) : "—"} ${R.ok ? "(radius of the largest ball around the origin inside the hull)" : "(≤ 0: origin on or outside the hull)"}` + ng +
        (m === 0 ? `<br>μ = 0: each cone collapses to its normal. In the plane you need at least 4 frictionless contacts, so 2–3 can never hold.` : "");
    }
    draw();
  });
})();
