/* ───────── IRoC-U Base Station · dashboard client ───────── */
const $ = (id) => document.getElementById(id);
const api = (p, opt) => fetch(p, opt);
const post = (p, body) => fetch(p, {
  method: "POST",
  headers: { "Content-Type": "application/json" },
  body: body ? JSON.stringify(body) : undefined,
});

let PHASES = [];
let fieldW = null, fieldH = null;          // arena size (m) when known
let dyn = { minX: -2, maxX: 6, minY: -3, maxY: 5 };  // fallback bounds
let trail = [];
const TRAIL_MAX = 40;
let arenaZoom = 1;
let lastTargets = [];     // most recent target list (for stage-3 visual count)
let lastUpdated = -1;     // results "updated" timestamp we last rendered images for

/* ───────── WebSocket ───────── */
let ws = null;
function connectWS() {
  const proto = location.protocol === "https:" ? "wss" : "ws";
  ws = new WebSocket(`${proto}://${location.host}/ws`);
  ws.onopen = () => setWS(true);
  ws.onclose = () => { setWS(false); setTimeout(connectWS, 1500); };
  ws.onerror = () => ws.close();
  ws.onmessage = (e) => handle(JSON.parse(e.data));
}
function setWS(on) {
  const el = $("wsState");
  el.textContent = on ? "● connected" : "● disconnected";
  el.className = on ? "ws-on" : "ws-off";
}

function handle(m) {
  switch (m.type) {
    case "hello":
      PHASES = m.phases; renderPhases();
      applySettings(m.settings);
      updateTelemetry(m.telemetry);
      (m.logs || []).forEach(addLog);
      (m.dock_logs || []).forEach(addDockLog);
      applySummary(m.summary);
      loadConfig();
      break;
    case "telemetry": updateTelemetry(m.data); break;
    case "log": addLog(m.data); break;
    case "dock": addDockLog(m.data); break;
    case "refresh":
      if (m.summary) applySummary(m.summary);
      if (m.what === "targets" || m.what === "results") { refreshArena(); refreshTargets(); refreshStages(); }
      if (m.what === "map") { refreshArena(); refreshStages(); }
      if (m.what === "3d") { refreshModel(true); refreshStages(); }
      if (m.what === "photos") refreshPhotos();
      break;
  }
}

/* ───────── mission phases ───────── */
function renderPhases() {
  const box = $("phases");
  if (!box) return;            // phase pills removed from UI
  box.innerHTML = "";
  PHASES.forEach((p) => {
    const d = document.createElement("div");
    d.className = "phase"; d.dataset.phase = p; d.textContent = p;
    box.appendChild(d);
  });
}
function setPhase(state) {
  const idx = PHASES.indexOf(state);
  [...document.querySelectorAll(".phase")].forEach((el, i) => {
    el.classList.toggle("active", el.dataset.phase === state);
    el.classList.toggle("done", idx >= 0 && i < idx);
  });
  const btn = $("btnStart");
  if (btn) btn.classList.toggle("armed",
    state && !["Idle", "Done", "Landed"].includes(state));
}

/* ───────── telemetry ───────── */
function fmt(v, d = 2) { return (v === null || v === undefined) ? "—" : Number(v).toFixed(d); }
function mmss(s) {
  s = Math.max(0, Math.floor(s || 0));
  return String((s / 60) | 0).padStart(2, "0") + ":" + String(s % 60).padStart(2, "0");
}
function setTxt(id, val) { const el = $(id); if (el) el.textContent = val; }

function updateTelemetry(t) {
  if (!t) return;
  // battery
  const bp = t.battery_pct ?? 0;
  setTxt("batPct", fmt(bp, 0) + "%");
  const fill = $("batFill");
  if (fill) {
    fill.style.width = Math.max(0, Math.min(100, bp)) + "%";
    fill.style.background = bp < 20 ? "linear-gradient(90deg,#c0263a,#ff5c6c)"
      : bp < 45 ? "linear-gradient(90deg,#b8860b,#ffb454)"
        : "linear-gradient(90deg,#16b888,#3ef0b0)";
  }
  // (PHOTOS counter shows received-on-disk count, set in refreshPhotos)

  // data transfer bar
  const xw = $("xferWrap");
  if (xw) {
    if (t.state === "Data Transfer" || (t.transfer_pct ?? 0) > 0) {
      xw.style.display = "block";
      $("xferFill").style.width = (t.transfer_pct ?? 0) + "%";
      setTxt("xferPct", fmt(t.transfer_pct, 0) + "%");
    } else xw.style.display = "none";
  }

  // mission control header
  setTxt("timer", mmss(t.mission_elapsed_s));
  setTxt("sortie", t.sortie ?? 0);
  const dot = $("linkDot"); if (dot) dot.classList.toggle("on", !!t.connected);
  setTxt("linkLabel", (t.link_mode || t.link_type || "—").toUpperCase());
  setPhase(t.state);
  // No live position: the drone has no link to the base station in flight.
}

/* ───────── live position HUD (shown on 2D + 3D) ───────── */
function updatePosHud(t) {
  const link = t.connected ? "● LINK" : "○ LINK";
  const txt =
    `${link}   STATE ${t.state || "—"}\n` +
    `X ${fmt(t.x)}   Y ${fmt(t.y)}   Z ${fmt(t.z)} m\n` +
    `ALT ${fmt(t.altitude_m)} m   YAW ${fmt(t.yaw_deg, 0)}°\n` +
    `SPD ${fmt(t.speed_mps)} m/s   BAT ${fmt(t.battery_pct, 0)}%`;
  const a = $("posHud2d"), b = $("posHud3d");
  if (a) a.textContent = txt;
  if (b) b.textContent = txt;
}

/* ───────── arena live marker ───────── */
function pct(x, y) {
  let lx, ty;
  if (fieldW && fieldH) {
    lx = (x / fieldW) * 100;
    ty = (1 - y / fieldH) * 100;
  } else {
    const w = Math.max(0.5, dyn.maxX - dyn.minX), h = Math.max(0.5, dyn.maxY - dyn.minY);
    lx = ((x - dyn.minX) / w) * 100;
    ty = (1 - (y - dyn.minY) / h) * 100;
  }
  return [Math.max(2, Math.min(98, lx)), Math.max(2, Math.min(98, ty))];
}
function placeDrone(x, y) {
  if (x === undefined || y === undefined) return;
  if (!(fieldW && fieldH)) {           // grow fallback bounds
    dyn.minX = Math.min(dyn.minX, x - 1); dyn.maxX = Math.max(dyn.maxX, x + 1);
    dyn.minY = Math.min(dyn.minY, y - 1); dyn.maxY = Math.max(dyn.maxY, y + 1);
  }
  const [lx, ty] = pct(x, y);
  const d = $("drone");
  d.style.left = lx + "%"; d.style.top = ty + "%";
  $("droneLabel").textContent = `${x.toFixed(1)}, ${y.toFixed(1)}`;
  // trail
  trail.push([lx, ty]); if (trail.length > TRAIL_MAX) trail.shift();
  const tb = $("trail");
  tb.innerHTML = trail.map((p, i) =>
    `<span style="left:${p[0]}%;top:${p[1]}%;opacity:${0.05 + 0.3 * i / trail.length}"></span>`).join("");
}

/* ───────── arena image + zoom/scroll ───────── */
function refreshArena() {
  const img = $("arenaImg");
  const url = `/api/image/annotated?t=${lastUpdated}`;   // stable token
  if (img.src.endsWith(`t=${lastUpdated}`)) return;        // already current
  img.src = url;
  img.onload = () => { img.style.display = "block"; $("mapEmpty").style.display = "none"; };
  img.onerror = () => { img.style.display = "none"; $("mapEmpty").style.display = "block"; };
}
function applyZoom() {
  const c = $("mapCanvas"), f = $("mapFrame");
  c.style.transform = `scale(${arenaZoom})`;
  f.style.overflow = arenaZoom > 1.001 ? "auto" : "hidden";
  $("zoomLvl").textContent = Math.round(arenaZoom * 100) + "%";
}
document.querySelectorAll("[data-zoom]").forEach((b) => {
  b.onclick = () => {
    const k = b.dataset.zoom;
    if (k === "in") arenaZoom = Math.min(4, arenaZoom + 0.25);
    else if (k === "out") arenaZoom = Math.max(1, arenaZoom - 0.25);
    else arenaZoom = 1;
    applyZoom();
  };
});

/* ───────── 3D model ───────── */
function refreshModel(force) {
  const mv = $("mv");
  if (!mv) return;
  const url = `/api/model?t=${lastUpdated}`;        // stable token
  if (!mv.src || (force && !mv.src.endsWith(`t=${lastUpdated}`))) mv.src = url;
}
// model-viewer tells us itself whether the .glb loaded
(function wireModelEvents() {
  const mv = $("mv");
  if (!mv) return;
  mv.addEventListener("load", () => { const e = $("modelEmpty"); if (e) e.style.display = "none"; });
  mv.addEventListener("error", () => { const e = $("modelEmpty"); if (e) e.style.display = "block"; });
})();

/* ───────── fullscreen helper + per-section buttons ───────── */
function toggleFullscreen(el) {
  if (!el) return;
  if (!document.fullscreenElement) {
    (el.requestFullscreen || el.webkitRequestFullscreen || (() => {})).call(el);
  } else {
    (document.exitFullscreen || document.webkitExitFullscreen || (() => {})).call(document);
  }
}
document.querySelectorAll("[data-full]").forEach((b) => {
  b.onclick = () => toggleFullscreen(document.getElementById(b.dataset.full));
});
document.addEventListener("fullscreenchange", () => {
  document.querySelectorAll("[data-full]").forEach((b) =>
    b.textContent = document.fullscreenElement ? "⛶ Exit" : "⛶ Full");
});

/* ───────── pipeline stages gallery ─────────
   Loads every stage image directly and drops the ones that 404, so it works
   even before the /api/stages endpoint exists (no server restart needed). */
function addStageCard(box, tag, sub, src) {
  const card = document.createElement("div");
  card.className = "stage-card";
  const head = document.createElement("div");
  head.className = "stage-h";
  head.innerHTML = `<b>${tag}</b><span>${sub}</span>`;
  const fbtn = document.createElement("button");
  fbtn.className = "btn tiny card-full";
  fbtn.textContent = "⛶ Full";
  fbtn.onclick = (e) => { e.stopPropagation(); toggleFullscreen(card); };
  head.appendChild(fbtn);
  const img = document.createElement("img");
  img.loading = "lazy";
  img.src = src + (src.includes("?") ? "" : `?t=${Date.now()}`);
  img.onerror = () => { card.remove(); checkStagesEmpty(box); };
  card.appendChild(head);
  card.appendChild(img);
  card.onclick = () => openCardModal(card);
  box.appendChild(card);
}
function checkStagesEmpty(box) {
  if (!box.querySelector(".stage-card"))
    box.innerHTML = `<div class="map-empty">No stage outputs yet — run the pipeline.</div>`;
}
let stagesVer = null;
function refreshStages() {
  const box = $("stagesBox");
  if (!box) return;
  // only rebuild when results actually changed (avoids per-second flicker)
  if (stagesVer === lastUpdated && box.querySelector(".stage-card")) return;
  stagesVer = lastUpdated;
  box.innerHTML = "";
  const t = lastUpdated;                 // stable token → no reload until data changes
  addStageCard(box, "Stage 1", "Orthomosaic (stitched arena)", `/api/image/ortho?t=${t}`);
  addStageCard(box, "Stage 2", "Rectified field (ENU calibrated)", `/api/image/rectified?t=${t}`);
  const n = lastTargets.length || 8;
  for (let i = 1; i <= n; i++)
    addStageCard(box, "Stage 3", `Match visual ${i}.jpg`, `/api/visual/${i}?t=${t}`);
  addStageCard(box, "Stage 4", "Annotated field — targets circled", `/api/image/annotated?t=${t}`);
  setTimeout(() => checkStagesEmpty(box), 2000);
}

/* ───────── received-photos: square tiles (all, lazy-loaded) ───────── */
function refreshPhotos() {
  const g = $("photosGrid");
  if (!g) return;
  api("/api/photos").then((r) => r.json()).then((d) => {
    const photos = d.photos || [];
    const cnt = $("photoCount"); if (cnt) cnt.textContent = `${photos.length} photos`;
    setTxt("tPhotos", photos.length);          // top-bar PHOTOS = received count
    if (!photos.length) {
      g.innerHTML = `<div class="map-empty">No photos received yet.</div>`;
      return;
    }
    // render ALL tiles; images load lazily as they scroll into view
    g.innerHTML = "";
    photos.forEach((name) => {
      const src = "/api/photo/" + encodeURIComponent(name);
      const tile = document.createElement("div");
      tile.className = "photo-tile";
      tile.innerHTML = `<img src="${src}" loading="lazy"><div class="pname">${name}</div>`;
      tile.onclick = () => openCardModal(tile);
      g.appendChild(tile);
    });
  }).catch(() => {});
}
/* ───────── result files (csv / json / xlsx) ───────── */
function humanSize(n) {
  if (n < 1024) return n + " B";
  if (n < 1048576) return (n / 1024).toFixed(1) + " KB";
  return (n / 1048576).toFixed(1) + " MB";
}
let filesSig = null;
function refreshFiles() {
  const box = $("filesList");
  if (!box) return;
  api("/api/datafiles").then((r) => r.json()).then((d) => {
    const files = d.files || [];
    const sig = JSON.stringify(files.map((f) => [f.rel, f.size]));
    if (sig === filesSig) return;
    filesSig = sig;
    if (!files.length) {
      box.innerHTML = `<div class="map-empty">No result files yet — run the pipeline.</div>`;
      return;
    }
    box.innerHTML = "";
    files.forEach((f) => {
      const enc = f.rel.split("/").map(encodeURIComponent).join("/");
      const row = document.createElement("div");
      row.className = "file-row";
      row.innerHTML = `<span class="fext">${f.ext}</span>
        <span class="fname">${f.rel}</span>
        <span class="fsize">${humanSize(f.size)}</span>
        <a class="file-act view" href="/api/viewfile/${enc}" target="_blank" rel="noopener">View</a>
        <a class="file-act" href="/api/datafile/${enc}" download="${f.name}">Download</a>`;
      box.appendChild(row);
    });
  }).catch(() => {});
}
$("btnOpenResults").onclick = () => {
  post("/api/open_results").then((r) => r.json()).then((r) => {
    if (!r.ok) addLog({ level: "warn", msg: "open results folder: " + (r.error || "failed") });
  }).catch(() => {});
};

/* ───────── all-results gallery ───────── */
let resultsSig = null;
function renderResults(targets) {
  const g = $("resultsGrid");
  if (!g) return;
  const sig = JSON.stringify(targets.map((t) => [t.name, t.x, t.y, t.z, t.confidence, t.proof]));
  if (sig === resultsSig) return;        // unchanged → don't rebuild (no flicker)
  resultsSig = sig;
  if (!targets.length) {
    g.innerHTML = `<div class="map-empty">No results yet — run the pipeline.</div>`;
    return;
  }
  g.innerHTML = "";
  targets.forEach((t) => {
    const card = document.createElement("div");
    card.className = "rcard";
    const conf = (t.confidence || "").toString().toUpperCase();
    const img = t.has_proof
      ? `<img src="/api/proof/${encodeURIComponent(t.proof)}" loading="lazy">`
      : `<img src="/api/visual/${encodeURIComponent(t.name)}.jpg" loading="lazy" onerror="this.style.visibility='hidden'">`;
    // title row first: number · identity · x y z · confidence
    card.innerHTML = `
      <div class="rhead">
        <span class="rnum rname">${t.name || "—"}</span>
        <span class="rident">${t.identity || ""}</span>
        <span class="rxyz">x=${t.x ?? "—"}  y=${t.y ?? "—"}  z=${t.z ?? "—"} m
          ${conf ? `<span class="conf ${conf}">${conf}</span>` : ""}</span>
        <button class="btn tiny card-full">⛶ Full</button>
      </div>
      ${img}`;
    card.querySelector(".card-full").onclick = (e) => { e.stopPropagation(); toggleFullscreen(card); };
    card.onclick = () => openCardModal(card);
    g.appendChild(card);
  });
}

/* ───────── targets ───────── */
function applySummary(s) {
  if (!s) return;
  const badge = `Found ${s.found} / ${s.total}`;
  $("foundBadge").textContent = badge;
  const fb2 = $("foundBadge2"); if (fb2) fb2.textContent = badge;
  if (s.field && s.field.raw) {
    const m = s.field.raw.match(/([\d.]+)\s*m\s*x\s*([\d.]+)\s*m/i);
    if (m) { fieldW = parseFloat(m[1]); fieldH = parseFloat(m[2]); }
  }
  lastTargets = s.targets || [];
  renderTargets(lastTargets);
  renderResults(lastTargets);
}

/* ───────── auto-poll: keep checking for new results ───────── */
function poll() {
  api("/api/targets").then((r) => r.json()).then((s) => {
    applySummary(s);
    // every section is always visible now → refresh images on any change
    if (s.updated !== undefined && s.updated !== lastUpdated) {
      lastUpdated = s.updated;
      refreshArena();
      refreshModel(true);
      refreshStages();
      refreshFiles();
    }
  }).catch(() => {});
}
function refreshTargets() { api("/api/targets").then(r => r.json()).then(applySummary); }

function renderTargets(targets) {
  const body = $("targetsBody");
  if (!body) return;            // table removed in scroll layout
  if (!targets.length) {
    body.innerHTML = `<tr class="empty"><td colspan="6">No targets yet — run the pipeline.</td></tr>`;
    return;
  }
  body.innerHTML = "";
  targets.forEach((t) => {
    const tr = document.createElement("tr");
    tr.className = t.found ? "found-row" : "notfound";
    const thumb = t.has_proof
      ? `<img class="thumb" src="/api/proof/${encodeURIComponent(t.proof)}" loading="lazy">`
      : `<div class="thumb"></div>`;
    const conf = (t.confidence || "").toString().toUpperCase();
    tr.innerHTML = `
      <td>${thumb}</td>
      <td><div class="tname">${t.name || "—"}</div>
          ${t.identity ? `<div class="tident">${t.identity}</div>` : ""}</td>
      <td class="mono">${t.x ?? "—"}</td>
      <td class="mono">${t.y ?? "—"}</td>
      <td class="mono">${t.z ?? "—"}</td>
      <td>${conf ? `<span class="conf ${conf}">${conf}</span>` : "—"}</td>`;
    if (t.found && t.has_proof) {
      tr.onclick = () => openModal(t);   // table row → single proof
    }
    body.appendChild(tr);
  });
}

/* ───────── modal lightbox (with prev/next slider) ───────── */
let modalList = [];
let modalIdx = 0;

function showModalAt(i) {
  if (!modalList.length) return;
  modalIdx = (i + modalList.length) % modalList.length;
  const it = modalList[modalIdx];
  $("modalTitle").textContent = it.title || "";
  $("modalImg").src = it.src;
  $("modalCount").textContent = modalList.length > 1 ? `${modalIdx + 1} / ${modalList.length}` : "";
  $("modal").classList.add("open");
}

// open from a clicked card; build the slide list from its sibling cards
function openCardModal(card) {
  const cards = [...card.parentElement.children].filter((c) => c.querySelector && c.querySelector("img"));
  modalList = cards.map((c) => {
    const im = c.querySelector("img");
    let title = (c.querySelector(".rname")?.textContent
      || c.querySelector(".pname")?.textContent
      || c.querySelector(".stage-h")?.textContent
      || im.getAttribute("alt") || "").trim();
    const xyz = c.querySelector(".rxyz")?.textContent?.trim();
    if (xyz) title += "   " + xyz;
    return { src: im.src, title };
  });
  showModalAt(cards.indexOf(card));
}

function openImageModal(title, src) {
  modalList = [{ title, src: src + (src.includes("?") ? "" : `?t=${Date.now()}`) }];
  showModalAt(0);
}
function openModal(t) {
  modalList = [{
    title: `x=${t.x ?? "?"}  y=${t.y ?? "?"}  z=${t.z ?? "?"} m`,
    src: `/api/proof/${encodeURIComponent(t.proof)}?t=${Date.now()}`,
  }];
  showModalAt(0);
}

function closeModal() { $("modal").classList.remove("open"); }
$("modalClose").onclick = closeModal;
$("modalPrev").onclick = (e) => { e.stopPropagation(); showModalAt(modalIdx - 1); };
$("modalNext").onclick = (e) => { e.stopPropagation(); showModalAt(modalIdx + 1); };
$("modal").onclick = (e) => { if (e.target.id === "modal") closeModal(); };
document.addEventListener("keydown", (e) => {
  if (!$("modal").classList.contains("open")) return;
  if (e.key === "ArrowLeft") showModalAt(modalIdx - 1);
  else if (e.key === "ArrowRight") showModalAt(modalIdx + 1);
  else if (e.key === "Escape") closeModal();
});

/* ───────── logs ───────── */
function addLog(d) {
  const c = $("logConsole");
  const near = c.scrollHeight - c.scrollTop - c.clientHeight < 60;
  const line = document.createElement("div");
  const lvl = (d.level || "info").toLowerCase();
  line.className = "log-line log-" + (["info", "warn", "error", "pipe"].includes(lvl) ? lvl : "info");
  const ts = new Date((d.t || Date.now() / 1000) * 1000).toLocaleTimeString();
  line.innerHTML = `<span class="ts">${ts}</span>  ${escapeHtml(d.msg || "")}`;
  c.appendChild(line);
  while (c.childElementCount > 600) c.removeChild(c.firstChild);
  if (near) c.scrollTop = c.scrollHeight;
}
function escapeHtml(s) { return s.replace(/[&<>]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;" }[c])); }
$("clearLogs").onclick = () => ($("logConsole").innerHTML = "");

/* ───────── docking / charging (separate ESP stream) ───────── */
function addDockLog(d) {
  const c = $("dockConsole");
  if (!c) return;
  const near = c.scrollHeight - c.scrollTop - c.clientHeight < 60;
  const line = document.createElement("div");
  const lvl = (d.level || "esp").toLowerCase();
  const cls = { esp: "log-info", cmd: "log-pipe", info: "log-pipe", warn: "log-warn", error: "log-error" }[lvl] || "log-info";
  line.className = "log-line " + cls;
  const ts = new Date((d.t || Date.now() / 1000) * 1000).toLocaleTimeString();
  line.innerHTML = `<span class="ts">${ts}</span>  ${escapeHtml(d.msg || "")}`;
  c.appendChild(line);
  while (c.childElementCount > 800) c.removeChild(c.firstChild);
  if (near) c.scrollTop = c.scrollHeight;
}
$("clearDock").onclick = () => ($("dockConsole").innerHTML = "");
$("btnDockStart").onclick = () => post("/api/dock/start");
$("btnDockStop").onclick = () => post("/api/dock/stop");
function loadDockState() {
  api("/api/dock_state").then((r) => r.json()).then((d) => {
    const el = $("dockEsp");
    if (el) el.textContent = "ESP: " + (d.esp_ip || "not registered");
  }).catch(() => {});
}

/* ───────── settings / config ───────── */
function applySettings(s) {
  if (!s) return;
  $("runPipe").checked = !!s.run_pipeline_on_mission;
  if (s.pipeline_job) $("pipeJob").value = s.pipeline_job;
}
function loadConfig() {
  api("/api/config").then(r => r.json()).then((c) => {
    $("cfgBase").textContent = "base: " + c.base_dir;
    $("cfgCsv").textContent = "csv: " + (c.coordinates_csv ? shorten(c.coordinates_csv) : "none");
    $("cfgModel").textContent = "3d: " + (c.model_glb ? shorten(c.model_glb) : "—");
    if (c.model_glb) $("cfgModel").title = c.model_glb;   // full path on hover
    if (c.link_mode) $("linkMode").value = c.link_mode === "simulator" ? "simulator" : $("linkMode").value;
    if (c.result_set) $("resultSet").value = c.result_set;   // reflect active result view
  });
}
function shorten(p) { const a = p.split("/"); return a.slice(-2).join("/"); }

/* ───────── controls wiring ───────── */
{ const _bs = $("btnStart"); if (_bs) _bs.onclick = () => post("/api/command/start_mission"); }
document.querySelectorAll("[data-cmd]").forEach((b) => {
  b.onclick = () => post("/api/command/" + b.dataset.cmd);
});
$("linkMode").onchange = (e) => post("/api/link/" + e.target.value);
$("runPipe").onchange = (e) => post("/api/settings", { run_pipeline_on_mission: e.target.checked });
$("pipeJob").onchange = (e) => post("/api/settings", { pipeline_job: e.target.value });
$("btnRunPipe").onclick = () => post("/api/pipeline/" + $("pipeJob").value)
  .then(r => r.json()).then(r => { if (!r.ok) addLog({ level: "error", msg: r.error }); });
$("btnStopPipe").onclick = () => post("/api/pipeline_stop");
$("resultSet").onchange = (e) => post("/api/result_set/" + e.target.value)
  .then(r => r.json()).then((r) => {
    if (r && r.ok === false) { addLog({ level: "warn", msg: "64×64 result nahi mila (pehle MATCH 64×64 run karo)" }); return; }
    location.reload();                       // switch view -> reload all panels from the new path
  });

/* ───────── sidebar: jump to a section ───────── */
const sideLinks = [...document.querySelectorAll(".side-link")];
sideLinks.forEach((b) => {
  b.onclick = () => {
    const el = document.getElementById(b.dataset.go);
    if (el) el.scrollIntoView({ behavior: "smooth", block: "start" });
  };
});
// highlight the section currently in view
(function wireScrollSpy() {
  const sp = $("scrollPage");
  if (!sp) return;
  sp.addEventListener("scroll", () => {
    let cur = null;
    sideLinks.forEach((b) => {
      const el = document.getElementById(b.dataset.go);
      if (el && el.getBoundingClientRect().top <= 140) cur = b;
    });
    sideLinks.forEach((b) => b.classList.toggle("active", b === cur));
  });
})();

/* ───────── boot ───────── */
connectWS();
applyZoom();
refreshArena();
refreshModel(false);
refreshStages();
refreshPhotos();
refreshFiles();
loadDockState();
poll();                       // immediate first load (targets + results)
setInterval(poll, 2000);      // keep checking for new results every 2 s
setInterval(loadDockState, 15000);   // refresh ESP registration status
