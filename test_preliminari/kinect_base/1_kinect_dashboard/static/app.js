/* Kinect v1 dashboard frontend: Socket.IO client, Chart.js accel plot, mic arrow. */
(() => {
  "use strict";

  const WINDOW_S = 5;           // rolling chart window
  const $ = (id) => document.getElementById(id);
  const fmt = (v, d = 2) => (v === null || v === undefined || Number.isNaN(v) ? "--" : Number(v).toFixed(d));

  // ------------------------------------------------------------ helpers
  function toast(msg, kind = "danger") {
    const el = document.createElement("div");
    el.className = `toast align-items-center text-bg-${kind} border-0 show mb-2`;
    el.innerHTML = `<div class="d-flex"><div class="toast-body"></div>
      <button type="button" class="btn-close btn-close-white me-2 m-auto"></button></div>`;
    el.querySelector(".toast-body").textContent = msg;
    el.querySelector("button").onclick = () => el.remove();
    $("toast-area").appendChild(el);
    setTimeout(() => el.remove(), 4000);
  }

  async function post(url, body) {
    try {
      const r = await fetch(url, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(body),
      });
      const j = await r.json();
      if (!j.ok) toast(j.error || `Errore ${r.status}`);
      return j;
    } catch (e) {
      toast(`Richiesta fallita: ${e}`);
      return { ok: false };
    }
  }

  function setBadge(el, text, kind) {
    el.textContent = text;
    el.className = `badge text-bg-${kind}`;
  }

  // ------------------------------------------------------------ tilt
  const slider = $("tilt-slider");
  const TILT_MIN = Number(slider.min), TILT_MAX = Number(slider.max);
  let tiltTimer = null;

  function sendTilt(angle) {
    angle = Math.max(TILT_MIN, Math.min(TILT_MAX, Math.round(angle)));
    slider.value = angle;
    $("tilt-target").textContent = angle;
    clearTimeout(tiltTimer);
    // debounce: the motor is slow, avoid flooding it while dragging
    tiltTimer = setTimeout(() => post("/api/tilt", { angle }), 150);
  }

  slider.addEventListener("input", () => sendTilt(Number(slider.value)));
  document.querySelectorAll("[data-tilt-step]").forEach((b) =>
    b.addEventListener("click", () => sendTilt(Number(slider.value) + Number(b.dataset.tiltStep))));
  document.querySelectorAll("[data-tilt-set]").forEach((b) =>
    b.addEventListener("click", () => sendTilt(Number(b.dataset.tiltSet))));

  // ------------------------------------------------------------ LED
  document.querySelectorAll("[data-led]").forEach((b) =>
    b.addEventListener("click", async () => {
      const j = await post("/api/led", { state: b.dataset.led });
      if (j.ok) $("led-current").textContent = j.state;
    }));

  // ------------------------------------------------------------ video mode
  const videoImg = $("video-img");
  document.querySelectorAll("input[name=vmode]").forEach((r) =>
    r.addEventListener("change", async () => {
      if (!r.checked) return;
      const j = await post("/api/video_mode", { mode: r.value });
      if (j.ok) videoImg.src = `/video_${j.mode}?t=${Date.now()}`;
    }));

  // reconnect MJPEG <img> if the stream drops (e.g. server restart)
  [videoImg, $("depth-img")].forEach((img) => {
    img.addEventListener("error", () => {
      setTimeout(() => { img.src = img.src.split("?")[0] + `?t=${Date.now()}`; }, 1000);
    });
  });

  // ------------------------------------------------------------ accel chart
  const mkDs = (label, color) => ({
    label, data: [], borderColor: color, backgroundColor: color,
    borderWidth: 1.5, pointRadius: 0, tension: 0,
  });
  const chart = new Chart($("accel-chart"), {
    type: "line",
    data: { datasets: [mkDs("X", "#dc3545"), mkDs("Y", "#20c997"), mkDs("Z", "#0d6efd")] },
    options: {
      animation: false,
      responsive: true,
      maintainAspectRatio: false,
      parsing: false,
      normalized: true,
      spanGaps: true,
      plugins: { legend: { labels: { color: "#ccc", boxWidth: 12 } }, tooltip: { enabled: false } },
      scales: {
        x: {
          type: "linear",
          ticks: { color: "#888", callback: (v) => `${Number(v).toFixed(0)}s`, maxTicksLimit: 6 },
          grid: { color: "#2a313c" },
        },
        y: { suggestedMin: -12, suggestedMax: 12, ticks: { color: "#888" }, grid: { color: "#2a313c" } },
      },
    },
  });

  let lastT = 0;
  let chartDirty = false;

  function addAccelSamples(samples) {
    const [dx, dy, dz] = chart.data.datasets.map((d) => d.data);
    for (const [t, x, y, z] of samples) {
      if (t < lastT) { dx.length = dy.length = dz.length = 0; } // server restarted
      lastT = t;
      dx.push({ x: t, y: x });
      dy.push({ x: t, y: y });
      dz.push({ x: t, y: z });
    }
    const tMin = lastT - WINDOW_S;
    for (const arr of [dx, dy, dz]) {
      let i = 0;
      while (i < arr.length && arr[i].x < tMin) i++;
      if (i) arr.splice(0, i);
    }
    chart.options.scales.x.min = tMin;
    chart.options.scales.x.max = lastT;
    chartDirty = true;
  }

  // ------------------------------------------------------------ mic arrow
  (function drawTicks() {
    const ticks = $("mic-ticks"), labels = $("mic-tick-labels");
    for (let a = -50; a <= 50; a += 10) {
      const r = (a * Math.PI) / 180, s = Math.sin(r), c = Math.cos(r);
      const big = a % 50 === 0 || a === 0;
      const r1 = 110, r2 = big ? 100 : 105;
      const line = document.createElementNS("http://www.w3.org/2000/svg", "line");
      line.setAttribute("x1", 150 + r1 * s); line.setAttribute("y1", 160 - r1 * c);
      line.setAttribute("x2", 150 + r2 * s); line.setAttribute("y2", 160 - r2 * c);
      ticks.appendChild(line);
      if (big || a % 20 === 0) {
        const t = document.createElementNS("http://www.w3.org/2000/svg", "text");
        t.setAttribute("x", 150 + 122 * s); t.setAttribute("y", 163 - 122 * c);
        t.textContent = `${a}°`;
        labels.appendChild(t);
      }
    }
  })();

  const arrow = $("mic-arrow"), arrowLine = $("mic-arrow-line"), arrowHead = $("mic-arrow-head");
  let micData = null;

  function drawMic(d) {
    const sign = $("mic-invert").checked ? -1 : 1;
    const angle = sign * d.angle;
    const conf = Math.max(0, Math.min(1, d.confidence));
    const len = 25 + 75 * conf;                // arrow length proportional to confidence
    const tipY = 160 - len;
    arrowLine.setAttribute("y2", tipY + 10);
    arrowHead.setAttribute("points", `150,${tipY} 141,${tipY + 16} 159,${tipY + 16}`);
    arrow.style.transform = `rotate(${angle}deg)`;
    arrow.style.opacity = 0.35 + 0.65 * conf;
    $("mic-angle").textContent = fmt(angle, 1);
    $("mic-conf").textContent = fmt(conf, 2);
    $("mic-db").textContent = fmt(d.rms_db, 0);
    const bars = document.querySelectorAll("#mic-levels .level-bar > div");
    (d.levels_db || []).forEach((db, i) => {
      if (bars[i]) bars[i].style.width = `${Math.max(0, Math.min(100, ((db + 70) / 60) * 100))}%`;
    });
    $("st-mic").textContent = `${fmt(angle, 0)}° (${fmt(conf, 2)})`;
  }

  // ------------------------------------------------------------ render loop
  let accelData = null;
  function frame() {
    if (chartDirty) { chart.update("none"); chartDirty = false; }
    if (accelData) {
      const a = accelData;
      $("acc-x").textContent = fmt(a.x);
      $("acc-y").textContent = fmt(a.y);
      $("acc-z").textContent = fmt(a.z);
      $("acc-pitch").textContent = fmt(a.pitch, 1);
      $("acc-roll").textContent = fmt(a.roll, 1);
      $("acc-norm").textContent = fmt(Math.hypot(a.x, a.y, a.z));
      $("tilt-current").textContent = fmt(a.tilt, 1);
      $("tilt-status").textContent = a.tilt_status || "--";
      $("st-tilt").textContent = fmt(a.tilt, 1);
      $("st-accel").textContent = `${fmt(a.x, 1)} / ${fmt(a.y, 1)} / ${fmt(a.z, 1)}`;
      $("st-fps-accel").textContent = fmt(a.hz, 0);
      accelData = null;
    }
    if (micData) { drawMic(micData); micData = null; }
    requestAnimationFrame(frame);
  }
  requestAnimationFrame(frame);

  // ------------------------------------------------------------ socket
  const socket = io("/data", { transports: ["websocket", "polling"] });

  socket.on("connect", () => setBadge($("st-socket"), "Socket: connesso", "success"));
  socket.on("disconnect", () => {
    setBadge($("st-socket"), "Socket: disconnesso", "danger");
    setBadge($("st-kinect"), "Kinect: ?", "secondary");
  });

  socket.on("accel_data", (d) => {
    addAccelSamples(d.samples || []);
    accelData = d;
  });

  socket.on("mic_direction", (d) => { micData = d; });

  let firstStatus = true;
  socket.on("status", (s) => {
    setBadge($("st-kinect"), s.connected ? "Kinect: connessa" : "Kinect: disconnessa", s.connected ? "success" : "danger");
    const fps = s.fps || {};
    $("st-fps-rgb").textContent = fmt(fps.rgb, 0);
    $("st-fps-ir").textContent = fmt(fps.ir, 0);
    $("st-fps-depth").textContent = fmt(fps.depth, 0);
    if (fps.accel !== undefined) $("st-fps-accel").textContent = fmt(fps.accel, 0);
    if (s.tilt !== undefined) $("st-tilt").textContent = fmt(s.tilt, 1);
    $("center-depth").textContent = fmt(s.center_depth_m, 2);
    if (s.led) $("led-current").textContent = s.led;
    $("st-error").textContent = s.error || "";
    $("st-error").title = s.error || "";

    if (s.video_mode) {
      const radio = $(`vmode-${s.video_mode}`);
      if (radio && !radio.checked) {
        radio.checked = true;
        videoImg.src = `/video_${s.video_mode}?t=${Date.now()}`;
      }
    }
    if (firstStatus && s.tilt_target !== undefined) {
      slider.value = s.tilt_target;
      $("tilt-target").textContent = s.tilt_target;
      firstStatus = false;
    }

    const m = s.mic || {};
    if (m.available) {
      $("mic-info").textContent = `${m.device} — ${m.channels} canali @ ${m.rate} Hz, ${fmt(m.hz, 0)} stime/s`;
    } else {
      $("mic-info").textContent = `Microfoni non disponibili: ${m.error || "?"}`;
      $("st-mic").textContent = "off";
    }
  });
})();
