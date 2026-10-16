/* Hero: a Franka FR3 with a UMI gripper, rendered offline from the real FR3 meshes (MuJoCo ray casting) and drawn here
   as a fine halftone, one dot per cell.
   Story: the "R" of the headline starts mirrored at its home. The arm grasps it from above, flips it back about the
   vertical axis, the R slips at the end of the flip and falls (rigid-body drop simulated offline in MuJoCo, replayed
   here), the arm reacts, grabs it on the floor, seats it at home and lets go, then dissolves toward the name. Plays once.
   Video frames hold two layers stacked vertically: the scene, and the gripper parts that pass in front of the R (drawn
   above the text). Pixel coding: grey = surface luminance (<= 240), orange = Fin Ray fingers, green = base LED ring,
   pure white = nothing. The R itself is DOM text: a CSS 3D block of stacked glyph layers, so at home it is a flat
   silhouette like the other letters and its thickness shows only while it moves. */
(function () {
  'use strict';
  var canvas = document.getElementById('hero-arm'), fcan = document.getElementById('hero-front'),
      ccan = document.getElementById('hero-cables'), video = document.getElementById('hero-video'),
      caption = document.getElementById('hero-caption'), letter = document.getElementById('hero-R');
  if (!canvas || !canvas.getContext || !fcan || !ccan || !video || !caption || !letter) return;
  var h1 = caption.querySelector('h1'), section = canvas.closest('section');
  var reduce = window.matchMedia('(prefers-reduced-motion: reduce)').matches;
  var ctx = canvas.getContext('2d'), fctx = fcan.getContext('2d'), cctx = ccan.getContext('2d');
  var off = document.createElement('canvas'), octx = off.getContext('2d', { willReadFrequently: true });
  var meta, NX, NY, FPS, N, WORLD;
  var W = 0, H = 0, DPR = 1, pxPerM = 1, pitch = 1;          // css px
  var homePx = [0, 0], inkOff = [0, 0], nameBox = null;
  var LAYERS = 20, built = false;
  var INK = [22, 22, 22], ORANGE = [236, 112, 20], GREEN = [34, 197, 94], ACCENT = [53, 82, 242];

  // ---------- dot stamping into an ImageData buffer (device pixels, sub-pixel phase) ----------
  var stamps = {};
  function stamp(r, fx, fy) {
    var key = (Math.round(r * 16) * 4 + fx) * 4 + fy;
    if (stamps[key]) return stamps[key];
    var rr = Math.round(r * 16) / 16, o = Math.ceil(rr) + 1, n = 2 * o + 1, a = new Float32Array(n * n);
    var cx = o + fx / 4, cy = o + fy / 4;
    for (var y = 0; y < n; y++) for (var x = 0; x < n; x++) {
      var cov = 0;
      for (var sy = 0; sy < 4; sy++) for (var sx = 0; sx < 4; sx++) {
        var dx = x + (sx + 0.5) / 4 - cx, dy = y + (sy + 0.5) / 4 - cy;
        if (dx * dx + dy * dy <= rr * rr) cov++;
      }
      a[y * n + x] = cov / 16;
    }
    return (stamps[key] = { n: n, o: o, a: a });
  }
  function Layer(cv, c2d) { this.cv = cv; this.c2d = c2d; this.img = null; }
  Layer.prototype.reset = function () {
    if (!this.img || this.img.width !== this.cv.width || this.img.height !== this.cv.height) this.img = this.c2d.createImageData(this.cv.width, this.cv.height);
    else this.img.data.fill(0);
  };
  Layer.prototype.plot = function (x, y, r, col, alpha) {      // x, y, r in css px
    var d = this.img.data, bw = this.img.width, bh = this.img.height;
    x *= DPR; y *= DPR; r *= DPR;
    if (r < 0.15) return;
    var ix = Math.floor(x), iy = Math.floor(y), s = stamp(r, Math.floor((x - ix) * 4), Math.floor((y - iy) * 4));
    var x0 = ix - s.o, y0 = iy - s.o, A = 255 * (alpha === undefined ? 1 : alpha);
    for (var j = 0; j < s.n; j++) {
      var py = y0 + j; if (py < 0 || py >= bh) continue;
      for (var i = 0; i < s.n; i++) {
        var px = x0 + i; if (px < 0 || px >= bw) continue;
        var c = s.a[j * s.n + i]; if (!c) continue;
        var k = (py * bw + px) * 4, al = c * A;
        if (al > d[k + 3]) { d[k] = col[0]; d[k + 1] = col[1]; d[k + 2] = col[2]; d[k + 3] = al; }
      }
    }
  };
  Layer.prototype.flush = function () { this.c2d.putImageData(this.img, 0, 0); };
  var base = new Layer(canvas, ctx), fore = new Layer(fcan, fctx);

  // ---------- geometry ----------
  function toPx(x, z) { return [(x - WORLD.x0) * pxPerM, (WORLD.z1 - z) * pxPerM]; }
  function sizeCanvas(cv, w, h) { cv.width = Math.max(1, Math.round(w * DPR)); cv.height = Math.max(1, Math.round(h * DPR)); cv.style.width = w + 'px'; cv.style.height = h + 'px'; }
  function layout() {
    var w = Math.round(canvas.getBoundingClientRect().width);
    if (!w) return false;
    W = w; H = Math.round(W * NY / NX); DPR = Math.min(window.devicePixelRatio || 1, 2);
    pxPerM = W / (WORLD.x1 - WORLD.x0); pitch = W / NX;
    sizeCanvas(canvas, W, H); sizeCanvas(fcan, W, H);
    var sr = section.getBoundingClientRect(), cr = canvas.getBoundingClientRect();
    sizeCanvas(ccan, W, Math.max(0, Math.round(sr.bottom - cr.bottom)));
    homePx = toPx(meta.home_world[0], meta.home_world[1]);
    placeText();
    if (cableCols) drawCables(1);
    return true;
  }
  // font size from the world scale (R ink height = glyph.height), then put the R's ink centre on its home
  function placeText() {
    caption.style.transform = 'none'; var keep = letter.style.transform; letter.style.transform = 'none';
    var cs = getComputedStyle(h1), probe = document.createElement('canvas').getContext('2d');
    probe.font = cs.fontStyle + ' ' + cs.fontWeight + ' 100px ' + cs.fontFamily;
    var m100 = probe.measureText('R'), inkH = m100.actualBoundingBoxAscent + m100.actualBoundingBoxDescent;
    if (inkH > 0) h1.style.fontSize = (meta.glyph.height * pxPerM * 100 / inkH).toFixed(2) + 'px';
    var ls = getComputedStyle(letter);
    probe.font = ls.fontStyle + ' ' + ls.fontWeight + ' ' + ls.fontSize + ' ' + ls.fontFamily;
    var mt = probe.measureText('R');
    var mark = document.createElement('span'); mark.style.cssText = 'display:inline-block;width:0;height:0;vertical-align:baseline';
    letter.appendChild(mark);
    var lr = letter.getBoundingClientRect(), br = mark.getBoundingClientRect(); letter.removeChild(mark);
    var baseY = br.top - lr.top;
    inkOff = [(mt.actualBoundingBoxRight - mt.actualBoundingBoxLeft) / 2, baseY - (mt.actualBoundingBoxAscent - mt.actualBoundingBoxDescent) / 2];
    letter.style.transformOrigin = inkOff[0].toFixed(2) + 'px ' + inkOff[1].toFixed(2) + 'px';
    var cr = canvas.getBoundingClientRect();
    var dx = homePx[0] - (lr.left - cr.left + inkOff[0]), dy = homePx[1] - (lr.top - cr.top + inkOff[1]);
    caption.style.transform = 'translate(' + dx.toFixed(1) + 'px,' + dy.toFixed(1) + 'px)';
    var hr = h1.getBoundingClientRect(), lr2 = letter.getBoundingClientRect();
    h1.style.perspective = Math.round(W * 0.75) + 'px';
    h1.style.perspectiveOrigin = (lr2.left - hr.left + inkOff[0]).toFixed(1) + 'px ' + (lr2.top - hr.top + inkOff[1]).toFixed(1) + 'px';
    nameBox = { x0: hr.left - cr.left, x1: hr.right - cr.left, y0: hr.top - cr.top + 0.15 * hr.height, y1: hr.bottom - cr.top - 0.15 * hr.height };
    buildLayers();
    letter.style.transform = keep;
    caption.classList.add('is-placed');
  }
  // the R as a block: the in-flow glyph stays (transparent) for layout and accessibility; 20 stacked copies draw it
  function buildLayers() {
    var T = meta.glyph.thickness * pxPerM;
    if (!built) {
      for (var k = 0; k < LAYERS; k++) {
        var s = document.createElement('span'); s.className = 'hl'; s.setAttribute('aria-hidden', 'true'); s.textContent = 'R';
        var u = k / (LAYERS - 1), edge = k === 0 || k === LAYERS - 1, v = Math.round(78 - 30 * u);
        s.style.color = edge ? '#111' : 'rgb(' + v + ',' + v + ',' + (v + 2) + ')';
        letter.appendChild(s);
      }
      letter.classList.add('is-3d'); built = true;
    }
    var hl = letter.querySelectorAll('.hl');
    for (var j = 0; j < hl.length; j++) hl[j].style.transform = 'translateZ(' + (-T / 2 + T * j / (LAYERS - 1)).toFixed(2) + 'px)';
  }

  // ---------- letter pose ----------
  function fallPose(t) {
    var tr = meta.fall, f = Math.max(0, Math.min(tr.length - 1.001, t * 240)), i = Math.floor(f), u = f - i, a = tr[i], b = tr[i + 1];
    return [a[1] + (b[1] - a[1]) * u, a[2] + (b[2] - a[2]) * u, a[3] + (b[3] - a[3]) * u];
  }
  function setLetter(t) {                                   // t: video time (s)
    var f = Math.min(N - 1, t * FPS), i = Math.floor(f), u = f - i, A = meta.frames[i].L, B = meta.frames[Math.min(N - 1, i + 1)].L;
    var x, z, th = 0, yaw = 0;
    if (A.st === 'fall') { var p = fallPose(t - meta.t_drop); x = p[0]; z = p[1]; th = p[2] * 180 / Math.PI; }
    else {
      var same = B.st === A.st;
      x = same ? A.x + (B.x - A.x) * u : A.x; z = same ? A.z + (B.z - A.z) * u : A.z;
      th = A.th; yaw = same ? A.yaw + (B.yaw - A.yaw) * u : A.yaw;
    }
    var p2 = toPx(x, z), dx = p2[0] - homePx[0], dy = p2[1] - homePx[1];
    var flat = Math.abs(dx) + Math.abs(dy) + Math.abs(th) < 1e-3 && Math.abs(((yaw % 360) + 360) % 360) < 1e-3;
    var tf = flat ? 'none' : 'translate(' + dx.toFixed(2) + 'px,' + dy.toFixed(2) + 'px) rotate(' + th.toFixed(2) + 'deg) rotateY(' + yaw.toFixed(2) + 'deg)';
    if (letter.style.transform !== tf) letter.style.transform = tf;
  }

  // ---------- frame decoding and drawing ----------
  var lastDots = [], trail = [], cableCols = null, floorAlpha = 1;
  function readFrame() {
    octx.drawImage(video, 0, 0, NX, 2 * NY);
    return octx.getImageData(0, 0, NX, 2 * NY).data;
  }
  function dotsFrom(px, row0, out, breath) {
    var rmax = pitch / 2 * 1.04;
    for (var j = 0; j < NY; j++) for (var i = 0; i < NX; i++) {
      var k = ((row0 + j) * NX + i) * 4, r = px[k], g = px[k + 1], b = px[k + 2];
      if (r > 246 && g > 246 && b > 246) continue;
      var x = (i + 0.5) * pitch, y = (j + 0.5) * pitch;
      if (g > r + 60 && g > b + 60) { out.push(x, y, rmax * (0.72 + 0.28 * breath), 2, 0.45 + 0.55 * breath); continue; }
      if (r > g + 25 && r > b + 60 && r > 150) {
        var tt = Math.max(0, Math.min(1, (g / 255 - 0.439) / 0.411));
        out.push(x, y, rmax * Math.sqrt(0.12 + 0.88 * (1 - 0.85 * tt)), 1, 1); continue;
      }
      var lum = (r + g + b) / 765, c = 0.03 + 0.97 * Math.pow(Math.max(0, 1 - lum), 1.3);
      out.push(x, y, rmax * Math.sqrt(c), 0, 1);
    }
  }
  var COLS = [INK, ORANGE, GREEN, ACCENT];
  function drawFloor(L) {
    var j0 = Math.round(WORLD.z1 / (WORLD.z1 - WORLD.z0) * NY), rmax = pitch / 2 * 1.04;
    if (floorAlpha <= 0) return;
    for (var j = j0; j < NY; j++) {
      var depth = (j - j0) / Math.max(1, NY - j0), y = (j + 0.5) * pitch;
      for (var i = 0; i < NX; i++) {
        var xf = Math.min(1, Math.min(i, NX - 1 - i) / (0.08 * NX));
        var c = (j === j0 ? 0.2 : 0.11 * Math.pow(1 - depth, 1.4)) * xf;
        if (c > 0.01) L.plot((i + 0.5) * pitch, y, rmax * Math.sqrt(c), INK, floorAlpha);
      }
    }
  }
  function findCables(px) {                                  // dark runs in the bottom row, left part
    var cols = [], j = NY - 1, run = null;
    for (var i = 0; i < NX * 0.35; i++) {
      var k = (j * NX + i) * 4, lum = (px[k] + px[k + 1] + px[k + 2]) / 765, dark = lum < 0.6;
      if (dark && !run) run = [i, i]; else if (dark) run[1] = i; else if (run) { cols.push(run); run = null; }
    }
    if (run) cols.push(run);
    return cols;
  }
  function drawCables(alpha) {
    if (!cableCols || ccan.height < 2) return;
    cctx.setTransform(1, 0, 0, 1, 0, 0); cctx.clearRect(0, 0, ccan.width, ccan.height);
    cctx.fillStyle = 'rgba(22,22,22,' + alpha + ')';
    var rows = Math.ceil(ccan.height / DPR / pitch), r = pitch / 2 * 0.86 * DPR;
    cctx.beginPath();
    for (var c = 0; c < cableCols.length; c++) for (var i = cableCols[c][0]; i <= cableCols[c][1]; i++)
      for (var j = 0; j < rows; j++) {
        var x = (i + 0.5) * pitch * DPR, y = (j + 0.5) * pitch * DPR;
        cctx.moveTo(x + r, y); cctx.arc(x, y, r, 0, 6.2832);
      }
    cctx.fill();
  }
  function drawFrame(frame, now) {
    var px = readFrame(), breath = 0.5 + 0.5 * Math.sin(now * 2 * Math.PI / 2.6);
    if (!cableCols) { cableCols = findCables(px); drawCables(1); }
    var dots = []; dotsFrom(px, 0, dots, breath);
    base.reset(); drawFloor(base);
    for (var k = 0; k < dots.length; k += 5) base.plot(dots[k], dots[k + 1], dots[k + 2], COLS[dots[k + 3]], dots[k + 4]);
    var tp = meta.frames[frame].tip, last = trail[trail.length - 1];
    if (!last || Math.hypot(tp[0] - last[0], tp[1] - last[1]) > 0.6) trail.push([tp[0], tp[1], now]);
    while (trail.length && now - trail[0][2] > 3.2) trail.shift();
    for (var t = 0; t < trail.length; t++) {
      var v = 1 - (now - trail[t][2]) / 3.2; if (v <= 0) continue;
      var tx = (Math.floor(trail[t][0]) + 0.5) * pitch, ty = (Math.floor(trail[t][1]) + 0.5) * pitch, tr = pitch / 2 * 0.62 * Math.sqrt(v);
      base.plot(tx, ty, tr, ACCENT, 0.9); dots.push(tx, ty, tr, 3, 0.9);
    }
    base.flush();
    fore.reset();
    if (meta.frames[frame].front) {
      var fd = []; dotsFrom(px, NY, fd, breath);
      for (var q = 0; q < fd.length; q += 5) fore.plot(fd[q], fd[q + 1], fd[q + 2], COLS[fd[q + 3]], fd[q + 4]);
      Array.prototype.push.apply(dots, fd);
    }
    fore.flush();
    lastDots = dots;
  }

  // ---------- dissolve: every dot drifts toward the name as mist and fades ----------
  var particles = null, d0 = 0;
  function mulberry(a) { return function () { a |= 0; a = a + 0x6d2b79f5 | 0; var t = Math.imul(a ^ a >>> 15, 1 | a); t = t + Math.imul(t ^ t >>> 7, 61 | t) ^ t; return ((t ^ t >>> 14) >>> 0) / 4294967296; }; }
  function startDissolve(now) {
    var P = [], nb = nameBox, rnd = mulberry(7);
    for (var k = 0; k < lastDots.length; k += 5) {
      var x = lastDots[k], y = lastDots[k + 1];
      var tx = nb.x0 + rnd() * (nb.x1 - nb.x0), ty = nb.y0 + rnd() * (nb.y1 - nb.y0);
      var life = 0.62 + 0.3 * rnd(), reach = 0.42 + 0.28 * rnd();
      var delay = 0.24 * Math.max(0, Math.min(1, (nb.x0 - x) / Math.max(1, nb.x0)));
      P.push({ x: x, y: y, r: lastDots[k + 2], c: lastDots[k + 3], a: lastDots[k + 4], vx: (tx - x) * reach / life, vy: (ty - y) * reach / life,
               life: life, delay: delay, ph: rnd() * 6.283, sw: (0.6 + rnd()) * pitch * 6 });
    }
    particles = P; d0 = now;
    ccan.style.transition = 'opacity 0.5s ease'; ccan.style.opacity = '0';
    fore.reset(); fore.flush();
  }
  function stepDissolve(now) {
    var t = now - d0, alive = 0;
    floorAlpha = Math.max(0, 1 - t / 0.55);
    base.reset(); drawFloor(base);
    for (var k = 0; k < particles.length; k++) {
      var p = particles[k], tau = t - p.delay;
      if (tau < 0) { base.plot(p.x, p.y, p.r, COLS[p.c], p.a); alive++; continue; }
      var u = tau / p.life; if (u >= 1) continue;
      alive++;
      var e = 1 - Math.pow(1 - u, 2.2);
      var x = p.x + p.vx * p.life * e + Math.sin(p.ph + 7 * tau) * p.sw * u * 0.25;
      var y = p.y + p.vy * p.life * e + Math.cos(p.ph + 5 * tau) * p.sw * u * 0.25 - 10 * u * pitch;
      base.plot(x, y, p.r * (1 + 0.9 * u), COLS[p.c], p.a * Math.pow(1 - u, 2));
    }
    base.flush();
    return alive > 0 || floorAlpha > 0;
  }

  // ---------- run ----------
  function showStill() {                                     // reduced motion / no autoplay: arm at rest, name readable
    letter.style.transform = 'none';
    var once = function () { drawFrame(0, 0); canvas.classList.add('is-ready'); };
    if (video.readyState >= 2) once(); else video.addEventListener('loadeddata', once, { once: true });
  }
  function run() {
    var t0 = performance.now(), lastFrame = -1, dissolving = false;
    function loop(nowMs) {
      var now = (nowMs - t0) / 1000;
      if (dissolving) {
        if (stepDissolve(now)) requestAnimationFrame(loop);
        else { base.reset(); base.flush(); video.pause(); }
        return;
      }
      requestAnimationFrame(loop);
      if (video.readyState < 2) return;
      var vt = video.currentTime, f = Math.min(N - 1, Math.floor(vt * FPS + 1e-3));
      setLetter(vt);
      if (f !== lastFrame || meta.frames[f].L.st === 'home') { drawFrame(f, now); lastFrame = f; }
      if (!canvas.classList.contains('is-ready')) canvas.classList.add('is-ready');
      if (vt >= meta.t_release + 0.08 || video.ended) {
        letter.style.transform = 'none'; dissolving = true; startDissolve(now);
      }
    }
    requestAnimationFrame(loop);
  }
  fetch('images/hero/hero.json').then(function (r) { return r.json(); }).then(function (m) {
    meta = m; NX = m.nx; NY = m.ny; FPS = m.fps; N = m.frames.length; WORLD = m.world;
    meta.home_world = [WORLD.x0 + m.home[0] * (WORLD.x1 - WORLD.x0), WORLD.z1 - m.home[1] * (WORLD.z1 - WORLD.z0)];
    off.width = NX; off.height = 2 * NY;
    var fonts = document.fonts && document.fonts.load ? document.fonts.load('800 100px Raleway') : Promise.resolve();
    return fonts.catch(function () {}).then(function () {
      return new Promise(function (res) { (function poll() { if (canvas.getBoundingClientRect().width > 0) res(); else requestAnimationFrame(poll); })(); });
    });
  }).then(function () {
    if (!reduce) letter.style.transform = 'rotateY(180deg)';
    layout();
    window.addEventListener('resize', function () { if (meta && layout() && video.readyState >= 2 && !particles) drawFrame(Math.min(N - 1, Math.floor(video.currentTime * FPS)), 0); });
    if (reduce) { showStill(); return; }
    var p = video.play();
    if (p && p.then) p.then(run, showStill); else run();
  }).catch(function (e) { if (window.console) console.warn('hero failed to load', e); letter.style.transform = 'none'; caption.classList.add('is-placed'); });
})();
