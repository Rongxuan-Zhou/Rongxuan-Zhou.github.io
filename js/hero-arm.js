/* Hero: a Franka FR3 with a UMI gripper, rendered as a halftone dot grid from pre-computed frames.
   The frames were ray-cast from the real FR3 geometry (MuJoCo) and stored as an atlas:
   R = luminance, G = mask (0 none, 127 arm, 254 orange finger pads), B = how face-on the surface is.
   The arm reaches a few targets, then picks the mirrored "R" of the headline, flips it and places it.
   The letter itself is a DOM span that follows the gripper. With prefers-reduced-motion the final frame is shown. */
(function () {
  var canvas = document.getElementById('hero-arm');
  var letter = document.getElementById('hero-R');
  var caption = document.getElementById('hero-caption');
  if (!canvas || !canvas.getContext || !letter || !caption) return;
  var ctx = canvas.getContext('2d');
  var reduce = window.matchMedia('(prefers-reduced-motion: reduce)').matches;
  var ATLAS = 'images/hero/atlas.webp', META = 'images/hero/hero.json';
  var meta = null, cells = null, NX = 0, NY = 0, COLS = 0;
  var W = 0, H = 0, pitch = 4, x0 = 0, y0 = 0, grain = null;
  var ink = '#161616', orange = '#ec7014', accent = '#3552f2', dark = false;
  var trail = [], TRAIL_LIFE = 6, FPS = 15;

  function readTheme() {
    dark = window.matchMedia('(prefers-color-scheme: dark)').matches && document.documentElement.getAttribute('data-theme') === 'dark';
    ink = dark ? '#e6e6e6' : '#161616';
  }
  function resize() {
    var r = canvas.getBoundingClientRect(), w = Math.round(r.width);
    if (!w) return false;
    W = w; H = Math.round(w * NY / NX);
    var dpr = Math.min(window.devicePixelRatio || 1, 2);
    canvas.width = W * dpr; canvas.height = H * dpr; canvas.style.height = H + 'px';
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    pitch = W / NX; x0 = 0; y0 = 0;
    if (!grain) { grain = new Float32Array(NX * NY); for (var i = 0; i < grain.length; i++) grain[i] = 0.9 + 0.2 * (((Math.sin(i * 12.9898) * 43758.5453) % 1 + 1) % 1); }
    placeCaption();
    return true;
  }
  // Put the headline so that the centre of its "R" sits on the gripper's place point.
  var placePx = [0, 0], pickPx = [0, 0];
  function placeCaption() {
    var p = meta.place, q = meta.pick;
    placePx = [p[0] * W, p[1] * H]; pickPx = [q[0] * W, q[1] * H];
    caption.style.transform = 'none'; letter.style.transform = 'none';
    var cr = canvas.getBoundingClientRect(), lr = letter.getBoundingClientRect(), capr = caption.getBoundingClientRect();
    var lcx = lr.left + lr.width / 2 - cr.left, lcy = lr.top + lr.height / 2 - cr.top;
    var dx = placePx[0] - lcx, dy = placePx[1] - lcy;
    caption.style.transform = 'translate(' + dx.toFixed(1) + 'px,' + dy.toFixed(1) + 'px)';
  }
  function cellAt(frame, i, j) {
    var c = frame % COLS, r = (frame / COLS) | 0;
    var idx = ((r * NY + j) * (COLS * NX) + c * NX + i) * 4;
    return idx;
  }
  function draw(frame, now) {
    var fr = meta.frames[frame];
    ctx.clearRect(0, 0, W, H);
    var rmax = pitch / 2 * 1.02, onArm = new Uint8Array(NX * NY);
    // arm in ink
    ctx.beginPath(); ctx.fillStyle = ink;
    var pathO = [];
    for (var j = 0; j < NY; j++) for (var i = 0; i < NX; i++) {
      var idx = cellAt(frame, i, j), m = cells[idx + 1];
      if (!m) continue;
      var lum = cells[idx] / 255, rim = cells[idx + 2] / 255;
      var c = dark ? 0.05 + 0.95 * Math.pow(lum, 1.6) : 0.05 + 0.95 * Math.pow(1 - lum, 1.7);
      c *= 0.35 + 0.65 * Math.sqrt(rim);
      var rad = rmax * Math.sqrt(c) * grain[j * NX + i];
      if (rad < 0.3) continue;
      onArm[j * NX + i] = 1;
      var cx = x0 + (i + 0.5) * pitch, cy = y0 + (j + 0.5) * pitch;
      if (m > 200) { pathO.push(cx, cy, Math.max(rad, rmax * 0.55)); continue; }
      ctx.moveTo(cx + rad, cy); ctx.arc(cx, cy, rad, 0, 6.2832);
    }
    ctx.fill();
    // finger pads in orange
    ctx.beginPath(); ctx.fillStyle = orange;
    for (var k = 0; k < pathO.length; k += 3) { ctx.moveTo(pathO[k] + pathO[k + 2], pathO[k + 1]); ctx.arc(pathO[k], pathO[k + 1], pathO[k + 2], 0, 6.2832); }
    ctx.fill();
    // the gripper's trace, snapped to the grid, in the accent colour
    ctx.beginPath(); ctx.fillStyle = accent;
    for (var t = 0; t < trail.length; t++) {
      var age = (now - trail[t][2]) / TRAIL_LIFE, v = 1 - age; if (v <= 0) continue;
      var ci = Math.floor(trail[t][0]), cj = Math.floor(trail[t][1]);
      if (ci < 0 || cj < 0 || ci >= NX || cj >= NY || onArm[cj * NX + ci]) continue;
      var rr = rmax * 0.6 * Math.sqrt(v); if (rr < 0.3) continue;
      var tx = x0 + (ci + 0.5) * pitch, ty = y0 + (cj + 0.5) * pitch;
      ctx.moveTo(tx + rr, ty); ctx.arc(tx, ty, rr, 0, 6.2832);
    }
    ctx.fill();
    // the letter: on the wall (mirrored) -> held by the gripper -> placed in the headline
    var tipPx = [fr.tip[0] * pitch, fr.tip[1] * pitch];
    if (fr.state === 'wall') {
      letter.style.transform = 'translate(' + (pickPx[0] - placePx[0]).toFixed(1) + 'px,' + (pickPx[1] - placePx[1]).toFixed(1) + 'px) rotate(180deg)';
    } else if (fr.state === 'held') {
      var ang = (fr.theta + Math.PI / 2) * 180 / Math.PI;   // place roll is -90 deg -> 0 deg on screen
      letter.style.transform = 'translate(' + (tipPx[0] - placePx[0]).toFixed(1) + 'px,' + (tipPx[1] - placePx[1]).toFixed(1) + 'px) rotate(' + ang.toFixed(1) + 'deg)';
    } else {
      letter.style.transform = 'none';
    }
  }
  function load() {
    var img = new Image();
    var p1 = fetch(META).then(function (r) { return r.json(); });
    var p2 = new Promise(function (res, rej) { img.onload = function () { res(img); }; img.onerror = rej; img.src = ATLAS; });
    return Promise.all([p1, p2]).then(function (v) {
      meta = v[0]; NX = meta.nx; NY = meta.ny; COLS = meta.cols; FPS = meta.fps;
      var off = document.createElement('canvas'); off.width = img.width; off.height = img.height;
      var octx = off.getContext('2d', { willReadFrequently: true }); octx.drawImage(img, 0, 0);
      cells = octx.getImageData(0, 0, img.width, img.height).data;
    });
  }
  function whenVisible(cb) { (function poll() { if (canvas.getBoundingClientRect().width > 0) cb(); else requestAnimationFrame(poll); })(); }
  load().then(function () { return new Promise(function (res) { whenVisible(res); }); }).then(function () {
    readTheme(); resize();
    var n = meta.frames.length, lastFrame = -1;
    if (reduce) { draw(n - 1, 0); canvas.classList.add('is-ready'); return; }
    // warm up the trace so the first paint already carries one
    var t0 = performance.now(), warm = 3.5;
    for (var s = 0; s < warm; s += 1 / FPS) { var f0 = Math.floor(s * FPS) % n; var tp = meta.frames[f0].tip; trail.push([tp[0], tp[1], s - warm]); }
    canvas.classList.add('is-ready');
    function frame(now) {
      requestAnimationFrame(frame);
      var t = (now - t0) / 1000, f = Math.floor(t * FPS) % n;
      if (f === lastFrame) return;
      lastFrame = f;
      var tp = meta.frames[f].tip, last = trail[trail.length - 1];
      if (!last || Math.hypot(tp[0] - last[0], tp[1] - last[1]) > 0.3) trail.push([tp[0], tp[1], t]);
      while (trail.length && t - trail[0][2] > TRAIL_LIFE) trail.shift();
      draw(f, t);
    }
    requestAnimationFrame(frame);
    window.addEventListener('resize', function () { if (resize()) draw(Math.max(lastFrame, 0), (performance.now() - t0) / 1000); });
    if (document.fonts && document.fonts.ready) document.fonts.ready.then(placeCaption);
  }).catch(function (e) { if (window.console) console.warn('hero arm failed to load', e); });
})();
