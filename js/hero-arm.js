/* Hero: a Franka FR3 with a UMI gripper, drawn as a fine halftone from a pre-rendered video.
   The video frames were ray-cast from the real FR3 meshes (MuJoCo) with their materials; pixels are coded as
   grey = arm (luminance), orange = finger pads, green = base LED ring, white = nothing. The browser decodes
   each frame and draws one dot per cell, so the look follows the page's ink colour.
   The letter "R" of the headline is a DOM element: it starts upside down on the wall, the gripper carries it into
   the headline, it falls under gravity (simulated here), lands on its side, and the arm puts it back on the wall. */
(function () {
  var canvas = document.getElementById('hero-arm');
  var video = document.getElementById('hero-video');
  var letter = document.getElementById('hero-R');
  var caption = document.getElementById('hero-caption');
  if (!canvas || !canvas.getContext || !video || !letter || !caption) return;
  var ctx = canvas.getContext('2d');
  var reduce = window.matchMedia('(prefers-reduced-motion: reduce)').matches;
  var META = 'images/hero/hero.json';
  var meta = null, NX = 0, NY = 0, FPS = 15, N = 0;
  var W = 0, H = 0, pitch = 2.4;
  var ink = '#161616', orange = '#ec7014', green = '#22c55e', accent = '#3552f2';
  var off = document.createElement('canvas'), octx = off.getContext('2d', { willReadFrequently: true });
  var trail = [], TRAIL_LIFE = 5;
  var placePx = [0, 0], pickPx = [0, 0], floorPx = [0, 0], halfPx = 0, halfWPx = 0, letterW = 0, letterH = 0;

  function resize() {
    var r = canvas.getBoundingClientRect(), w = Math.round(r.width);
    if (!w) return false;
    W = w; H = Math.round(w * NY / NX);
    var dpr = Math.min(window.devicePixelRatio || 1, 2);
    canvas.width = W * dpr; canvas.height = H * dpr; canvas.style.height = H + 'px';
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    pitch = W / NX;
    placeCaption();
    return true;
  }
  // Put the headline so that the centre of its "R" sits on the place point.
  function placeCaption() {
    placePx = [meta.place[0] * W, meta.place[1] * H]; pickPx = [meta.pick[0] * W, meta.pick[1] * H]; floorPx = [meta.floor[0] * W, meta.floor[1] * H];
    halfPx = meta.letter_half_cells * pitch; halfWPx = meta.letter_half_w_cells * pitch;
    caption.style.transform = 'none'; letter.style.transform = 'none';
    var cr = canvas.getBoundingClientRect(), lr = letter.getBoundingClientRect();
    letterW = lr.width; letterH = lr.height;
    var dx = placePx[0] - (lr.left + lr.width / 2 - cr.left), dy = placePx[1] - (lr.top + lr.height / 2 - cr.top);
    caption.style.transform = 'translate(' + dx.toFixed(1) + 'px,' + dy.toFixed(1) + 'px)';
  }

  // ---- the letter's fall: a planar rigid body with gravity, a floor, restitution and settling ----
  var fall = null;
  function startFall() {
    var g = 9.81 * H;                                  // the canvas is one metre tall in the world
    fall = { x: placePx[0], y: placePx[1], a: 0, vx: (floorPx[0] - placePx[0]) * 3.2, vy: 0, w: 5.2, g: g, t: 0, bounces: 0, done: false };
  }
  function stepFall(dt) {
    var f = fall; if (!f || f.done) return;
    var floorY = floorPx[1] + halfWPx;                 // the floor is where the lying letter rests
    var sub = 4, h = dt / sub;
    for (var s = 0; s < sub; s++) {
      f.vy += f.g * h; f.y += f.vy * h; f.x += f.vx * h; f.a += f.w * h; f.t += h;
      var ca = Math.abs(Math.cos(f.a)), sa = Math.abs(Math.sin(f.a));
      var halfDown = 0.5 * (letterH * ca + letterW * sa);  // lowest corner of the rotated box
      if (f.y + halfDown > floorY) {
        f.y = floorY - halfDown;
        if (f.vy > 0) {
          f.bounces++;
          f.vy = -f.vy * (f.bounces === 1 ? 0.32 : 0.18);
          f.vx *= 0.55;
          var err = Math.PI / 2 - f.a;                 // the impact kicks the rotation toward lying flat
          f.w = f.w * 0.35 + err * 6.0;
          if (Math.abs(f.vy) < 0.06 * H) f.vy = 0;
        }
      }
      if (f.bounces >= 1) {                            // on the floor: settle the angle to +90 deg, friction on x
        var e2 = Math.PI / 2 - f.a; f.w += (e2 * 60 - f.w * 7) * h; f.vx *= Math.pow(0.02, h);
      }
      if (f.bounces >= 2 && Math.abs(f.vy) < 0.05 * H && Math.abs(f.w) < 0.05 && Math.abs(Math.PI / 2 - f.a) < 0.01) {
        f.a = Math.PI / 2; f.w = 0; f.vy = 0; f.vx = 0; f.x = floorPx[0]; f.y = floorPx[1]; f.done = true; break;
      }
      if (f.t > 2.5) { f.a = Math.PI / 2; f.x = floorPx[0]; f.y = floorPx[1]; f.done = true; break; }
    }
  }

  // ---- drawing ----
  function decodeAndDraw(frame, now) {
    octx.drawImage(video, 0, 0, NX, NY);
    var px = octx.getImageData(0, 0, NX, NY).data;
    ctx.clearRect(0, 0, W, H);
    var rmax = pitch / 2 * 1.06, onArm = new Uint8Array(NX * NY);
    var pads = [], leds = [];
    ctx.beginPath(); ctx.fillStyle = ink;
    for (var j = 0; j < NY; j++) for (var i = 0; i < NX; i++) {
      var k = (j * NX + i) * 4, r = px[k], g = px[k + 1], b = px[k + 2];
      if (r > 236 && g > 236 && b > 236) continue;
      var cx = (i + 0.5) * pitch, cy = (j + 0.5) * pitch;
      onArm[j * NX + i] = 1;
      if (g > r + 70 && g > b + 70) { leds.push(cx, cy); continue; }
      if (r > g + 50 && r > b + 90) { pads.push(cx, cy, rmax * (0.6 + 0.4 * (r / 255))); continue; }
      var lum = (r + g + b) / 765;
      var c = 0.04 + 0.96 * Math.pow(1 - lum, 1.35);
      var rad = rmax * Math.sqrt(c);
      if (rad < 0.22) continue;
      ctx.moveTo(cx + rad, cy); ctx.arc(cx, cy, rad, 0, 6.2832);
    }
    ctx.fill();
    ctx.beginPath(); ctx.fillStyle = orange;
    for (var p = 0; p < pads.length; p += 3) { ctx.moveTo(pads[p] + pads[p + 2], pads[p + 1]); ctx.arc(pads[p], pads[p + 1], pads[p + 2], 0, 6.2832); }
    ctx.fill();
    var breath = 0.5 + 0.5 * Math.sin(now * 2 * Math.PI / 2.6);
    ctx.beginPath(); ctx.fillStyle = green; ctx.globalAlpha = 0.45 + 0.55 * breath;
    var rg = rmax * (0.6 + 0.4 * breath);
    for (var q = 0; q < leds.length; q += 2) { ctx.moveTo(leds[q] + rg, leds[q + 1]); ctx.arc(leds[q], leds[q + 1], rg, 0, 6.2832); }
    ctx.fill(); ctx.globalAlpha = 1;
    ctx.beginPath(); ctx.fillStyle = accent;        // the gripper's trace
    for (var t = 0; t < trail.length; t++) {
      var v = 1 - (now - trail[t][2]) / TRAIL_LIFE; if (v <= 0) continue;
      var ci = Math.floor(trail[t][0]), cj = Math.floor(trail[t][1]);
      if (ci < 0 || cj < 0 || ci >= NX || cj >= NY || onArm[cj * NX + ci]) continue;
      var rr = rmax * 0.7 * Math.sqrt(v); if (rr < 0.25) continue;
      ctx.moveTo((ci + 0.5) * pitch + rr, (cj + 0.5) * pitch); ctx.arc((ci + 0.5) * pitch, (cj + 0.5) * pitch, rr, 0, 6.2832);
    }
    ctx.fill();
    placeLetter(frame);
  }
  // letter centre = gripper tip + half-extent along the approach (screen direction (sin phi, cos phi));
  // its angle follows the gripper from the angle it had when grasped.
  var graspPhi = { held: -Math.PI / 2, held2: 0 }, graspAngle = { held: 180, held2: 90 };
  function placeLetter(frame) {
    var fr = meta.frames[frame], tf, dX = placePx[0], dY = placePx[1];
    if (fr.state === 'wall') {
      tf = 'translate(' + (pickPx[0] - dX).toFixed(1) + 'px,' + (pickPx[1] - dY).toFixed(1) + 'px) rotate(180deg)';
    } else if (fr.state === 'held' || fr.state === 'held2') {
      var half = fr.state === 'held' ? halfPx : halfWPx;
      var cx = fr.tip[0] * pitch + half * Math.sin(fr.phi), cy = fr.tip[1] * pitch + half * Math.cos(fr.phi);
      var ang = graspAngle[fr.state] - (fr.phi - graspPhi[fr.state]) * 180 / Math.PI;
      tf = 'translate(' + (cx - dX).toFixed(1) + 'px,' + (cy - dY).toFixed(1) + 'px) rotate(' + ang.toFixed(1) + 'deg)';
    } else if (fr.state === 'falling') {
      if (!fall) startFall();
      tf = 'translate(' + (fall.x - dX).toFixed(1) + 'px,' + (fall.y - dY).toFixed(1) + 'px) rotate(' + (fall.a * 180 / Math.PI).toFixed(1) + 'deg)';
    } else { tf = 'none'; }
    letter.style.transform = tf;
  }

  fetch(META).then(function (r) { return r.json(); }).then(function (m) {
    meta = m; NX = m.nx; NY = m.ny; FPS = m.fps; N = m.frames.length;
    graspAngle.held2 = m.floor_angle;
    off.width = NX; off.height = NY;
    (function whenVisible() { if (canvas.getBoundingClientRect().width > 0) begin(); else requestAnimationFrame(whenVisible); })();
  }).catch(function (e) { if (window.console) console.warn('hero failed to load', e); });

  function begin() {
    resize();
    var fallStart = meta.frames.findIndex(function (f) { return f.state === 'falling'; });
    var placedFrame = Math.max(0, fallStart - 1);
    var lastFrame = -1, lastT = 0, t0 = performance.now();
    function paint(frame, now) {
      var tp = meta.frames[frame].tip, last = trail[trail.length - 1];
      if (!last || Math.hypot(tp[0] - last[0], tp[1] - last[1]) > 0.5) trail.push([tp[0], tp[1], now]);
      while (trail.length && now - trail[0][2] > TRAIL_LIFE) trail.shift();
      decodeAndDraw(frame, now);
    }
    if (reduce) {
      video.addEventListener('seeked', function once() { video.removeEventListener('seeked', once); paint(placedFrame, 0); canvas.classList.add('is-ready'); });
      video.currentTime = placedFrame / FPS;
      return;
    }
    var playing = video.play();
    if (playing && playing.catch) playing.catch(function () { /* autoplay blocked: nothing to decode */ });
    function frameLoop(now) {
      requestAnimationFrame(frameLoop);
      if (video.readyState < 2) return;
      var f = Math.min(N - 1, Math.floor(video.currentTime * FPS + 1e-3)), nowS = (now - t0) / 1000;
      if (f !== lastFrame) {
        if (f < lastFrame) { fall = null; trail = []; }            // looped
        if (meta.frames[f].state !== 'falling') fall = null;
        lastFrame = f;
      }
      if (fall && !fall.done) stepFall(Math.min(0.05, nowS - lastT));
      lastT = nowS;
      paint(f, nowS);
      if (!canvas.classList.contains('is-ready')) canvas.classList.add('is-ready');
    }
    requestAnimationFrame(frameLoop);
    window.addEventListener('resize', function () { if (resize() && lastFrame >= 0) paint(lastFrame, (performance.now() - t0) / 1000); });
    if (document.fonts && document.fonts.ready) document.fonts.ready.then(placeCaption);
  }
})();
