// ---------- sound: physical synthesizer stream (simemu-acoustics) ----------
// The server synthesizes 48 kHz pressure (Pa) in PLANT time from the physics and streams it over a WebSocket.
// The AudioWorklet below turns plant time into wall-clock audio with a jitter buffer (~70 ms) and one of two
// policies when the simulation is slower/faster than real time:
//   'stretch' — pitch-true time-scale modification (WSOLA, Verhelst & Roelands 1993): frequencies stay correct,
//               events are spread / compressed in time; 85 ms grains (2 periods of 23 Hz idle firing) → ~140 ms buffer;
//   'follow'  — follow plant time (varispeed): playback rate = simulation speed, so pitch scales with it; ~70 ms;
//   'auto'    — varispeed while the simulation runs within 3 % of real time (pitch error < 0.5 semitone), else WSOLA.
// Nothing here generates sound: it only plays, meters and analyses what the server sends.
Object.assign(T, {
  v_sound: ['Звук', 'Sound'],
  p_snd_ctl: ['Звук: прослушивание', 'Sound: listening'], p_snd_spec: ['Спектрограмма, дБ SPL', 'Spectrogram, dB SPL'],
  p_snd_mix: ['Микшер источников (уровни в точке прослушивания, дБ без коррекции)', 'Source mixer (levels at the listener, dB unweighted)'],
  p_snd_harn: ['Жгут проводки: отключить разъём', 'Wiring harness: unplug a connector'],
  snd_connect: ['▶ Слушать', '▶ Listen'], snd_disconnect: ['■ Стоп', '■ Stop'],
  snd_listener: ['Точка', 'Position'], snd_policy: ['Если симуляция не 1:1', 'If simulation is not 1:1'],
  snd_auto: ['авто: ≈1:1 — как есть, иначе растяжение', 'auto: ≈1:1 as is, else time-stretch'],
  snd_stretch: ['высота тона верна (растяжение, +~70 мс)', 'pitch-true (time-stretch, +~70 ms)'], snd_follow: ['время симуляции (высота плывёт)', 'plant time (pitch follows)'],
  snd_master: ['Громкость', 'Master'], snd_cyl: ['Цилиндры', 'Cylinders'], snd_target: ['Стетоскоп на', 'Stethoscope on'],
  snd_calib: ['0 дБFS =', '0 dBFS ='], snd_reset: ['Сброс микшера', 'Reset mixer'],
  snd_note: ['Звук рассчитывается сервером из физики plant (потоки через клапаны и турбину, давление в цилиндрах, детонация, удары клапанов/форсунок/ТНВД, стартер, генератор, вентилятор, сцепления, шестерни, насос) и проходит через модели труб, конструкции и пути до слушателя. Сэмплов и сценариев нет. Модель и непроверенные параметры — docs/ACOUSTICS.md.',
    'The server computes the sound from the plant physics (valve and turbine flows, cylinder pressure, knock, valve/injector/HPFP impacts, starter, alternator, fan, clutches, gears, pump) through duct, structure and listener-path models. No samples, no scripts. Model and unverified parameters: docs/ACOUSTICS.md.'],
  snd_coil: ['Катушка', 'Coil'], snd_inj: ['Форсунка', 'Injector'],
  snd_harn_note: ['Неисправность железа на стенде (как разъём, снятый с катушки/форсунки): прошивка продолжает управлять, а цилиндр не получает искру/топливо. Слушайте пропуск в выхлопе и опорах.',
    'A bench hardware fault (like a connector pulled off a coil/injector): the firmware keeps commanding, the cylinder gets no spark/fuel. Listen to the misfire in the exhaust and mounts.'],
  tip_sound: ['Звук двигателя и трансмиссии из физической модели (раздел «Звук»).', 'Engine and driveline sound from the physics model (Sound view).'],
});

const SND = {
  on: false, ws: null, ac: null, node: null, an: null, gain: null, meta: null, stat: null, info: null, raf: 0,
  cfg: Object.assign({ db: {}, mute: {}, solo: {}, cyl: [true, true, true, true], listener: 0, master: -20, policy: 'auto', target: 'exhaust', tcyl: -1 }, LS.get('snd', {})),
};
const SND_FS = 48000;
const SND_WORKLET = `
// mode 0 = follow plant time (varispeed), 1 = pitch-true WSOLA, 2 = auto (varispeed while the input rate is within
// 3 % of real time, WSOLA otherwise). WSOLA grains are 85 ms: two periods of the 23 Hz idle firing frequency, the
// lowest periodicity that must keep its pitch; that is why the stretch buffer is ~140 ms.
class Player extends AudioWorkletProcessor {
  constructor() {
    super();
    this.N = 1 << 18; this.m = this.N - 1; this.buf = new Float32Array(this.N);
    this.w = 0; this.r = 0; this.mode = 2; this.speed = 1; this.lastIn = -1; this.fade = 0;
    this.rate = 1; this.rx = 0; this.t0 = -1; this.ws = false;
    this.G = 4096; this.H = 2048; this.S = 1100; this.win = new Float32Array(this.G);
    for (let i = 0; i < this.G; i++) this.win[i] = 0.5 - 0.5 * Math.cos(2 * Math.PI * i / this.G);
    this.tail = new Float32Array(this.H); this.q = new Float32Array(this.H); this.qn = 0; this.qi = 0; this.prev = -1; this.n = 0;
    this.port.onmessage = e => {
      const d = e.data;
      if (d.pcm) {
        const x = d.pcm;
        for (let i = 0; i < x.length; i++) this.buf[(this.w + i) & this.m] = x[i];
        this.w += x.length; this.rx += x.length; this.lastIn = currentTime;
        if (this.w - this.r > 48000) { this.r = this.w - this.target(); this.prev = -1; this.qi = this.qn; }
      }
      if (d.mode !== undefined) this.mode = d.mode;
    };
  }
  target() { return this.ws ? this.G + this.S + 1500 : 3360; }
  at(i) { return this.buf[i & this.m]; }
  hop() {
    const H = this.H, S = this.S, a = Math.floor(this.r);
    let best = 0;
    if (this.prev >= 0) {
      const nat = this.prev + H; let bc = -Infinity;
      for (let d = -S; d <= S; d += 4) {
        let c = 0;
        for (let k = 0; k < H; k += 8) c += this.at(a + d + k) * this.at(nat + k);
        if (c > bc) { bc = c; best = d; }
      }
    }
    const s = a + best;
    for (let k = 0; k < H; k++) { this.q[k] = this.tail[k] + this.at(s + k) * this.win[k]; this.tail[k] = this.at(s + H + k) * this.win[H + k]; }
    this.prev = s; this.r += H * this.speed; this.qn = H; this.qi = 0;
  }
  process(_, outputs) {
    const out = outputs[0][0], n = out.length;
    if (this.t0 < 0) this.t0 = currentTime;
    if (currentTime - this.t0 >= 0.25) {
      const r = this.rx / ((currentTime - this.t0) * sampleRate);
      if (r > 0) this.rate += (r - this.rate) * 0.3;
      this.rx = 0; this.t0 = currentTime;
    }
    const ws = this.mode === 1 || (this.mode === 2 && Math.abs(this.rate - 1) > 0.03);
    if (ws !== this.ws) { this.ws = ws; this.prev = -1; this.qi = this.qn; this.tail.fill(0); }
    const fill = this.w - this.r;
    // Input rate (plant s per wall s) × buffer-level correction keeps the jitter buffer at its target.
    const want = this.rate * Math.min(2, Math.max(0.5, fill / this.target()));
    this.speed += (Math.min(4, Math.max(0.1, want)) - this.speed) * 0.02;
    const starving = this.lastIn < 0 || currentTime - this.lastIn > 0.3 || fill < (ws ? this.G + this.S + 200 : 8);
    if (starving) this.prev = -1;
    for (let i = 0; i < n; i++) {
      let y = 0;
      if (!starving) {
        if (ws) { if (this.qi >= this.qn) this.hop(); y = this.q[this.qi++]; }
        else { const i0 = Math.floor(this.r), f = this.r - i0; y = this.at(i0) * (1 - f) + this.at(i0 + 1) * f; this.r += this.speed; }
      }
      this.fade += ((starving ? 0 : 1) - this.fade) * 0.003;
      out[i] = y * this.fade;
    }
    for (let c = 1; c < outputs[0].length; c++) outputs[0][c].set(out);
    if (++this.n % 30 === 0) this.port.postMessage({ fill, speed: this.speed, starving, stretch: ws });
    return true;
  }
}
registerProcessor('simemu-player', Player);`;

function sndSave() { LS.set('snd', SND.cfg); }
function sndGains() {
  const src = (SND.info && SND.info.sources) || [];
  const c = SND.cfg;
  if (c.listener === 3) return src.map(s => (s.key === c.target ? 1 : 0));
  const anySolo = src.some(s => c.solo[s.key]);
  return src.map(s => ((anySolo ? c.solo[s.key] : !c.mute[s.key]) ? Math.pow(10, (c.db[s.key] || 0) / 20) : 0));
}
function sndSendMix() {
  const c = SND.cfg;
  const cyl = c.listener === 3 && c.tcyl >= 0 ? [0, 1, 2, 3].map(i => (i === c.tcyl ? 1 : 0)) : c.cyl.map(x => (x ? 1 : 0));
  if (SND.ws && SND.ws.readyState === 1) SND.ws.send(JSON.stringify({ gain: sndGains(), cyl, master: 1, listener: c.listener }));
  if (SND.gain) SND.gain.gain.setTargetAtTime(Math.pow(10, c.master / 20), SND.ac.currentTime, 0.02);
  if (SND.node) SND.node.port.postMessage({ mode: c.policy === 'stretch' ? 1 : c.policy === 'follow' ? 0 : 2 });
  sndSave();
}
async function sndStart() {
  if (!SND.info) SND.info = await api('/api/audio/info');
  if (!SND.ac) {
    try { SND.ac = new AudioContext({ sampleRate: SND_FS, latencyHint: 'interactive' }); } catch (e) { SND.ac = new AudioContext(); }
    if (SND.ac.sampleRate !== SND_FS) say(L(['Браузер не дал 48 кГц: высота тона будет смещена', 'Browser refused 48 kHz: pitch will be off']), true);
    await SND.ac.audioWorklet.addModule(URL.createObjectURL(new Blob([SND_WORKLET], { type: 'application/javascript' })));
    const ac = SND.ac;
    SND.node = new AudioWorkletNode(ac, 'simemu-player', { outputChannelCount: [2] });
    SND.gain = ac.createGain();
    const lim = ac.createDynamicsCompressor(); // protection limiter for the speakers (not part of the model)
    lim.threshold.value = -3; lim.knee.value = 0; lim.ratio.value = 20; lim.attack.value = 0.002; lim.release.value = 0.1;
    SND.an = ac.createAnalyser(); SND.an.fftSize = 8192; SND.an.smoothingTimeConstant = 0;
    SND.node.connect(SND.an);
    SND.node.connect(SND.gain).connect(lim).connect(ac.destination);
    SND.node.port.onmessage = e => { SND.stat = e.data; };
  }
  await SND.ac.resume();
  const ws = new WebSocket(`ws://${location.host}/api/audio/ws`);
  ws.binaryType = 'arraybuffer';
  ws.onopen = () => sndSendMix();
  ws.onmessage = e => {
    if (typeof e.data === 'string') { try { SND.meta = JSON.parse(e.data); } catch (x) { /* ignore */ } return; }
    const x = new Float32Array(e.data);
    SND.node.port.postMessage({ pcm: x }, [x.buffer]);
  };
  ws.onclose = () => { if (SND.ws === ws) { SND.ws = null; SND.on = false; renderTopButtons(); sndRenderCtl(); } };
  SND.ws = ws; SND.on = true;
  renderTopButtons(); sndRenderCtl();
  sndSpecLoop();
}
function sndStop() {
  SND.on = false;
  if (SND.ws) SND.ws.close();
  SND.ws = null;
  if (SND.ac) SND.ac.suspend();
}
function toggleSound() {
  if (SND.on) sndStop(); else sndStart().catch(e => say(e.message, true));
  renderTopButtons(); sndRenderCtl();
}

function sndRenderCtl() {
  const el = $('snd-ctl');
  if (!el) return;
  const c = SND.cfg, info = SND.info;
  const lis = info ? info.listeners.map((l, i) => `<option value="${i}"${i === c.listener ? ' selected' : ''}>${esc(L([l.ru, l.en]))}</option>`).join('') : '';
  const tgt = info ? info.sources.map(s => `<option value="${esc(s.key)}"${s.key === c.target ? ' selected' : ''}>${esc(L([s.ru, s.en]))}</option>`).join('') : '';
  el.innerHTML = `<div class="row"><button id="snd-go" class="${SND.on ? 'on' : 'primary'}">${esc(t(SND.on ? 'snd_disconnect' : 'snd_connect'))}</button>
      <label>${esc(t('snd_listener'))}</label><select id="snd-lis">${lis}</select></div>
    <div class="row" id="snd-steth"${c.listener === 3 ? '' : ' style="display:none"'}><label>${esc(t('snd_target'))}</label><select id="snd-tgt">${tgt}</select>
      <select id="snd-tcyl"><option value="-1">${esc(t('all'))}</option>${[0, 1, 2, 3].map(i => `<option value="${i}"${i === c.tcyl ? ' selected' : ''}>${esc(L(['цил. ', 'cyl ']))}${i + 1}</option>`).join('')}</select></div>
    <div class="row"><label>${esc(t('snd_cyl'))}</label>${c.cyl.map((x, i) => `<label><input type="checkbox" data-cyl="${i}"${x ? ' checked' : ''}> ${i + 1}</label>`).join('')}</div>
    <div class="slider"><label>${esc(t('snd_master'))}</label><input id="snd-master" type="range" min="-60" max="20" step="1" value="${c.master}"><b id="snd-master-v">${c.master} dB</b></div>
    <div class="row"><label>${esc(t('snd_policy'))}</label><select id="snd-pol"><option value="auto"${c.policy === 'auto' ? ' selected' : ''}>${esc(t('snd_auto'))}</option><option value="stretch"${c.policy === 'stretch' ? ' selected' : ''}>${esc(t('snd_stretch'))}</option><option value="follow"${c.policy === 'follow' ? ' selected' : ''}>${esc(t('snd_follow'))}</option></select></div>
    <div class="row"><span class="muted">${esc(t('snd_calib'))} ${(94 - c.master).toFixed(0)} dB SPL</span><button id="snd-reset">${esc(t('snd_reset'))}</button></div>
    <div id="snd-stat" class="kv"></div>
    <div class="note">${esc(t('snd_note'))}</div>`;
  $('snd-go').onclick = toggleSound;
  $('snd-lis').onchange = e => { c.listener = +e.target.value; sndSendMix(); sndRenderCtl(); sndBuildMix(); };
  if ($('snd-tgt')) $('snd-tgt').onchange = e => { c.target = e.target.value; sndSendMix(); };
  if ($('snd-tcyl')) $('snd-tcyl').onchange = e => { c.tcyl = +e.target.value; sndSendMix(); };
  el.querySelectorAll('[data-cyl]').forEach(b => { b.onchange = () => { c.cyl[+b.dataset.cyl] = b.checked; sndSendMix(); }; });
  $('snd-master').oninput = e => { c.master = +e.target.value; $('snd-master-v').textContent = `${c.master} dB`; sndSendMix(); };
  $('snd-pol').onchange = e => { c.policy = e.target.value; sndSendMix(); };
  $('snd-reset').onclick = () => { Object.assign(c, { db: {}, mute: {}, solo: {}, cyl: [true, true, true, true] }); sndSendMix(); sndRenderCtl(); sndBuildMix(); };
}

function sndBuildMix() {
  const el = $('snd-mix');
  if (!el || !SND.info) return;
  const c = SND.cfg;
  el.innerHTML = `<table class="mixer"><tbody>${SND.info.sources.map(s => `<tr data-src="${esc(s.key)}">
      <td style="white-space:nowrap">${esc(L([s.ru, s.en]))}${s.cyl ? ' <span class="muted">[цил]</span>' : ''}</td>
      <td><button data-m="mute" class="${c.mute[s.key] ? 'on' : ''}">M</button> <button data-m="solo" class="${c.solo[s.key] ? 'on' : ''}">S</button></td>
      <td><input type="range" min="-40" max="20" step="1" value="${c.db[s.key] || 0}" data-m="db"> <span class="mono" data-v="db">${c.db[s.key] || 0} dB</span></td>
      <td style="width:45%"><div class="sndbar"><div data-v="bar"></div></div></td><td class="mono" data-v="lv" style="width:60px">—</td></tr>`).join('')}</tbody></table>`;
  el.onclick = e => {
    const b = e.target.closest('button[data-m]'); if (!b) return;
    const k = b.closest('tr').dataset.src, m = b.dataset.m;
    c[m][k] = !c[m][k]; b.classList.toggle('on', !!c[m][k]); sndSendMix();
  };
  el.oninput = e => {
    if (e.target.dataset.m !== 'db') return;
    const tr = e.target.closest('tr'), k = tr.dataset.src;
    c.db[k] = +e.target.value; tr.querySelector('[data-v="db"]').textContent = `${c.db[k]} dB`; sndSendMix();
  };
}

function sndMeters() {
  const el = $('snd-mix'), m = SND.meta;
  if (el && m && m.lv && SND.info) {
    SND.info.sources.forEach((s, i) => {
      const tr = el.querySelector(`tr[data-src="${s.key}"]`); if (!tr) return;
      const db = m.lv[i];
      tr.querySelector('[data-v="bar"]').style.width = `${clamp((db - 20) / 100, 0, 1) * 100}%`;
      tr.querySelector('[data-v="lv"]').textContent = db > -90 ? db.toFixed(0) : DASH;
    });
  }
  const st = $('snd-stat');
  if (st) {
    const s = SND.stat || {}, mm = SND.meta || {};
    const kv = [[L(['буфер', 'buffer']), isNum(s.fill) ? `${(s.fill / 48).toFixed(0)} ms` : DASH],
      [L(['скорость воспроизведения', 'playback rate']), isNum(s.speed) ? `${s.speed.toFixed(2)}×${s.stretch ? L([' (растяжение)', ' (stretch)']) : ''}` : DASH],
      [L(['задержка (оценка)', 'latency (est.)']), isNum(s.fill) && SND.ac ? `${(s.fill / 48 + 1000 * (SND.ac.baseLatency || 0) + 1000 * (SND.ac.outputLatency || 0)).toFixed(0)} ms` : DASH],
      [L(['нагрузка синтезатора', 'synthesizer load']), isNum(mm.load) ? `${(100 * mm.load).toFixed(1)} % CPU` : DASH],
      [L(['потеряно записей', 'dropped records']), isNum(mm.dropped) ? String(mm.dropped) : DASH],
      [L(['время plant', 'plant time']), isNum(mm.sim_t) ? `${mm.sim_t.toFixed(2)} s` : DASH]];
    st.innerHTML = kv.map(([a, b]) => `<div><span>${esc(a)}</span><b>${esc(b)}</b></div>`).join('') + (s.starving && SND.on ? `<div class="w"><span class="warn">${esc(L(['нет данных (пауза / симуляция стоит)', 'no data (paused / simulation stalled)']))}</span></div>` : '');
  }
}

// Scrolling log-frequency spectrogram (analyser on the calibrated Pa signal: dB SPL = dBFS + 94) with order lines.
function sndSpecLoop() {
  cancelAnimationFrame(SND.raf);
  const cv = $('snd-spec');
  const frame = () => {
    if (!SND.on || !SND.an || CUR_VIEW !== 'sound') { SND.raf = 0; return; }
    SND.raf = requestAnimationFrame(frame);
    sndMeters();
    if (!cv) return;
    const W = cv.clientWidth, H = cv.clientHeight;
    if (cv.width !== W || cv.height !== H) { cv.width = W; cv.height = H; }
    const g = cv.getContext('2d'), an = SND.an, n = an.frequencyBinCount, d = SND.spec || (SND.spec = new Float32Array(n));
    an.getFloatFrequencyData(d);
    const plot = W - 70, step = 2;
    g.drawImage(cv, step, 0, plot - step, H, 0, 0, plot - step, H);
    const fmin = 20, fmax = 20000, bin = SND.ac.sampleRate / an.fftSize;
    for (let y = 0; y < H; y++) {
      const f = fmin * Math.pow(fmax / fmin, 1 - y / H);
      const v = clamp((d[Math.min(n - 1, Math.round(f / bin))] + 94 - 10) / 100, 0, 1);
      g.fillStyle = `hsl(${240 - 240 * v},90%,${8 + 52 * v}%)`;
      g.fillRect(plot - step, y, step, 1);
    }
    g.fillStyle = '#0e1420'; g.fillRect(plot, 0, 70, H);
    const yf = f => H * (1 - Math.log(f / fmin) / Math.log(fmax / fmin));
    g.font = '10px monospace';
    for (const f of [50, 100, 200, 500, 1000, 2000, 5000, 10000]) { g.fillStyle = '#66768f'; g.fillText(f >= 1000 ? `${f / 1000}k` : String(f), plot + 4, yf(f) + 3); }
    const p = (S && S.plant) || {};
    const lines = [];
    if (isNum(p.rpm) && p.rpm > 60) { lines.push([p.rpm / 60 * 2, L(['2-й', 'ord 2']), '#ffd166']); lines.push([p.rpm / 60 * 4, L(['4-й', 'ord 4']), '#f5a623']); lines.push([p.rpm / 60 * 21, L(['цепь', 'chain']), '#8d8cff']); }
    if (isNum(p.turbo_rpm) && p.turbo_rpm > 3000) lines.push([p.turbo_rpm / 60 * 6, 'BPF', '#2ecc71']);
    for (const [f, lab, col] of lines) {
      if (f < fmin || f > fmax) continue;
      const y = yf(f);
      g.strokeStyle = col; g.setLineDash([3, 4]); g.beginPath(); g.moveTo(plot - 40, y); g.lineTo(plot, y); g.stroke(); g.setLineDash([]);
      g.fillStyle = col; g.fillText(lab, plot + 30, y + 3);
    }
  };
  SND.raf = requestAnimationFrame(frame);
}

async function sndHarness(change) {
  try {
    const h = await api('/api/audio/harness', Object.assign(SND.harness || { coil_open: [false, false, false, false], injector_open: [false, false, false, false] }, change || {}));
    SND.harness = h;
  } catch (e) { say(e.message, true); }
  const el = $('snd-harn');
  if (!el || !SND.harness) return;
  const h = SND.harness;
  el.innerHTML = `<div class="row">${[0, 1, 2, 3].map(i => `<label><input type="checkbox" data-h="coil_open" data-i="${i}"${h.coil_open[i] ? ' checked' : ''}> ${esc(t('snd_coil'))} ${i + 1}</label>`).join('')}</div>
    <div class="row">${[0, 1, 2, 3].map(i => `<label><input type="checkbox" data-h="injector_open" data-i="${i}"${h.injector_open[i] ? ' checked' : ''}> ${esc(t('snd_inj'))} ${i + 1}</label>`).join('')}</div>
    <div class="note">${esc(t('snd_harn_note'))}</div>`;
  el.onchange = e => {
    const b = e.target.closest('[data-h]'); if (!b) return;
    const arr = h[b.dataset.h].slice(); arr[+b.dataset.i] = b.checked;
    sndHarness({ [b.dataset.h]: arr });
  };
}

view({
  id: 'sound',
  async init() {
    try { SND.info = await api('/api/audio/info'); } catch (e) { /* server without audio */ }
    sndRenderCtl(); sndBuildMix();
    try { SND.harness = await api('/api/audio/harness'); } catch (e) { /* ignore */ }
    sndHarness();
  },
  show() { sndRenderCtl(); sndBuildMix(); if (SND.on) sndSpecLoop(); },
  i18n() { sndRenderCtl(); sndBuildMix(); sndHarness(); },
  render() { if (!SND.on) sndMeters(); },
});
