// ---------- shared chart drawing ----------
function fitCanvas(cv) {
  const w = cv.clientWidth, h = cv.clientHeight, dpr = window.devicePixelRatio || 1;
  if (!w || !h) return null;
  if (cv.width !== Math.round(w * dpr) || cv.height !== Math.round(h * dpr)) { cv.width = Math.round(w * dpr); cv.height = Math.round(h * dpr); }
  const c = cv.getContext('2d');
  c.setTransform(dpr, 0, 0, dpr, 0, 0);
  c.clearRect(0, 0, w, h);
  c.font = '10.5px ' + getComputedStyle(document.body).getPropertyValue('--mono');
  return { c, w, h };
}
function niceTicks(a, b, n) {
  const span = b - a;
  if (!(span > 0)) return [a];
  const raw = span / n, mag = Math.pow(10, Math.floor(Math.log10(raw)));
  const step = [1, 2, 5, 10].map(m => m * mag).find(s => span / s <= n) || 10 * mag;
  const out = [];
  for (let x = Math.ceil(a / step) * step; x <= b + 1e-9; x += step) out.push(+x.toPrecision(12));
  return out;
}
// Index of the last sample with ts[i] <= t (ts ascending), -1 if none.
function bsearch(ts, n, t) {
  let lo = 0, hi = n - 1, r = -1;
  while (lo <= hi) { const m = (lo + hi) >> 1; if (ts[m] <= t) { r = m; lo = m + 1; } else hi = m - 1; }
  return r;
}
const valueAt = (sr, t) => { const i = bsearch(sr.ts, sr.n, t - (sr.off || 0)); return i >= 0 ? sr.vs[i] : NaN; };
// Polyline with gaps on NaN and per-pixel min/max decimation for long series.
function drawSeries(c, sr, sx, sy, x0, x1) {
  const n = sr.n, off = sr.off || 0;
  let i = Math.max(0, bsearch(sr.ts, n, x0 - off) - 1);
  const iEnd = Math.min(n - 1, bsearch(sr.ts, n, x1 - off) + 1);
  c.strokeStyle = sr.color; c.lineWidth = sr.dashed ? 1.2 : 1.6;
  c.setLineDash(sr.dashed ? [5, 4] : []);
  c.beginPath();
  let pen = false, col = null, cmin = 0, cmax = 0, clast = 0;
  const flush = () => { if (col === null) return; c.lineTo(col, sy(cmin)); c.lineTo(col, sy(cmax)); c.lineTo(col, sy(clast)); };
  for (; i <= iEnd; i++) {
    const v = sr.vs[i];
    if (!isNum(v)) { flush(); col = null; pen = false; continue; }
    const x = Math.round(sx(sr.ts[i] + off));
    if (!pen) { c.moveTo(x, sy(v)); pen = true; col = x; cmin = cmax = clast = v; continue; }
    if (x === col) { cmin = Math.min(cmin, v); cmax = Math.max(cmax, v); clast = v; continue; }
    flush();
    c.lineTo(x, sy(v)); col = x; cmin = cmax = clast = v;
  }
  flush();
  c.stroke();
  c.setLineDash([]);
}
// Stacked lanes sharing one time axis. lanes: [{key, label, unit, color, series:[{ts,vs,n,off,color,dashed}], lo, hi}]
function laneChart(cv, o) {
  const g = fitCanvas(cv);
  if (!g) return;
  const { c, w, h } = g;
  const l = 6, r = 52, top = 2, bot = 18, pw = w - l - r;
  const x0 = o.x0, x1 = o.x1 > o.x0 ? o.x1 : o.x0 + 1;
  const sx = t => l + ((t - x0) / (x1 - x0)) * pw;
  cv._geo = { l, pw, x0, x1 };
  const n = o.lanes.length;
  c.fillStyle = '#66768f';
  if (!n) { c.fillText(o.empty || DASH, l + 10, 20); return; }
  const lh = (h - top - bot) / n;
  for (const tk of niceTicks(x0, x1, Math.max(2, Math.floor(pw / 110)))) {
    const x = sx(tk);
    c.strokeStyle = '#16202f'; c.beginPath(); c.moveTo(x, top); c.lineTo(x, h - bot); c.stroke();
    c.fillStyle = '#66768f'; c.textAlign = 'center'; c.fillText(fmtN(tk) + ' s', x, h - 5);
  }
  o.lanes.forEach((ln, li) => {
    const y0 = top + li * lh, y1 = y0 + lh;
    let lo = Infinity, hi = -Infinity;
    for (const sr of ln.series) {
      const off = sr.off || 0;
      const a = Math.max(0, bsearch(sr.ts, sr.n, x0 - off)), b = Math.min(sr.n - 1, bsearch(sr.ts, sr.n, x1 - off) + 1);
      for (let i = a; i <= b; i++) { const v = sr.vs[i]; if (isNum(v)) { if (v < lo) lo = v; if (v > hi) hi = v; } }
    }
    if (!Number.isFinite(lo)) { lo = 0; hi = 1; }
    // Minimum visible span = 1 % of the channel's typical range, so numerical noise on a steady value stays flat.
    const mr = ln.key && meta(ln.key).r, minSpan = mr ? (mr[1] - mr[0]) * 0.01 : 0;
    if (hi - lo < minSpan) { const c0 = (hi + lo) / 2; lo = c0 - minSpan / 2; hi = c0 + minSpan / 2; }
    if (hi - lo < 1e-9) { const p = Math.abs(hi) * 0.05 || 0.5; lo -= p; hi += p; }
    const pad = (hi - lo) * 0.08; lo -= pad; hi += pad;
    const sy = v => y1 - 3 - ((v - lo) / (hi - lo)) * (lh - 16);
    c.strokeStyle = '#22304a'; c.beginPath(); c.moveTo(l, y1 - 0.5); c.lineTo(l + pw, y1 - 0.5); c.stroke();
    c.textAlign = 'left'; c.fillStyle = '#66768f';
    c.fillText(fmtN(hi), l + pw + 4, y0 + 14); c.fillText(fmtN(lo), l + pw + 4, y1 - 4);
    c.save(); c.beginPath(); c.rect(l, y0, pw, lh); c.clip();
    for (const [lim, col] of [[ln.lo, '#ff4d4f'], [ln.hi, '#ff4d4f']]) {
      if (!isNum(lim) || lim < lo || lim > hi) continue;
      c.strokeStyle = col; c.setLineDash([2, 3]); c.beginPath(); c.moveTo(l, sy(lim)); c.lineTo(l + pw, sy(lim)); c.stroke(); c.setLineDash([]);
    }
    for (const sr of ln.series) drawSeries(c, sr, sx, sy, x0, x1);
    c.restore();
    const at = o.cursor != null ? o.cursor : x1;
    const v = ln.series.length ? valueAt(ln.series[0], at) : NaN;
    const txt = `${ln.label}  ${isNum(v) ? fmtN(v) : DASH} ${ln.unit || ''}`;
    c.font = '600 11px sans-serif';
    c.fillStyle = '#080b12cc'; c.fillRect(l + 2, y0 + 2, c.measureText(txt).width + 8, 15);
    c.fillStyle = ln.color; c.fillText(txt, l + 6, y0 + 13);
    c.font = '10.5px monospace';
  });
  for (const ev of o.events || []) {
    if (ev.t < x0 || ev.t > x1) continue;
    const x = sx(ev.t);
    c.strokeStyle = '#8d8cff'; c.setLineDash([4, 3]); c.beginPath(); c.moveTo(x, top); c.lineTo(x, h - bot); c.stroke(); c.setLineDash([]);
    c.fillStyle = '#8d8cff'; c.textAlign = 'left'; c.fillText('⚑ ' + ev.label, x + 3, h - bot - 4);
  }
  if (o.cursor != null && o.cursor >= x0 && o.cursor <= x1) {
    const x = sx(o.cursor);
    c.strokeStyle = '#ffffffaa'; c.beginPath(); c.moveTo(x, top); c.lineTo(x, h - bot); c.stroke();
    c.fillStyle = '#fff'; c.textAlign = 'center'; c.fillText(fmtN(o.cursor) + ' s', x, h - 5);
  }
}
// Zoom (wheel), pan (drag), cursor (hover), live/full (double-click). st: {span, end, cursor}
function chartNav(cv, st, redraw, onClick) {
  let drag = null;
  cv.addEventListener('wheel', e => {
    const g = cv._geo; if (!g) return;
    e.preventDefault();
    const fx = clamp((e.offsetX - g.l) / g.pw, 0, 1), at = g.x0 + fx * (g.x1 - g.x0);
    const span = (g.x1 - g.x0) * (e.deltaY > 0 ? 1.25 : 0.8);
    st.span = Math.max(0.05, span); st.end = at + (1 - fx) * st.span;
    redraw();
  }, { passive: false });
  cv.addEventListener('mousedown', e => { const g = cv._geo; if (g) drag = { x: e.clientX, x0: g.x0, x1: g.x1, moved: false, ox: e.offsetX }; });
  addEventListener('mousemove', e => {
    if (!drag) return;
    const g = cv._geo, dx = e.clientX - drag.x;
    if (Math.abs(dx) > 3) drag.moved = true;
    if (!drag.moved) return;
    st.span = drag.x1 - drag.x0; st.end = drag.x1 - (dx / g.pw) * st.span;
    redraw();
  });
  addEventListener('mouseup', () => {
    if (drag && !drag.moved && onClick) { const g = cv._geo; onClick(g.x0 + clamp((drag.ox - g.l) / g.pw, 0, 1) * (g.x1 - g.x0)); }
    drag = null;
  });
  cv.addEventListener('mousemove', e => { const g = cv._geo; if (!g || drag) return; st.cursor = g.x0 + clamp((e.offsetX - g.l) / g.pw, 0, 1) * (g.x1 - g.x0); redraw(); });
  cv.addEventListener('mouseleave', () => { st.cursor = null; redraw(); });
  cv.addEventListener('dblclick', () => { st.end = null; st.span = null; redraw(); });
}
// x/y chart with a left (axis 0) and right (axis 1) scale. series: [{xs, ys, color, dashed, axis, label}]
function xyChart(cv, o) {
  const g = fitCanvas(cv);
  if (!g) return;
  const { c, w, h } = g;
  const l = 52, r = 52, top = 24, bot = 22, pw = w - l - r, ph = h - top - bot;
  const all = o.series;
  const xsAll = all.flatMap(s => s.xs.filter(isNum));
  const x0 = o.xr ? o.xr[0] : xsAll.length ? Math.min(...xsAll) : 0, x1 = o.xr ? o.xr[1] : xsAll.length ? Math.max(...xsAll) : 1;
  const yr = [0, 1].map(ax => {
    const ys = all.filter(s => (s.axis || 0) === ax).flatMap(s => s.ys.filter(isNum));
    return ys.length ? [Math.min(0, ...ys), Math.max(...ys) * 1.08 || 1] : [0, 1];
  });
  const sx = x => l + ((x - x0) / (x1 - x0 || 1)) * pw;
  const sy = (v, ax) => top + ph * (1 - (v - yr[ax][0]) / (yr[ax][1] - yr[ax][0] || 1));
  cv._geo = { l, pw, x0, x1 };
  for (let i = 0; i <= 5; i++) {
    const y = top + (ph * i) / 5;
    c.strokeStyle = '#16202f'; c.beginPath(); c.moveTo(l, y); c.lineTo(l + pw, y); c.stroke();
    [0, 1].forEach(ax => {
      c.fillStyle = (o.axisColors || ['#36b5ff', '#ff6b6b'])[ax]; c.textAlign = ax ? 'left' : 'right';
      c.fillText(fmtN(yr[ax][1] - ((yr[ax][1] - yr[ax][0]) * i) / 5), ax ? l + pw + 5 : l - 5, y + 3);
    });
  }
  c.fillStyle = '#66768f'; c.textAlign = 'center';
  for (const tk of niceTicks(x0, x1, 8)) { const x = sx(tk); c.strokeStyle = '#16202f'; c.beginPath(); c.moveTo(x, top); c.lineTo(x, top + ph); c.stroke(); c.fillText(fmtN(tk), x, h - 6); }
  c.textAlign = 'right'; c.fillText(o.xLabel || '', l + pw, h - 6 - 11);
  let lx = l;
  c.textAlign = 'left';
  for (const s of all.filter(s => s.label)) { c.fillStyle = s.color; const tx = (s.dashed ? '┄ ' : '— ') + s.label; c.fillText(tx, lx, 12); lx += c.measureText(tx).width + 14; }
  c.save(); c.beginPath(); c.rect(l, top, pw, ph); c.clip();
  for (const s of all) {
    c.strokeStyle = s.color; c.lineWidth = s.dashed ? 1.3 : 2; c.setLineDash(s.dashed ? [6, 4] : []);
    c.beginPath();
    let pen = false;
    s.ys.forEach((v, i) => {
      if (!isNum(v) || !isNum(s.xs[i])) { pen = false; return; }
      const X = sx(s.xs[i]), Y = sy(v, s.axis || 0);
      if (pen) c.lineTo(X, Y); else c.moveTo(X, Y);
      pen = true;
    });
    c.stroke();
    if (s.points) { c.fillStyle = s.color; s.ys.forEach((v, i) => { if (isNum(v) && isNum(s.xs[i])) { c.beginPath(); c.arc(sx(s.xs[i]), sy(v, s.axis || 0), 3, 0, 7); c.fill(); } }); }
  }
  c.restore(); c.setLineDash([]);
  if (o.cursorX != null && o.cursorX >= x0 && o.cursorX <= x1) {
    const x = sx(o.cursorX);
    c.strokeStyle = '#ffffffaa'; c.beginPath(); c.moveTo(x, top); c.lineTo(x, top + ph); c.stroke();
    if (o.cursorText) { c.fillStyle = '#fff'; c.textAlign = x > l + pw / 2 ? 'right' : 'left'; o.cursorText.forEach((tx, i) => c.fillText(tx, x + (x > l + pw / 2 ? -6 : 6), top + 14 + i * 13)); }
  }
  if (!xsAll.length && o.empty) { c.fillStyle = '#66768f'; c.textAlign = 'center'; c.fillText(o.empty, l + pw / 2, top + ph / 2); }
}
// UI history of one channel inside [x0, x1] as a series.
function histSeries(k, color) {
  const [i0, i1] = histRange(), n = i1 - i0;
  const ts = new Float64Array(n), vs = new Float64Array(n);
  for (let j = 0; j < n; j++) { ts[j] = histT(i0 + j); vs[j] = dv(k, histV(k, i0 + j)); }
  return { ts, vs, n, color };
}
const histSpan = () => { const [i0, i1] = histRange(); return i1 > i0 ? [histT(i0), histT(i1 - 1)] : [0, 1]; };
function laneFor(k, color, series) {
  const l = limitsOf(k);
  return { key: k, label: chName(k), unit: meta(k).unit, color, series, lo: l.on ? l.lo : null, hi: l.on ? l.hi : null };
}

// ---------- overview: key values, "what is happening" ----------
Object.assign(T, {
  p_keyvals: ['Главные показатели', 'Key values'], p_why: ['Что происходит', 'What is happening'],
  p_alarms: ['Активные тревоги', 'Active alarms'], p_ovchart: ['Обороты, момент и наддув — последние 60 с', 'Rpm, torque and boost — last 60 s'],
  no_alarms: ['Тревог нет.', 'No alarms.'], edit_limits: ['пределы', 'limits'],
  v_ok_run: ['Мотор работает.', 'The engine is running.'], v_why: ['Почему мотор не работает: ', 'Why the engine is not running: '],
  v_seized: ['Мотор заклинило — нужен «Холодный старт» (сброс физики).', 'The engine has seized — use Cold soak (plant reset).'],
});
const TILE_KEYS = ['plant.rpm', 'plant.torque_brake_nm', 'plant.power_ps', 'plant.map_kpa', 'plant.lambda_sensor', 'plant.throttle_pct', 'plant.coolant_c', 'plant.oil_c',
  'plant.oil_bar', 'plant.egt_c', 'plant.vehicle_kmh', 'plant.battery_v'];

// Firmware liveness: the board's STM timer must advance while simulation time advances.
const ALIVE = { ecu: { stm: NaN, t: NaN, stalled: 0 }, tcu: { stm: NaN, t: NaN, stalled: 0 } };
HOOKS.push(s => {
  const tp = s.plant && s.plant.sim_time_s;
  for (const b of ['ecu', 'tcu']) {
    const x = s[b], a = ALIVE[b];
    if (!x || !x.present || !isNum(tp)) { a.stalled = 0; continue; }
    if (x.stm_ms !== a.stm || !isNum(a.t) || tp < a.t) { a.stm = x.stm_ms; a.t = tp; }
    a.stalled = tp - a.t > 0.5 ? tp - a.t : 0;
  }
});
// Plain-language status derived only from backend fields. lvl: ok / warn / bad / off.
function diagnose(s) {
  const p = s.plant || {}, d = s.driver || {}, o = p.ecu_outputs || {}, im = s.immo || {}, ws = s.workstation || {};
  const out = [];
  const add = (lvl, title, detail, src, blocks) => out.push({ lvl, title, detail, src, blocks: !!blocks });
  const e = s.ecu || {};
  if (!e.present) add('bad', ['ЭБУ не эмулируется', 'ECU not emulated'], ['Плата ЭБУ отсутствует.', 'No ECU board.'], 'ecu.present', true);
  else if (e.error) add('bad', ['Прошивка ЭБУ остановилась с ошибкой', 'ECU firmware stopped with an error'], [`Эмулятор процессора сообщил: ${e.error}`, `CPU emulator reported: ${e.error}`], 'ecu.error', true);
  else if (ALIVE.ecu.stalled) add('bad', ['Прошивка ЭБУ стоит', 'ECU firmware stalled'], [`Таймер STM ЭБУ не растёт уже ${f(ALIVE.ecu.stalled, 1)} с времени симуляции (PC ${e.pc}, ${e.region}): процессор не исполняется, мотор не управляется. Поможет загрузка образа («Прошивки») — она перезагружает ЭБУ.`,
    `The ECU STM timer has not advanced for ${f(ALIVE.ecu.stalled, 1)} s of simulation time (PC ${e.pc}, ${e.region}): the CPU is not executing, the engine is not controlled. Loading an image (Firmware) power-cycles the ECU.`], 'ecu.stm_ms', true);
  else add(e.warm_resets ? 'warn' : 'ok', ['Прошивка ЭБУ выполняется', 'ECU firmware is running'],
    [`Процессор TC1791 исполняет код (PC ${e.pc}, ${e.region}); прерываний ${e.irq_count}, задач ОС ${e.rtos_tasks_seen}.` + (e.warm_resets ? ` Перезапусков процессора: ${e.warm_resets} — прошивка сама перезагружалась (сторожевой таймер или проверка железа).` : ''),
      `The TC1791 is executing (PC ${e.pc}, ${e.region}); ${e.irq_count} interrupts, ${e.rtos_tasks_seen} RTOS tasks.` + (e.warm_resets ? ` CPU resets: ${e.warm_resets} — the firmware restarted itself (watchdog or hardware check).` : '')], 'ecu.*');
  const tc = s.tcu || {};
  if (!tc.present) add('warn', ['КПП не эмулируется', 'Gearbox not emulated'], [tc.error || 'Плата ТКУ отсутствует.', tc.error || 'No TCU board.'], 'tcu.present');
  else if (tc.error) add('warn', ['Прошивка КПП: ошибка', 'TCU firmware error'], [tc.error, tc.error], 'tcu.error');
  else if (ALIVE.tcu.stalled) add('warn', ['Прошивка КПП стоит', 'TCU firmware stalled'], [`Таймер STM ТКУ не растёт уже ${f(ALIVE.tcu.stalled, 1)} с (PC ${tc.pc}, ${tc.region}).`, `The TCU STM timer has not advanced for ${f(ALIVE.tcu.stalled, 1)} s (PC ${tc.pc}, ${tc.region}).`], 'tcu.stm_ms');
  else add('ok', ['Прошивка КПП выполняется', 'TCU firmware is running'], [`PC ${tc.pc}, ${tc.region}; прерываний ${tc.irq_count}.`, `PC ${tc.pc}, ${tc.region}; ${tc.irq_count} interrupts.`], 'tcu.*');
  if (!d.ignition_on) add('off', ['Зажигание выключено', 'Ignition is off'], ['Нажмите «Зажигание» (клавиша I): без клеммы 15 блоки не работают.', 'Press Ignition (key I): without terminal 15 the units do not work.'], 'driver.ignition_on = 0', true);
  else add('ok', ['Зажигание включено', 'Ignition is on'], ['Клемма 15 подана.', 'Terminal 15 is on.'], 'driver.ignition_on = 1');
  if (!p.ecu_outputs_connected) add('bad', ['Команды прошивки ЭБУ не доходят до мотора', 'ECU firmware commands do not reach the engine'],
    ['Эмулятор пока не переводит сигналы с выводов процессора (искра, впрыск, дроссель, реле, насосы) в действия механизмов. Поэтому мотор завестись не может — это ограничение эмулятора, а не ошибка пользователя или калибровки.',
      'The emulator does not yet turn the CPU pin signals (spark, injection, throttle, relays, pumps) into actuator actions. So the engine cannot start — an emulator limitation, not a user or calibration error.'], 'plant.ecu_outputs_connected = 0', true);
  else add('ok', ['Команды ЭБУ управляют мотором', 'ECU commands drive the engine'], ['Выходы прошивки декодируются и подаются в физику.', 'Firmware outputs are decoded and drive the physics.'], 'plant.ecu_outputs_connected = 1');
  if (tc.present && !p.tcu_outputs_connected) add('warn', ['Команды КПП не доходят до мехатроника', 'TCU commands do not reach the mechatronic'],
    ['Коробка не включает передачи и не замыкает сцепления: в режиме «Дорога» машина не поедет. Моторному стенду КПП не нужна.', 'The gearbox engages no gears and closes no clutches: in Road mode the car will not move. The engine dyno does not need the gearbox.'], 'plant.tcu_outputs_connected = 0');
  if (!im.ecu_key_known) add('warn', ['Ключ иммобилайзера ЭБУ неизвестен', 'ECU immobiliser key unknown'],
    [`Виртуальная приборка отвечает ЭБУ без настоящего ключа (запросов 0x010: ${im.ecu_challenges}, ответов: ${im.ecu_answers}). Прошивка должна отвергнуть такой ответ и запретить пуск, как с чужой приборкой.`,
      `The virtual cluster answers the ECU without the real key (0x010 challenges: ${im.ecu_challenges}, answers: ${im.ecu_answers}). The firmware should reject it and block the start, as with a foreign cluster.`], 'immo.ecu_key_known = 0', true);
  else if (d.ignition_on && !im.ecu_challenges) add('warn', ['ЭБУ ещё не запрашивал иммобилайзер', 'ECU has not queried the immobiliser yet'], ['Кадров 0x010 от ЭБУ пока не было.', 'No 0x010 frames from the ECU yet.'], 'immo.ecu_challenges = 0');
  else add('ok', ['Иммобилайзер: ключ ЭБУ известен', 'Immobiliser: ECU key known'], [`Запросов ${im.ecu_challenges}, ответов ${im.ecu_answers}. ${im.binding || ''}`, `${im.ecu_challenges} challenges, ${im.ecu_answers} answers. ${im.binding || ''}`], 'immo.*');
  if (d.start_button || p.engine_state === 'cranking') {
    if (o.starter_relay == null) add('warn', ['Стартер: команда прошивки не декодирована', 'Starter: firmware command not decoded'], ['Запрос пуска передан, но реле стартера не управляется.', 'Start requested, but the starter relay is not driven.'], 'plant.ecu_outputs.starter_relay = null', true);
    else if (!o.starter_relay) add('warn', ['Прошивка не включает стартер', 'The firmware does not engage the starter'], ['Запрос пуска есть, но реле стартера выключено (например, иммобилайзер, селектор не в P/N, не нажат тормоз).', 'Start requested but the starter relay is off (e.g. immobiliser, selector not in P/N, brake not pressed).'], 'plant.ecu_outputs.starter_relay = 0', true);
  }
  if (d.ignition_on) {
    if (!(p.lp_fuel_bar > 0.5)) add('warn', ['Нет давления подкачки топлива', 'No low-pressure fuel'], [`Давление подкачки ${f(p.lp_fuel_bar, 2)} бар; насос в баке ${o.lpfp_duty == null ? 'не управляется прошивкой (команда не декодирована)' : 'скважность ' + f(o.lpfp_duty, 2)}.`,
      `Low-pressure fuel ${f(p.lp_fuel_bar, 2)} bar; in-tank pump ${o.lpfp_duty == null ? 'not driven (command not decoded)' : 'duty ' + f(o.lpfp_duty, 2)}.`], 'plant.lp_fuel_bar', true);
    if (p.engine_state === 'cranking' && !(p.rail_mpa > 3)) add('warn', ['Мало давления в рампе', 'Low rail pressure'], [`${f(p.rail_mpa, 2)} МПа — форсунки непосредственного впрыска не смогут распылить топливо.`, `${f(p.rail_mpa, 2)} MPa — the direct injectors cannot atomise fuel.`], 'plant.rail_mpa', true);
    if (p.engine_state === 'cranking' && Array.isArray(o.ignition) && o.ignition.every(x => x == null)) add('warn', ['Нет искры', 'No spark'], ['Прошивка не выдаёт (или эмулятор не декодирует) импульсы зажигания.', 'The firmware issues no (or the emulator does not decode) ignition pulses.'], 'plant.ecu_outputs.ignition', true);
  }
  if (isNum(p.battery_v) && p.battery_v < 11.5) add('warn', ['Низкое напряжение бортсети', 'Low system voltage'], [`${f(p.battery_v, 2)} В — электроника может сбрасываться, стартер крутит слабо.`, `${f(p.battery_v, 2)} V — electronics may reset, weak cranking.`], 'plant.battery_v');
  const st = p.engine_state;
  if (st === 'running') add('ok', ['Мотор работает', 'Engine running'], [`${f(p.rpm, 0)} об/мин.`, `${f(p.rpm, 0)} rpm.`], 'plant.engine_state');
  else if (st === 'seized') add('bad', ['Мотор заклинило', 'Engine seized'], ['Подшипники разрушены. Восстановление — «Холодный старт» в «Настройке стенда».', 'The bearings are destroyed. Recover with Cold soak in Bench setup.'], 'plant.engine_state', true);
  else add('off', ['Мотор: ' + stName(st), 'Engine: ' + stName(st)], ['Физическая модель: двигатель не работает.', 'Physics: the engine is not running.'], 'plant.engine_state');
  if (st === 'running' && isNum(p.oil_bar) && p.oil_bar < 0.5 && p.rpm > 1000) add('bad', ['Нет давления масла!', 'No oil pressure!'], ['Подшипники разрушаются без масла. Заглушите мотор.', 'Bearings fail without oil. Stop the engine.'], 'plant.oil_bar');
  // Only integrity channels (100 % = intact); damage / wear channels count up from 0 and are listed in «Ресурс узлов».
  const dmg = Array.from(FLAT.keys()).filter(k => meta(k).group === 'hlth' && /health/.test(k) && isNum(FLAT.get(k)) && dv(k, FLAT.get(k)) < 99.5);
  if (dmg.length) add('bad', ['Есть повреждения узлов', 'Components are damaged'], [dmg.map(k => `${chName(k)} ${fmtCh(k, FLAT.get(k))} %`).join('; ') + '. Подробности — «Ресурс узлов».', dmg.map(k => `${chName(k)} ${fmtCh(k, FLAT.get(k))} %`).join('; ') + '. Details — Health view.'], 'plant.*health*');
  const sf = ws.safety;
  if (sf && sf.trip) add('warn', ['Сработала защита бокса', 'Test-cell interlock tripped'], [`${chName(sf.trip.channel)} = ${fmtN(sf.trip.value)} (предел ${fmtN(sf.trip.limit)}) на t = ${f(sf.trip.t_s, 2)} с. Педаль удерживается в 0, пока защиту не сбросят («Диностенд»).`,
    `${chName(sf.trip.channel)} = ${fmtN(sf.trip.value)} (limit ${fmtN(sf.trip.limit)}) at t = ${f(sf.trip.t_s, 2)} s. The pedal is held at 0 until reset (Dyno view).`], 'workstation.safety.trip', true);
  if (s.paused) add('warn', ['Симуляция на паузе', 'Simulation paused'], ['Время не идёт. Кнопка «Продолжить» или клавиша «.».', 'Time is frozen. Press Resume or key ".".'], 'paused', true);
  else if (isNum(eff.x) && eff.x < 0.9) add('warn', ['Симуляция медленнее реального времени', 'Simulation slower than real time'], [`Скорость ${eff.x.toFixed(2)}×: компьютер не успевает считать прошивки и физику в реальном времени. Результаты верны, просто идут медленнее.`,
    `Speed ${eff.x.toFixed(2)}×: the host cannot run firmware and physics in real time. Results are correct, just slower.`], 'frame.realtime_factor');
  return out;
}
function verdict(s, dg) {
  const st = s.plant && s.plant.engine_state;
  if (st === 'running') return ['ok', t('v_ok_run')];
  if (st === 'seized') return ['bad', t('v_seized')];
  const b = dg.find(x => x.blocks && x.lvl !== 'ok');
  return b ? [b.lvl === 'bad' ? 'bad' : 'warn', t('v_why') + L(b.title)] : ['warn', L(['Мотор не работает.', 'The engine is not running.'])];
}
const lightsHTML = dg => dg.map(x => `<div><span class="lamp ${x.lvl === 'off' ? '' : x.lvl}"></span><b>${esc(L(x.title))}</b><div class="d">${esc(L(x.detail))}</div><div class="d src">${esc(x.src)}</div></div>`).join('');

// ---------- gearbox at a glance (TCU's own CAN signals + plant) ----------
Object.assign(T, {
  p_ovgbx: ['Коробка передач', 'Gearbox'], gbx_read: ['Читать ошибки ТКУ', 'Read TCU DTCs'], gbx_dtc_none: ['не читались', 'not read yet'],
  gbx_note: ['Передача, позиция и целевая передача — то, что сама прошивка ТКУ передаёт по CAN (WBA_03, Getriebe_11); давления, температура и проскальзывание — физика. Ошибки — чтение памяти неисправностей ТКУ (19 02), коды по таблице самой прошивки.',
    'Gear, position and target gear are what the TCU firmware itself sends on CAN (WBA_03, Getriebe_11); pressures, temperature and slip are physics. DTCs: the TCU fault memory (19 02), codes by the firmware\'s own table.'],
});
// GE_Fahrstufe values (DQ200 table 0x801382BC, MQB): 5 P, 6 R, 7 N, 8 D, 9 S, 14 tiptronic.
const FAHRSTUFE = { 5: 'P', 6: 'R', 7: 'N', 8: 'D', 9: 'S', 14: 'M' };
const GBX_DTC = { list: null, at: NaN };
HOOKS.push(s => {
  const u = s.uds_tcu || {};
  if (/ReadDTC/.test(u.request || '') && !u.pending && u.response_hex) GBX_DTC.list = (u.dtcs || []).map((d, i) => ({ raw: d, sae: (u.dtc_sae || [])[i], st: parseInt((/status 0x([0-9A-F]{2})/i.exec(d) || [])[1] || '0', 16) })), GBX_DTC.at = s.plant ? s.plant.sim_time_s : NaN;
});
function gbxHTML(s) {
  const b = s.gearbox_bus || {}, d = s.driver || {}, p = s.plant || {}, pr = s.tcu_prep || {};
  const g = v => (v == null ? DASH : String(v)), kv = (k, v, cls) => `<div data-ch="${esc(k)}"><span>${esc(chName(k))}</span><b class="${cls || ''}">${esc(v)}</b></div>`;
  const ch = k => `${fmtCh(k, FLAT.get(k))} ${meta(k).unit || ''}`;
  // active = failing now or in this operation cycle (status bits 0/1); history = confirmed / pending earlier
  const faults = (GBX_DTC.list || []).filter(x => x.st & 0x03), hist = (GBX_DTC.list || []).filter(x => !(x.st & 0x03) && x.st & 0x2C);
  const dtc = GBX_DTC.list == null ? t('gbx_dtc_none') : (faults.length ? faults.map(x => (x.sae && x.sae !== '?' ? x.sae.replace(/ \(.*\)$/, '') : x.raw.slice(0, 6))).join(', ') : L(['нет активных', 'none active']))
    + (hist.length ? ` · ${L(['история', 'history'])}: ${hist.length}` : '');
  return `<div class="kv">${kv('gearbox_bus.gear', g(b.gear))}${kv('driver.selector', `${d.selector || DASH} · ${L(['ТКУ', 'TCU'])}: ${b.fahrstufe == null ? DASH : FAHRSTUFE[b.fahrstufe] || b.fahrstufe}`)}`
    + `${kv('gearbox_bus.zielgang', g(b.zielgang))}${kv('plant.clutch_bar[0]', ch('plant.clutch_bar[0]'))}${kv('plant.clutch_bar[1]', ch('plant.clutch_bar[1]'))}`
    + `${kv('plant.mechatronic_oil_c', ch('plant.mechatronic_oil_c'))}${kv('plant.clutch_slip_rpm[0]', ch('plant.clutch_slip_rpm[0]'))}${kv('plant.clutch_slip_rpm[1]', ch('plant.clutch_slip_rpm[1]'))}`
    + `<div class="w"><span>${esc(L(['Ошибки ТКУ', 'TCU DTCs']))}${isNum(GBX_DTC.at) ? ` (t ${f(GBX_DTC.at, 1)} s)` : ''}</span><b class="${faults.length ? 'bad' : ''}">${esc(dtc)}</b></div></div>`
    + `<div class="row"><button data-gbxdtc>${esc(t('gbx_read'))}</button>${pr.running ? `<span class="warn">${esc(L(['Мехатроник готовится в мастерской', 'Mechatronic being prepared in the workshop']))}: ${esc((pr.progress && pr.progress.step) || '')} ${f(100 * ((pr.progress && pr.progress.frac) || 0), 0)} %</span>` : ''}</div>`
    + `<div class="note">${esc(t('gbx_note'))}</div>`;
}

let ovChartSt = { span: 60, end: null, cursor: null };
view({
  id: 'overview',
  init() {
    $('ov-tiles').innerHTML = TILE_KEYS.map(k => `<div class="tile" id="tile-${esc(k)}" data-ch="${esc(k)}"><div class="n"></div><div class="v">—</div></div>`).join('');
    $('ov-alarms').onclick = e => { const b = e.target.closest('[data-lim]'); if (b) editLimits(b.dataset.lim); };
    chartNav($('ov-chart'), ovChartSt, () => S && VIEWS.overview.render(S));
    $('ov-gbx').onclick = e => { const b = e.target.closest('[data-gbxdtc]'); if (b) act(b, async () => { onState(await api('/api/uds', { target: 'tcu', service: 'read_dtc' })); say('UDS read_dtc → TCU'); }); };
  },
  render(s) {
    for (const k of TILE_KEYS) {
      const el = document.getElementById('tile-' + k), v = FLAT.get(k);
      el.querySelector('.n').textContent = chName(k);
      el.querySelector('.v').innerHTML = `${esc(fmtCh(k, v))}<span class="u">${esc(meta(k).unit || '')}</span>`;
      el.classList.toggle('alarm', ACTIVE.has(k));
    }
    const dg = diagnose(s), [lv, txt] = verdict(s, dg);
    $('ov-verdict').innerHTML = `<div class="verdict ${lv}">${esc(txt)}</div>`;
    $('ov-lights').innerHTML = lightsHTML(dg);
    $('p-ovgbx').hidden = !(s.tcu && s.tcu.present);
    if ((s.tcu && s.tcu.present) && !$('ov-gbx').contains(document.activeElement)) {
      $('ov-gbx').innerHTML = gbxHTML(s);
      $('ov-gbx-sub').textContent = `${(s.drivetrain_profile && s.drivetrain_profile.tcu_code) || ''} · ${(s.tcu.sw_id || '').replace(/\s+\S+$/, '')}`;
    }
    $('ov-alarms').innerHTML = ACTIVE.size ? `<table><tr><th></th><th>${esc(L(['значение', 'value']))}</th><th>${esc(t('limits'))}</th><th>${esc(L(['с t, с', 'since t, s']))}</th><th></th></tr>` + Array.from(ACTIVE.entries()).map(([k, a]) => {
      const l = limitsOf(k);
      return `<tr><td data-ch="${esc(k)}">${esc(chName(k))}</td><td class="r bad">${esc(fmtCh(k, FLAT.get(k)))} ${esc(meta(k).unit || '')}</td><td class="r">${a.side === 2 ? '> ' + fmtN(l.hi) : '< ' + fmtN(l.lo)}</td><td class="r">${f(a.since, 2)}</td><td><button data-lim="${esc(k)}">${esc(t('edit_limits'))}</button></td></tr>`;
    }).join('') + '</table>' : `<div class="muted">${esc(t('no_alarms'))}</div>`;
    const [a, b] = histSpan(), end = ovChartSt.end ?? b, span = ovChartSt.span || 60;
    const ks = ['plant.rpm', 'plant.torque_brake_nm', 'plant.map_kpa', 'plant.lambda_sensor'];
    laneChart($('ov-chart'), { x0: Math.max(a, end - span), x1: end, cursor: ovChartSt.cursor, lanes: ks.map((k, i) => laneFor(k, COLORS[i], [histSeries(k, COLORS[i])])) });
  },
});

// ---------- gauges ----------
Object.assign(T, {
  p_gauges: ['Приборные панели', 'Gauge dashboards'], g_addpage: ['+ Страница', '+ Page'], g_renpage: ['Переименовать', 'Rename'], g_delpage: ['Удалить страницу', 'Delete page'],
  g_edit: ['Редактировать', 'Edit layout'], g_editing: ['Готово', 'Done editing'], g_reset: ['Сброс раскладки', 'Reset layouts'],
  tip_g_edit: ['Режим правки: добавление, удаление и настройка приборов. Раскладка сохраняется в браузере.', 'Edit mode: add, remove and configure gauges. Layouts are saved in the browser.'],
  g_note: ['Приборы показывают каналы телеметрии сервера; цветные зоны — пределы тревог канала. Наведите курсор — описание канала.', 'Gauges show server telemetry channels; coloured zones are the channel alarm limits. Hover for the channel description.'],
  g_add: ['+ Прибор', '+ Gauge'], g_cfg: ['Настройка прибора', 'Gauge settings'], g_type: ['Тип', 'Type'], g_dial: ['Стрелочный', 'Dial'], g_bar: ['Шкала', 'Bar'], g_digital: ['Цифровой', 'Digital'],
  g_min: ['Мин', 'Min'], g_max: ['Макс', 'Max'], g_page: ['Страница', 'Page'], g_confirm_del: ['Удалить страницу приборов?', 'Delete this gauge page?'], g_confirm_reset: ['Вернуть стандартные страницы приборов? Ваши раскладки будут удалены.', 'Restore the default gauge pages? Your layouts will be deleted.'],
  g_newpage: ['Новая страница', 'New page'], g_pagename: ['Название', 'Name'],
});
const G_DEFAULT = () => [
  { name: L(['Двигатель', 'Engine']), items: [['plant.rpm', 'dial'], ['plant.torque_brake_nm', 'dial'], ['plant.power_ps', 'dial'], ['plant.map_kpa', 'dial'], ['plant.lambda_sensor', 'bar'], ['plant.throttle_pct', 'bar'],
    ['plant.coolant_c', 'bar'], ['plant.oil_c', 'bar'], ['plant.oil_bar', 'bar'], ['plant.egt_c', 'bar'], ['plant.iat_c', 'digital'], ['plant.battery_v', 'digital']].map(([key, type]) => ({ key, type })) },
  { name: L(['Коробка', 'Gearbox']), items: [['plant.vehicle_kmh', 'dial'], ['gearbox_bus.gear', 'digital'], ['plant.driving_gear', 'digital'], ['plant.shaft_rpm[0]', 'dial'], ['plant.shaft_rpm[1]', 'dial'],
    ['plant.clutch_bar[0]', 'bar'], ['plant.clutch_bar[1]', 'bar'], ['plant.clutch_temp_c[0]', 'bar'], ['plant.clutch_temp_c[1]', 'bar'],
    ['plant.clutch_slip_rpm[0]', 'digital'], ['plant.clutch_slip_rpm[1]', 'digital'], ['plant.mechatronic_oil_c', 'bar'], ['plant.gearbox_hydraulic_bar', 'bar']].map(([key, type]) => ({ key, type })) },
  { name: L(['Прошивка', 'Firmware']), items: [['plant.ecu_outputs_connected', 'digital'], ['plant.ecu_outputs.etc_duty', 'bar'], ['plant.ecu_outputs.ignition[0].spark_btdc_deg', 'digital'], ['plant.ecu_outputs.injection[0][0].width_ms', 'digital'],
    ['ecu.irq_count', 'digital'], ['ecu.warm_resets', 'digital'], ['frame.realtime_factor', 'dial'], ['immo.ecu_challenges', 'digital']].map(([key, type]) => ({ key, type })) },
];
let GP = LS.get('gauges', null) || G_DEFAULT(), GPAGE = clamp(LS.get('gpage', 0), 0, GP.length - 1), GEDIT = false;
const saveGauges = () => { LS.set('gauges', GP); LS.set('gpage', GPAGE); };
function gaugeRange(g) {
  const m = meta(g.key), st = STATS.get(g.key);
  const mn = isNum(g.min) ? g.min : m.r ? m.r[0] : st ? Math.min(0, dv(g.key, st.min)) : 0;
  let mx = isNum(g.max) ? g.max : m.r ? m.r[1] : st ? dv(g.key, st.max) * 1.2 : 1;
  if (!(mx > mn)) mx = mn + 1;
  return [mn, mx];
}
function drawGauge(cv, g) {
  const gg = fitCanvas(cv);
  if (!gg) return;
  const { c, w, h } = gg, v = dv(g.key, FLAT.get(g.key)), m = meta(g.key), l = limitsOf(g.key), [mn, mx] = gaugeRange(g);
  const al = ACTIVE.has(g.key), txt = fmtCh(g.key, FLAT.get(g.key)), col = al ? '#ff4d4f' : '#e8eef7';
  const frac = x => clamp((x - mn) / (mx - mn), 0, 1);
  c.textAlign = 'center';
  if (g.type === 'dial') {
    const cx = w / 2, cy = h * 0.58, R = Math.min(w * 0.42, h * 0.5), a0 = Math.PI * 0.75, a1 = Math.PI * 2.25, ang = x => a0 + frac(x) * (a1 - a0);
    c.lineWidth = 9; c.strokeStyle = '#1b2638'; c.beginPath(); c.arc(cx, cy, R, a0, a1); c.stroke();
    if (l.on && isNum(l.hi) && l.hi < mx) { c.strokeStyle = '#ff4d4f88'; c.beginPath(); c.arc(cx, cy, R, ang(l.hi), a1); c.stroke(); }
    if (l.on && isNum(l.lo) && l.lo > mn) { c.strokeStyle = '#ff4d4f88'; c.beginPath(); c.arc(cx, cy, R, a0, ang(l.lo)); c.stroke(); }
    if (isNum(v)) { c.strokeStyle = al ? '#ff4d4f' : '#36b5ff'; c.beginPath(); c.arc(cx, cy, R, a0, ang(v)); c.stroke(); }
    c.lineWidth = 1; c.fillStyle = '#66768f'; c.font = '9.5px monospace';
    for (let i = 0; i <= 5; i++) { const x = mn + ((mx - mn) * i) / 5, a = ang(x); c.fillText(fmtN(x), cx + Math.cos(a) * (R - 18), cy + Math.sin(a) * (R - 18) + 3); }
    if (isNum(v)) { const a = ang(v); c.strokeStyle = '#fff'; c.lineWidth = 2; c.beginPath(); c.moveTo(cx, cy); c.lineTo(cx + Math.cos(a) * (R - 4), cy + Math.sin(a) * (R - 4)); c.stroke(); }
    c.fillStyle = col; c.font = '600 20px monospace'; c.fillText(txt, cx, cy + R * 0.55);
    c.fillStyle = '#9fb0c8'; c.font = '11px sans-serif'; c.fillText(m.unit || '', cx, cy + R * 0.55 + 14);
    c.fillText(chName(g.key).slice(0, 34), cx, 12);
  } else if (g.type === 'bar') {
    c.fillStyle = '#9fb0c8'; c.font = '11px sans-serif'; c.textAlign = 'left'; c.fillText(chName(g.key).slice(0, 38), 6, 13);
    c.textAlign = 'right'; c.fillStyle = col; c.font = '600 18px monospace'; c.fillText(`${txt} ${m.unit || ''}`, w - 6, 40);
    const y = 50, bw = w - 12;
    c.fillStyle = '#1b2638'; c.fillRect(6, y, bw, 12);
    if (isNum(v)) { c.fillStyle = al ? '#ff4d4f' : '#36b5ff'; c.fillRect(6, y, bw * frac(v), 12); }
    c.fillStyle = '#ff4d4f';
    for (const x of [l.on && l.lo, l.on && l.hi]) if (isNum(x) && x >= mn && x <= mx) c.fillRect(6 + bw * frac(x) - 1, y - 3, 2, 18);
    c.fillStyle = '#66768f'; c.font = '9.5px monospace'; c.textAlign = 'left'; c.fillText(fmtN(mn), 6, y + 25); c.textAlign = 'right'; c.fillText(fmtN(mx), w - 6, y + 25);
  } else {
    c.fillStyle = '#9fb0c8'; c.font = '11px sans-serif'; c.fillText(chName(g.key).slice(0, 38), w / 2, 13);
    c.fillStyle = col; c.font = '600 28px monospace'; c.fillText(txt, w / 2, 50);
    c.fillStyle = '#66768f'; c.font = '11px sans-serif'; c.fillText(m.unit || '', w / 2, 68);
  }
}
function buildGauges() {
  const pg = GP[GPAGE] || { items: [] };
  $('g-pages').innerHTML = GP.map((p, i) => `<button data-pg="${i}" class="${i === GPAGE ? 'on' : ''}">${esc(p.name)}</button>`).join('');
  $('g-grid').classList.toggle('edit', GEDIT);
  $('g-grid').innerHTML = pg.items.map((g, i) => `<div class="gauge ${esc(g.type)}" data-ch="${esc(g.key)}"><canvas></canvas><div class="gtools"><button data-gcfg="${i}">⚙</button><button data-gdel="${i}" class="danger">✕</button></div></div>`).join('')
    + (GEDIT ? `<div class="gadd" id="g-add">${esc(t('g_add'))}</div>` : '');
  $('g-edit').textContent = t(GEDIT ? 'g_editing' : 'g_edit');
  $('g-edit').classList.toggle('on', GEDIT);
  $('g-sub').textContent = pg.name || '';
}
function gaugeCfg(i) {
  const g = GP[GPAGE].items[i];
  modal(t('g_cfg'), `<div class="row"><b data-ch="${esc(g.key)}">${esc(chName(g.key))}</b><span class="mono muted">${esc(g.key)}</span><button id="gc-ch">${esc(t('pick_channel'))}</button></div>
    <div class="row"><label>${esc(t('g_type'))}</label><select id="gc-type">${['dial', 'bar', 'digital'].map(x => `<option value="${x}"${g.type === x ? ' selected' : ''}>${esc(t('g_' + x))}</option>`).join('')}</select>
    <label>${esc(t('g_min'))}</label><input id="gc-min" type="number" step="any" value="${isNum(g.min) ? g.min : ''}"><label>${esc(t('g_max'))}</label><input id="gc-max" type="number" step="any" value="${isNum(g.max) ? g.max : ''}"></div>
    <div class="note">${esc(L(['Пустые мин/макс — типичный диапазон канала из реестра или наблюдаемый.', 'Empty min/max = the channel typical range from the registry, or the observed range.']))}</div>
    <div class="row"><button id="gc-lim">${esc(t('limits'))}…</button></div>`,
  [{ label: t('cancel') }, { label: t('apply'), cls: 'primary', fn: () => {
    const num = id => { const v = $(id).value.trim(); return v === '' || !Number.isFinite(+v) ? null : +v; };
    Object.assign(g, { type: $('gc-type').value, min: num('gc-min'), max: num('gc-max') });
    saveGauges(); buildGauges();
  } }]);
  $('gc-ch').onclick = async () => { const k = await pickChannel(); if (k) { g.key = k; saveGauges(); buildGauges(); } };
  $('gc-lim').onclick = () => editLimits(g.key);
}
view({
  id: 'gauges',
  init() {
    $('g-pages').onclick = e => { const b = e.target.closest('[data-pg]'); if (b) { GPAGE = +b.dataset.pg; saveGauges(); buildGauges(); } };
    $('g-edit').onclick = () => { GEDIT = !GEDIT; buildGauges(); };
    $('g-addpage').onclick = async () => { const n = await promptBox(t('g_addpage'), t('g_pagename'), t('g_newpage')); if (n) { GP.push({ name: n, items: [] }); GPAGE = GP.length - 1; GEDIT = true; saveGauges(); buildGauges(); } };
    $('g-renpage').onclick = async () => { const p = GP[GPAGE]; if (!p) return; const n = await promptBox(t('g_renpage'), t('g_pagename'), p.name); if (n) { p.name = n; saveGauges(); buildGauges(); } };
    $('g-delpage').onclick = async () => { if (GP.length && await confirmBox(t('g_confirm_del'))) { GP.splice(GPAGE, 1); if (!GP.length) GP = G_DEFAULT(); GPAGE = 0; saveGauges(); buildGauges(); } };
    $('g-reset').onclick = async () => { if (await confirmBox(t('g_confirm_reset'))) { GP = G_DEFAULT(); GPAGE = 0; saveGauges(); buildGauges(); } };
    $('g-grid').onclick = async e => {
      const cfg = e.target.closest('[data-gcfg]'), del = e.target.closest('[data-gdel]');
      if (cfg) return gaugeCfg(+cfg.dataset.gcfg);
      if (del) { GP[GPAGE].items.splice(+del.dataset.gdel, 1); saveGauges(); buildGauges(); return; }
      if (e.target.closest('#g-add')) { const k = await pickChannel(); if (k) { GP[GPAGE].items.push({ key: k, type: meta(k).bool ? 'digital' : 'dial' }); saveGauges(); buildGauges(); } }
    };
  },
  show: buildGauges,
  i18n: buildGauges,
  render() {
    const items = (GP[GPAGE] || { items: [] }).items;
    $$('#g-grid .gauge canvas').forEach((cv, i) => { if (items[i]) drawGauge(cv, items[i]); });
  },
});

// ---------- strip charts ----------
Object.assign(T, {
  p_chart: ['Многоканальный самописец', 'Multi-channel strip chart'], window: ['Окно', 'Window'], follow_live: ['▶ Живой режим', '▶ Live'],
  tip_c_live: ['Вернуться к последним данным (или двойной клик по графику).', 'Back to the newest data (or double-click the chart).'],
  c_save: ['Сохранить набор', 'Save set'], c_sets: ['— наборы каналов —', '— channel sets —'], tip_c_sets: ['Сохранённые наборы каналов самописца.', 'Saved strip-chart channel sets.'],
  chart_help: ['Колесо мыши — масштаб по времени, перетаскивание — сдвиг, двойной клик — живой режим. Курсор показывает значения всех каналов. Красный пунктир — пределы тревог. Окно хранит до 5 мин с частотой опроса интерфейса (для 100 Гц — «Запись»).',
    'Mouse wheel zooms time, drag pans, double-click returns to live. The cursor shows all channel values. Red dashes = alarm limits. The window keeps up to 5 min at the UI poll rate (for 100 Hz use the Recorder).'],
  st_cur: ['сейчас', 'now'], st_cursor: ['курсор', 'cursor'], st_min: ['мин (окно)', 'min (window)'], st_max: ['макс (окно)', 'max (window)'], st_avg: ['средн. (окно)', 'avg (window)'],
  c_setname: ['Название набора', 'Set name'],
});
let CH_KEYS = LS.get('chart_keys', ['plant.rpm', 'plant.torque_brake_nm', 'plant.map_kpa', 'plant.lambda_sensor', 'plant.coolant_c']);
let CSETS = LS.get('chart_sets', {});
const chartSt = { span: null, end: null, cursor: null };
function chartChips(el, ks, onRemove) {
  el.innerHTML = ks.map((k, i) => `<span class="cchip" data-ch="${esc(k)}"><i style="background:${COLORS[i % COLORS.length]}"></i>${esc(chName(k))}<span class="x" data-rm="${i}">✕</span></span>`).join('');
  el.onclick = e => { const x = e.target.closest('[data-rm]'); if (x) onRemove(+x.dataset.rm); };
}
function buildChartUI() {
  chartChips($('c-chips'), CH_KEYS, i => { CH_KEYS.splice(i, 1); LS.set('chart_keys', CH_KEYS); buildChartUI(); });
  $('c-sets').innerHTML = `<option value="">${esc(t('c_sets'))}</option>` + Object.keys(CSETS).map(n => `<option>${esc(n)}</option>`).join('');
}
function drawChart() {
  const [a, b] = histSpan(), win = +$('c-win').value;
  const span = chartSt.span || (win || Math.max(1, b - a)), end = chartSt.end ?? b;
  const x0 = chartSt.end == null && !chartSt.span ? (win ? Math.max(a, end - span) : a) : end - span;
  const lanes = CH_KEYS.map((k, i) => laneFor(k, COLORS[i % COLORS.length], [histSeries(k, COLORS[i % COLORS.length])]));
  laneChart($('c-canvas'), { x0, x1: end, cursor: chartSt.cursor, lanes, empty: L(['Добавьте каналы кнопкой «+ Канал».', 'Add channels with "+ Channel".']) });
  $('c-live').classList.toggle('on', chartSt.end == null);
  // window statistics
  const rows = CH_KEYS.map((k, i) => {
    const sr = lanes[i].series[0];
    let mn = Infinity, mx = -Infinity, sum = 0, n = 0;
    for (let j = 0; j < sr.n; j++) { const tt = sr.ts[j], v = sr.vs[j]; if (tt < x0 || tt > end || !isNum(v)) continue; mn = Math.min(mn, v); mx = Math.max(mx, v); sum += v; n++; }
    const l = limitsOf(k), cur = chartSt.cursor != null ? valueAt(sr, chartSt.cursor) : NaN;
    return `<tr><td data-ch="${esc(k)}"><i style="display:inline-block;width:9px;height:9px;border-radius:50%;background:${COLORS[i % COLORS.length]}"></i> ${esc(chName(k))}</td><td>${esc(meta(k).unit || '')}</td>`
      + `<td class="r${ACTIVE.has(k) ? ' bad' : ''}">${esc(fmtCh(k, FLAT.get(k)))}</td><td class="r">${fmtN(cur)}</td><td class="r">${n ? fmtN(mn) : DASH}</td><td class="r">${n ? fmtN(mx) : DASH}</td><td class="r">${n ? fmtN(sum / n) : DASH}</td>`
      + `<td class="r">${l.on ? `${isNum(l.lo) ? '&lt; ' + fmtN(l.lo) : ''} ${isNum(l.hi) ? '&gt; ' + fmtN(l.hi) : ''}` : DASH}</td><td><button data-lim="${esc(k)}">${esc(t('edit_limits'))}</button></td></tr>`;
  });
  $('c-stats').innerHTML = `<tr><th></th><th></th><th class="r">${esc(t('st_cur'))}</th><th class="r">${esc(t('st_cursor'))}</th><th class="r">${esc(t('st_min'))}</th><th class="r">${esc(t('st_max'))}</th><th class="r">${esc(t('st_avg'))}</th><th class="r">${esc(t('limits'))}</th><th></th></tr>` + rows.join('');
  $('c-sub').textContent = `${fmtN(x0)} … ${fmtN(end)} s`;
}
view({
  id: 'charts',
  init() {
    chartNav($('c-canvas'), chartSt, drawChart);
    $('c-add').onclick = async () => { const k = await pickChannel(); if (k && !CH_KEYS.includes(k)) { CH_KEYS.push(k); LS.set('chart_keys', CH_KEYS); buildChartUI(); drawChart(); } };
    $('c-win').onchange = () => { chartSt.span = null; chartSt.end = null; drawChart(); };
    $('c-live').onclick = () => { chartSt.span = null; chartSt.end = null; drawChart(); };
    $('c-stats').onclick = e => { const b = e.target.closest('[data-lim]'); if (b) editLimits(b.dataset.lim); };
    $('c-save').onclick = async () => { const n = await promptBox(t('c_save'), t('c_setname'), ''); if (n) { CSETS[n] = CH_KEYS.slice(); LS.set('chart_sets', CSETS); buildChartUI(); } };
    $('c-sets').onchange = e => { const v = CSETS[e.target.value]; if (v) { CH_KEYS = v.slice(); LS.set('chart_keys', CH_KEYS); buildChartUI(); drawChart(); } };
  },
  show: buildChartUI,
  i18n: buildChartUI,
  render: drawChart,
});

// ---------- all channels ----------
Object.assign(T, {
  p_allch: ['Все каналы телеметрии', 'All telemetry channels'], only_alarms: ['только тревоги', 'alarms only'], show_unavail: ['показывать недоступные («—»)', 'show unavailable ("—")'],
  reset_stats: ['Сброс статистики', 'Reset statistics'], tip_reset_stats: ['Обнулить мин/макс/среднее с момента сброса.', 'Clear min/max/average since reset.'],
  ch_note: ['Список строится автоматически из того, что прислал сервер: новые поля физики появляются сами (группа «Новые каналы»). Наведите курсор на имя — пояснение. Кнопки: ∿ — в самописец, ◔ — на текущую страницу приборов, ⚠ — пределы тревоги.',
    'Built automatically from what the server sends: new physics fields appear by themselves (group "New channels"). Hover a name for an explanation. Buttons: ∿ add to chart, ◔ add to the current gauge page, ⚠ alarm limits.'],
  all_groups: ['все группы', 'all groups'], c_key: ['ключ', 'key'], c_val: ['значение', 'value'], c_unit: ['ед.', 'unit'], c_min: ['мин', 'min'], c_max: ['макс', 'max'], c_avg: ['средн.', 'avg'],
  text_ch: ['Текстовые поля', 'Text fields'],
});
let chSig = '';
const chRows = new Map();
function buildChannels(force) {
  const q = $('ch-filter').value.trim().toLowerCase(), grp = $('ch-group').value, onlyAl = $('ch-alarms').checked, nan = $('ch-nan').checked;
  const all = Array.from(FLAT.keys());
  const groups = groupKeys(all.filter(k => typeof FLAT.get(k) === 'number'));
  const gsig = groups.map(g => g[0]).join('|');
  if ($('ch-group').dataset.sig !== gsig + LANG) {
    const cur = grp;
    $('ch-group').innerHTML = `<option value="">${esc(t('all_groups'))}</option>` + groups.map(([g, ks]) => `<option value="${esc(g)}">${esc(groupName(g))} (${ks.length})</option>`).join('');
    $('ch-group').value = cur; $('ch-group').dataset.sig = gsig + LANG;
  }
  const keep = k => {
    const v = FLAT.get(k);
    if (q && !k.toLowerCase().includes(q) && !chName(k).toLowerCase().includes(q)) return false;
    if (grp && meta(k).group !== grp) return false;
    if (onlyAl && !ACTIVE.has(k)) return false;
    if (!nan && typeof v === 'number' && !isNum(v)) return false;
    return true;
  };
  const vis = groups.map(([g, ks]) => [g, ks.filter(keep)]).filter(([, ks]) => ks.length);
  const txt = all.filter(k => typeof FLAT.get(k) === 'string' && (!q || k.toLowerCase().includes(q)) && !onlyAl);
  const sig = JSON.stringify([vis, txt, LANG]);
  if (sig === chSig && !force) return;
  chSig = sig;
  chRows.clear();
  let html = `<tr><th>${esc(L(['канал', 'channel']))}</th><th>${esc(t('c_key'))}</th><th class="r">${esc(t('c_val'))}</th><th>${esc(t('c_unit'))}</th><th class="r">${esc(t('c_min'))}</th><th class="r">${esc(t('c_max'))}</th><th class="r">${esc(t('c_avg'))}</th><th class="r">${esc(t('limits'))}</th><th></th></tr>`;
  for (const [g, ks] of vis) {
    html += `<tr><th colspan="9" class="acc">${esc(groupName(g))}</th></tr>`;
    for (const k of ks) html += `<tr data-row="${esc(k)}"><td data-ch="${esc(k)}">${esc(chName(k))}${meta(k).auto ? ' <span class="warn">*</span>' : ''}</td><td class="muted">${esc(k)}</td><td class="r"></td><td>${esc(meta(k).unit || '')}</td><td class="r"></td><td class="r"></td><td class="r"></td><td class="r"></td>`
      + `<td><button data-a="chart" data-k="${esc(k)}" data-tipraw="∿">∿</button><button data-a="gauge" data-k="${esc(k)}">◔</button><button data-a="lim" data-k="${esc(k)}">⚠</button></td></tr>`;
  }
  if (txt.length) html += `<tr><th colspan="9" class="acc">${esc(t('text_ch'))}</th></tr>` + txt.map(k => `<tr data-row="${esc(k)}"><td>${esc(chName(k))}</td><td class="muted">${esc(k)}</td><td colspan="7" style="white-space:normal"></td></tr>`).join('');
  $('ch-table').innerHTML = html;
  for (const tr of $$('#ch-table tr[data-row]')) chRows.set(tr.dataset.row, tr.cells);
}
function updateChannels() {
  buildChannels(false);
  for (const [k, cells] of chRows) {
    const v = FLAT.get(k);
    if (typeof v === 'string' || v === undefined) { cells[2].textContent = v || DASH; continue; }
    const st = STATS.get(k), l = limitsOf(k);
    cells[2].textContent = fmtCh(k, v);
    cells[2].className = 'r' + (ACTIVE.has(k) ? ' bad' : '');
    if (!isNum(v)) cells[2].dataset.tipraw = L(meta(k).na || T.na_generic); else delete cells[2].dataset.tipraw;
    cells[4].textContent = st ? fmtN(dv(k, st.min)) : DASH;
    cells[5].textContent = st ? fmtN(dv(k, st.max)) : DASH;
    cells[6].textContent = st && st.n ? fmtN(dv(k, st.sum / st.n)) : DASH;
    cells[7].textContent = l.on ? `${isNum(l.lo) ? '<' + fmtN(l.lo) : ''} ${isNum(l.hi) ? '>' + fmtN(l.hi) : ''}`.trim() || DASH : DASH;
  }
  $('ch-sub').textContent = `${FLAT.size} · ${L(['тревог', 'alarms'])}: ${ACTIVE.size}`;
}
view({
  id: 'channels',
  init() {
    for (const id of ['ch-filter', 'ch-group', 'ch-alarms', 'ch-nan']) $(id).addEventListener(id === 'ch-filter' ? 'input' : 'change', () => { buildChannels(true); updateChannels(); });
    $('ch-reset').onclick = () => { STATS.clear(); say(t('done')); };
    $('ch-table').onclick = e => {
      const b = e.target.closest('[data-a]');
      if (!b) return;
      const k = b.dataset.k;
      if (b.dataset.a === 'lim') editLimits(k);
      if (b.dataset.a === 'chart' && !CH_KEYS.includes(k)) { CH_KEYS.push(k); LS.set('chart_keys', CH_KEYS); say(`∿ ${chName(k)}`); }
      if (b.dataset.a === 'gauge' && GP[GPAGE]) { GP[GPAGE].items.push({ key: k, type: meta(k).bool ? 'digital' : 'bar' }); saveGauges(); say(`◔ ${chName(k)} → ${GP[GPAGE].name}`); }
    };
  },
  show() { buildChannels(true); },
  render: updateChannels,
});

// ---------- recorder (backend ring buffer at the 10 ms slice rate) ----------
Object.assign(T, {
  p_rec: ['Регистратор данных', 'Data recorder'], p_datasets: ['Наборы данных', 'Datasets'], p_play: ['Просмотр и воспроизведение', 'Review & playback'],
  rec_hint: ['Запись идёт на сервере на каждом 10-мс шаге симуляции (100 Гц) — без пропусков, даже если вкладка браузера свёрнута. Здесь выбирают каналы, ставят метки событий, выгружают CSV и сравнивают записи.',
    'Recording runs on the server at every 10 ms simulation slice (100 Hz) — no gaps even when the browser tab is in the background. Choose channels, mark events, export CSV and compare recordings here.'],
  rec_channels: ['Каналы записи', 'Recorded channels'], r_fromchart: ['из самописца', 'from chart'], r_all: ['все', 'all'], r_none: ['убрать все', 'none'],
  r_start: ['● Начать запись', '● Start recording'], r_stop: ['■ Стоп', '■ Stop'], r_mark: ['⚑ Метка', '⚑ Mark'], r_clear: ['Удалить запись', 'Delete capture'], r_label_ph: ['текст метки', 'event label'],
  tip_r_mark: ['Поставить метку события в записи (клавиша M).', 'Mark an event in the recording (key M).'],
  r_fetch: ['⟳ Загрузить с сервера', '⟳ Fetch from server'], r_csv: ['Экспорт CSV (запись сервера)', 'Export CSV (server capture)'], r_import: ['Импорт CSV…', 'Import CSV…'],
  r_sets_note: ['Отмеченные наборы рисуются вместе: первый — сплошными линиями, остальные — пунктиром (наложение). Сдвиг выравнивает записи по времени.',
    'Checked datasets are drawn together: the first solid, the others dashed (overlay). Offset aligns recordings in time.'],
  r_play: ['▶ Воспроизвести', '▶ Play'], r_pause: ['❚❚ Пауза', '❚❚ Pause'],
  rs_active: ['идёт запись', 'recording'], rs_rows: ['строк', 'rows'], rs_cap: ['ёмкость, строк', 'capacity, rows'], rs_span: ['длительность, с', 'duration, s'], rs_events: ['меток', 'events'], rs_rate: ['частота', 'rate'],
  ds_name: ['набор', 'dataset'], ds_rows: ['строк', 'rows'], ds_dur: ['длит., с', 'dur., s'], ds_off: ['сдвиг, с', 'offset, s'], ds_backend: ['запись сервера', 'server capture'],
  r_need_ch: ['Выберите хотя бы один канал.', 'Select at least one channel.'], r_confirm_clear: ['Удалить запись на сервере?', 'Delete the server capture?'],
  r_confirm_restart: ['Новая запись заменит текущую запись на сервере. Продолжить?', 'A new recording replaces the current server capture. Continue?'],
  r_bad_csv: ['Не удалось разобрать CSV', 'Could not parse the CSV'], r_empty: ['Нет данных: начните запись или импортируйте CSV.', 'No data: start a recording or import a CSV.'],
});
let REC_CH = LS.get('rec_channels', ['plant.rpm', 'plant.torque_brake_nm', 'plant.power_ps', 'plant.map_kpa', 'plant.put_kpa', 'plant.lambda_sensor', 'plant.throttle_pct', 'plant.knock_integral[0]',
  'plant.knock_integral[1]', 'plant.knock_integral[2]', 'plant.knock_integral[3]', 'plant.coolant_c', 'plant.oil_c', 'plant.oil_bar', 'plant.egt_c', 'plant.turbo_rpm', 'driver.pedal_pct']);
// Dataset: {id, name, src, keys, t:[], cols:[[]], events:[], off, on}
const DSETS = [];
let BACK = null, recLastSeq = null, recFetchAt = 0, recFetching = false;
const playSt = { span: null, end: null, cursor: null, playing: false };
function recStatus() { return S && S.workstation && S.workstation.recorder; }
function newBackDataset(keys) {
  BACK = { id: 'backend', name: t('ds_backend'), src: 'backend', keys: keys.slice(), t: [], cols: keys.map(() => []), events: [], off: 0, on: true };
  const i = DSETS.findIndex(d => d.id === 'backend');
  if (i >= 0) DSETS[i] = BACK; else DSETS.unshift(BACK);
  recLastSeq = null;
}
async function recFetch(all) {
  if (recFetching) return;
  recFetching = true;
  try {
    for (let guard = 0; guard < 500; guard++) {
      const d = await api('/api/ws/rec' + (recLastSeq != null ? '?since=' + recLastSeq : ''));
      if (!BACK || BACK.keys.join() !== d.channels.join()) newBackDataset(d.channels);
      for (const r of d.rows) { recLastSeq = r[0]; BACK.t.push(r[1]); for (let c = 0; c < BACK.keys.length; c++) BACK.cols[c].push(r[c + 2] == null ? NaN : dv(BACK.keys[c], r[c + 2])); }
      BACK.events = d.events.map(e => ({ t: e.t_s, label: e.label }));
      if (!d.more || !all) break;
    }
  } finally { recFetching = false; }
  if (CUR_VIEW === 'rec') buildDatasets();
}
async function recStart() {
  if (!REC_CH.length) return say(t('r_need_ch'), true);
  const rs = recStatus();
  if (rs && rs.rows && !(await confirmBox(t('r_confirm_restart'), t('r_start'), false))) return;
  await api('/api/ws/rec', { action: 'start', channels: REC_CH });
  newBackDataset(REC_CH);
  say(t('r_start'));
}
async function recStop() { await api('/api/ws/rec', { action: 'stop' }); await recFetch(true); say(t('r_stop')); }
function recToggle() { const rs = recStatus(); act(null, rs && rs.active ? recStop : recStart); }
function recMark() {
  const rs = recStatus();
  if (!rs || !rs.active) return;
  const label = ($('r-label').value || '').trim() || `#${(BACK ? BACK.events.length : 0) + 1}`;
  act(null, async () => { await api('/api/ws/rec', { action: 'mark', label }); say(`⚑ ${label}`); });
}
function dsToCSV(d) {
  const lines = [`# SIMEMU Studio recording "${d.name}"; image=${S ? S.image_label : ''}; preset=${S ? S.active_preset : ''}; exported=${new Date().toISOString()}`];
  for (const e of d.events) lines.push(`# event,${e.t},${String(e.label).replace(/[\r\n,]/g, ' ')}`);
  lines.push(['t_s'].concat(d.keys.map(k => `${k} [${meta(k).unit || ''}]`)).join(','));
  for (let i = 0; i < d.t.length; i++) lines.push([d.t[i]].concat(d.cols.map(c => (isNum(c[i]) ? c[i] : ''))).join(','));
  return lines.join('\n') + '\n';
}
function parseCSV(text, name) {
  const lines = text.split(/\r?\n/);
  const events = [];
  let hdr = null;
  const rows = [];
  for (const ln of lines) {
    if (!ln.trim()) continue;
    if (ln.startsWith('#')) { const m = ln.match(/^# event,([^,]+),(.*)$/); if (m) events.push({ t: +m[1], label: m[2] }); continue; }
    const cells = ln.split(',');
    if (!hdr) { hdr = cells.map(c => c.trim().replace(/\s+\[[^\]]*\]$/, '')); continue; }
    rows.push(cells.map(c => (c.trim() === '' ? NaN : +c)));
  }
  if (!hdr || hdr.length < 2 || !rows.length) throw new Error(t('r_bad_csv'));
  const ti = Math.max(0, hdr.findIndex(h => /^t(_s|ime)?$/i.test(h)));
  const keys = hdr.filter((_, i) => i !== ti);
  const cols = keys.map(() => []), tt = [];
  for (const r of rows) { tt.push(r[ti]); let c = 0; hdr.forEach((_, i) => { if (i !== ti) cols[c++].push(r[i]); }); }
  return { id: 'csv' + Date.now(), name, src: 'csv', keys, t: tt, cols, events, off: 0, on: true };
}
function buildRecChips() {
  chartChips($('r-chips'), REC_CH, i => { REC_CH.splice(i, 1); LS.set('rec_channels', REC_CH); buildRecChips(); });
  $('r-nch').textContent = `(${REC_CH.length})`;
}
function buildDatasets() {
  $('r-sets').innerHTML = `<tr><th></th><th>${esc(t('ds_name'))}</th><th class="r">${esc(t('ds_rows'))}</th><th class="r">${esc(t('ds_dur'))}</th><th class="r">${esc(t('ds_off'))}</th><th></th></tr>` + DSETS.map((d, i) => {
    const dur = d.t.length ? d.t[d.t.length - 1] - d.t[0] : 0;
    return `<tr><td><input type="checkbox" data-on="${i}"${d.on ? ' checked' : ''}></td><td>${esc(d.name)} <span class="muted">(${d.keys.length} ch)</span></td><td class="r">${d.t.length}</td><td class="r">${f(dur, 2)}</td>`
      + `<td class="r"><input type="number" step="0.1" data-off="${i}" value="${d.off}" style="width:70px"></td><td><button data-csv="${i}">CSV</button>${d.src === 'csv' ? `<button data-rm="${i}" class="danger">✕</button>` : ''}</td></tr>`;
  }).join('');
}
function playData() {
  const sets = DSETS.filter(d => d.on && d.t.length);
  if (!sets.length) return null;
  const p = sets[0], t0 = p.t[0];
  const keys = p.keys.slice(0, 10);
  const lanes = keys.map((k, i) => {
    const color = COLORS[i % COLORS.length];
    const series = sets.map((d, j) => { const c = d.keys.indexOf(k); return c < 0 ? null : { ts: d.t, vs: d.cols[c], n: d.t.length, off: -d.t[0] + d.off, color, dashed: j > 0 }; }).filter(Boolean);
    return laneFor(k, color, series);
  });
  const dur = Math.max(...sets.map(d => d.t[d.t.length - 1] - d.t[0] + d.off));
  return { sets, lanes, t0, dur: Math.max(0.01, dur), events: p.events.map(e => ({ t: e.t - t0 + p.off, label: e.label })) };
}
function drawPlay() {
  const pd = playData();
  if (!pd) { laneChart($('r-canvas'), { x0: 0, x1: 1, lanes: [], empty: t('r_empty') }); $('r-values').innerHTML = ''; return; }
  const span = playSt.span || pd.dur, end = playSt.end ?? (playSt.span ? span : pd.dur);
  const cur = playSt.playCursor;
  laneChart($('r-canvas'), { x0: end - span, x1: end, cursor: playSt.cursor ?? cur, lanes: pd.lanes, events: pd.events });
  const at = playSt.cursor ?? cur ?? pd.dur;
  $('r-cursor').textContent = `${f(at, 2)} / ${f(pd.dur, 2)} s`;
  $('r-scrub').value = String(Math.round((clamp(at, 0, pd.dur) / pd.dur) * 1000));
  $('r-play-sub').textContent = pd.sets.map(d => d.name).join(' vs ');
  $('r-values').innerHTML = `<tr><th></th>${pd.sets.map((d, j) => `<th class="r">${esc(d.name)}${j ? ' ┄' : ''}</th>`).join('')}${pd.sets.length > 1 ? `<th class="r">Δ</th>` : ''}</tr>` + pd.lanes.map(ln => {
    const vals = ln.series.map(sr => valueAt(sr, at));
    return `<tr><td data-ch="${esc(ln.key)}" style="color:${ln.color}">${esc(ln.label)} <span class="muted">${esc(ln.unit || '')}</span></td>${vals.map(v => `<td class="r">${fmtN(v)}</td>`).join('')}${vals.length > 1 ? `<td class="r">${fmtN(vals[1] - vals[0])}</td>` : ''}</tr>`;
  }).join('');
}
let playRAF = 0, playLast = 0;
function playTick(ts) {
  if (!playSt.playing) return;
  const pd = playData();
  if (!pd) { playSt.playing = false; return; }
  const dt = playLast ? (ts - playLast) / 1000 : 0;
  playLast = ts;
  playSt.playCursor = (playSt.playCursor || 0) + dt * +$('r-speed').value;
  if (playSt.playCursor >= pd.dur) { playSt.playCursor = pd.dur; playSt.playing = false; $('r-play').textContent = t('r_play'); }
  if (playSt.span && playSt.playCursor > (playSt.end ?? playSt.span)) playSt.end = playSt.playCursor + playSt.span * 0.1;
  drawPlay();
  if (playSt.playing) playRAF = requestAnimationFrame(playTick);
}
view({
  id: 'rec',
  init() {
    $('r-addch').onclick = async () => { const ks = await pickChannels(t('pick_channels'), true, REC_CH); if (ks) { REC_CH = ks; LS.set('rec_channels', REC_CH); buildRecChips(); } };
    $('r-fromchart').onclick = () => { REC_CH = Array.from(new Set(REC_CH.concat(CH_KEYS))); LS.set('rec_channels', REC_CH); buildRecChips(); };
    $('r-all').onclick = () => { REC_CH = numKeys(); LS.set('rec_channels', REC_CH); buildRecChips(); };
    $('r-none').onclick = () => { REC_CH = []; LS.set('rec_channels', REC_CH); buildRecChips(); };
    $('r-start').onclick = e => act(e.target, recStart);
    $('r-stop').onclick = e => act(e.target, recStop);
    $('r-mark').onclick = recMark;
    $('r-clear').onclick = e => act(e.target, async () => {
      if (!(await confirmBox(t('r_confirm_clear')))) return;
      await api('/api/ws/rec', { action: 'clear' });
      const i = DSETS.findIndex(d => d.id === 'backend'); if (i >= 0) DSETS.splice(i, 1);
      BACK = null; recLastSeq = null; buildDatasets(); drawPlay();
    });
    $('r-fetch').onclick = e => act(e.target, async () => { BACK = null; recLastSeq = null; await recFetch(true); drawPlay(); say(t('done')); });
    $('r-csv').onclick = e => act(e.target, async () => { await recFetch(true); if (!BACK || !BACK.t.length) throw new Error(t('r_empty')); download(`simemu_rec_${stamp()}.csv`, dsToCSV(BACK), 'text/csv'); });
    $('r-file').onchange = e => {
      const file = e.target.files[0];
      if (!file) return;
      file.text().then(txt => { DSETS.push(parseCSV(txt, file.name)); buildDatasets(); drawPlay(); say(`CSV: ${file.name}`); }).catch(err => say(err.message, true));
      e.target.value = '';
    };
    $('r-sets').onchange = e => {
      const on = e.target.dataset.on, off = e.target.dataset.off;
      if (on != null) DSETS[+on].on = e.target.checked;
      if (off != null && Number.isFinite(+e.target.value)) DSETS[+off].off = +e.target.value;
      drawPlay();
    };
    $('r-sets').onclick = e => {
      const c = e.target.closest('[data-csv]'), r = e.target.closest('[data-rm]');
      if (c) { const d = DSETS[+c.dataset.csv]; download(`${d.name.replace(/[^\w.-]+/g, '_')}_${stamp()}.csv`, dsToCSV(d), 'text/csv'); }
      if (r) { DSETS.splice(+r.dataset.rm, 1); buildDatasets(); drawPlay(); }
    };
    chartNav($('r-canvas'), playSt, drawPlay, at => { playSt.playCursor = at; drawPlay(); });
    $('r-scrub').oninput = e => { const pd = playData(); if (pd) { playSt.playCursor = (+e.target.value / 1000) * pd.dur; drawPlay(); } };
    $('r-play').onclick = () => {
      playSt.playing = !playSt.playing;
      $('r-play').textContent = t(playSt.playing ? 'r_pause' : 'r_play');
      const pd = playData();
      if (playSt.playing && pd && (playSt.playCursor || 0) >= pd.dur) playSt.playCursor = 0;
      playLast = 0; cancelAnimationFrame(playRAF); playRAF = requestAnimationFrame(playTick);
    };
    HOOKS.push(s => {
      const rs = s.workstation && s.workstation.recorder;
      if (rs && rs.active && performance.now() - recFetchAt > 1000) { recFetchAt = performance.now(); recFetch(false).catch(e => say(e.message, true)); }
    });
  },
  show() { buildRecChips(); buildDatasets(); drawPlay(); },
  i18n() { buildRecChips(); if (BACK) BACK.name = t('ds_backend'); buildDatasets(); $('r-play').textContent = t(playSt.playing ? 'r_pause' : 'r_play'); },
  render(s) {
    const rs = (s.workstation && s.workstation.recorder) || {};
    const dur = isNum(rs.t_last_s) && isNum(rs.t_first_s) ? rs.t_last_s - rs.t_first_s : NaN;
    $('r-status').innerHTML = [[t('rs_active'), rs.active ? '● ' + t('yes') : t('no')], [t('rs_rows'), rs.rows], [t('rs_cap'), rs.capacity_rows], [t('rs_span'), f(dur, 2)],
      [t('rs_rate'), isNum(dur) && dur > 0 ? f((rs.rows - 1) / dur, 0) + ' Hz' : DASH], [t('rs_events'), rs.events]]
      .map(([a, b]) => `<div><span>${esc(a)}</span><b class="${a === t('rs_active') && rs.active ? 'bad' : ''}">${esc(b ?? DASH)}</b></div>`).join('');
    $('r-start').disabled = !!rs.active; $('r-stop').disabled = !rs.active; $('r-mark').disabled = !rs.active;
    if (rs.active && !playSt.playing) { if (BACK && $('r-sets').rows.length - 1 !== DSETS.length) buildDatasets(); drawPlay(); }
  },
});
