'use strict';
// SIMEMU Studio workstation UI — core. Shows only what the backend reports; anything missing is
// "—" with a reason. Every value inserted as HTML goes through esc().
const $ = id => document.getElementById(id);
const $$ = (sel, root = document) => Array.from(root.querySelectorAll(sel));
const DASH = '—';
const esc = s => String(s ?? '').replace(/[&<>"']/g, c => '&#' + c.charCodeAt(0) + ';');
const isNum = v => typeof v === 'number' && Number.isFinite(v);
const f = (v, d = 1) => (isNum(v) ? v.toFixed(d) : DASH);
const fmtN = v => (isNum(v) ? (Math.abs(v) < 1e-9 ? '0' : String(+v.toPrecision(5))) : DASH); // display rounding only
const hex = v => (typeof v === 'bigint' || isNum(v) ? '0x' + v.toString(16).toUpperCase() : DASH);
const clamp = (x, a, b) => Math.min(b, Math.max(a, x));
const LS = {
  get(k, def) { try { const v = localStorage.getItem('simemu.' + k); return v == null ? def : JSON.parse(v); } catch (e) { return def; } },
  set(k, v) { try { localStorage.setItem('simemu.' + k, JSON.stringify(v)); } catch (e) { /* storage full / disabled */ } },
};
const download = (name, data, type) => {
  const url = URL.createObjectURL(data instanceof Blob ? data : new Blob([data], { type }));
  const a = document.createElement('a');
  a.href = url; a.download = name;
  document.body.appendChild(a); a.click(); a.remove();
  setTimeout(() => URL.revokeObjectURL(url), 2000);
};
const stamp = () => new Date().toISOString().slice(0, 19).replace(/[:T]/g, '-');

// ---------- i18n ----------
const DEFAULT_LANG = (typeof window !== 'undefined' && window.__SIMEMU_DEFAULT_LANG === 'ru') ? 'ru' : 'en';
let LANG = LS.get('lang', DEFAULT_LANG) === 'ru' ? 'ru' : 'en';
const L = pair => (Array.isArray(pair) ? pair[LANG === 'ru' ? 0 : 1] : pair);
const T = {};
const t = k => (T[k] ? L(T[k]) : k);
Object.assign(T, {
  preset: ['Пресет', 'Preset'], alarms: ['Тревоги', 'Alarms'],
  pause: ['Пауза', 'Pause'], resume: ['Продолжить', 'Resume'], paused: ['пауза', 'paused'],
  online: ['связь', 'online'], offline: ['нет связи', 'offline'],
  sound_on: ['♪ вкл', '♪ on'], sound_off: ['♪ выкл', '♪ off'],
  asound_on: ['🔔', '🔔'], asound_off: ['🔕', '🔕'],
  rtf_tip: ['Скорость симуляции. Первое число — фактическая (прирост времени симуляции / время в браузере), второе — расчётная (время симуляции / время расчёта шага на сервере). 1× = реальное время.',
    'Simulation speed. First = actual (sim-time advance / browser wall time), second = compute (simulated time / server compute time of the step). 1× = real time.'],
  tip_conn: ['Связь с сервером эмулятора (каждые 50 мс браузер просит сервер продвинуть симуляцию и получает телеметрию).', 'Link to the emulator server (every 50 ms the browser asks the server to advance the simulation and receives telemetry).'],
  tip_h_ecu: ['Блок управления двигателем SIMOS18.1: загруженный образ прошивки и состояние процессора. Клик — раздел «Прошивки».', 'Engine ECU SIMOS18.1: loaded firmware image and CPU state. Click — Firmware view.'],
  tip_h_tcu: ['Блок управления коробкой (мехатроник DQ200 0CW или DQ250 0D9). ✓ — EEPROM DQ250 подготовлен самой прошивкой (кодирование, базовые установки). Клик — раздел «Прошивки», сервис КПП — «Диагностика».',
    'Gearbox control unit (DQ200 0CW or DQ250 0D9 mechatronic). ✓ — DQ250 EEPROM prepared by the firmware itself (coding, basic settings). Click — Firmware view; gearbox service — Diagnostics.'],
  tip_h_preset: ['Набор калибровок, применённый к образу ЭБУ (stock = заводской).', 'Calibration set applied to the ECU image (stock = factory).'],
  tip_h_time: ['Время симуляции, с (сбрасывается при сбросе физики).', 'Simulation time, s (resets with the plant).'],
  tip_h_engine: ['Состояние двигателя по физической модели. Клик — обзор с объяснением.', 'Engine state from the physics. Click — overview with explanations.'],
  tip_h_alarms: ['Каналы, вышедшие за пределы тревог. Клик — список каналов (только тревоги).', 'Channels outside their alarm limits. Click — channel list (alarms only).'],
  tip_h_safety: ['Защита испытательного бокса (оборудование стенда, не логика ЭБУ): ВЫКЛ / ВЗВЕДЕНА / СРАБОТАЛА. Клик — настройки на вкладке «Диностенд».',
    'Test-cell interlock (bench equipment, not ECU logic): OFF / ARMED / TRIPPED. Click — settings in the Dyno view.'],
  tip_h_rec: ['Регистратор данных (запись каналов каждые 10 мс на сервере). Клик — раздел «Запись».', 'Data recorder (server records channels every 10 ms). Click — Recorder view.'],
  tip_pause: ['Остановить/продолжить симуляцию (клавиша .)', 'Pause/resume the simulation (key .)'],
  tip_sound: ['Звук мотора, синтезированный из оборотов и вспышек в цилиндрах физической модели.', 'Engine sound synthesised from plant rpm and cylinder firing.'],
  tip_asound: ['Звуковой сигнал при новых тревогах (для каналов с включённым звуком).', 'Audible beep on new alarms (for channels with sound enabled).'],
  tip_dock: ['Показать/скрыть панель управления справа (клавиша \\)', 'Show/hide the control dock (key \\)'],
  tip_help: ['Горячие клавиши и справка (клавиша ?)', 'Keyboard shortcuts and help (key ?)'],
  working: ['Выполняется…', 'Working…'], done: ['Готово', 'Done'], cancel: ['Отмена', 'Cancel'], ok: ['OK', 'OK'], close: ['Закрыть', 'Close'],
  yes: ['да', 'yes'], no: ['нет', 'no'], none: ['нет', 'none'], all: ['все', 'all'], clear: ['Очистить', 'Clear'], apply: ['Применить', 'Apply'],
  add_channel: ['+ Канал', '+ Channel'], pick_channel: ['Выбор канала', 'Pick a channel'], pick_channels: ['Выбор каналов', 'Pick channels'],
  filter_ph: ['Фильтр: имя или ключ…', 'Filter: name or key…'],
  collapse: ['Свернуть/развернуть', 'Collapse/expand'], maximize: ['Развернуть на весь экран (Esc — назад)', 'Maximise (Esc to restore)'],
  // nav
  v_overview: ['Обзор', 'Overview'], v_gauges: ['Приборы', 'Gauges'], v_charts: ['Графики', 'Charts'], v_channels: ['Все каналы', 'All channels'],
  v_rec: ['Запись', 'Recorder'], v_dyno: ['Диностенд', 'Dyno'], v_health: ['Ресурс узлов', 'Health'], v_cal: ['Калибровка', 'Calibration'],
  v_diag: ['Диагностика', 'Diagnostics'], v_can: ['CAN-монитор', 'CAN monitor'], v_fw: ['Прошивки', 'Firmware'], v_setup: ['Настройка стенда', 'Bench setup'],
  // dock
  d_power: ['Питание и пуск', 'Power & start'], d_pedals: ['Педали', 'Pedals'], d_quick: ['Сводка', 'Quick look'],
  ign_on: ['Зажигание: ВКЛ', 'Ignition: ON'], ign_off: ['Зажигание: ВЫКЛ', 'Ignition: OFF'],
  start_btn: ['СТАРТ (держать)', 'START (hold)'],
  tip_ign: ['Клемма 15 (замок зажигания). Включает электронику; прошивки начинают работу. Клавиша I.', 'Terminal 15 (ignition switch). Powers the electronics; firmware starts working. Key I.'],
  tip_start: ['Запрос пуска (клемма 50). Держите, пока мотор не заведётся. Крутить ли стартер — решает прошивка ЭБУ, не интерфейс. Клавиша Пробел (держать).',
    'Start request (terminal 50). Hold until the engine runs. Whether the starter cranks is decided by the ECU firmware, not the UI. Key Space (hold).'],
  pedal: ['Газ', 'Pedal'], brake: ['Тормоз', 'Brake'],
  tip_pedal: ['Педаль акселератора, %. Прошивка сама решает, насколько открыть дроссель. W/↑ — 100 % пока удерживается. Справа — значение, принятое сервером.',
    'Accelerator pedal, %. The firmware decides how far to open the throttle. W/↑ holds 100 %. The number is what the server applied.'],
  tip_brake: ['Педаль тормоза, %. S/↓ — 100 % пока удерживается.', 'Brake pedal, %. S/↓ holds 100 %.'],
  selector: ['Селектор КПП', 'Gear selector'],
  tip_tiptronic: ['Типтроник +/− недоступен: в сигналах шины, которые эмулятор передаёт в коробку (ChassisInputs), нет сигнала подрулевых лепестков/качелек. Нужна доработка бэкенда.',
    'Tiptronic +/− unavailable: the bus signals the emulator sends to the gearbox (ChassisInputs) have no paddle/lever tip signal. Needs a backend addition.'],
  drive_profile: ['Профиль', 'Profile'], tip_drive_profile: ['Режим движения, передаваемый блокам по CAN (Charisma_01, значения не проверены).', 'Drive mode sent to the units over CAN (Charisma_01, values unverified).'],
  bench: ['Режим стенда', 'Bench mode'], road: ['Дорога (КПП подключена)', 'Road (gearbox connected)'], engine_dyno: ['Моторный стенд (вихретоковый тормоз)', 'Engine dyno (eddy-current absorber)'],
  ac_dyno: ['Моторный стенд (AC, 4 квадранта)', 'Engine dyno (AC, 4-quadrant)'], chassis_dyno: ['Барабанный стенд (весь автомобиль)', 'Chassis dyno (whole car)'],
  tip_bench_mode: ['Дорога — мотор через КПП крутит колёса, обдув радиатора и интеркулера от скорости хода (стоя — только вентилятор J293). Моторный стенд (вихретоковый / AC) — КПП снята, обдув бокса 90 км/ч, стендовый restbus нагружает ЭБУ без стояночной отсечки 3800 об/мин. Барабанный стенд — синхронизированные 4WD-барабаны, передачи включает прошивка КПП (включите D/S/M).',
    'Road — the engine drives the wheels through the gearbox; ram air from road speed (at standstill only the J293 fan). Engine dyno (eddy-current / AC) — gearbox removed, 90 km/h cell blower, testbed restbus loads the ECU without the 3800 rpm standstill limiter. Chassis dyno — linked 4WD rollers driven through the gearbox (select D/S/M).'],
  target_rpm: ['Цель, об/мин', 'Target rpm'], target_kmh: ['Цель, км/ч', 'Target km/h'],
  bench_ac: ['Сейчас: AC-стенд → {r} об/мин', 'Now: AC dyno → {r} rpm'], bench_chassis: ['Сейчас: барабанный стенд → {v} км/ч', 'Now: chassis dyno → {v} km/h'],
  keys_hint: ['Клавиши: W/↑ газ, S/↓ тормоз, Пробел — старт, I — зажигание, Shift+P/R/N/D/S/M — селектор, . — пауза, ? — справка.',
    'Keys: W/↑ pedal, S/↓ brake, Space start, I ignition, Shift+P/R/N/D/S/M selector, . pause, ? help.'],
  q_rpm: ['Обороты', 'Engine rpm'], q_tq: ['Момент, Нм', 'Torque, Nm'], q_speed: ['Скорость, км/ч', 'Speed, km/h'], q_gear: ['Передача', 'Gear'],
  q_state: ['Мотор', 'Engine'], q_map: ['MAP, кПа', 'MAP, kPa'],
  road_load_dyno: ['Барабанный стенд 4WD, дорожная нагрузка', '4WD chassis dyno, road load'],
  bench_rld: ['Сейчас: барабанный стенд 4WD, дорожная нагрузка (масса, аэродинамика, качение шин сборки)', 'Now: 4WD chassis dyno with road load (mass, drag, rolling resistance of the build)'],
  bench_road: ['Сейчас: дорога', 'Now: road'], bench_dyno: ['Сейчас: моторный стенд → {r} об/мин', 'Now: engine dyno → {r} rpm'],
  st_running: ['работает', 'running'], st_cranking: ['прокрутка', 'cranking'], st_stopped: ['остановлен', 'stopped'], st_spinning: ['вращается', 'spinning'], st_seized: ['ЗАКЛИНИЛ', 'SEIZED'],
  sf_off: ['Защита: ВЫКЛ', 'Interlock: OFF'], sf_armed: ['Защита: ВЗВЕДЕНА', 'Interlock: ARMED'], sf_tripped: ['Защита: СРАБОТАЛА', 'Interlock: TRIPPED'],
  rec_off: ['REC выкл', 'REC off'],
  help_title: ['Справка и горячие клавиши', 'Help & keyboard shortcuts'],
  help_body: [
    '<p>SIMEMU Studio — рабочее место виртуального моторного стенда. Настоящие прошивки ЭБУ SIMOS18.1 и КПП DQ200 выполняются в эмуляторе процессора, а физическая модель мотора и трансмиссии отвечает на их команды. Интерфейс ничего не выдумывает: всё, что показано, пришло с сервера; «—» значит «нет данных», причина — в подсказке.</p>',
    '<p>SIMEMU Studio is a virtual dyno workstation. The real SIMOS18.1 ECU and DQ200 TCU firmware run in a CPU emulator and the engine/driveline physics answers their commands. The UI invents nothing: everything shown comes from the server; "—" means "no data", the reason is in the tooltip.</p>'],
  help_keys: ['Клавиши', 'Keys'],
  log: ['Журнал действий', 'Action log'],
  confirm_title: ['Подтверждение', 'Confirm'],
  unknown_ch: ['Канал есть в телеметрии, но в реестре интерфейса для него нет описания. Название и единица угаданы по имени поля.', 'The channel is in the telemetry but the UI registry has no description for it. Name and unit are guessed from the field name.'],
  na_generic: ['нет значения (поле отсутствует или null в телеметрии)', 'no value (field missing or null in telemetry)'],
  limits: ['Пределы тревоги', 'Alarm limits'], range: ['Типичный диапазон', 'Typical range'],
});

// ---------- API ----------
// ECM3 checksums are 64-bit: keep them exact as BigInt where the browser exposes the JSON source text.
const parseJSON = text => JSON.parse(text, (k, v, ctx) => ((k === 'stored' || k === 'computed') && ctx && ctx.source ? BigInt(ctx.source) : v));
async function api(path, body) {
  const opt = body === undefined ? {} : { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) };
  const res = await fetch(path, opt);
  const text = await res.text();
  let data = null;
  try { data = parseJSON(text); } catch (e) { if (res.ok) throw new Error(`${path}: invalid JSON`); }
  if (!res.ok) throw new Error(`${path}: ${(data && data.error) || res.status + ' ' + res.statusText}`);
  return data;
}

// ---------- messages / log ----------
const LOGS = [];
function say(text, isErr) {
  if (!text) { $('foot-msg').textContent = ''; return; }
  const simT = S && S.plant ? S.plant.sim_time_s : NaN;
  LOGS.push({ at: new Date(), sim: simT, text, err: !!isErr });
  if (LOGS.length > 500) LOGS.shift();
  $('foot-msg').textContent = `${new Date().toLocaleTimeString()} — ${text}`;
  $('foot-msg').className = 'msg' + (isErr ? ' err' : '');
  if (CUR_VIEW === 'setup') renderLog();
}
function renderLog() {
  $('ui-log').innerHTML = '<tr><th>time</th><th>t sim, s</th><th></th></tr>' + LOGS.slice().reverse().map(l =>
    `<tr><td>${esc(l.at.toLocaleTimeString())}</td><td class="r">${f(l.sim, 2)}</td><td class="${l.err ? 'bad' : ''}" style="white-space:normal">${esc(l.text)}</td></tr>`).join('');
}
// Runs a user action with the button disabled; reports errors in the footer.
async function act(btn, fn) {
  if (btn) btn.disabled = true;
  $('foot-msg').textContent = t('working');
  try { await fn(); } catch (e) { say(e.message, true); } finally { if (btn) btn.disabled = false; }
}

// ---------- modal ----------
let modalClose = null;
function modal(title, html, buttons = [], onClose) {
  $('m-title').textContent = title;
  $('m-body').innerHTML = html;
  $('m-foot').innerHTML = '';
  const close = v => { $('modal').classList.remove('on'); modalClose = null; if (onClose) onClose(v); };
  for (const b of buttons) {
    const el = document.createElement('button');
    el.textContent = b.label;
    if (b.cls) el.className = b.cls;
    el.onclick = () => { if (!b.fn || b.fn() !== false) close(b.value); };
    $('m-foot').appendChild(el);
  }
  modalClose = close;
  $('modal').classList.add('on');
  return close;
}
$('m-x').onclick = () => modalClose && modalClose(undefined);
$('modal').onclick = e => { if (e.target.id === 'modal' && modalClose) modalClose(undefined); };
const confirmBox = (text, okLabel, danger = true) => new Promise(res => {
  modal(t('confirm_title'), `<p>${esc(text)}</p>`, [
    { label: t('cancel'), value: false }, { label: okLabel || t('ok'), cls: danger ? 'danger big' : 'primary big', value: true }], v => res(!!v));
});
const promptBox = (title, label, value) => new Promise(res => {
  let out = null;
  modal(title, `<div class="row"><label>${esc(label)}</label><input id="m-in" type="text" style="flex:1" value="${esc(value || '')}"></div>`, [
    { label: t('cancel') }, { label: t('ok'), cls: 'primary', fn: () => { out = $('m-in').value; } }], () => res(out));
  setTimeout(() => { const i = $('m-in'); if (i) { i.focus(); i.select(); i.onkeydown = e => { if (e.key === 'Enter') { out = i.value; modalClose(); } }; } }, 0);
});

// ---------- channel flattening, history, statistics, alarms ----------
// Channel key = JSON path into /api/state, e.g. plant.rpm, plant.clutch_temp_c[1], ecu.irq_count.
const ROOTS = ['plant', 'driver', 'frame', 'ecu', 'tcu', 'immo'];
function flatten(s) {
  const out = new Map();
  const walk = (pfx, v, depth) => {
    if (v === null || v === undefined) out.set(pfx, NaN);
    else if (typeof v === 'number') out.set(pfx, v);
    else if (typeof v === 'boolean') out.set(pfx, v ? 1 : 0);
    else if (typeof v === 'string') out.set(pfx, v);
    else if (Array.isArray(v)) { if (v.length <= 32) v.forEach((x, i) => walk(`${pfx}[${i}]`, x, depth + 1)); }
    else if (typeof v === 'object' && depth < 5) for (const k in v) walk(`${pfx}.${k}`, v[k], depth + 1);
  };
  for (const r of ROOTS) if (s[r] !== undefined) walk(r, s[r], 0);
  return out;
}
const numKeys = () => Array.from(FLAT.keys()).filter(k => typeof FLAT.get(k) === 'number');

const HCAP = 6000; // 5 min at 20 Hz of UI-side history (charts); full-rate data is the recorder's job
const H = { n: 0, t: new Float64Array(HCAP), ch: new Map() };
function histClear() { H.n = 0; H.ch.clear(); }
function histPush(flat, tSim) {
  if (!isNum(tSim)) return;
  const last = H.n ? H.t[(H.n - 1) % HCAP] : NaN;
  if (H.n && tSim < last) { histClear(); STATS.clear(); }
  else if (H.n && tSim === last) return; // paused: no new sample
  const i = H.n % HCAP;
  H.t[i] = tSim;
  for (const [k, v] of flat) {
    if (typeof v !== 'number') continue;
    let a = H.ch.get(k);
    if (!a) { a = new Float64Array(HCAP).fill(NaN); H.ch.set(k, a); }
    a[i] = v;
  }
  for (const [k, a] of H.ch) if (!flat.has(k)) a[i] = NaN;
  H.n++;
  for (const [k, v] of flat) {
    if (!isNum(v)) continue;
    let st = STATS.get(k);
    if (!st) STATS.set(k, st = { min: v, max: v, sum: 0, n: 0 });
    st.min = Math.min(st.min, v); st.max = Math.max(st.max, v); st.sum += v; st.n++;
  }
}
// Index range [i0, i1) of history samples (oldest → newest) as absolute counters.
const histRange = () => [Math.max(0, H.n - HCAP), H.n];
const histT = j => H.t[j % HCAP];
const histV = (k, j) => { const a = H.ch.get(k); return a ? a[j % HCAP] : NaN; };
const STATS = new Map();

// Alarms: limits are in display units. User overrides persist; defaults come from the registry.
let ALIM = LS.get('alarms', {});
const limVal = x => (typeof x === 'function' ? (() => { try { return x(S); } catch (e) { return undefined; } })() : x);
function limitsOf(key) {
  const m = meta(key), a = m.alarm || {}, u = ALIM[key];
  const pick = n => (u && n in u ? u[n] : limVal(a[n]));
  return { lo: pick('lo'), hi: pick('hi'), on: u && 'on' in u ? !!u.on : !!m.alarm, snd: u && 'snd' in u ? !!u.snd : !!a.snd, when: a.when };
}
function alarmSide(key, raw) {
  if (!isNum(raw)) return 0;
  const l = limitsOf(key);
  if (!l.on) return 0;
  if (l.when && !limVal(l.when)) return 0;
  const v = dv(key, raw);
  if (isNum(l.hi) && v > l.hi) return 2;
  if (isNum(l.lo) && v < l.lo) return 1;
  return 0;
}
const ACTIVE = new Map(); // key -> {since, side}
let ASOUND = LS.get('asound', true), lastBeep = 0;
function alarmsUpdate() {
  const now = S && S.plant ? S.plant.sim_time_s : 0;
  const seen = new Set();
  for (const [k, v] of FLAT) {
    if (typeof v !== 'number') continue;
    const side = alarmSide(k, v);
    if (!side) continue;
    seen.add(k);
    if (!ACTIVE.has(k)) {
      ACTIVE.set(k, { since: now, side });
      const l = limitsOf(k);
      say(`⚠ ${chName(k)} = ${fmtCh(k, v)} ${meta(k).unit || ''} (${side === 2 ? '>' : '<'} ${fmtN(side === 2 ? l.hi : l.lo)})`, true);
      if (l.snd && ASOUND) beep();
    }
  }
  for (const k of Array.from(ACTIVE.keys())) if (!seen.has(k)) ACTIVE.delete(k);
}

// ---------- audio: alarm beep (UI signal). Engine / driveline sound: sound.js (physical synthesizer). ----------
let AC = null;
function audioCtx() { if (!AC) AC = new AudioContext(); return AC; }
function beep() {
  const now = performance.now();
  if (now - lastBeep < 800) return;
  lastBeep = now;
  try {
    const ac = audioCtx(), o = ac.createOscillator(), g = ac.createGain();
    o.type = 'square'; o.frequency.value = 880; g.gain.value = 0.06;
    o.connect(g).connect(ac.destination); o.start(); o.stop(ac.currentTime + 0.15);
  } catch (e) { /* audio blocked until a user gesture */ }
}

// ---------- views / navigation / panels ----------
const ORDER = [['overview', '◉', '1'], ['gauges', '◔', '2'], ['charts', '∿', '3'], ['channels', '☰', '4'], ['rec', '●', '5'], ['dyno', '⚙', '6'],
  ['health', '♥', '7'], ['cal', '▦', '8'], ['diag', '⚕', '9'], ['can', '⇄', '0'], ['fw', '⬢', '-'], ['check', '✓', 'c'], ['setup', '⚒', '='], ['build', '⛭', 'b'], ['sound', '♪', 'a']];
const VIEWS = {};
const view = def => { VIEWS[def.id] = def; };
let CUR_VIEW = LS.get('view', 'overview');
function buildNav() {
  $('nav').innerHTML = ORDER.map(([id, ico, key], i) => (i === 5 || i === 8 ? '<hr>' : '')
    + `<button data-view="${id}" data-tip="v_${id}"><span class="ico">${ico}</span><span data-i18n="v_${id}"></span><span class="badge" id="nb-${id}" hidden></span><span class="key">${key}</span></button>`).join('');
  $('nav').onclick = e => { const b = e.target.closest('[data-view]'); if (b) go(b.dataset.view); };
}
function go(id) {
  if (!VIEWS[id]) id = 'overview';
  CUR_VIEW = id;
  LS.set('view', id);
  $$('#nav button').forEach(b => b.classList.toggle('on', b.dataset.view === id));
  $$('#main > .view').forEach(v => v.classList.toggle('on', v.dataset.view === id));
  const v = VIEWS[id];
  if (v.show) try { v.show(); } catch (e) { console.error(e); }
  if (S && v.render) try { v.render(S); } catch (e) { console.error(e); }
}
function initPanels() {
  const col = LS.get('collapsed', {});
  for (const p of $$('.panel')) {
    const h = p.querySelector(':scope > h3');
    if (!h) continue;
    if (h.dataset.i18n) { h.innerHTML = `<span data-i18n="${esc(h.dataset.i18n)}"></span>`; delete h.dataset.i18n; }
    const b = document.createElement('span');
    b.className = 'pbtn';
    b.innerHTML = '<span data-act="col" data-tip="collapse">▾</span><span data-act="max" data-tip="maximize">⤢</span>';
    h.appendChild(b);
    if (col[p.id]) p.classList.add('collapsed');
    b.onclick = e => {
      const a = e.target.dataset.act;
      if (a === 'col') { p.classList.toggle('collapsed'); const c = LS.get('collapsed', {}); c[p.id] = p.classList.contains('collapsed'); LS.set('collapsed', c); }
      if (a === 'max') { p.classList.toggle('max'); if (S) go(CUR_VIEW); }
    };
  }
}

// ---------- tooltips ----------
const tip = $('tip');
function chTipHTML(k) {
  const m = meta(k), v = FLAT.get(k), l = limitsOf(k), st = STATS.get(k);
  const r = m.r ? `${t('range')}: ${fmtN(m.r[0])} … ${fmtN(m.r[1])} ${m.unit || ''}` : '';
  const lim = l.on && (isNum(l.lo) || isNum(l.hi)) ? `${t('limits')}: ${isNum(l.lo) ? '< ' + fmtN(l.lo) : ''} ${isNum(l.hi) ? '> ' + fmtN(l.hi) : ''}` : '';
  const na = typeof v === 'number' && !isNum(v) ? `<div class="warn">${esc(L(m.na || T.na_generic))}</div>` : '';
  return `<b>${esc(chName(k))}</b> <span class="muted">${esc(m.unit || '')}</span><div class="k">${esc(k)}</div>`
    + `<div class="d">${esc(L(m.desc))}</div>${m.auto ? `<div class="d warn">${esc(t('unknown_ch'))}</div>` : ''}${na}`
    + `<div class="d mono">= ${esc(fmtCh(k, v))} ${esc(m.unit || '')}${st ? ` · min ${fmtN(dv(k, st.min))} · max ${fmtN(dv(k, st.max))}` : ''}</div>`
    + (r ? `<div class="d">${esc(r)}</div>` : '') + (lim ? `<div class="d">${esc(lim)}</div>` : '');
}
function showTip(el) {
  let html = '';
  if (el.dataset.ch) html = chTipHTML(el.dataset.ch);
  else if (el.dataset.tipraw) html = esc(el.dataset.tipraw);
  else if (el.dataset.tip) html = esc(t(el.dataset.tip));
  if (!html) return hideTip();
  tip.innerHTML = html;
  tip.style.display = 'block';
}
function hideTip() { tip.style.display = 'none'; tipEl = null; }
let tipEl = null;
document.addEventListener('mouseover', e => {
  const el = e.target.closest && e.target.closest('[data-tip],[data-ch],[data-tipraw]');
  if (el === tipEl) return;
  tipEl = el;
  if (el) showTip(el); else hideTip();
});
document.addEventListener('mousemove', e => {
  if (tip.style.display !== 'block') return;
  const w = tip.offsetWidth, h = tip.offsetHeight;
  tip.style.left = Math.min(innerWidth - w - 8, e.clientX + 14) + 'px';
  tip.style.top = (e.clientY + 16 + h > innerHeight ? e.clientY - h - 10 : e.clientY + 16) + 'px';
});

// ---------- channel picker ----------
function pickChannels(title, multi, preselected = []) {
  return new Promise(res => {
    const chosen = new Set(preselected);
    let result = null;
    const draw = () => {
      const q = ($('pk-q') ? $('pk-q').value : '').toLowerCase();
      const keys = numKeys().filter(k => !q || k.toLowerCase().includes(q) || chName(k).toLowerCase().includes(q));
      const groups = groupKeys(keys);
      $('pk-list').innerHTML = groups.map(([g, ks]) => `<div class="grp">${esc(groupName(g))}</div>` + ks.map(k =>
        `<div class="it${chosen.has(k) ? ' sel' : ''}" data-k="${esc(k)}" data-ch="${esc(k)}">${multi ? `<span><input type="checkbox"${chosen.has(k) ? ' checked' : ''}> ` : '<span>'}${esc(chName(k))} <small>${esc(k)}</small></span><small>${esc(fmtCh(k, FLAT.get(k)))} ${esc(meta(k).unit || '')}</small></div>`).join('')).join('')
        || `<div class="muted">${DASH}</div>`;
    };
    modal(title, `<input id="pk-q" type="search" style="width:100%" placeholder="${esc(t('filter_ph'))}"><div id="pk-list" class="picklist"></div>`,
      multi ? [{ label: t('cancel') }, { label: t('ok'), cls: 'primary', fn: () => { result = Array.from(chosen); } }] : [{ label: t('cancel') }],
      () => { hideTip(); res(result); });
    $('pk-q').oninput = draw;
    $('pk-list').onclick = e => {
      const it = e.target.closest('[data-k]');
      if (!it) return;
      const k = it.dataset.k;
      if (!multi) { result = k; modalClose(); return; }
      if (chosen.has(k)) chosen.delete(k); else chosen.add(k);
      draw();
    };
    draw();
    $('pk-q').focus();
  });
}
const pickChannel = title => pickChannels(title || t('pick_channel'), false);

// Alarm-limit editor for one channel (display units).
function editLimits(k) {
  const l = limitsOf(k), m = meta(k);
  modal(`${t('limits')}: ${chName(k)}`, `<div class="k mono muted">${esc(k)}</div>
    <div class="row"><label><input id="al-on" type="checkbox"${l.on ? ' checked' : ''}> ${esc(L(['включена', 'enabled']))}</label>
    <label><input id="al-snd" type="checkbox"${l.snd ? ' checked' : ''}> ${esc(L(['звук', 'sound']))}</label></div>
    <div class="row"><label>${esc(L(['Нижний предел', 'Low limit']))}</label><input id="al-lo" type="number" step="any" value="${isNum(l.lo) ? l.lo : ''}">
    <label>${esc(L(['Верхний предел', 'High limit']))}</label><input id="al-hi" type="number" step="any" value="${isNum(l.hi) ? l.hi : ''}"> ${esc(m.unit || '')}</div>
    <div class="note">${esc(L(['Пустое поле — предел не проверяется. Тревога — только сигнал на экране и звук; на стенд не влияет (для этого есть защита бокса на вкладке «Диностенд»).',
    'Empty = not checked. An alarm is only a visual/audible signal; it does not act on the bench (that is the test-cell interlock in the Dyno view).']))}</div>
    ${l.when ? `<div class="note">${esc(L(['Условие проверки задано в реестре (например, только при работающем моторе).', 'A check condition is defined in the registry (e.g. only while running).']))}</div>` : ''}`,
  [{ label: L(['По умолчанию', 'Defaults']), fn: () => { delete ALIM[k]; LS.set('alarms', ALIM); } }, { label: t('cancel') },
    { label: t('apply'), cls: 'primary', fn: () => {
      const num = id => { const v = $(id).value.trim(); return v === '' || !Number.isFinite(+v) ? null : +v; };
      ALIM[k] = { on: $('al-on').checked, snd: $('al-snd').checked, lo: num('al-lo'), hi: num('al-hi') };
      LS.set('alarms', ALIM);
    } }]);
}

// ---------- co-simulation loop ----------
const STEP_MS = 50;
let S = null, FLAT = new Map(), CONNECTED = false;
let lastStep = performance.now();
const pending = {}; // one-shot fields merged into the next /api/step
const keys = new Set();
let startHeld = false, startLatch = false;
let wasUp = false, wasDown = false, wasStart = false, lastOverride = null;
const eff = { t: 0, sim: 0, x: NaN }; // effective sim speed measured here (EMA)
const HOOKS = []; // fn(s, prev) run on every new state (before rendering)
const pedalOverride = { v: null }; // dyno step sequencer drives the pedal through this

async function loop() {
  const t0 = performance.now();
  const up = keys.has('up'), down = keys.has('down'), start = startHeld || startLatch || keys.has('start');
  const body = Object.assign({
    dt_sec: Math.min(0.1, (t0 - lastStep) / 1000), // measured wall-clock dt; the backend clamps to 0.01..0.1
  }, pending);
  if (up) body.pedal_pct = 100;
  else if (pedalOverride.v != null) body.pedal_pct = pedalOverride.v;
  else if (wasUp || lastOverride != null) body.pedal_pct = +$('pedal').value;
  if (down) body.brake_pct = 100;
  else if (wasDown) body.brake_pct = +$('brake').value;
  if (start) body.start_button = true;
  else if (wasStart) body.start_button = false;
  wasUp = up; wasDown = down; wasStart = start; lastOverride = pedalOverride.v;
  if (S && S.dyno && S.dyno.running) delete body.pedal_pct; // the bench sweep program holds the pedal; do not fight it
  lastStep = t0;
  for (const k in pending) delete pending[k];
  startLatch = false;
  let wait = STEP_MS;
  try {
    const s = await api('/api/step', body);
    const now = performance.now(), sim = s.plant && s.plant.sim_time_s;
    if (eff.t && isNum(sim) && sim >= eff.sim && !s.paused) {
      const x = (sim - eff.sim) / ((now - eff.t) / 1000);
      eff.x = isNum(eff.x) ? 0.8 * eff.x + 0.2 * x : x;
    }
    eff.t = now; eff.sim = sim;
    onState(s);
  } catch (e) {
    if (CONNECTED) say(`${t('offline')}: ${e.message}`, true);
    CONNECTED = false;
    renderConn();
    wait = 1000;
  }
  setTimeout(loop, Math.max(0, wait - (performance.now() - t0)));
}

let firstState = true;
function onState(s) {
  const prev = S;
  S = s;
  CONNECTED = true;
  FLAT = flatten(s);
  histPush(FLAT, s.plant && s.plant.sim_time_s);
  alarmsUpdate();
  for (const h of HOOKS) try { h(s, prev); } catch (e) { console.error(e); }
  if (firstState) { firstState = false; for (const v of Object.values(VIEWS)) if (v.first) try { v.first(s); } catch (e) { console.error(e); } }
  renderTop(s);
  renderDock(s);
  const v = VIEWS[CUR_VIEW];
  if (v && v.render) try { v.render(s); } catch (e) { console.error(e); }
}

// ---------- top bar ----------
function lamp(el, cls, blink) { el.className = 'lamp' + (cls ? ' ' + cls : '') + (blink ? ' blink' : ''); }
function renderConn() {
  lamp($('h-conn'), CONNECTED ? 'ok' : 'bad', !CONNECTED);
  $('h-conn-t').textContent = t(CONNECTED ? 'online' : 'offline');
}
function boardLamp(b) { return !b || !b.present ? '' : b.error ? 'bad' : b.warm_resets > 0 ? 'warn' : 'ok'; }
const stName = st => (T['st_' + st] ? t('st_' + st) : st || DASH);
function renderTopButtons() {
  $('b-sound').textContent = t(SND.on ? 'sound_on' : 'sound_off');
  $('b-asound').textContent = t(ASOUND ? 'asound_on' : 'asound_off');
}
function renderTop(s) {
  renderConn();
  const p = s.plant || {};
  lamp($('h-ecu-l'), ALIVE.ecu.stalled ? 'bad' : boardLamp(s.ecu), !!ALIVE.ecu.stalled);
  // Chip: box code + post-programming mark + immobiliser binding; the file name and all load notes are in the tooltip
  // and on the Firmware page.
  $('h-ecu').textContent = s.image_label ? s.image_label.replace(/ \([^)]*\.(?:bin|frf|odx)\)/i, '').replace('completed after programming', L(['дозаписан после программирования', 'completed after programming'])) : DASH;
  $('h-ecu').parentElement.dataset.tipraw = [t('tip_h_ecu'), s.image_label || ''].concat(s.image_notes || []).join('\n');
  // TCU chip: software id + the DQ250 mechatronic preparation (studio::dq250_prep); its notes in the tooltip.
  const pr = s.tcu_prep || {}, rec = pr.record;
  lamp($('h-tcu-l'), ALIVE.tcu.stalled ? 'warn' : pr.running ? 'warn' : rec && !rec.ok ? 'warn' : boardLamp(s.tcu), !!pr.running);
  $('h-tcu').textContent = (s.tcu && s.tcu.present ? (s.tcu.sw_id || DASH).replace(/\s+\S+$/, '') : L(['нет', 'absent']))
    + (pr.running ? ` · ${L(['подготовка', 'preparing'])} ${f(100 * ((pr.progress && pr.progress.frac) || 0), 0)} %` : rec ? (rec.ok ? ' · ✓' : ' · ⚠') : '');
  $('h-tcu').parentElement.dataset.tipraw = [t('tip_h_tcu')].concat(pr.running && pr.progress ? [`${L(['Сейчас', 'Now'])}: ${pr.progress.step}`] : [], s.tcu_notes || []).join('\n');
  $('h-preset').textContent = s.active_preset || DASH;
  $('h-time').textContent = f(p.sim_time_s, 2);
  const rtf = s.frame && s.frame.realtime_factor;
  $('h-rtf').textContent = s.paused ? t('paused') : `${isNum(eff.x) ? eff.x.toFixed(2) + '×' : DASH} / ${isNum(rtf) && rtf > 0 ? rtf.toFixed(2) + '×' : DASH}`;
  $('h-rtf').className = 'mono ' + (!s.paused && isNum(eff.x) && eff.x < 0.95 ? 'warn' : '');
  const pc = s.frame && s.frame.pacing;
  $('h-rtf').parentElement.dataset.tipraw = t('rtf_tip') + (pc ? ' ' + L([`Темп сервера: задано ${f(pc.speed, 2)}×, по часам ${f(pc.rtf, 2)}×, отставание ${f(pc.lag_s, 3)} с, сброшено отставания ${f(pc.slip_s, 2)} с, перегрузок шага ${pc.overruns}.`,
    `Server pacing: requested ${f(pc.speed, 2)}×, wall-clock ${f(pc.rtf, 2)}×, lag ${f(pc.lag_s, 3)} s, dropped lag ${f(pc.slip_s, 2)} s, step overruns ${pc.overruns}.`]) : '');
  const st = p.engine_state;
  lamp($('h-eng-l'), st === 'running' ? 'ok' : st === 'seized' ? 'bad' : st === 'cranking' ? 'warn' : '', st === 'seized');
  $('h-eng').textContent = stName(st);
  $('h-alarms').textContent = ACTIVE.size;
  lamp($('h-alarm-l'), ACTIVE.size ? 'bad' : 'ok', ACTIVE.size > 0);
  const nb = $('nb-channels');
  nb.hidden = !ACTIVE.size; nb.textContent = ACTIVE.size;
  const ws = s.workstation || {}, sf = ws.safety;
  if (sf && sf.limits) {
    const tripped = !!sf.trip, armed = sf.limits.enabled;
    lamp($('h-safe-l'), tripped ? 'bad' : armed ? 'ok' : '', tripped);
    $('h-safe').textContent = t(tripped ? 'sf_tripped' : armed ? 'sf_armed' : 'sf_off');
  } else { lamp($('h-safe-l'), ''); $('h-safe').textContent = DASH; }
  const rc = ws.recorder;
  if (rc && rc.active) {
    const dur = isNum(rc.t_last_s) && isNum(rc.t_first_s) ? rc.t_last_s - rc.t_first_s : 0;
    $('h-rec').innerHTML = `<span class="rec-dot">●</span> REC ${esc(f(dur, 1))} s`;
  } else $('h-rec').textContent = rc ? `${t('rec_off')}${rc.rows ? ` (${rc.rows})` : ''}` : DASH;
  $('b-pause').textContent = t(s.paused ? 'resume' : 'pause');
  $('b-pause').classList.toggle('on', !!s.paused);
  $('foot-stat').textContent = `ECU PC ${s.ecu ? s.ecu.pc : DASH} ${s.ecu ? s.ecu.region : ''} · IRQ ${s.ecu ? s.ecu.irq_count : DASH} · TCU PC ${s.tcu && s.tcu.present ? s.tcu.pc : DASH}`;
}

// ---------- control dock ----------
function renderDock(s) {
  const d = s.driver || {}, p = s.plant || {};
  $('b-ign').textContent = t(d.ignition_on ? 'ign_on' : 'ign_off');
  $('b-ign').classList.toggle('on', !!d.ignition_on);
  $('b-start').classList.toggle('held', !!d.start_button);
  $('pedal-v').textContent = f(d.pedal_pct, 0);
  $('brake-v').textContent = f(d.brake_pct, 0);
  if (pedalOverride.v != null) $('pedal').value = pedalOverride.v;
  else if (document.activeElement !== $('pedal') && !keys.has('up') && isNum(d.pedal_pct)) $('pedal').value = d.pedal_pct;
  if (document.activeElement !== $('brake') && !keys.has('down') && isNum(d.brake_pct)) $('brake').value = d.brake_pct;
  if (document.activeElement !== $('drive-profile') && isNum(s.drive_profile)) $('drive-profile').value = s.drive_profile;
  $$('#sel button').forEach(b => b.classList.toggle('on', b.textContent === d.selector));
  const [bm, bt, bnow] = benchOf(s.bench), bsig = JSON.stringify(s.bench);
  if (bsig !== renderDock.bsig) { renderDock.bsig = bsig; if (document.activeElement !== $('bench-mode')) $('bench-mode').value = bm; if (bt != null && document.activeElement !== $('bench-rpm')) $('bench-rpm').value = bt; }
  benchLabel();
  $('bench-now').textContent = bnow;
  const gear = g => (g === -1 ? 'R' : g > 0 ? String(g) : g === 0 ? 'N' : DASH);
  const q = [[t('q_state'), stName(p.engine_state)], [t('q_rpm'), f(p.rpm, 0)], [t('q_tq'), f(p.torque_brake_nm, 0)], [t('q_map'), f(p.map_kpa, 0)],
    [t('q_speed'), f(p.vehicle_kmh, 1)], [t('q_gear'), gear(p.driving_gear)]];
  $('quick').innerHTML = q.map(([a, b]) => `<span>${esc(a)}</span><b>${esc(b)}</b>`).join('');
}
// Bench mode of the state → [selector value, target, "now" text].
function benchOf(b) {
  const x = b && typeof b === 'object' ? b : {};
  if (x.EngineDyno) return ['engine_dyno', x.EngineDyno.target_rpm, t('bench_dyno').replace('{r}', f(x.EngineDyno.target_rpm, 0))];
  if (x.AcDyno) return ['ac_dyno', x.AcDyno.target_rpm, t('bench_ac').replace('{r}', f(x.AcDyno.target_rpm, 0))];
  if (x.ChassisDyno) return ['chassis_dyno', x.ChassisDyno.target_kmh, t('bench_chassis').replace('{v}', f(x.ChassisDyno.target_kmh, 0))];
  if (b === 'RoadLoadDyno') return ['road_load_dyno', null, t('bench_rld')];
  return ['road', null, b === 'Road' ? t('bench_road') : JSON.stringify(b ?? DASH)];
}
const benchLabel = () => { $('bench-tl').textContent = t($('bench-mode').value === 'chassis_dyno' ? 'target_kmh' : 'target_rpm'); };
function initDock() {
  $('sel').innerHTML = 'PRNDSM'.split('').map(x => `<button data-tipraw="${x}">${x}</button>`).join('');
  $('sel').onclick = e => { if (e.target.tagName === 'BUTTON') pending.selector = e.target.textContent; };
  $('b-ign').onclick = () => { if (S) pending.ignition_on = !S.driver.ignition_on; };
  const sb = $('b-start');
  sb.onpointerdown = e => { startHeld = startLatch = true; sb.setPointerCapture(e.pointerId); };
  sb.onpointerup = sb.onpointercancel = () => { startHeld = false; };
  sb.onclick = e => { if (!e.detail) startLatch = true; }; // keyboard activation: one-step tap
  $('drive-profile').onchange = e => { pending.drive_profile = +e.target.value; };
  $('b-bench').onclick = () => {
    const m = $('bench-mode').value, v = +$('bench-rpm').value;
    pending.bench_mode = m;
    if (Number.isFinite(v)) pending[m === 'chassis_dyno' ? 'dyno_target_kmh' : 'dyno_target_rpm'] = v;
  };
  $('bench-mode').onchange = () => { $('bench-rpm').value = $('bench-mode').value === 'chassis_dyno' ? 0 : 2000; benchLabel(); };
  $('pedal').oninput = $('pedal').onchange = () => { pedalOverride.v = null; pending.pedal_pct = +$('pedal').value; };
  $('brake').oninput = $('brake').onchange = () => { pending.brake_pct = +$('brake').value; };
}
function syncDock(s) {
  const set = (id, v) => { if (v != null) $(id).value = v; };
  set('drive-profile', s.drive_profile);
  set('pedal', s.driver && s.driver.pedal_pct);
  set('brake', s.driver && s.driver.brake_pct);
  const [bm, bt] = benchOf(s.bench);
  set('bench-mode', bm);
  set('bench-rpm', bt);
}

// ---------- keyboard ----------
const inField = e => e.target.closest && e.target.closest('input, select, textarea, [contenteditable]');
addEventListener('keydown', e => {
  if (e.key === 'Escape') {
    if (modalClose) return modalClose(undefined);
    const mx = document.querySelector('.panel.max');
    if (mx) { mx.classList.remove('max'); if (S) go(CUR_VIEW); }
    return;
  }
  if (modalClose || inField(e) || e.ctrlKey || e.metaKey || e.altKey) return;
  const v = VIEWS[CUR_VIEW];
  if (v && v.key && v.key(e)) { e.preventDefault(); return; }
  const nav = ORDER.find(o => o[2] === e.key);
  if (nav && !e.shiftKey) { go(nav[0]); e.preventDefault(); return; }
  if (e.shiftKey && 'PRNDSM'.includes(e.key.toUpperCase()) && e.code.startsWith('Key')) { pending.selector = e.key.toUpperCase(); e.preventDefault(); return; }
  const hold = { KeyW: 'up', ArrowUp: 'up', KeyS: 'down', ArrowDown: 'down', Space: 'start' }[e.code];
  if (hold) { keys.add(hold); e.preventDefault(); return; }
  if (e.code === 'KeyI' && !e.repeat) $('b-ign').click();
  else if (e.key === '.') $('b-pause').click();
  else if (e.key === '?') $('b-help').click();
  else if (e.key === '\\') $('b-dock').click();
  else if (e.code === 'KeyR' && e.shiftKey && typeof recToggle === 'function') recToggle();
  else if (e.code === 'KeyM' && typeof recMark === 'function') recMark();
  else return;
  e.preventDefault();
});
addEventListener('keyup', e => { keys.delete({ KeyW: 'up', ArrowUp: 'up', KeyS: 'down', ArrowDown: 'down', Space: 'start' }[e.code]); });
addEventListener('blur', () => keys.clear());

function showHelp() {
  const k = [['1 … 9, 0, -, =, B, A', L(['разделы (по порядку в левом меню)', 'views (in left-menu order)'])], ['W / ↑', L(['газ 100 % (удерживать)', 'pedal 100 % (hold)'])],
    ['S / ↓', L(['тормоз 100 % (удерживать)', 'brake 100 % (hold)'])], [L(['Пробел', 'Space']), L(['СТАРТ (удерживать)', 'START (hold)'])], ['I', L(['зажигание', 'ignition'])],
    ['Shift + P R N D S M', L(['селектор КПП', 'gear selector'])], ['.', L(['пауза', 'pause'])], ['\\', L(['панель управления', 'control dock'])],
    ['Shift + R', L(['запись старт/стоп', 'recorder start/stop'])], ['M', L(['метка события в записи', 'mark event in recording'])], ['Esc', L(['закрыть окно / свернуть панель', 'close dialog / restore panel'])],
    [L(['Калибровка', 'Calibration']), L(['стрелки — выбор ячеек, Shift+стрелки — область, Enter — правка, Ctrl+Z / Ctrl+Y — отмена/повтор, Ctrl+C / Ctrl+V — копирование таблиц (TSV)',
      'arrows select cells, Shift+arrows extend, Enter edit, Ctrl+Z / Ctrl+Y undo/redo, Ctrl+C / Ctrl+V copy tables (TSV)'])],
    [L(['Графики', 'Charts']), L(['колесо — масштаб по времени, перетаскивание — сдвиг, двойной клик — снова «живой» режим', 'wheel zoom time, drag pan, double-click back to live'])]];
  modal(t('help_title'), L(T.help_body) + `<h4>${esc(t('help_keys'))}</h4><table>${k.map(([a, b]) => `<tr><td><kbd>${esc(a)}</kbd></td><td style="white-space:normal">${esc(b)}</td></tr>`).join('')}</table>`, [{ label: t('close') }]);
}

function applyI18n() {
  document.documentElement.lang = LANG;
  $$('[data-i18n]').forEach(el => { el.textContent = t(el.dataset.i18n); });
  $$('[data-i18n-ph]').forEach(el => { el.placeholder = t(el.dataset.i18nPh); });
  renderTopButtons();
  for (const v of Object.values(VIEWS)) if (v.i18n) try { v.i18n(); } catch (e) { console.error(e); }
  if (S) onState(S);
}

function boot() {
  buildNav();
  initPanels();
  initDock();
  for (const v of Object.values(VIEWS)) if (v.init) try { v.init(); } catch (e) { console.error(e); }
  $('b-pause').onclick = () => { if (S) pending.paused = !S.paused; };
  $('b-lang').onclick = () => { LANG = LANG === 'ru' ? 'en' : 'ru'; LS.set('lang', LANG); MCACHE.clear(); applyI18n(); };
  $('b-sound').onclick = toggleSound;
  $('b-asound').onclick = () => { ASOUND = !ASOUND; LS.set('asound', ASOUND); renderTopButtons(); };
  $('b-dock').onclick = () => { const on = !$('app').classList.toggle('nodock'); LS.set('dock', on); };
  $('b-help').onclick = showHelp;
  if (LS.get('dock', true) === false) $('app').classList.add('nodock');
  document.addEventListener('click', e => {
    const g = e.target.closest('[data-go]');
    if (!g) return;
    if (g.dataset.alarmfilter) { $('ch-alarms').checked = true; }
    go(g.dataset.go);
  });
  HOOKS.unshift((s, prev) => { if (!prev) syncDock(s); });
  applyI18n();
  go(CUR_VIEW);
  loop();
}
