// ---------- dyno: test modes, curves, correction, run history, reports ----------
Object.assign(T, {
  p_dymode: ['Режим испытания', 'Test mode'], p_dychart: ['Кривые момента и мощности', 'Torque & power curves'], p_runs: ['История замеров', 'Run history'],
  p_corr: ['Приведение к стандартным условиям', 'Correction to standard conditions'], p_safety: ['Защита испытательного бокса', 'Test-cell interlock'],
  p_loss: ['Потери в трансмиссии (барабанный стенд)', 'Drivetrain loss (chassis dyno)'], p_dytable: ['Точки текущего замера', 'Current run points'],
  m_sweep: ['Развёртка', 'Sweep'], m_steady: ['Стационар', 'Steady state'], m_steps: ['Ступени', 'Step test'], m_torque: ['Удержание момента', 'Hold torque'],
  tip_m_torque: ['Недоступно: тормоза стендов в физической модели держат скорость (обороты или км/ч), режима заданного тормозного момента нет.',
    'Unavailable: the physics absorbers hold speed (rpm or km/h); there is no torque-control mode.'],
  rpm_start: ['от, об/мин', 'from, rpm'], rpm_end: ['до, об/мин', 'to, rpm'], ramp: ['темп, об/мин/с', 'rate, rpm/s'], start_sweep: ['▶ Начать развёртку', '▶ Start sweep'], stop: ['■ Стоп', '■ Stop'],
  sweep_note: ['Стенд выводит мотор на начальные обороты (2 с стабилизации наддува) и ведёт рампу вихретоковым тормозом при педали 100 % и обдуве бокса 90 км/ч (КПП отсоединена, стендовый restbus снимает стояночную отсечку 3800 об/мин). Всё управление — у прошивки ЭБУ; точки каждые 100 об/мин.',
    'The bench settles boost at the start speed (2 s) and ramps the eddy-current absorber at 100 % pedal with the 90 km/h cell fan (gearbox uncoupled; testbed restbus lifts the 3800 rpm standstill limiter). All engine control is the firmware\'s; points every 100 rpm.'],
  steady_note: ['Тормоз держит обороты, педаль — ваша. «Записать точку» усредняет последнюю секунду.', 'The absorber holds speed, the pedal is yours. "Log point" averages the last second.'],
  steps_note: ['Программа ступеней: на каждой ступени тормоз держит обороты, педаль ставится автоматически, в конце ступени усредняется последняя секунда. Время — симуляционное.',
    'Step program: each step holds speed with the given pedal; the last second of each step is averaged. Time is simulation time.'],
  pedal_pct: ['педаль, %', 'pedal, %'], hold_s: ['выдержка, с', 'hold, s'], apply_hold: ['Удерживать', 'Hold'], release: ['Отпустить', 'Release'], log_point: ['Записать точку', 'Log point'],
  save_points: ['Сохранить точки как замер', 'Save points as run'], add_step: ['+ Ступень', '+ Step'], run_steps: ['▶ Запустить программу', '▶ Run program'],
  running: ['идёт', 'running'], spooling: ['выход на старт', 'stabilising'], idle: ['остановлен', 'idle'], aborted: ['прерван', 'aborted'], std: ['Стандарт', 'Standard'], raw: ['без приведения', 'raw (observed)'],
  dy_need_eng: ['Сначала запустите двигатель.', 'Start the engine first.'],
  power_unit: ['Мощность в', 'Power in'], show_raw: ['показать неприведённые', 'show raw'],
  dy_chart_note: ['Сплошные — текущий/последний замер; пунктир — отмеченные в истории. Приведённое значение = наблюдаемое × CF (коэффициент сохраняется с замером).',
    'Solid = current/latest run; dashed = runs ticked in the history. Corrected = observed × CF (the factor is stored with the run).'],
  dy_report: ['Отчёт HTML', 'HTML report'], dy_csv: ['CSV', 'CSV'], dy_compare: ['Сравнить 2 отмеченных', 'Compare 2 ticked'], dy_delall: ['Удалить все', 'Delete all'],
  dy_confirm_del: ['Удалить все замеры из истории?', 'Delete all runs from the history?'], dy_need2: ['Отметьте два замера.', 'Tick two runs.'], dy_none: ['Нет замеров.', 'No runs.'],
  corr_note: ['Коэффициенты считает сервер (dyno_correction.rs) по атмосфере стенда: давление, влажность (парциальное давление пара) и температура воздуха на входе.',
    'The server computes the factors (dyno_correction.rs) from the bench atmosphere: pressure, humidity (vapour partial pressure) and inlet air temperature.'],
  sf_hint: ['Это оборудование стенда, а не логика ЭБУ. По умолчанию ВЫКЛЮЧЕНО: физика может разрушить мотор, если калибровка плохая. Взведённая защита при превышении предела только отпускает педаль и прерывает программу стенда (как реальная защита бокса) и держит педаль в 0 до сброса.',
    'This is bench equipment, not ECU logic. OFF by default: the physics may destroy the engine with a bad calibration. When armed and a limit is crossed it only lifts the pedal and aborts the bench program (like a real cell interlock) and holds the pedal at 0 until reset.'],
  sf_arm: ['Защита взведена', 'Interlock armed'], sf_reset: ['Сбросить срабатывание', 'Reset trip'], sf_lim: ['предел', 'limit'],
  sf_rpm: ['Обороты выше', 'Engine speed above'], sf_egt: ['EGT выше', 'EGT above'], sf_cool: ['ОЖ выше', 'Coolant above'], sf_oilt: ['Т масла выше', 'Oil temp above'],
  sf_oilp: ['Давление масла ниже', 'Oil pressure below'], sf_oilrpm: ['…при оборотах выше', '…when rpm above'], sf_knock: ['Интеграл детонации выше', 'Knock integral above'],
  sf_tripped_at: ['Сработала: ', 'Tripped: '], sf_total: ['срабатываний всего', 'trips total'],
  loss_na: ['Включите в панели справа «Режим стенда → Барабанный стенд»: потери считаются только там, где есть замер на барабанах.',
    'Select "Bench mode → Chassis dyno" in the right panel: losses exist only with a roller measurement.'],
  loss_note: ['Колёсная мощность = сила тормоза стенда × скорость барабанов; потери = мощность двигателя − колёсная (КПП, приводы, шины на барабанах). Верно в установившемся режиме: при разгоне часть мощности идёт в инерцию барабанов и автомобиля.',
    'Wheel power = absorber force × roller speed; loss = engine power − wheel power (gearbox, shafts, tyres on rollers). Valid at steady speed: while accelerating part of the power goes into roller and vehicle inertia.'],
  peak_tq: ['макс. момент', 'peak torque'], peak_pw: ['макс. мощность', 'peak power'], run_name: ['Название замера', 'Run name'],
  cf: ['CF', 'CF'], ref: ['опорные условия', 'reference'], formula: ['формула', 'formula'], valid: ['допустимо', 'valid'],
  a_p: ['Давление полное', 'Total pressure'], a_pv: ['Парциальное давление пара', 'Vapour pressure'], a_pd: ['Давление сухого воздуха', 'Dry-air pressure'], a_t: ['Т воздуха на входе', 'Inlet air temp'],
});
const PS_PER_KW = 1.3596216, KW_PER_HP = 0.745699872;
let DYNO_RUNS = LS.get('dyno_runs', []);
const saveRuns = () => { while (DYNO_RUNS.length > 40) DYNO_RUNS.shift(); LS.set('dyno_runs', DYNO_RUNS); };
let DY_MODE = LS.get('dy_mode', 'sweep');
let STEPS = LS.get('dy_steps', [{ rpm: 2000, pedal: 30, hold: 5 }, { rpm: 3000, pedal: 60, hold: 5 }, { rpm: 4000, pedal: 100, hold: 5 }]);
let STEPRUN = null; // {i, t0, applied, points}
let POINTS = [];
const dyChartSt = { cursor: null };
const SAMPLE_KEYS = [['rpm', 'plant.rpm'], ['torque_nm', 'plant.torque_brake_nm'], ['power_ps', 'plant.power_ps'], ['map_kpa', 'plant.map_kpa'], ['put_kpa', 'plant.put_kpa'],
  ['lambda', 'plant.lambda_sensor'], ['iat_c', 'plant.iat_c'], ['egt_c', 'plant.egt_c'], ['turbo_rpm', 'plant.turbo_rpm']];
const corrOf = s => (s && s.workstation && s.workstation.correction) || null;
function cfOf(corr, std) {
  if (!std || std === 'raw') return 1;
  const fct = corr && corr.factors && corr.factors.find(x => x.id === std);
  return fct ? fct.cf : NaN;
}
const pwConv = ps => { const u = $('dy-pu').value; return u === 'kw' ? ps / PS_PER_KW : u === 'hp' ? ps / PS_PER_KW / KW_PER_HP : ps; };
const pwUnit = () => ({ ps: 'PS', kw: 'kW', hp: 'hp' }[$('dy-pu').value]);
function runMeta(s, type, samples) {
  return {
    id: Date.now(), name: `${type} ${new Date().toLocaleString()}`, ts: new Date().toISOString(), type, samples,
    image: s.image_label, preset: s.active_preset, engine: s.engine_profile && s.engine_profile.name, drivetrain: s.drivetrain_profile && s.drivetrain_profile.name,
    ambient: s.ambient, fuel_ron: s.fuel_ron, corr: corrOf(s), on: false,
  };
}
// Average of the last `sec` seconds of UI history for the dyno sample fields.
function avgLast(sec) {
  const [i0, i1] = histRange();
  if (i1 <= i0) return null;
  const tEnd = histT(i1 - 1), o = {};
  for (const [f2, k] of SAMPLE_KEYS) {
    let sum = 0, n = 0;
    for (let j = i1 - 1; j >= i0 && histT(j) >= tEnd - sec; j--) { const v = histV(k, j); if (isNum(v)) { sum += v; n++; } }
    o[f2] = n ? sum / n : NaN;
  }
  return o;
}
function peak(samples, k) {
  let best = null;
  for (const p of samples) if (isNum(p[k]) && (!best || p[k] > best[k])) best = p;
  return best;
}
function buildModes() {
  $('dy-modes').innerHTML = ['sweep', 'steady', 'steps', 'torque'].map(m => `<button data-mode="${m}" class="${m === DY_MODE ? 'on' : ''}"${m === 'torque' ? ' disabled data-tip="tip_m_torque"' : ''}>${esc(t('m_' + m))}</button>`).join('');
  const dy = (S && S.dyno) || {};
  const b = $('dy-mode-body');
  if (DY_MODE === 'sweep') {
    b.innerHTML = `<div class="row"><label>${esc(t('rpm_start'))}</label><input id="dy-start" type="number" step="100" value="${dy.rpm_start ?? 1500}">
      <label>${esc(t('rpm_end'))}</label><input id="dy-end" type="number" step="100" value="${dy.rpm_end ?? 6500}">
      <label>${esc(t('ramp'))}</label><input id="dy-ramp" type="number" step="50" value="${dy.ramp_rpm_s ?? 300}"></div>
      <div class="row"><button id="dy-go" class="big primary">${esc(t('start_sweep'))}</button><button id="dy-stop" class="big">${esc(t('stop'))}</button><span id="dy-status" class="mono"></span></div>
      <div class="prog"><i id="dy-prog"></i></div><div class="note">${esc(t('sweep_note'))}</div>`;
    const num = id => { const v = +$(id).value; return Number.isFinite(v) ? v : undefined; };
    $('dy-go').onclick = e => act(e.target, async () => {
      if (!S || !S.plant || S.plant.engine_state !== 'running') { say(t('dy_need_eng'), true); return; }
      onState(await api('/api/dyno', { action: 'start', rpm_start: num('dy-start'), rpm_end: num('dy-end'), ramp_rpm_s: num('dy-ramp') }));
      say(t('start_sweep'));
    });
    $('dy-stop').onclick = e => act(e.target, async () => { onState(await api('/api/dyno', { action: 'stop' })); say(t('stop')); });
  } else if (DY_MODE === 'steady') {
    const dyb = S && S.bench && S.bench.EngineDyno;
    b.innerHTML = `<div class="row"><label>${esc(t('target_rpm'))}</label><input id="ss-rpm" type="number" step="100" value="${dyb ? dyb.target_rpm : 2500}">
      <label>${esc(t('pedal_pct'))}</label><input id="ss-ped" type="number" step="5" min="0" max="100" value="${pedalOverride.v ?? $('pedal').value}"></div>
      <div class="row"><button id="ss-go" class="primary big">${esc(t('apply_hold'))}</button><button id="ss-rel">${esc(t('release'))}</button><button id="ss-log">${esc(t('log_point'))}</button><button id="ss-save">${esc(t('save_points'))}</button></div>
      <div class="note">${esc(t('steady_note'))}</div><div id="ss-points" class="scroll" style="max-height:160px"></div>`;
    $('ss-go').onclick = () => { pending.bench_mode = 'engine_dyno'; pending.dyno_target_rpm = +$('ss-rpm').value; pedalOverride.v = clamp(+$('ss-ped').value || 0, 0, 100); };
    $('ss-rel').onclick = () => { pedalOverride.v = null; $('pedal').value = 0; };
    $('ss-log').onclick = () => { const p = avgLast(1); if (p) { POINTS.push(p); drawPoints(); } };
    $('ss-save').onclick = () => { if (!POINTS.length || !S) return; DYNO_RUNS.push(runMeta(S, 'points', POINTS.slice().sort((a, c) => a.rpm - c.rpm))); POINTS = []; saveRuns(); drawPoints(); buildRuns(); say(t('done')); };
    drawPoints();
  } else if (DY_MODE === 'steps') {
    b.innerHTML = `<table id="st-table"></table><div class="row"><button id="st-add">${esc(t('add_step'))}</button><span class="sp"></span>
      <button id="st-go" class="primary big">${esc(t('run_steps'))}</button><button id="st-stop" class="big">${esc(t('stop'))}</button></div><div id="st-status" class="mono"></div><div class="note">${esc(t('steps_note'))}</div>`;
    const draw = () => {
      $('st-table').innerHTML = `<tr><th>#</th><th>${esc(t('target_rpm'))}</th><th>${esc(t('pedal_pct'))}</th><th>${esc(t('hold_s'))}</th><th></th></tr>` + STEPS.map((s2, i) =>
        `<tr class="${STEPRUN && STEPRUN.i === i ? 'sel' : ''}"><td>${i + 1}</td><td><input type="number" step="100" data-i="${i}" data-f="rpm" value="${s2.rpm}"></td><td><input type="number" step="5" data-i="${i}" data-f="pedal" value="${s2.pedal}"></td>`
        + `<td><input type="number" step="1" data-i="${i}" data-f="hold" value="${s2.hold}"></td><td><button data-del="${i}" class="danger">✕</button></td></tr>`).join('');
    };
    draw();
    $('st-table').onchange = e => { const i = e.target.dataset.i; if (i != null && Number.isFinite(+e.target.value)) { STEPS[+i][e.target.dataset.f] = +e.target.value; LS.set('dy_steps', STEPS); } };
    $('st-table').onclick = e => { const d = e.target.closest('[data-del]'); if (d) { STEPS.splice(+d.dataset.del, 1); LS.set('dy_steps', STEPS); draw(); } };
    $('st-add').onclick = () => { const l = STEPS[STEPS.length - 1] || { rpm: 2000, pedal: 50, hold: 5 }; STEPS.push({ rpm: l.rpm + 500, pedal: l.pedal, hold: l.hold }); LS.set('dy_steps', STEPS); draw(); };
    $('st-go').onclick = () => { if (STEPS.length && S) { STEPRUN = { i: 0, t0: null, points: [] }; say(t('run_steps')); } };
    $('st-stop').onclick = () => { STEPRUN = null; pedalOverride.v = null; $('pedal').value = 0; say(t('stop')); };
    STEPRUN && draw();
    b._draw = draw;
  }
}
function drawPoints() {
  const el = $('ss-points');
  if (!el) return;
  el.innerHTML = POINTS.length ? `<table><tr>${SAMPLE_KEYS.map(([k]) => `<th class="r">${k}</th>`).join('')}</tr>${POINTS.map(p => `<tr>${SAMPLE_KEYS.map(([k]) => `<td class="r">${fmtN(p[k])}</td>`).join('')}</tr>`).join('')}</table>` : '';
}
// Step-test sequencer: driven by simulation time, it only sets bench equipment and the pedal.
function stepTick(s) {
  if (!STEPRUN) return;
  const tnow = s.plant.sim_time_s, st = STEPS[STEPRUN.i];
  if (!st) {
    pedalOverride.v = null; $('pedal').value = 0;
    if (STEPRUN.points.length) { DYNO_RUNS.push(runMeta(s, 'steps', STEPRUN.points)); saveRuns(); buildRuns(); }
    STEPRUN = null; say(t('done'));
    return;
  }
  if (STEPRUN.t0 == null || tnow < STEPRUN.t0) { STEPRUN.t0 = tnow; pending.bench_mode = 'engine_dyno'; pending.dyno_target_rpm = st.rpm; pedalOverride.v = clamp(st.pedal, 0, 100); }
  const el = tnow - STEPRUN.t0;
  if ($('st-status')) $('st-status').textContent = `${t('running')}: ${STEPRUN.i + 1}/${STEPS.length} · ${f(el, 1)} / ${f(st.hold, 1)} s`;
  if (el >= st.hold) {
    const p = avgLast(Math.min(1, st.hold / 2));
    if (p) STEPRUN.points.push(Object.assign(p, { step: STEPRUN.i + 1, pedal: st.pedal }));
    STEPRUN.i++; STEPRUN.t0 = null;
    if (CUR_VIEW === 'dyno' && $('dy-mode-body')._draw) $('dy-mode-body')._draw();
  }
}
const RUN_COLORS = ['#c48cff', '#ffd23f', '#5ee27a', '#ff9f43', '#2ee6d6', '#ff5fd2', '#9aa9ff'];
function curvesFor(samples, cf, color, dashed, label, std) {
  const xs = samples.map(p => p.rpm);
  const out = [{ xs, ys: samples.map(p => p.torque_nm * cf), color, dashed, axis: 0, label: label ? `${label} Nm` : '' },
    { xs, ys: samples.map(p => pwConv(p.power_ps) * cf), color: dashed ? color : '#ff6b6b', dashed, axis: 1, label: label ? `${label} ${pwUnit()}` : '' }];
  if (std !== 'raw' && $('dy-showraw').checked && !dashed) out.push({ xs, ys: samples.map(p => p.torque_nm), color: '#36b5ff66', axis: 0 }, { xs, ys: samples.map(p => pwConv(p.power_ps)), color: '#ff6b6b66', axis: 1 });
  if (out[0].label === '' && !dashed) { out[0].label = `${L(['Момент', 'Torque'])}, Nm`; out[1].label = `${L(['Мощность', 'Power'])}, ${pwUnit()}`; }
  return out;
}
function currentRun(s) {
  const sm = (s.dyno && s.dyno.samples) || [];
  if (s.dyno && s.dyno.running) return { samples: sm, corr: corrOf(s), live: true };
  if (STEPRUN && STEPRUN.points.length) return { samples: STEPRUN.points, corr: corrOf(s), live: true };
  const last = DYNO_RUNS[DYNO_RUNS.length - 1];
  return last ? { samples: last.samples, corr: last.corr, name: last.name } : { samples: sm, corr: corrOf(s) };
}
function drawDyno(s) {
  const std = $('dy-std').value, cr = currentRun(s);
  const cf = cfOf(cr.corr, std);
  let series = curvesFor(cr.samples, cf, '#36b5ff', false, '', std);
  DYNO_RUNS.filter(r => r.on).forEach((r, i) => { series = series.concat(curvesFor(r.samples, cfOf(r.corr, std), RUN_COLORS[i % RUN_COLORS.length], true, r.name.slice(0, 18), std)); });
  let cursorText = null;
  if (dyChartSt.cursor != null && cr.samples.length) {
    const p = cr.samples.reduce((a, b) => (Math.abs(b.rpm - dyChartSt.cursor) < Math.abs(a.rpm - dyChartSt.cursor) ? b : a));
    cursorText = [`${fmtN(p.rpm)} rpm`, `${fmtN(p.torque_nm * cf)} Nm`, `${fmtN(pwConv(p.power_ps) * cf)} ${pwUnit()}`, `MAP ${fmtN(p.map_kpa)} kPa · λ ${fmtN(p.lambda)}`];
  }
  xyChart($('dy-canvas'), { series, xLabel: 'rpm', cursorX: dyChartSt.cursor, cursorText, empty: L(['Нет точек. Запустите замер.', 'No points. Start a run.']) });
  $('dy-sub').textContent = `${cr.live ? '● ' + t('running') : cr.name || ''} · ${std === 'raw' ? t('raw') : `CF ${fmtN(cf)}`}`;
  const sig = cr.samples.length + std + (cr.name || '') + $('dy-pu').value + LANG;
  if ($('dy-table').dataset.sig !== sig) {
    $('dy-table').dataset.sig = sig;
    const cols = SAMPLE_KEYS.map(([k]) => k);
    $('dy-table').innerHTML = `<tr>${cols.map(c => `<th class="r">${c}</th>`).join('')}<th class="r">torque corr</th><th class="r">power corr, ${esc(pwUnit())}</th></tr>`
      + cr.samples.map(p => `<tr>${cols.map(c => `<td class="r">${fmtN(p[c])}</td>`).join('')}<td class="r">${fmtN(p.torque_nm * cf)}</td><td class="r">${fmtN(pwConv(p.power_ps) * cf)}</td></tr>`).join('');
  }
}
function buildRuns() {
  const std = $('dy-std').value;
  $('dy-runs').innerHTML = `<tr><th></th><th>${esc(L(['замер', 'run']))}</th><th>${esc(L(['образ / пресет', 'image / preset']))}</th><th class="r">${esc(t('peak_tq'))}</th><th class="r">${esc(t('peak_pw'))}</th><th class="r">CF</th><th></th></tr>`
    + (DYNO_RUNS.length ? DYNO_RUNS.map((r, i) => {
      const cf = cfOf(r.corr, std), pt = peak(r.samples, 'torque_nm'), pp = peak(r.samples, 'power_ps');
      const ci = DYNO_RUNS.filter(x => x.on).indexOf(r);
      return `<tr><td><input type="checkbox" data-on="${i}"${r.on ? ' checked' : ''}>${ci >= 0 ? `<i style="display:inline-block;width:10px;height:3px;background:${RUN_COLORS[ci % RUN_COLORS.length]};margin-left:3px;vertical-align:middle"></i>` : ''}</td>`
        + `<td class="click" data-ren="${i}" style="cursor:pointer" data-tipraw="${esc(L(['Клик — переименовать', 'Click to rename']))}">${esc(r.name)}<div class="muted">${esc(r.type)} · ${esc(new Date(r.ts).toLocaleString())}${r.aborted ? ' · ' + esc(t('aborted')) : ''}</div></td>`
        + `<td style="white-space:normal;max-width:220px">${esc(r.image || '')}<div class="muted">${esc(r.preset || '')}</div></td>`
        + `<td class="r">${pt ? `${fmtN(pt.torque_nm * cf)} Nm<div class="muted">@ ${fmtN(pt.rpm)}</div>` : DASH}</td><td class="r">${pp ? `${fmtN(pwConv(pp.power_ps) * cf)} ${esc(pwUnit())}<div class="muted">@ ${fmtN(pp.rpm)}</div>` : DASH}</td>`
        + `<td class="r">${fmtN(cf)}</td><td><button data-del="${i}" class="danger">✕</button></td></tr>`;
    }).join('') : `<tr><td colspan="7" class="muted">${esc(t('dy_none'))}</td></tr>`);
}
function buildCorr(s) {
  const c = corrOf(s);
  if (!c) { $('dy-corr').textContent = DASH; return; }
  const sig = JSON.stringify(c) + LANG;
  if ($('dy-corr').dataset.sig === sig) return;
  $('dy-corr').dataset.sig = sig;
  $('dy-corr').innerHTML = `<div class="kv">${[[t('a_p'), f(c.p_total_kpa, 2) + ' kPa'], [t('a_pv'), f(c.p_vapour_kpa, 3) + ' kPa'], [t('a_pd'), f(c.p_dry_kpa, 2) + ' kPa'], [t('a_t'), f(c.inlet_c, 1) + ' °C']]
    .map(([a, b]) => `<div><span>${esc(a)}</span><b>${esc(b)}</b></div>`).join('')}</div>`
    + `<table style="margin-top:6px"><tr><th>${esc(t('std'))}</th><th class="r">CF</th><th>${esc(t('ref'))}</th><th>${esc(t('formula'))}</th><th>${esc(t('valid'))}</th></tr>`
    + c.factors.map(x => `<tr><td>${esc(x.name)}</td><td class="r"><b>${f(x.cf, 4)}</b></td><td class="wrap">${esc(x.reference)}</td><td class="mono wrap">${esc(x.formula)}</td>`
      + `<td class="${x.in_range === false ? 'bad' : x.in_range ? 'good' : 'muted'}">${x.valid ? `${x.valid[0]}…${x.valid[1]} ${x.in_range ? '✓' : '✗'}` : DASH}</td></tr>`).join('') + '</table>';
}
function buildStd() {
  const c = corrOf(S), cur = $('dy-std').value || LS.get('dy_std', 'raw');
  $('dy-std').innerHTML = `<option value="raw">${esc(t('raw'))}</option>` + ((c && c.factors) || []).map(x => `<option value="${esc(x.id)}">${esc(x.name)}</option>`).join('');
  $('dy-std').value = Array.from($('dy-std').options).some(o => o.value === cur) ? cur : 'raw';
}
// ---- interlock UI ----
const SF_ROWS = [['rpm_max', 'sf_rpm', 'rpm'], ['egt_max_c', 'sf_egt', '°C'], ['coolant_max_c', 'sf_cool', '°C'], ['oil_max_c', 'sf_oilt', '°C'], ['oil_min_bar', 'sf_oilp', 'bar'], ['knock_max', 'sf_knock', '']];
let sfDirty = false, sfLast = null;
function buildSafety(s) {
  const sf = s.workstation && s.workstation.safety;
  if (!sf) return;
  if (!sfDirty || !sfLast) {
    sfLast = JSON.parse(JSON.stringify(sf.limits));
    $('sf-en').checked = !!sf.limits.enabled;
    $('sf-table').innerHTML = `<tr><th></th><th></th><th>${esc(t('sf_lim'))}</th><th></th></tr>` + SF_ROWS.map(([k, lab, u]) => `<tr><td><input type="checkbox" data-sfon="${k}"${sf.limits[k] != null ? ' checked' : ''}></td>`
      + `<td>${esc(t(lab))}</td><td><input type="number" step="any" data-sf="${k}" value="${sf.limits[k] != null ? sf.limits[k] : SF_DEF[k]}"></td><td>${esc(u)}</td></tr>`).join('')
      + `<tr><td></td><td>${esc(t('sf_oilrpm'))}</td><td><input type="number" step="100" data-sf="oil_check_above_rpm" value="${sf.limits.oil_check_above_rpm}"></td><td>rpm</td></tr>`;
  }
  $('sf-sub').textContent = t(sf.trip ? 'sf_tripped' : sf.limits.enabled ? 'sf_armed' : 'sf_off');
  $('sf-trip').innerHTML = sf.trip ? `<div class="verdict bad">${esc(t('sf_tripped_at'))}${esc(chName(sf.trip.channel))} = ${esc(fmtN(sf.trip.value))} (${esc(t('sf_lim'))} ${esc(fmtN(sf.trip.limit))}), t = ${esc(f(sf.trip.t_s, 2))} s</div>`
    : `<div class="note">${esc(t('sf_total'))}: ${esc(sf.trips_total)}</div>`;
}
const SF_DEF = { rpm_max: 7000, egt_max_c: 980, coolant_max_c: 115, oil_max_c: 140, oil_min_bar: 1, knock_max: 1 };
function safetyLimitsFromUI() {
  const lim = { enabled: $('sf-en').checked };
  for (const [k] of SF_ROWS) { const on = document.querySelector(`[data-sfon="${k}"]`).checked, v = +document.querySelector(`[data-sf="${k}"]`).value; lim[k] = on && Number.isFinite(v) ? v : null; }
  lim.oil_check_above_rpm = +document.querySelector('[data-sf="oil_check_above_rpm"]').value || 0;
  return lim;
}
// ---- reports ----
function svgCurves(runs, std) {
  const W = 760, H = 340, l = 50, r = 50, top = 20, bot = 30, pw = W - l - r, ph = H - top - bot;
  const all = runs.flatMap(rr => rr.samples);
  if (!all.length) return '';
  const x0 = Math.min(...all.map(p => p.rpm)), x1 = Math.max(...all.map(p => p.rpm));
  const tq = Math.max(1, ...runs.flatMap(rr => rr.samples.map(p => p.torque_nm * cfOf(rr.corr, std)))) * 1.1;
  const pw2 = Math.max(1, ...runs.flatMap(rr => rr.samples.map(p => pwConv(p.power_ps) * cfOf(rr.corr, std)))) * 1.1;
  const X = v => l + ((v - x0) / (x1 - x0 || 1)) * pw, Y = (v, m) => top + ph * (1 - v / m);
  let g = '';
  for (let i = 0; i <= 5; i++) { const y = top + (ph * i) / 5; g += `<line x1="${l}" x2="${l + pw}" y1="${y}" y2="${y}" stroke="#ddd"/><text x="${l - 4}" y="${y + 4}" text-anchor="end" fill="#1565c0">${fmtN(tq * (1 - i / 5))}</text><text x="${l + pw + 4}" y="${y + 4}" fill="#c62828">${fmtN(pw2 * (1 - i / 5))}</text>`; }
  for (const tk of niceTicks(x0, x1, 8)) g += `<text x="${X(tk)}" y="${H - 10}" text-anchor="middle">${fmtN(tk)}</text>`;
  runs.forEach((rr, i) => {
    const cf = cfOf(rr.corr, std), dash = i ? ' stroke-dasharray="6 4"' : '';
    g += `<polyline fill="none" stroke="#1565c0" stroke-width="2"${dash} points="${rr.samples.map(p => `${X(p.rpm).toFixed(1)},${Y(p.torque_nm * cf, tq).toFixed(1)}`).join(' ')}"/>`;
    g += `<polyline fill="none" stroke="#c62828" stroke-width="2"${dash} points="${rr.samples.map(p => `${X(p.rpm).toFixed(1)},${Y(pwConv(p.power_ps) * cf, pw2).toFixed(1)}`).join(' ')}"/>`;
  });
  return `<svg xmlns="http://www.w3.org/2000/svg" width="${W}" height="${H}" font-family="sans-serif" font-size="11">${g}<text x="${l}" y="12" fill="#1565c0">Nm</text><text x="${l + pw}" y="12" fill="#c62828" text-anchor="end">${esc(pwUnit())}</text><text x="${l + pw}" y="${H - 22}" text-anchor="end">rpm</text></svg>`;
}
function reportHTML(runs) {
  const std = $('dy-std').value, stdName = $('dy-std').selectedOptions[0].textContent;
  const meta2 = rr => `<table><tr><th>${esc(L(['Дата', 'Date']))}</th><td>${esc(new Date(rr.ts).toLocaleString())}</td></tr><tr><th>${esc(L(['Образ ЭБУ', 'ECU image']))}</th><td>${esc(rr.image)}</td></tr>`
    + `<tr><th>${esc(t('preset'))}</th><td>${esc(rr.preset)}</td></tr><tr><th>${esc(L(['Двигатель', 'Engine']))}</th><td>${esc(rr.engine)}</td></tr><tr><th>${esc(L(['Трансмиссия', 'Drivetrain']))}</th><td>${esc(rr.drivetrain)}</td></tr>`
    + `<tr><th>${esc(L(['Атмосфера', 'Atmosphere']))}</th><td>${rr.ambient ? `${esc(rr.ambient.temp_c)} °C, ${esc(rr.ambient.altitude_m)} m, RH ${esc(rr.ambient.humidity_pct)} %` : DASH}${rr.corr ? `, p ${f(rr.corr.p_total_kpa, 2)} kPa, p_dry ${f(rr.corr.p_dry_kpa, 2)} kPa` : ''}</td></tr>`
    + `<tr><th>RON</th><td>${esc(rr.fuel_ron ?? DASH)}</td></tr><tr><th>${esc(stdName)}</th><td>CF ${fmtN(cfOf(rr.corr, std))}</td></tr></table>`;
  const tbl = rr => { const cf = cfOf(rr.corr, std); return `<table class="d"><tr>${SAMPLE_KEYS.map(([k]) => `<th>${k}</th>`).join('')}<th>torque corr</th><th>power corr ${esc(pwUnit())}</th></tr>${rr.samples.map(p => `<tr>${SAMPLE_KEYS.map(([k]) => `<td>${fmtN(p[k])}</td>`).join('')}<td>${fmtN(p.torque_nm * cf)}</td><td>${fmtN(pwConv(p.power_ps) * cf)}</td></tr>`).join('')}</table>`; };
  return `<!DOCTYPE html><html><head><meta charset="utf-8"><title>SIMEMU dyno report</title><style>body{font:13px sans-serif;margin:24px;color:#111}table{border-collapse:collapse;margin:8px 0}th,td{border:1px solid #ccc;padding:2px 6px;text-align:left}table.d td{text-align:right;font-family:monospace}h2{margin-top:28px}</style></head><body>`
    + `<h1>SIMEMU — ${esc(L(['Отчёт моторного стенда', 'Engine dyno report']))}</h1><p>${esc(L(['Стандарт приведения', 'Correction standard']))}: <b>${esc(stdName)}</b>. ${esc(L(['Сплошные — первый замер, пунктир — сравнение. Данные — физическая модель SIMEMU под управлением заводской прошивки ЭБУ.', 'Solid = first run, dashed = comparison. Data = SIMEMU physics driven by the ECU firmware.']))}</p>`
    + svgCurves(runs, std) + runs.map(rr => `<h2>${esc(rr.name)}</h2>${meta2(rr)}${tbl(rr)}`).join('') + `<p style="color:#777">${esc(new Date().toISOString())}</p></body></html>`;
}
const selRuns = () => { const on = DYNO_RUNS.filter(r => r.on); return on.length ? on : DYNO_RUNS.slice(-1); };
function compareRuns(a, b) {
  const std = $('dy-std').value, ca = cfOf(a.corr, std), cb = cfOf(b.corr, std);
  const lo = Math.max(Math.min(...a.samples.map(p => p.rpm)), Math.min(...b.samples.map(p => p.rpm)));
  const hi = Math.min(Math.max(...a.samples.map(p => p.rpm)), Math.max(...b.samples.map(p => p.rpm)));
  const interp = (sm, rpm, k) => {
    const s2 = sm.slice().sort((x, y) => x.rpm - y.rpm);
    for (let i = 1; i < s2.length; i++) if (s2[i].rpm >= rpm && s2[i - 1].rpm <= rpm) { const u = (rpm - s2[i - 1].rpm) / (s2[i].rpm - s2[i - 1].rpm || 1); return s2[i - 1][k] + u * (s2[i][k] - s2[i - 1][k]); }
    return NaN;
  };
  if (!(hi > lo)) return `<div class="note">${esc(L(['Нет общего диапазона оборотов.', 'No common rpm range.']))}</div>`;
  const rows = niceTicks(lo, hi, 12).map(rpm => {
    const ta = interp(a.samples, rpm, 'torque_nm') * ca, tb = interp(b.samples, rpm, 'torque_nm') * cb;
    const pa = pwConv(interp(a.samples, rpm, 'power_ps')) * ca, pb = pwConv(interp(b.samples, rpm, 'power_ps')) * cb;
    const pc = (x, y) => (isNum(x) && isNum(y) && x ? `${((y / x - 1) * 100).toFixed(1)} %` : DASH);
    return `<tr><td class="r">${fmtN(rpm)}</td><td class="r">${fmtN(ta)}</td><td class="r">${fmtN(tb)}</td><td class="r">${fmtN(tb - ta)}</td><td class="r">${pc(ta, tb)}</td><td class="r">${fmtN(pa)}</td><td class="r">${fmtN(pb)}</td><td class="r">${fmtN(pb - pa)}</td><td class="r">${pc(pa, pb)}</td></tr>`;
  });
  return `<h4>${esc(a.name)} → ${esc(b.name)}</h4><table><tr><th class="r">rpm</th><th class="r">Nm A</th><th class="r">Nm B</th><th class="r">Δ</th><th class="r">%</th><th class="r">${esc(pwUnit())} A</th><th class="r">${esc(pwUnit())} B</th><th class="r">Δ</th><th class="r">%</th></tr>${rows.join('')}</table>`;
}
view({
  id: 'dyno',
  init() {
    $('dy-modes').onclick = e => { const b = e.target.closest('[data-mode]'); if (b && !b.disabled) { DY_MODE = b.dataset.mode; LS.set('dy_mode', DY_MODE); buildModes(); } };
    $('dy-std').onchange = () => { LS.set('dy_std', $('dy-std').value); buildRuns(); };
    $('dy-pu').value = LS.get('dy_pu', 'ps');
    $('dy-pu').onchange = () => { LS.set('dy_pu', $('dy-pu').value); buildRuns(); };
    const cv = $('dy-canvas');
    cv.addEventListener('mousemove', e => { const g = cv._geo; if (g) { dyChartSt.cursor = g.x0 + clamp((e.offsetX - g.l) / g.pw, 0, 1) * (g.x1 - g.x0); if (S) drawDyno(S); } });
    cv.addEventListener('mouseleave', () => { dyChartSt.cursor = null; if (S) drawDyno(S); });
    $('dy-runs').onchange = e => { const i = e.target.dataset.on; if (i != null) { DYNO_RUNS[+i].on = e.target.checked; saveRuns(); buildRuns(); } };
    $('dy-runs').onclick = async e => {
      const d = e.target.closest('[data-del]'), rn = e.target.closest('[data-ren]');
      if (d) { DYNO_RUNS.splice(+d.dataset.del, 1); saveRuns(); buildRuns(); }
      if (rn) { const r = DYNO_RUNS[+rn.dataset.ren]; const n = await promptBox(t('run_name'), t('run_name'), r.name); if (n) { r.name = n; saveRuns(); buildRuns(); } }
    };
    $('dy-delall').onclick = async () => { if (await confirmBox(t('dy_confirm_del'))) { DYNO_RUNS = []; saveRuns(); buildRuns(); } };
    $('dy-report').onclick = () => { const r = selRuns(); if (!r.length) return say(t('dy_none'), true); download(`simemu_dyno_${stamp()}.html`, reportHTML(r), 'text/html'); };
    $('dy-csv').onclick = () => {
      const r = selRuns(), std = $('dy-std').value;
      if (!r.length) return say(t('dy_none'), true);
      const lines = [['run', 'std', 'cf'].concat(SAMPLE_KEYS.map(([k]) => k), ['torque_corr_nm', `power_corr_${$('dy-pu').value}`]).join(',')];
      for (const rr of r) { const cf = cfOf(rr.corr, std); for (const p of rr.samples) lines.push([`"${rr.name.replace(/"/g, "'")}"`, std, cf].concat(SAMPLE_KEYS.map(([k]) => p[k]), [p.torque_nm * cf, pwConv(p.power_ps) * cf]).join(',')); }
      download(`simemu_dyno_${stamp()}.csv`, lines.join('\n') + '\n', 'text/csv');
    };
    $('dy-compare').onclick = () => { const on = DYNO_RUNS.filter(r => r.on); $('dy-cmp').innerHTML = on.length >= 2 ? compareRuns(on[0], on[1]) : `<div class="note warn">${esc(t('dy_need2'))}</div>`; };
    $('sf-table').oninput = () => { sfDirty = true; };
    $('sf-en').onchange = () => { sfDirty = true; };
    $('sf-apply').onclick = e => act(e.target, async () => { await api('/api/ws/safety', { action: 'set', limits: safetyLimitsFromUI() }); sfDirty = false; sfLast = null; say(t('done')); });
    $('sf-reset').onclick = e => act(e.target, async () => { await api('/api/ws/safety', { action: 'reset' }); say(t('sf_reset')); });
    // Auto-save sweeps when the bench program finishes (or is aborted by the interlock / stop).
    HOOKS.push((s, prev) => {
      stepTick(s);
      if (prev && prev.dyno && prev.dyno.running && s.dyno && !s.dyno.running && s.dyno.samples.length > 1) {
        const r = runMeta(s, 'sweep', s.dyno.samples.slice());
        r.aborted = !s.dyno.completed;
        DYNO_RUNS.push(r); saveRuns(); buildRuns();
        say(`${L(['Замер сохранён', 'Run saved'])}: ${r.name}${r.aborted ? ' (' + t('aborted') + ')' : ''}`);
      }
    });
  },
  first() { buildStd(); },
  show() { buildModes(); buildStd(); buildRuns(); sfLast = null; sfDirty = false; },
  i18n() { buildModes(); buildStd(); buildRuns(); renderLoss(S); sfLast = null; },
  render(s) {
    const dy = s.dyno || {}, p = s.plant || {};
    if ($('dy-status')) {
      $('dy-status').textContent = `${t(dy.running ? (dy.sweeping ? 'running' : 'spooling') : 'idle')} · target ${f(dy.target_rpm, 0)} rpm · ${(dy.samples || []).length} pts`;
      $('dy-prog').style.width = (dy.running ? clamp((dy.target_rpm - dy.rpm_start) / (dy.rpm_end - dy.rpm_start || 1), 0, 1) * 100 : 0).toFixed(0) + '%';
    }
    const kn = Array.isArray(p.knock_integral) ? Math.max(...p.knock_integral) : NaN;
    $('dy-live').innerHTML = [['plant.rpm'], ['plant.dyno_absorber_nm'], ['plant.torque_brake_nm'], ['plant.power_ps'], ['plant.map_kpa'], ['plant.lambda_sensor'], ['plant.egt_c']]
      .map(([k]) => `<div data-ch="${esc(k)}"><span>${esc(chName(k))}</span><b class="${ACTIVE.has(k) ? 'bad' : ''}">${esc(fmtCh(k, FLAT.get(k)))} ${esc(meta(k).unit || '')}</b></div>`).join('')
      + `<div><span>${esc(L(['Детонация, макс. по цил.', 'Knock, max over cyl.']))}</span><b>${fmtN(kn)}</b></div>`;
    drawDyno(s);
    buildCorr(s);
    buildSafety(s);
    renderLoss(s);
  },
});
function renderLoss(s) {
  const p = (s && s.plant) || {}, cd = !!(s && s.bench && s.bench.ChassisDyno);
  const wheel = cd && isNum(p.roller_absorber_n) && isNum(p.roller_kmh) ? (p.roller_absorber_n * p.roller_kmh) / 3.6 / 1000 : NaN;
  const eng = cd ? p.power_kw : NaN, loss = eng - wheel;
  const rows = [[L(['Двигатель, кВт', 'Engine, kW']), f(eng, 1)], [L(['На барабанах, кВт', 'At the rollers, kW']), f(wheel, 1)], [L(['Барабаны, км/ч', 'Rollers, km/h']), f(cd ? p.roller_kmh : NaN, 1)],
    [L(['Потери, кВт', 'Loss, kW']), f(loss, 1)], [L(['Потери, %', 'Loss, %']), f(eng > 1 ? (100 * loss) / eng : NaN, 1)]];
  $('dy-loss').innerHTML = `<div class="kv">${rows.map(([a, b]) => `<div><span>${esc(a)}</span><b>${esc(b)}</b></div>`).join('')}</div><div class="note">${esc(t(cd ? 'loss_note' : 'loss_na'))}</div>`;
}

// ---------- health: component diagram + damage event log ----------
Object.assign(T, {
  p_hdiag: ['Схема узлов: двигатель и коробка', 'Component map: engine and gearbox'], p_hdetail: ['Узел', 'Component'], p_hlog: ['Журнал повреждений', 'Damage log'],
  h_color: ['Окраска', 'Colour by'], h_m_health: ['целостность', 'integrity'], h_m_temp: ['температура', 'temperature'], h_m_stress: ['нагрузка', 'stress'],
  h_note: ['Цвет берётся только из телеметрии. Серый — для узла пока нет поля в телеметрии физики (появится автоматически, когда физика начнёт его отдавать: поля *_health / *_wear / *_damage с именем узла).',
    'Colours come from telemetry only. Grey = no telemetry field for that component yet (appears automatically once the physics reports it: *_health / *_wear / *_damage fields named after the component).'],
  h_pick: ['Выберите узел на схеме.', 'Pick a component on the map.'], h_nodata: ['Нет данных: физика пока не отдаёт состояние этого узла.', 'No data: the physics does not report this component yet.'],
  h_combined: ['общая целостность цилиндра (поршень, кольца, клапаны вместе)', 'combined cylinder integrity (piston, rings, valves together)'],
  h_rate: ['скорость износа', 'damage rate'], h_src: ['источники', 'sources'], h_when: ['условия в момент события', 'conditions at the moment'],
  h_dropped: ['снизилась', 'dropped'], h_ev_seized: ['Двигатель заклинило', 'Engine seized'], h_ev_trip: ['Сработала защита бокса', 'Test-cell interlock tripped'],
  h_backend: ['События, переданные физикой', 'Events reported by the physics'], h_nolog: ['Повреждений не было.', 'No damage so far.'],
  h_health: ['Целостность', 'Integrity'], h_temp: ['Температура', 'Temperature'], h_stress: ['Нагрузка', 'Stress'],
});
const HCOMP = [
  { id: 'valves', n: 4, name: ['Клапаны, цил. {i}', 'Valves, cyl {i}'], tok: /valve/, fb: 'plant.cylinder_health[{i}]' },
  { id: 'piston', n: 4, name: ['Поршень, цил. {i}', 'Piston, cyl {i}'], tok: /piston/, fb: 'plant.cylinder_health[{i}]', stress: ['plant.knock_integral[{i}]'] },
  { id: 'rings', n: 4, name: ['Кольца, цил. {i}', 'Rings, cyl {i}'], tok: /ring/, fb: 'plant.cylinder_health[{i}]' },
  { id: 'rod', n: 4, name: ['Шатун, цил. {i}', 'Con-rod, cyl {i}'], tok: /(^|_)(con_?)?rod/ },
  { id: 'gasket', n: 1, name: ['Прокладка ГБЦ', 'Head gasket'], tok: /gasket/, temp: ['plant.coolant_c'] },
  { id: 'bearing', n: 1, name: ['Подшипники коленвала', 'Crank bearings'], tok: /bearing/, temp: ['plant.oil_c'], stress: ['plant.oil_bar'] },
  { id: 'turbo', n: 1, name: ['Турбокомпрессор', 'Turbocharger'], tok: /turbo|turbine|compressor/, temp: ['plant.egt_c'], stress: ['plant.turbo_rpm'] },
  { id: 'clutch', n: 2, il: ['K1', 'K2'], name: ['Сцепление {i}', 'Clutch {i}'], tok: /clutch/, temp: ['plant.clutch_temp_c[{i}]'], stress: ['plant.clutch_slip_rpm[{i}]'] },
  { id: 'gear', n: 8, il: ['1', '2', '3', '4', '5', '6', '7', 'R'], name: ['Шестерни {i}-й передачи', 'Gear {i} pair'], tok: /(^|_)gears?(?!box)(_|\[|$)/ },
  { id: 'synchro', n: 4, name: ['Синхронизатор / вилка {i}', 'Synchro / fork {i}'], tok: /sync/, stress: ['plant.fork_mm[{i}]'] },
];
let H_SEL = null;
const leafOf = k => k.replace(/^.*\./, '');
function healthKeys() { return Array.from(FLAT.keys()).filter(k => typeof FLAT.get(k) === 'number' && !/_outputs\./.test(k) && /(health|integrity|wear|damage)/i.test(leafOf(k))); }
// Integrity %, from *_health (1 = intact) or *_damage (0..1 used). Wear in mm / µm has no "% intact" scale → NaN
// (still listed as source data of the component).
const hPct = (k, raw) => { if (!isNum(raw) || /_(mm|um)(\[\d+\])*$/.test(leafOf(k))) return NaN; const worn = /wear|damage/i.test(leafOf(k)); const x = Math.abs(raw) <= 1.0001 ? raw * 100 : raw; return worn ? 100 - x : x; };
const idxOf = k => { const m = k.match(/\[(\d+)\](?!.*\[)/); return m ? +m[1] : null; };
function compData(c, i) {
  const sub = s => s.replace('{i}', i);
  const fit = k => { const ix = idxOf(k); return ix == null || c.n === 1 || ix === i; };
  const own = healthKeys().filter(k => c.tok.test(leafOf(k).replace(/\[\d+\]/g, '')) && fit(k));
  const fb = !own.length && c.fb && FLAT.has(sub(c.fb)) ? [sub(c.fb)] : [];
  const hk = own.length ? own : fb;
  const hv = hk.map(k => hPct(k, FLAT.get(k))).filter(isNum);
  const extra = (list, re) => (list || []).map(sub).filter(k => FLAT.has(k)).concat(Array.from(FLAT.keys()).filter(k => typeof FLAT.get(k) === 'number' && !/_outputs\./.test(k) && c.tok.test(leafOf(k)) && re.test(leafOf(k)) && fit(k)));
  const tk = Array.from(new Set(extra(c.temp, /(_c|temp)(\[\d+\])?$/))), sk = Array.from(new Set(extra(c.stress, /(stress|load|force)/)));
  return { hk, health: hv.length ? Math.min(...hv) : NaN, combined: !own.length && fb.length > 0, tk, sk };
}
function stressFrac(k) {
  const v = Math.abs(dv(k, FLAT.get(k))), l = limitsOf(k), m = meta(k);
  const ref = isNum(l.hi) ? l.hi : m.r ? Math.max(Math.abs(m.r[0]), Math.abs(m.r[1])) : NaN;
  return isNum(v) && ref > 0 ? clamp(v / ref, 0, 1.5) : NaN;
}
function compColor(d, mode) {
  if (mode === 'health') return isNum(d.health) ? `hsl(${clamp(d.health, 0, 100) * 1.2},70%,${d.health >= 99.5 ? 38 : 45}%)` : '#2a3548';
  if (mode === 'temp') {
    const k = d.tk[0];
    if (!k) return '#2a3548';
    const l = limitsOf(k), m = meta(k), v = dv(k, FLAT.get(k)), hi = isNum(l.hi) ? l.hi : m.r ? m.r[1] : 150;
    return isNum(v) ? `hsl(${(1 - clamp(v / hi, 0, 1)) * 220},75%,45%)` : '#2a3548';
  }
  const fr = d.sk.map(stressFrac).filter(isNum);
  return fr.length ? `hsl(${(1 - clamp(Math.max(...fr), 0, 1)) * 120},75%,42%)` : '#2a3548';
}
function healthSVG(mode) {
  const parts = [];
  const comp = (id, i, shape) => {
    const c = HCOMP.find(x => x.id === id), d = compData(c, i);
    parts.push(shape.replace('/>', ` class="comp${H_SEL && H_SEL[0] === id && H_SEL[1] === i ? ' sel' : ''}" data-comp="${id}:${i}" fill="${compColor(d, mode)}"/>`));
  };
  parts.push('<text x="20" y="22">' + esc(L(['ДВИГАТЕЛЬ', 'ENGINE'])) + '</text><text x="740" y="22">' + esc(L(['КОРОБКА DSG', 'DSG GEARBOX'])) + '</text>');
  for (let i = 0; i < 4; i++) {
    const x = 30 + i * 128;
    parts.push(`<rect x="${x - 6}" y="40" width="122" height="44" rx="4" fill="#1b2638" stroke="#2f4263"/>`);
    comp('valves', i, `<rect x="${x + 14}" y="48" width="34" height="14" rx="2"/>`);
    comp('valves', i, `<rect x="${x + 62}" y="48" width="34" height="14" rx="2"/>`);
    parts.push(`<rect x="${x}" y="92" width="110" height="120" fill="none" stroke="#2f4263" stroke-width="2"/>`);
    comp('rings', i, `<rect x="${x + 4}" y="116" width="102" height="12"/>`);
    comp('piston', i, `<rect x="${x + 4}" y="128" width="102" height="44" rx="3"/>`);
    comp('rod', i, `<rect x="${x + 46}" y="172" width="18" height="96" rx="6"/>`);
    parts.push(`<text x="${x + 55}" y="108" text-anchor="middle">${esc(L(['цил.', 'cyl']))} ${i + 1}</text>`);
  }
  comp('gasket', 0, '<rect x="20" y="84" width="520" height="8"/>');
  parts.push('<rect x="20" y="276" width="520" height="12" rx="6" fill="#24324a"/>');
  for (let i = 0; i < 5; i++) comp('bearing', 0, `<circle cx="${30 + i * 128 - 10}" cy="282" r="13"/>`);
  for (let i = 0; i < 4; i++) comp('bearing', 0, `<circle cx="${85 + i * 128}" cy="282" r="9"/>`);
  parts.push('<text x="560" y="40">' + esc(L(['Турбина', 'Turbo'])) + '</text>');
  comp('turbo', 0, '<circle cx="600" cy="90" r="30"/>');
  comp('turbo', 0, '<circle cx="660" cy="90" r="24"/>');
  parts.push('<rect x="596" y="86" width="64" height="8" fill="#24324a"/>');
  comp('clutch', 0, '<rect x="740" y="40" width="22" height="110" rx="4"/>');
  comp('clutch', 1, '<rect x="768" y="40" width="22" height="110" rx="4"/>');
  parts.push('<text x="751" y="166" text-anchor="middle">K1</text><text x="779" y="166" text-anchor="middle">K2</text>');
  parts.push('<rect x="800" y="70" width="190" height="6" fill="#24324a"/><rect x="800" y="200" width="190" height="6" fill="#24324a"/>');
  [[0, 0], [2, 1], [4, 2], [6, 3]].forEach(([g, j]) => { comp('gear', g, `<rect x="${810 + j * 46}" y="40" width="30" height="66" rx="3"/>`); parts.push(`<text x="${825 + j * 46}" y="122" text-anchor="middle">${g + 1}</text>`); });
  [[1, 0], [3, 1], [5, 2], [7, 3]].forEach(([g, j]) => { comp('gear', g, `<rect x="${810 + j * 46}" y="170" width="30" height="66" rx="3"/>`); parts.push(`<text x="${825 + j * 46}" y="252" text-anchor="middle">${g === 7 ? 'R' : g + 1}</text>`); });
  for (let j = 0; j < 4; j++) comp('synchro', j, `<rect x="${806 + j * 46}" y="${j % 2 ? 268 : 130}" width="38" height="14" rx="3"/>`);
  parts.push(`<text x="810" y="${300}">${esc(L(['синхронизаторы / вилки 1–4', 'synchros / forks 1–4']))}</text>`);
  return `<svg class="health-svg" viewBox="0 0 1000 310">${parts.join('')}</svg>`;
}
function healthDetail() {
  if (!H_SEL) { $('h-detail').innerHTML = `<div class="muted">${esc(t('h_pick'))}</div>`; return; }
  const [id, i] = H_SEL, c = HCOMP.find(x => x.id === id), d = compData(c, i);
  const lab = c.il ? c.il[i] : String(i + 1);
  const row2 = k => `<div data-ch="${esc(k)}"><span>${esc(chName(k))}</span><b class="${ACTIVE.has(k) ? 'bad' : ''}">${esc(fmtCh(k, FLAT.get(k)))} ${esc(meta(k).unit || '')}</b></div>`;
  let rate = '';
  if (d.hk.length) {
    const k = d.hk[0], [i0, i1] = histRange();
    if (i1 - i0 > 2) {
      const tEnd = histT(i1 - 1);
      let j = i1 - 1;
      while (j > i0 && histT(j) > tEnd - 10) j--;
      const dtt = tEnd - histT(j), dh = hPct(k, histV(k, i1 - 1)) - hPct(k, histV(k, j));
      if (dtt > 0 && isNum(dh)) rate = `<div><span>${esc(t('h_rate'))}</span><b>${fmtN(dh / dtt)} %/s</b></div>`;
    }
  }
  $('h-detail').innerHTML = `<h4>${esc(L(c.name).replace('{i}', lab))}</h4>`
    + `<div class="kv"><div><span>${esc(t('h_health'))}</span><b class="${isNum(d.health) && d.health < 99.5 ? 'bad' : ''}">${isNum(d.health) ? f(d.health, 2) + ' %' : DASH}</b></div>${rate}</div>`
    + (d.combined ? `<div class="note warn">${esc(t('h_combined'))}</div>` : '')
    + (!d.hk.length ? `<div class="note">${esc(t('h_nodata'))}</div>` : '')
    + (d.hk.length ? `<h4>${esc(t('h_src'))}</h4><div class="kv">${d.hk.map(row2).join('')}</div><div class="note">${esc(L(meta(d.hk[0]).desc))}</div>` : '')
    + (d.tk.length ? `<h4>${esc(t('h_temp'))}</h4><div class="kv">${d.tk.map(row2).join('')}</div>` : '')
    + (d.sk.length ? `<h4>${esc(t('h_stress'))}</h4><div class="kv">${d.sk.map(row2).join('')}</div>` : '');
}
const HLOG = [];
const hLast = new Map();
let hPrevState = null, hPrevTrip = null;
function hContext() {
  const p = (S && S.plant) || {};
  const kn = Array.isArray(p.knock_integral) ? Math.max(...p.knock_integral) : NaN;
  const base = [`rpm ${f(p.rpm, 0)}`, `knock max ${fmtN(kn)}`, `oil ${f(p.oil_bar, 2)} bar / ${f(p.oil_c, 0)} °C`, `ECT ${f(p.coolant_c, 0)} °C`, `EGT ${f(p.egt_c, 0)} °C`, `turbo ${f(p.turbo_rpm, 0)} rpm`];
  const al = Array.from(ACTIVE.keys()).filter(k => meta(k).group !== 'hlth').map(k => `⚠ ${chName(k)} ${fmtCh(k, FLAT.get(k))} ${meta(k).unit || ''}`);
  return al.concat(base).join(' · ');
}
function healthTick(s) {
  const tnow = s.plant ? s.plant.sim_time_s : NaN;
  if (hLast.size && isNum(tnow) && HLOG.length && tnow < HLOG[HLOG.length - 1].t) hLast.clear();
  for (const k of healthKeys()) {
    const v = hPct(k, FLAT.get(k));
    if (!isNum(v)) continue;
    const last = hLast.get(k);
    if (last == null) { hLast.set(k, v); continue; }
    if (v < last - 0.5 || (v <= 0 && last > 0)) {
      HLOG.push({ t: tnow, what: chName(k), key: k, from: last, to: v, ctx: hContext() });
      hLast.set(k, v);
    } else if (v > last) hLast.set(k, v); // reset (cold soak)
  }
  const st = s.plant && s.plant.engine_state;
  if (st === 'seized' && hPrevState !== 'seized') HLOG.push({ t: tnow, what: t('h_ev_seized'), key: 'plant.engine_state', ctx: hContext() });
  hPrevState = st;
  const trip = s.workstation && s.workstation.safety && s.workstation.safety.trip;
  if (trip && (!hPrevTrip || hPrevTrip.t_s !== trip.t_s)) HLOG.push({ t: trip.t_s, what: t('h_ev_trip'), key: trip.channel, ctx: `${chName(trip.channel)} ${fmtN(trip.value)} > ${fmtN(trip.limit)}` });
  hPrevTrip = trip;
  if (HLOG.length > 500) HLOG.splice(0, HLOG.length - 500);
  const nb = $('nb-health');
  nb.hidden = !HLOG.length; nb.textContent = HLOG.length;
}
function backendEvents(s) {
  const p = s.plant || {}, out = [];
  for (const k in p) if (/event|failure|fault|incident|damage_log/i.test(k) && p[k] && typeof p[k] === 'object') {
    const arr = Array.isArray(p[k]) ? p[k] : [p[k]];
    for (const e of arr) out.push(`<tr><td class="muted">plant.${esc(k)}</td><td style="white-space:normal">${esc(typeof e === 'string' ? e : JSON.stringify(e))}</td></tr>`);
  }
  return out.length ? `<h4>${esc(t('h_backend'))}</h4><table>${out.join('')}</table>` : '';
}
let hLogN = -1;
view({
  id: 'health',
  init() {
    $('h-svg').onclick = e => { const c = e.target.closest('[data-comp]'); if (c) { const [id, i] = c.dataset.comp.split(':'); H_SEL = [id, +i]; VIEWS.health.render(S); } };
    $('h-mode').onchange = () => S && VIEWS.health.render(S);
    $('h-logclear').onclick = () => { HLOG.length = 0; hLogN = -1; VIEWS.health.render(S); };
    HOOKS.push(healthTick);
  },
  render(s) {
    const mode = $('h-mode').value;
    $('h-svg').innerHTML = healthSVG(mode);
    $('h-legend').innerHTML = mode === 'health' ? `<span><i style="background:hsl(120,70%,38%)"></i>100 %</span><span><i style="background:hsl(60,70%,45%)"></i>50 %</span><span><i style="background:hsl(0,70%,45%)"></i>0 %</span><span><i style="background:#2a3548"></i>${esc(L(['нет данных', 'no data']))}</span>`
      : `<span><i style="background:hsl(${mode === 'temp' ? 220 : 120},75%,45%)"></i>${esc(L(['низк.', 'low']))}</span><span><i style="background:hsl(0,75%,45%)"></i>${esc(L(['предел', 'limit']))}</span><span><i style="background:#2a3548"></i>${esc(L(['нет данных', 'no data']))}</span>`;
    const hk = healthKeys();
    $('h-sub').textContent = `${hk.length} ${L(['полей ресурса в телеметрии', 'health fields in telemetry'])}`;
    healthDetail();
    if (HLOG.length !== hLogN) {
      hLogN = HLOG.length;
      $('h-log').innerHTML = HLOG.length ? `<tr><th class="r">t, s</th><th>${esc(L(['что', 'what']))}</th><th class="r">${esc(L(['было → стало', 'from → to']))}</th><th>${esc(t('h_when'))}</th></tr>` + HLOG.slice().reverse().map(e =>
        `<tr><td class="r">${f(e.t, 2)}</td><td data-ch="${esc(e.key)}">${esc(e.what)}</td><td class="r bad">${e.from != null ? `${f(e.from, 1)} → ${f(e.to, 1)} %` : DASH}</td><td style="white-space:normal">${esc(e.ctx)}</td></tr>`).join('')
        : `<tr><td class="muted">${esc(t('h_nolog'))}</td></tr>`;
    }
    $('h-log-sub').textContent = HLOG.length ? String(HLOG.length) : '';
    $('h-backend-events').innerHTML = backendEvents(s);
  },
});
