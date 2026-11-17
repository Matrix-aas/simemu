'use strict';
// SIMEMU Studio — bench build: the physical car (long block, turbo, bolt-ons, fuel, wear, gearbox, vehicle, ambient)
// chosen independently of the firmware image. Everything shown comes from /api/build; edits are staged on the server
// and re-create the plant at the next bench start. Firmware compatibility is advisory only.
Object.assign(T, {
  v_build: ['Конфигурация стенда', 'Bench build'],
  tip_h_build: ['Физический автомобиль на стенде (двигатель, турбина, доработки, КПП, кузов). Не зависит от прошивки. ● — есть правки, применятся при следующем запуске стенда. Клик — «Конфигурация стенда».',
    'Physical car on the bench (engine, turbo, mods, gearbox, body). Independent of the firmware. ● — staged edits, applied at the next bench start. Click — Bench build.'],
  p_bd_form: ['Сборка автомобиля', 'Car build'], p_bd_side: ['Запуск, пресеты, сохранённые сборки', 'Start, presets, saved builds'],
  p_bd_eff: ['Ожидаемый физический эффект (к той же сборке со стоковым навесным)', 'Expected physical effect (vs the same build with stock bolt-ons)'],
  p_bd_compat: ['Совместимость с загруженной прошивкой', 'Compatibility with the loaded firmware'],
  p_bd_specs: ['Данные компонентов и источники', 'Component data and sources'], p_bd_log: ['Журнал запусков стенда', 'Bench start log'],
  p_fwcompat: ['Совместимость образов с конфигурацией стенда', 'Images vs the bench build'],
  bd_engine: ['Двигатель (long block)', 'Engine (long block)'], bd_turbo: ['Турбина', 'Turbocharger'],
  bd_intake: ['Фильтр и патрубок', 'Filter and duct'], bd_pipe: ['патрубок, мм', 'duct, mm'], bd_exhaust: ['Выпуск (даунпайп + труба)', 'Exhaust (downpipe + pipe)'], bd_cat: ['катализатор', 'catalyst'],
  bd_ic: ['Интеркулер', 'Intercooler'], bd_plug: ['Свечи', 'Spark plugs'], bd_fuel: ['Топливо', 'Fuel'], bd_eth: ['этанол, % об.', 'ethanol, vol %'],
  bd_gearbox: ['КПП', 'Gearbox'], bd_awd: ['полный привод (Haldex 5)', 'AWD (Haldex 5)'],
  bd_tcu: ['Мехатроник (прошивка ТКУ)', 'Mechatronic (TCU firmware)'], bd_tcu_default: ['по умолчанию (стоковый образ DQ250 из vault/)', 'default (stock DQ250 image from vault/)'],
  bd_coding: ['кодирование', 'coding'],
  bd_tcu_note: ['Файлы .bin DQ250 из vault/firmwares/tcu/dq250/. Новый мехатроник (нет сохранённого EEPROM) студия готовит сама: кодирование (пусто — значение прошивки по умолчанию) и базовые установки выполняет сама прошивка. Передаточные числа — по калибровке ТКУ. Смена кодирования для уже подготовленного — «Подготовить заново» или запись в «Диагностике».',
    'DQ250 .bin files from vault/firmwares/tcu/dq250/. A new mechatronic (no saved EEPROM) is prepared by Studio: the firmware itself codes it (empty = firmware default) and runs its basic settings. Gear ratios follow the TCU calibration. To change the coding of a prepared unit: "Prepare again" or write it in Diagnostics.'],
  bd_vehicle: ['Автомобиль', 'Vehicle'], bd_tyre: ['радиус шины, мм (пусто — штатный)', 'tyre radius, mm (empty = standard)'], bd_amb: ['Атмосфера', 'Ambient'],
  bd_stockparts: ['свои (сток двигателя)', 'own (engine stock)'], bd_from: ['от', 'from'],
  bd_airbox: ['штатный корпус фильтра', 'stock airbox'], bd_cone: ['фильтр нулевого сопротивления (конусный)', 'high-flow cone filter'],
  bd_cat_oem: ['штатный', 'OEM'], bd_cat_sport: ['спорт 200 cpsi', 'sport 200 cpsi'], bd_cat_removed: ['удалён', 'removed'],
  bd_ic_stock: ['штатный', 'stock'], bd_ic_up: ['усиленный (Wagner/Forge/APR-класс)', 'upgraded (Wagner/Forge/APR class)'],
  bd_start: ['Запустить стенд с этой сборкой', 'Start the bench with this build'],
  tip_bd_start: ['Пересоздаёт физическую модель по сборке (холодный мотор, стоянка 12 ч, повреждения сброшены) и перезагружает ЭБУ и ТКУ на новой модели. Пользовательский образ ЭБУ сохраняется.',
    'Re-creates the plant from the build (cold engine, 12 h soak, damage reset) and reboots ECU and TCU on the new plant. A user-selected ECU image is kept.'],
  bd_discard: ['Отменить правки', 'Discard edits'], bd_save: ['Сохранить как', 'Save as'], bd_load: ['Загрузить', 'Load'], bd_del: ['Удалить', 'Delete'],
  bd_active: ['Работает', 'Running'], bd_staged: ['Правки ждут запуска стенда', 'Edits wait for the bench start'], bd_nostaged: ['правок нет', 'no edits'],
  bd_legacy: ['модель изменена старыми переключателями «Настройки стенда»', 'plant changed through the legacy Bench setup selectors'],
  bd_presets: ['Пресеты', 'Presets'], bd_saved: ['Сохранённые', 'Saved'], bd_none: ['нет', 'none'], bd_dir: ['Папка', 'Folder'],
  bd_started: ['Стенд запущен со сборкой', 'Bench started with build'], bd_staged_ok: ['Сборка подготовлена — применится при запуске стенда', 'Build staged — applies at the bench start'],
  bd_notes_none: ['Расхождений не найдено', 'No mismatches found'], bd_value: ['Сборка', 'Build'], bd_stock: ['Сток', 'Stock'],
  bd_pending: ['Пока не используется физикой (ждёт моделей свечи/предвоспламенения, MPI, старения катализатора, датчика детонации, AVS, топливной смеси)', 'Not yet used by the physics (awaits plug / pre-ignition, MPI, catalyst ageing, knock-sensor, AVS and fuel-blend models)'],
  bd_note: ['Сборка не выбирает прошивку, а загрузка образа не меняет сборку. Предупреждения совместимости ничего не блокируют: так ведёт себя настоящий автомобиль с чужой прошивкой. UNVERIFIED — значение без паспортных данных (ручка калибровки модели).',
    'The build never selects firmware and loading an image never changes the build. Compatibility warnings block nothing — exactly like a real car flashed with foreign software. UNVERIFIED = no data-sheet value (a model calibration knob).'],
  confirm_bd_start: ['Пересоздать физическую модель по сборке «{n}»? Мотор будет холодным, повреждения сброшены.', 'Re-create the plant from build "{n}"? The engine will be cold, damage reset.'],
});

let B = null; // last /api/build payload
let bBusy = false, bLast = 0, bSig = '';
async function buildApi(body) {
  B = await api('/api/build', body);
  bSig = '';
  renderBuild(); renderFwCompat();
  return B;
}
function buildPoll(force) {
  const now = performance.now();
  if (bBusy || (!force && now - bLast < 2000)) return;
  bLast = now; bBusy = true;
  api('/api/build').then(b => { B = b; renderBuild(); renderFwCompat(); }).catch(e => say(e.message, true)).finally(() => { bBusy = false; });
}
const bEdit = () => (B ? B.staged || B.active : null);
const bdFx = (v, d) => (isNum(v) ? (+v).toFixed(d) : DASH);
const bdOpt = (v, label, cur) => `<option value="${esc(v)}"${String(v) === String(cur) ? ' selected' : ''}>${esc(label)}</option>`;
const noteRow = n => `<div class="${n.level === 'warn' ? 'warn' : 'muted'}">${n.level === 'warn' ? '⚠' : 'ℹ'} ${esc(L([n.ru, n.en]))}</div>`;
const dispCc = e => (Math.PI / 4) * e.bore_mm * e.bore_mm * e.stroke_mm * 4 / 1000;
// Placeholder of the coding field: the default the preparation uses for the selected (or default) TCU image.
function tcuCodingDefault(c) {
  const imgs = B.tcu_images || [], i = imgs.find(x => x.file === c.tcu_image) || (!c.tcu_image && imgs[0]);
  return i && i.default_coding != null ? `${String(i.default_coding).padStart(6, '0')}` : L(['прошивки', 'firmware\'s']);
}

// ---------- form ----------
const PART_KM = [['engine_km', ['двигатель', 'engine']], ['di_injectors_km', ['форсунки DI', 'DI injectors']], ['mpi_injectors_km', ['форсунки MPI', 'MPI injectors']],
  ['spark_plugs_km', ['свечи', 'spark plugs']], ['turbo_km', ['турбина', 'turbo']], ['hpfp_km', ['ТНВД', 'HPFP']], ['catalyst_km', ['катализатор', 'catalyst']], ['clutches_km', ['сцепления', 'clutches']]];
function buildForm(c, cat) {
  const eng = cat.engines.find(e => e.id === c.engine) || {};
  const tur = (cat.turbos.find(t => t.spec.id === c.turbo) || {});
  const plug = cat.plugs.find(p => p.id === c.spark_plug) || {};
  const veh = cat.vehicles.find(v => v.id === c.vehicle) || {};
  const st = cat.stock || {};
  const sec = title => `<tr><th colspan="2" class="acc" style="padding-top:8px;text-transform:uppercase;font-size:11px">${esc(title)}</th></tr>`;
  const row = (label, ctl, fig) => `<tr><th style="white-space:normal;width:150px;vertical-align:top;padding-top:6px">${esc(label)}</th><td style="white-space:normal;padding:4px 6px">${ctl}${fig ? `<div class="note" style="margin-top:2px">${fig}</div>` : ''}</td></tr>`;
  const num = (k, v, step, w = 80, ph = '') => `<input data-k="${k}" type="number" min="0" step="${step}" value="${isNum(v) ? v : ''}" placeholder="${esc(ph)}" style="width:${w}px">`;
  const sel = (k, opts) => `<select data-k="${k}">${opts}</select>`;
  const part = k => sel('ancillaries.' + k, bdOpt('', t('bd_stockparts'), c.ancillaries[k] || '') + cat.engines.map(e => bdOpt(e.id, `${t('bd_from')} ${e.name}`, c.ancillaries[k] || '')).join(''));
  const thr = cat.throttles || {};
  return '<table>'
    + sec(L(['Двигатель и турбина', 'Engine and turbo']))
    + row(t('bd_engine'), sel('engine', cat.engines.map(e => bdOpt(e.id, e.name, c.engine)).join('')),
      `${bdFx(dispCc(eng), 0)} cm³ · ε ${bdFx(eng.compression_ratio, 1)} · ${bdFx(eng.rated_kw, 0)} kW / ${bdFx(eng.rated_nm, 0)} Nm · ${esc((eng.codes || []).join('/'))} · ${esc(L(['выпускной коллектор в ГБЦ', 'exhaust manifold integrated in the head']))}`)
    + row(t('bd_turbo'), sel('turbo', cat.turbos.map(x => bdOpt(x.spec.id, x.spec.name, c.turbo)).join('')),
      `${L(['макс. расход', 'max flow'])} ${bdFx(tur.max_flow_kg_s, 3)} kg/s · PR₀ ${bdFx(tur.max_pr, 2)} · ${L(['предел', 'limit'])} ${bdFx(tur.spec && tur.spec.max_speed_rpm / 1000, 0)} krpm · Ø ${bdFx(tur.spec && tur.spec.compressor_mm[1], 1)} mm`)
    + sec(L(['Впуск', 'Intake']))
    + row(t('bd_intake'), sel('intake.kind', bdOpt('airbox', t('bd_airbox'), c.intake.kind) + bdOpt('cone', t('bd_cone'), c.intake.kind)) + ` ${esc(t('bd_pipe'))} ` + num('intake.pipe_mm', c.intake.pipe_mm, 0.5, 72),
      `${L(['сток патрубок', 'stock duct'])} ${bdFx(st.intake_mm, 1)} mm · ${esc(L(['открытый фильтр: меньше перепад на фильтре, но подсос горячего подкапотного воздуха', 'cone filter: lower filter loss but hot under-hood air']))}`)
    + row(L(['Впускной коллектор', 'Intake manifold']), part('intake_manifold'), '')
    + row(L(['Дроссель', 'Throttle']), sel('ancillaries.throttle', bdOpt('gen3', `EA888 Gen3 Ø56 (CJSA/CJXC) · ${bdFx(thr.gen3 && thr.gen3[1], 1)} cm² @80°`, c.ancillaries.throttle)
      + bdOpt('cjha', `${L(['как в калибровке', 'as calibrated in'])} 264K/264F (CJHA/CJJG) · ${bdFx(thr.cjha && thr.cjha[1], 1)} cm² @80°`, c.ancillaries.throttle)), '')
    + sec(L(['Интеркулер и выпуск', 'Intercooler and exhaust']))
    + row(t('bd_ic'), sel('intercooler.core', bdOpt('stock', t('bd_ic_stock'), c.intercooler.core) + bdOpt('upgraded', t('bd_ic_up'), c.intercooler.core))
      + ` ${esc(L(['толщина, мм', 'core, mm']))} ` + num('intercooler.core_mm', c.intercooler.core_mm, 1, 65) + ` ${esc(L(['патрубки, мм', 'pipes, mm']))} ` + num('intercooler.pipe_mm', c.intercooler.pipe_mm, 0.5, 72),
      `${L(['сток', 'stock'])} ${bdFx(st.ic_core_mm, 0)} / ${bdFx(st.ic_pipe_mm, 1)} mm`)
    + row(t('bd_exhaust'), num('exhaust.pipe_mm', c.exhaust.pipe_mm, 1, 60) + ` mm ${esc(t('bd_cat'))} `
      + sel('exhaust.cat', ['oem', 'sport', 'removed'].map(k => bdOpt(k, t('bd_cat_' + k), c.exhaust.cat)).join('')), `${L(['сток', 'stock'])} ${bdFx(st.exhaust_mm, 0)} mm`)
    + sec(L(['Навесное: топливо и зажигание', 'Ancillaries: fuel and ignition']))
    + row(L(['Форсунки DI', 'DI injectors']), part('di_injectors'), esc(eng.di_injector || ''))
    + row(L(['Форсунки MPI', 'MPI injectors']), part('mpi_injectors'), esc(L(['одна и та же форсунка на 1.8 и 2.0 по калибровкам (2.30 мг/мс); в физике пока нет', 'same injector on 1.8 and 2.0 per the calibrations (2.30 mg/ms); not in the physics yet'])))
    + row(L(['ТНВД', 'HPFP']), part('hpfp'), '')
    + row(t('bd_plug'), sel('spark_plug', cat.plugs.map(p => bdOpt(p.id, p.name, c.spark_plug)).join('')),
      `${L(['калильное', 'heat range'])} ${bdFx(plug.heat_range, 0)} · ${L(['зазор', 'gap'])} ${bdFx(plug.gap_m * 1000, 1)} mm · ${esc(plug.electrodes || '')}`)
    + row(t('bd_fuel'), sel('fuel.ron', [92, 95, 98, 100].map(r => bdOpt(r, `RON ${r}`, c.fuel.ron)).join('') + (![92, 95, 98, 100].includes(c.fuel.ron) ? bdOpt(c.fuel.ron, `RON ${c.fuel.ron}`, c.fuel.ron) : ''))
      + ` ${esc(t('bd_eth'))} ` + num('fuel.ethanol_pct', c.fuel.ethanol_pct, 1, 60), '')
    + sec(L(['Пробег, км (0 у узла — как у двигателя)', 'Mileage, km (0 for a part = the engine\'s)']))
    + row(L(['Пробег', 'Mileage']), PART_KM.map(([k, n]) => `<label style="display:inline-block;margin:0 10px 4px 0">${esc(L(n))} ${num('mileage.' + k, c.mileage[k] || (k === 'engine_km' ? 0 : null), 1000, 90, k === 'engine_km' ? '0' : `= ${c.mileage.engine_km || 0}`)}</label>`).join(''),
      esc(L(['Износ: кольца/нагар, закоксовка форсунок, рост зазора свечей, КПД турбины, ТНВД, ёмкость катализатора, накладки сцеплений (коэффициенты UNVERIFIED).',
        'Wear: rings/deposits, injector coking, plug gap growth, turbo efficiency, HPFP, catalyst storage, clutch linings (UNVERIFIED rates).'])))
    + sec(L(['Трансмиссия, кузов, атмосфера', 'Driveline, body, ambient']))
    + row(t('bd_gearbox'), sel('gearbox', [['dq200', 'DQ200 0CW 7 (dry)'], ['dq250_02e', 'DQ250 02E 6 (wet)'], ['dq250_0d9', 'DQ250 0D9 6 (wet, MQB)'], ['dq381', 'DQ381 0GC 7 (wet)']].map(([k, n]) => bdOpt(k, n, c.gearbox)).join(''))
      + ` <label><input data-k="awd" type="checkbox"${c.awd ? ' checked' : ''}${c.gearbox === 'dq200' ? ' disabled' : ''}> ${esc(t('bd_awd'))}</label>`, '')
    + (c.gearbox === 'dq250_0d9' ? row(t('bd_tcu'), sel('tcu_image', bdOpt('', t('bd_tcu_default'), c.tcu_image || '') + (B.tcu_images || []).map(i => bdOpt(i.file, `${i.file} — ${i.asw_id.replace(/\s+\S+$/, '')}`, c.tcu_image || '')).join(''))
      + ` ${esc(t('bd_coding'))} ` + num('tcu_coding', c.tcu_coding, 1, 80, tcuCodingDefault(c)), esc(t('bd_tcu_note'))) : '')
    + row(t('bd_vehicle'), sel('vehicle', cat.vehicles.map(v => bdOpt(v.id, v.name, c.vehicle)).join('')) + ` ${esc(t('bd_tyre'))} `
      + num('tyre_radius_mm', c.tyre_radius_m != null ? +(c.tyre_radius_m * 1000).toFixed(1) : null, 0.5, 70),
      `${bdFx(veh.mass_fwd_kg, 0)} kg (FWD) · Cd ${bdFx(veh.cd, 3)} · A ${bdFx(veh.frontal_area_m2, 2)} m² · ${esc(veh.tyre || '')} r ${bdFx(veh.tyre_radius_m * 1000, 1)} mm`)
    + row(t('bd_amb'), `${esc(L(['Т, °C', 'T, °C']))} ` + num('ambient.temp_c', c.ambient.temp_c, 1, 55) + ` ${esc(L(['высота, м', 'altitude, m']))} ` + num('ambient.altitude_m', c.ambient.altitude_m, 10, 70)
      + ` ${esc(L(['влажн., %', 'humid., %']))} ` + num('ambient.humidity_pct', c.ambient.humidity_pct, 1, 55), '')
    + '</table>';
}
// Form → build JSON (edits a copy of the edited build; the server validates).
function formBuild() {
  const c = JSON.parse(JSON.stringify(bEdit()));
  for (const el of $$('#bd-form [data-k]')) {
    const k = el.dataset.k, raw = el.type === 'checkbox' ? el.checked : el.value;
    const n = typeof raw === 'string' && raw.trim() !== '' && Number.isFinite(+raw) ? +raw : null;
    if (k === 'tyre_radius_mm') { c.tyre_radius_m = n != null ? n / 1000 : null; continue; }
    if (k === 'tcu_image') { c.tcu_image = raw || null; continue; }
    if (k === 'tcu_coding') { c.tcu_coding = n != null && Number.isInteger(n) && n >= 0 && n <= 65535 ? n : null; continue; }
    const path = k.split('.'), last = path.pop();
    const o = path.reduce((a, p) => a[p], c);
    if (k.startsWith('mileage.')) o[last] = n ?? 0; // empty part = the engine's mileage
    else if (/^ancillaries\.(di_injectors|mpi_injectors|hpfp|intake_manifold)$/.test(k)) o[last] = raw || null;
    else o[last] = typeof raw === 'boolean' ? raw : el.type === 'number' || k === 'fuel.ron' ? (n ?? o[last]) : raw;
  }
  if (c.gearbox === 'dq200') c.awd = false;
  c.name = c.name.replace(/ \*$/, '') + ' *'; // edited
  return c;
}

// ---------- render ----------
function renderBuild() {
  if (!B || CUR_VIEW !== 'build') return;
  const c = bEdit(), cat = B.catalog;
  const sig = LANG + JSON.stringify(c) + JSON.stringify(B.saved) + JSON.stringify(B.tcu_images);
  const form = $('bd-form');
  if (sig !== bSig && !(document.activeElement && form.contains(document.activeElement) && document.activeElement.tagName === 'INPUT')) {
    bSig = sig;
    form.innerHTML = buildForm(c, cat);
  }
  const staged = !!B.staged;
  $('bd-status').innerHTML = `<div><span>${esc(t('bd_active'))}</span><b>${esc(B.active.name)}</b></div>`
    + `<div class="w"><span>${esc(L(['двигатель', 'engine']))}</span><b>${esc(S && S.engine_profile ? S.engine_profile.name : DASH)}</b></div>`
    + `<div class="w"><span>${esc(L(['трансмиссия', 'drivetrain']))}</span><b>${esc(S && S.drivetrain_profile ? S.drivetrain_profile.name : DASH)}</b></div>`
    + `<div class="w"><span>${esc(L(['правки', 'edits']))}</span><b class="${staged ? 'warn' : 'muted'}">${esc(staged ? `● ${t('bd_staged')}: ${B.edit_engine || ''}` : t('bd_nostaged'))}</b></div>`
    + (B.legacy ? `<div class="w"><span>legacy</span><b class="warn">${esc(t('bd_legacy'))}: ${esc(B.legacy)}</b></div>` : '');
  $('bd-discard').disabled = !staged;
  $('bd-presets').innerHTML = B.presets.map(p => `<button data-preset-b="${esc(p.id)}">${esc(L([p.ru, p.en]))}</button>`).join(' ');
  $('bd-saved').innerHTML = B.saved.length ? B.saved.map(n => `<tr><td style="white-space:normal">${esc(n)}</td><td class="r"><button data-load-b="${esc(n)}">${esc(t('bd_load'))}</button> <button class="danger" data-del-b="${esc(n)}">${esc(t('bd_del'))}</button></td></tr>`).join('')
    : `<tr><td class="muted">${esc(t('bd_none'))}</td></tr>`;
  $('bd-dir').textContent = `${t('bd_dir')}: ${B.builds_dir}`;
  const eff = B.effects || [];
  $('bd-effects').innerHTML = `<tr><th></th><th class="r">${esc(t('bd_value'))}</th><th class="r">${esc(t('bd_stock'))}</th><th class="r">Δ</th><th></th></tr>` + eff.map(e => {
    const d = e.value - e.stock, nd = Math.abs(e.value) >= 100 ? 0 : Math.abs(e.value) >= 10 ? 1 : 3;
    return `<tr><td style="white-space:normal">${esc(L([e.ru, e.en]))}</td><td class="r"><b>${bdFx(e.value, nd)}</b></td><td class="r muted">${bdFx(e.stock, nd)}</td>`
      + `<td class="r ${Math.abs(d) > 1e-9 ? 'acc' : 'muted'}">${Math.abs(d) > 1e-9 ? (d > 0 ? '+' : '') + bdFx(d, nd) : '='}</td><td class="muted">${esc(e.unit)}</td></tr>`;
  }).join('');
  const im = B.image || {}, f = im.facts || {};
  $('bd-compat').innerHTML = `<div class="kv"><div class="w"><span>${esc(L(['образ', 'image']))}</span><b>${esc(im.file || DASH)}</b></div>`
    + `<div><span>${esc(L(['номер', 'part']))}</span><b>${esc(f.part_number || DASH)}</b></div><div><span>${esc(L(['двигатель', 'engine']))}</span><b>${esc(f.engine_label || DASH)}</b></div>`
    + `<div><span>c_dpl</span><b>${bdFx(f.displacement_l, 2)} l</b></div><div><span>${esc(L(['форсунка (ip_ti_4_l)', 'injector (ip_ti_4_l)']))}</span><b>${bdFx(f.injector_cc_s, 1)} cm³/s</b></div>`
    + `<div><span>c_n_tcha_max_sp</span><b>${bdFx(f.n_tcha_max_sp_rpm, 0)} rpm</b></div></div>`
    + ((im.notes || []).length ? im.notes.map(noteRow).join('') : `<div class="good">${esc(t('bd_notes_none'))}</div>`);
  const ce = cat.engines.find(e => e.id === c.engine) || {}, ct = (cat.turbos.find(x => x.spec.id === c.turbo) || {}).spec || {};
  const cp = cat.plugs.find(p => p.id === c.spark_plug) || {}, cv = cat.vehicles.find(v => v.id === c.vehicle) || {};
  const kv = rows => rows.map(([a, b]) => `<div class="w"><span>${esc(a)}</span><b style="font-family:inherit">${esc(b ?? DASH)}</b></div>`).join('');
  const pd = B.pending || {};
  $('bd-specs').innerHTML = `<h4>${esc(ce.name || '')}</h4><div class="kv">${kv([['DI', ce.di_injector], ['MPI', ce.mpi], ['HPFP', ce.hpfp], [L(['поршни/шатуны', 'pistons/rods']), ce.pistons_rods],
    [L(['датчик детонации', 'knock sensor']), ce.knock_sensors], [L(['выпуск AVS, мм', 'exhaust AVS, mm']), (ce.exhaust_lift_mm || []).join(' / ')], [L(['источники', 'sources']), ce.sources]])}</div>`
    + `<h4>${esc(ct.name || '')}</h4><div class="kv">${kv([[L(['номера', 'part numbers']), ct.part_numbers], [L(['компрессор Ø вх/вых, мм', 'compressor Ø in/out, mm']), (ct.compressor_mm || []).map(v => bdFx(v, 1)).join(' / ')],
      [L(['турбина Ø вх/вых, мм', 'turbine Ø in/out, mm']), (ct.turbine_mm || []).map(v => bdFx(v, 1)).join(' / ')], [L(['вестгейт', 'wastegate']), ct.wastegate], [L(['подшипники', 'bearings']), ct.bearings], [L(['источники', 'sources']), ct.sources]])}</div>`
    + `<h4>${esc(cp.name || '')}</h4><div class="kv">${kv([[L(['источники', 'sources']), cp.sources]])}</div>`
    + `<h4>${esc(cv.name || '')}</h4><div class="kv">${kv([[L(['источники', 'sources']), cv.sources]])}</div>`
    + `<h4>${esc(t('bd_pending'))}</h4><div class="kv">${kv([[L(['калильное число / электроды', 'heat range / electrodes']), `${bdFx(pd.plug_heat_range, 0)} · ${pd.plug_electrodes || ''}`],
      [L(['нагар, мм', 'deposits, mm']), bdFx(pd.deposit_mm, 3)], ['MPI, mg/ms', bdFx(pd.mpi_mg_ms, 2)], [L(['ёмкость катализатора к новому', 'catalyst storage vs new']), bdFx(pd.cat_osc_factor, 2)],
      [L(['впускной коллектор', 'intake manifold']), pd.intake_manifold],
      [L(['топливо: RON / LHV / AFR / теплота испарения', 'fuel: RON / LHV / AFR / latent heat']), pd.fuel ? `${bdFx(pd.fuel.ron, 1)} / ${bdFx(pd.fuel.lhv_mj_kg, 2)} MJ/kg / ${bdFx(pd.fuel.afr_stoich, 2)} / ${bdFx(pd.fuel.latent_kj_kg, 0)} kJ/kg` : DASH],
      [L(['вестгейт', 'wastegate']), pd.wastegate]])}</div>`;
  $('bd-log').innerHTML = (B.log || []).slice().reverse().map(l => `<div class="mono" style="white-space:normal;word-break:break-all">${esc(l)}</div>`).join('') || `<div class="muted">${DASH}</div>`;
}
function renderFwCompat() {
  if (!B || CUR_VIEW !== 'fw') return;
  $('fw-compat').innerHTML = `<div class="note">${esc(L(['Сборка стенда', 'Bench build']))}: <b>${esc(B.active.name)}</b></div><table><tr><th>${esc(L(['образ', 'image']))}</th><th>${esc(L(['номер / двигатель', 'part / engine']))}</th><th></th></tr>`
    + (B.images || []).map(i => `<tr${S && i.file === S.image_file ? ' class="sel"' : ''}><td style="white-space:normal;word-break:break-all">${esc(i.file)}</td><td>${esc(i.part)}<br><span class="muted">${esc(i.label)}</span></td>`
      + `<td style="white-space:normal">${i.notes.length ? i.notes.map(noteRow).join('') : `<span class="good">${esc(t('bd_notes_none'))}</span>`}</td></tr>`).join('') + '</table>';
}

view({
  id: 'build',
  init() {
    $('bd-form').addEventListener('change', () => act(null, async () => { await buildApi({ action: 'stage', build: formBuild() }); say(t('bd_staged_ok')); }));
    $('bd-start').onclick = e => act(e.target, async () => {
      if (!(await confirmBox(t('confirm_bd_start').replace('{n}', bEdit().name), t('bd_start'), false))) return;
      await buildApi({ action: 'start' });
      say(`${t('bd_started')}: ${B.active.name}`);
    });
    $('bd-discard').onclick = e => act(e.target, () => buildApi({ action: 'discard' }));
    $('bd-saveb').onclick = e => act(e.target, async () => {
      const name = $('bd-name').value.trim();
      await buildApi({ action: 'save', name, build: formBuild() });
      say(`${t('bd_save')}: ${name}`);
    });
    $('p-bd-side').addEventListener('click', e => {
      const p = e.target.closest('[data-preset-b]'), l = e.target.closest('[data-load-b]'), d = e.target.closest('[data-del-b]');
      if (p) act(p, () => buildApi({ action: 'preset', preset: p.dataset.presetB }));
      if (l) act(l, () => buildApi({ action: 'load', name: l.dataset.loadB }));
      if (d) act(d, async () => { if (await confirmBox(`${t('bd_del')} «${d.dataset.delB}»?`)) await buildApi({ action: 'delete', name: d.dataset.delB }); });
    });
  },
  // The legacy Bench setup selectors show a composed plant as "build".
  first() { for (const id of ['cfg-engine', 'cfg-dt']) { const s = $(id); if (s && !s.querySelector('option[value="build"]')) s.insertAdjacentHTML('afterbegin', `<option value="build">${esc(t('v_build'))}</option>`); } if (typeof syncSetup === 'function' && S) syncSetup(S); },
  i18n() { bSig = ''; renderBuild(); },
  show() { buildPoll(true); },
  render() { buildPoll(false); },
});
HOOKS.push(s => {
  const b = s.bench_build;
  const el = $('h-build');
  if (el && b) el.textContent = `${b.name}${b.staged ? ' ●' : ''}${b.legacy ? ' (legacy)' : ''}`;
  if (CUR_VIEW === 'fw') buildPoll(false);
});
