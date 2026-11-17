// ---------- diagnostics: UDS / DTC, immobiliser, boards ----------
Object.assign(T, {
  read_dtc: ['Читать ошибки (19 02)', 'Read DTCs (19 02)'], clear_dtc: ['Стереть ошибки (OBD 04)', 'Clear DTCs (OBD 04)'], tester_present: ['TesterPresent (3E)', 'TesterPresent (3E)'],
  auto_tp: ['держать сессию (3E каждые 2 с)', 'keep session (3E every 2 s)'],
  uds_req: ['запрос', 'request'], uds_pending: ['ждём ответ', 'pending'], uds_resp: ['ответ', 'response'], uds_note: ['итог', 'result'],
  dtc_code: ['код', 'code'], dtc_desc: ['описание', 'description'], dtc_status: ['статус (ISO 14229)', 'status (ISO 14229)'], dtc_ftb: ['тип отказа', 'failure type'],
  dtc_none: ['Ошибок нет (или ещё не читали).', 'No DTCs (or not read yet).'], dtc_mfr: ['код VAG (показан как есть)', 'VAG code (shown raw)'],
  dtc_unknown: ['код SAE (нет в таблице UI)', 'SAE code (not in UI table)'],
  dtc_faults: ['только неисправности', 'faults only'], tip_dtc_faults: ['Скрыть коды без отказа: прошивка сообщает все коды, а у большинства лишь «тест не завершён» (биты 4/6). Неисправность = биты 0–3; «активна» / «была в этом цикле» — отказ сейчас, «история» — подтверждена раньше, сейчас тест проходит.',
    'Hide codes without a failure: the firmware reports every code, most only with "test not completed" (bits 4/6). Fault = bits 0–3; "active" / "failed this cycle" = failing now, "history" = confirmed earlier, the test passes now.'],
  dtc_shown: ['показано', 'shown'], ds_confirmed: ['история: подтверждена раньше, сейчас не отказывает', 'history: confirmed earlier, not failing now'], ds_pending: ['ожидает подтверждения', 'pending'], ds_failed: ['активна (отказ сейчас)', 'active (failing now)'],
  ds_cycle: ['была в этом цикле', 'failed this cycle'], ds_since: ['история: была после стирания', 'history: failed since clear'],
  ds_ok: ['отказа нет (тест не завершён)', 'no failure (test not completed)'], ds_ok2: ['отказа нет', 'no failure'],
  confirm_clear: ['Стереть память неисправностей? Команда OBD 04 идёт на функциональный адрес 0x7DF: стирают все блоки с OBD (ЭБУ и ТКУ), как со сканером; UDS 14 эти прошивки не поддерживают. Стирает сама прошивка.', 'Clear the fault memory? OBD 04 goes to the functional address 0x7DF: every OBD unit (ECU and TCU) clears, as with a scan tool; these firmwares have no UDS 14. The firmware does the clearing.'],
  uds_note_expl: ['Запросы идут по CAN через ISO-TP, как от внешнего сканера. Память ошибок хранит и обрабатывает прошивка, интерфейс лишь расшифровывает коды.',
    'Requests travel over CAN via ISO-TP, like an external scan tool. The firmware keeps the fault memory; the UI only decodes the codes.'],
  immo: ['Иммобилайзер WFS5', 'Immobiliser WFS5'],
  immo_expl: ['Ключи иммобилайзера: блоки (ЭБУ 0x010, КПП 0x012) каждые ~100 мс шлют случайный вызов, виртуальная приборка отвечает (0x011 / 0x013) подписью AES-128. Без верного ответа ЭБУ запрещает пуск, КПП держит сцепления разомкнутыми.',
    'Immobiliser: the units (ECU 0x010, TCU 0x012) send a random challenge every ~100 ms, the virtual cluster answers (0x011 / 0x013) with an AES-128 signature. Without a valid answer the ECU blocks the start and the TCU keeps the clutches open.'],
  boards: ['Платы ЭБУ / ТКУ', 'ECU / TCU boards'],
  boards_note: ['STM и «запусков задач» растут — прошивка живёт. Перезапуски > 0 — прошивка сама перезагружала процессор.', 'STM and task runs increasing = firmware alive. Resets > 0 = the firmware restarted the CPU itself.'],
  b_present: ['плата', 'board'], b_sw: ['ПО', 'software'], b_pc: ['счётчик команд PC', 'program counter'], b_region: ['область памяти', 'memory region'], b_stm: ['таймер STM, мс', 'STM timer, ms'],
  b_irq: ['прерываний', 'interrupts'], b_resets: ['перезапусков', 'resets'], b_tasks: ['задач ОС', 'RTOS tasks'], b_runs: ['запусков задач', 'task runs'], b_can: ['CAN TX', 'CAN TX'], b_err: ['ошибка', 'error'], b_alive: ['живость', 'alive'],
  alive_yes: ['работает', 'running'], alive_no: ['стоит', 'stalled'],
  dtc_hex: ['байты', 'bytes'], dtc_fmt_note: ['Расшифровка по SAE J2012-DA: 2 байта — код, 3-й — тип отказа. Блоки не сообщают формат DTC (запрос 19 01 → NRC 0x12), поэтому для кодов производителя ориентируйтесь на байты.',
    'Decoded as SAE J2012-DA: 2 bytes code + failure-type byte. The units do not report their DTC format (19 01 → NRC 0x12), so for manufacturer codes use the raw bytes.'],
  raw_ph: ['hex, напр. 22 F1 90', 'hex, e.g. 22 F1 90'], raw_send: ['Отправить', 'Send'], raw_bad: ['Запрос: 1…64 байт в hex.', 'Request: 1…64 hex bytes.'],
  tip_raw: ['Произвольный запрос тестера (длиннее 7 байт — несколькими кадрами ISO-TP): 22 F1 90 — VIN, 22 F1 87 — номер детали, 10 03 — расширенная сессия, 01 0C — обороты OBD. Ответ — строка «ответ».',
    'Any tester request (longer than 7 bytes: multi-frame ISO-TP): 22 F1 90 VIN, 22 F1 87 part number, 10 03 extended session, 01 0C OBD rpm. The reply is in the "response" line.'],
  obd_h: ['Живые данные ЭБУ (OBD-II, режим 01) — значения считает прошивка', 'ECU live data (OBD-II mode 01) — values computed by the firmware'],
  obd_go: ['▶ Опрос', '▶ Poll'], obd_stop: ['■ Стоп', '■ Stop'], obd_sup: ['поддерживаемые PID', 'supported PIDs'],
  obd_note: ['Запросы 01 xx идут по CAN к ЭБУ (0x7E0), как от сканера OBD; формулы — SAE J1979. Один запрос за раз, полный круг ≈ 2–3 с. Коррекции STFT/LTFT и угол опережения — это величины прошивки, а не физики.',
    'Requests 01 xx go over CAN to the ECU (0x7E0) like an OBD scan tool; formulas per SAE J1979. One request at a time, a full cycle ≈ 2–3 s. STFT/LTFT trims and spark advance are firmware values, not physics.'],
});
Object.assign(T, {
  ws_h: ['Сервис КПП: кодирование и базовые установки (как VCDS / ODIS)', 'Gearbox service: coding and basic settings (like VCDS / ODIS)'],
  ws_coding: ['Кодирование', 'Coding'], ws_write: ['Записать кодирование', 'Write coding'], ws_read: ['Прочитать', 'Read'],
  ws_basic: ['Базовая установка', 'Basic setting'], ws_start: ['▶ Запустить', '▶ Start'], ws_status: ['Статус', 'Status'], ws_stop: ['■ Стоп', '■ Stop'],
  ws_cur_coding: ['кодирование в блоке', 'coding in the unit'], ws_rstat: ['статус базовой установки (22 0102)', 'basic setting status (22 0102)'],
  ws_log: ['обмен с блоком', 'exchange with the unit'],
  ws_note: ['Всё идёт по UDS через CAN (0x7E1/0x7E9), как от сканера; решения принимает прошивка: проверяет условия, сама двигает вилки и сцепления, сама сохраняет результат в EEPROM. '
    + 'Кодирование: расширенная сессия 10 03 → код мастерской 2E F198 (без него прошивка отвечает NRC 0x24) → 2E 0600. Базовые установки: двигатель на холостом, P, тормоз, КПП прогрета; '
    + 'сначала 0x0380 (вилки, ~75 с), затем 0x04D4 (точки синхронизации, ~10 с). После — выключить зажигание на ~15 с: блок сохраняет адаптации при выключении. '
    + 'EEPROM блока хранится между запусками studio (build/studio_state/).',
    'Everything goes over UDS on CAN (0x7E1/0x7E9) like a scan tool; the firmware decides: it checks the conditions, moves the forks and clutches itself and stores the result in its EEPROM. '
    + 'Coding: extended session 10 03 → workshop code 2E F198 (without it the firmware answers NRC 0x24) → 2E 0600. Basic settings: engine idling, P, brake, gearbox warm; '
    + 'first 0x0380 (forks, ~75 s), then 0x04D4 (synchronisation points, ~10 s). Then ignition off for ~15 s: the unit stores its adaptations at shut-down. '
    + 'The unit\'s EEPROM is kept between studio runs (build/studio_state/).'],
  ws_tcu_only: ['Только для ТКУ DQ250 0D9 (адреса и процедуры из прошивки 0D9300012F).', 'DQ250 0D9 TCU only (addresses and procedures from the 0D9300012F firmware).'],
  wp_h: ['Подготовка мехатроника (EEPROM)', 'Mechatronic preparation (EEPROM)'],
  wp_prepare: ['Подготовить заново', 'Prepare again'], wp_erase: ['Стереть EEPROM (как новый мехатроник)', 'Erase EEPROM (as a new mechatronic)'],
  wp_running: ['идёт подготовка', 'preparing'], wp_ok: ['подготовлен самой прошивкой', 'prepared by the firmware itself'], wp_fail: ['подготовка не выполнена', 'preparation failed'],
  wp_none: ['записи о подготовке нет', 'no preparation record'], wp_log: ['ход подготовки (обмен тестера и шаги)', 'preparation log (tester exchanges and steps)'],
  wp_coding: ['кодирование', 'coding'], wp_routines: ['базовые установки', 'basic settings'], wp_time: ['время', 'time'], wp_car: ['автомобиль в мастерской', 'workshop car'],
  confirm_wp_prepare: ['Стереть EEPROM мехатроника и подготовить его заново (кодирование и базовые установки выполнит сама прошивка в отдельной симуляции, ≈ 1–2 мин)? ТКУ перезагрузится.',
    'Erase the mechatronic EEPROM and prepare it again (the firmware itself codes it and runs its basic settings in a separate simulation, ≈ 1–2 min)? The TCU reboots.'],
  confirm_wp_erase: ['Стереть EEPROM мехатроника (как новый, без кодирования и базовых установок)? ТКУ перезагрузится; подготовка — кнопкой «Подготовить заново».',
    'Erase the mechatronic EEPROM (as new, no coding, no basic settings)? The TCU reboots; prepare it with "Prepare again".'],
  wp_note: ['Дампа EEPROM DQ250 нет: образ стартует со стёртой микросхемой (без кодирования и выученных положений). Студия делает то, что сделала бы мастерская: в отдельной симуляции этой же машины (сборка стенда, загруженный образ ЭБУ, двигатель прогрет, P, тормоз) тестер выполняет 10 03 → 2E F198 → 2E 0600 → выключение зажигания → 31 01 0380 / 04D4 (опрос 22 0102 до 0x10, 31 02) → выключение зажигания. Все решения и все байты EEPROM — прошивки; результат сохраняется для образа ТКУ (build/studio_state/) и показывается при каждой загрузке. Кодирование — из сборки стенда (по умолчанию 000024 для 0D9300012F, иначе значение прошивки).',
    'No DQ250 EEPROM dump exists: an image starts with an erased part (no coding, no learnt positions). The studio does what a workshop would: in a separate simulation of the same car (bench build, loaded ECU image, engine warm, P, brake) the tester runs 10 03 → 2E F198 → 2E 0600 → ignition off → 31 01 0380 / 04D4 (poll 22 0102 to 0x10, 31 02) → ignition off. Every decision and every EEPROM byte is the firmware\'s; the result is kept per TCU image (build/studio_state/) and shown at every load. Coding from the bench build (default 000024 for 0D9300012F; otherwise the firmware\'s own).'],
  dtc_vag: ['номер VAG; код SAE — по таблице самой прошивки ТКУ', 'VAG number; SAE code from the TCU firmware\'s own table'],
});
function wpHTML(s) {
  const pr = s.tcu_prep || {}, rec = pr.record, pg = pr.progress || {};
  const st = pr.running ? `<b class="warn">${esc(t('wp_running'))}: ${esc(pg.step || '')}</b><div class="prog"><i style="width:${f(100 * (pg.frac || 0), 0)}%"></i></div>`
    : rec ? `<b class="${rec.ok ? 'good' : 'bad'}">${esc(t(rec.ok ? 'wp_ok' : 'wp_fail'))}</b>` : `<span class="muted">${esc(t('wp_none'))}</span>`;
  const rows = rec ? [[t('wp_coding'), `${rec.coding == null ? DASH : String(rec.coding).padStart(6, '0')} — ${rec.coding_source || ''}`],
    [t('wp_routines'), (rec.routines || []).map(([r, x]) => `0x${r.toString(16).toUpperCase().padStart(4, '0')}: ${x == null ? DASH : '0x' + x.toString(16).toUpperCase().padStart(2, '0') + ' ' + L(RSTAT[x] || ['?', '?'])}`).join(', ') || DASH],
    [t('wp_car'), `${rec.ecu_image} · ${rec.build}`], [t('wp_time'), `${f(rec.sim_s, 0)} s ${L(['модели за', 'simulated in'])} ${f(rec.wall_s, 0)} s · ${new Date(rec.unix_s * 1000).toLocaleString()}`]]
    .concat(rec.failed ? [[L(['ошибка', 'failure']), rec.failed]] : []) : [];
  return `<h4>${esc(t('wp_h'))}</h4><div>${st}</div>`
    + (rows.length ? `<div class="kv">${rows.map(([a, b]) => `<div class="w"><span>${esc(a)}</span><b style="font-family:inherit">${esc(b)}</b></div>`).join('')}</div>` : '')
    + (s.tcu_notes || []).map(n => `<div class="note ${/не выполнена|failed/.test(n) ? 'warn' : /подготовлен/.test(n) ? 'acc' : ''}">${esc(n)}</div>`).join('')
    + (rec && rec.log && rec.log.length ? `<details><summary class="muted">${esc(t('wp_log'))} (${rec.log.length})</summary><div class="scroll mono" style="max-height:240px;white-space:pre">${esc(rec.log.join('\n'))}</div></details>` : '');
}
// Basic settings of the DQ250 0D9300012F (routine control 31 01 <rid> 04 00 00; docs/DQ250_IO_MAP.md §8).
const DQ250_RIDS = [[0x0380, 'Адаптация вилок переключения (конечные положения всех передач)', 'Gear actuator adaptation (end positions of every gear)'],
  [0x04D4, 'Адаптация точек синхронизации (сцепления замкнуты, нагрузка — двигатель)', 'Synchronisation point adaptation (clutches closed, engine as the load)'],
  [0x0317, 'Сброс адаптаций сцеплений', 'Reset clutch adaptation values'], [0x0384, 'Сброс адаптации вилок', 'Reset gear actuator adaptation'],
  [0x0386, 'Сброс блока адаптаций 0x0386', 'Reset adaptation block 0x0386'], [0x0388, 'Сброс блока адаптаций 0x0388', 'Reset adaptation block 0x0388'],
  [0x0389, 'Сброс блока адаптаций 0x0389', 'Reset adaptation block 0x0389'], [0x038A, 'Сброс блока адаптаций 0x038A', 'Reset adaptation block 0x038A'],
  [0x038B, 'Сброс блока адаптаций 0x038B', 'Reset adaptation block 0x038B'], [0x038C, 'Сброс блока адаптаций 0x038C', 'Reset adaptation block 0x038C'],
  [0x03B0, 'Сброс блока адаптаций 0x03B0', 'Reset adaptation block 0x03B0']];
const RSTAT = { 0: ['нет', 'none'], 0xC0: ['выполняется', 'running'], 0x10: ['завершена успешно', 'completed'], 0x60: ['не удалась', 'failed'], 0x80: ['прервана', 'aborted'] };
const WS = { poll: 0 };
function wsHTML() {
  const cod = LS.get('ws_coding', 24);
  return `<div data-wsprep></div><div class="row"><button data-wsprepact="tcu_prepare" class="primary">${esc(t('wp_prepare'))}</button><button data-wsprepact="tcu_eeprom_erase" class="danger">${esc(t('wp_erase'))}</button></div>
    <div class="note">${esc(t('wp_note'))}</div>
    <h4>${esc(t('ws_h'))}</h4><div class="note">${esc(t('ws_tcu_only'))}</div>
    <div class="row"><span>${esc(t('ws_coding'))}</span><input data-wscod type="number" min="0" max="65535" style="width:110px" value="${esc(String(cod))}">
    <button data-ws="write_coding" class="primary">${esc(t('ws_write'))}</button><button data-ws="read_coding">${esc(t('ws_read'))}</button></div>
    <div class="row"><span>${esc(t('ws_basic'))}</span><select data-wsrid>${DQ250_RIDS.map(([r, ru, en]) => `<option value="${r}">${hexReq([r >> 8, r & 0xFF]).replace(' ', '')} — ${esc(L([ru, en]))}</option>`).join('')}</select>
    <button data-ws="basic_setting" class="primary">${esc(t('ws_start'))}</button><button data-ws="routine_status">${esc(t('ws_status'))}</button><button data-ws="routine_stop" class="danger">${esc(t('ws_stop'))}</button></div>
    <div class="kv" data-wsst></div><div class="scroll" style="max-height:220px"><table data-wslog></table></div><div class="note">${esc(t('ws_note'))}</div>`;
}
function wsBind(el) {
  el.querySelectorAll('[data-wsprepact]').forEach(b => { b.onclick = () => act(b, async () => {
    const action = b.dataset.wsprepact;
    if (!(await confirmBox(t(action === 'tcu_prepare' ? 'confirm_wp_prepare' : 'confirm_wp_erase'), b.textContent, action !== 'tcu_prepare'))) return;
    onState(await api('/api/preset', { action }));
    say(`${b.textContent}: ${(S.tcu_notes || [])[0] || ''}`);
  }); });
  el.querySelectorAll('[data-ws]').forEach(b => { b.onclick = () => act(b, async () => {
    const svc = b.dataset.ws, rid = Number(el.querySelector('[data-wsrid]').value), value = Number(el.querySelector('[data-wscod]').value);
    if (svc === 'write_coding') {
      if (!Number.isInteger(value) || value < 0 || value > 65535) throw new Error(t('ws_coding'));
      LS.set('ws_coding', value);
    }
    onState(await api('/api/uds', { target: 'tcu', service: svc, value, rid }));
    WS.poll = svc === 'basic_setting' ? rid : svc === 'routine_stop' ? 0 : WS.poll;
    say(`UDS ${svc} → TCU`);
  }); });
}
function wsRender(el, u, s) {
  const st = el.querySelector('[data-wsst]');
  if (!st) return;
  const wp = el.querySelector('[data-wsprep]'), h = wpHTML(s);
  if (wp.dataset.h !== h && !wp.querySelector('details[open]')) { wp.dataset.h = h; wp.innerHTML = h; }
  const dq250 = !!(s.tcu && s.tcu.present && s.drivetrain_profile && /^0D9/.test(s.drivetrain_profile.tcu_code || ''));
  el.querySelectorAll('[data-wsprepact]').forEach(b => { b.disabled = !dq250; });
  const rs = u.routine_status;
  st.innerHTML = [[t('ws_cur_coding'), u.coding == null ? DASH : String(u.coding).padStart(6, '0')],
    [t('ws_rstat'), rs == null ? DASH : `0x${rs.toString(16).toUpperCase().padStart(2, '0')} ${L(RSTAT[rs] || ['?', '?'])}`]]
    .map(([a, b], i) => `<div class="w"><span>${esc(a)}</span><b class="${i === 1 && rs === 0x10 ? 'good' : i === 1 && (rs === 0x60 || rs === 0x80) ? 'bad' : ''}">${esc(b)}</b></div>`).join('');
  const log = u.log || [];
  el.querySelector('[data-wslog]').innerHTML = log.length ? `<tr><th>${esc(t('ws_log'))}: ${esc(t('uds_req'))}</th><th>${esc(t('uds_resp'))}</th><th>${esc(t('uds_note'))}</th></tr>`
    + log.map(x => `<tr><td class="mono">${esc(x.request)}</td><td class="mono">${esc(x.response || '…')}</td><td class="wrap ${/^NRC/.test(x.note) ? 'bad' : ''}">${esc(x.note || '')}</td></tr>`).join('') : '';
  // a running basic setting is polled like a scan tool does (22 0102 every ~1 s)
  if (WS.poll && !u.pending && rs === 0xC0 && !WS.busy && performance.now() - (WS.at || 0) > 1000) {
    WS.busy = true; WS.at = performance.now();
    api('/api/uds', { target: 'tcu', service: 'routine_status' }).catch(e => say(e.message, true)).finally(() => { WS.busy = false; });
  }
  if (rs !== 0xC0 && !u.pending) WS.poll = 0;
}
Object.assign(T, {
  p_mech: ['Мехатроник КПП: клапаны, давление, сцепления, вилки', 'Gearbox mechatronic: valves, pressure, clutches, forks'],
  mech_note: ['Слева — что заказывает прошивка ТКУ (скважность, ток), справа — что получилось в железе (ток шунта, давления, ходы). На моторном стенде КПП снята — гидравлика и механика стоят.',
    'Left: what the TCU firmware commands (duty, current); right: what the hardware does (shunt current, pressures, travel). On the engine dyno the gearbox is removed — hydraulics and mechanics are idle.'],
});
function mechHTML(s) {
  const p = s.plant || {}, o = p.tcu_outputs || {}, names = p.solenoid_names || [];
  const v = (x, d) => (isNum(x) ? x.toFixed(d) : DASH);
  const sol = names.map((n, i) => [n, (o.solenoid_duty || [])[i], (o.solenoid_current_a || [])[i], (p.solenoid_current_a || [])[i]]).filter(r => r[0] && r[0] !== '-');
  const kv = rows => `<div class="kv">${rows.map(k => `<div data-ch="${esc(k)}"><span>${esc(chName(k))}</span><b>${esc(fmtCh(k, FLAT.get(k)))} ${esc(meta(k).unit || '')}</b></div>`).join('')}</div>`;
  const keys = pfx => Array.from(FLAT.keys()).filter(k => k.startsWith(pfx) && typeof FLAT.get(k) === 'number');
  return `<div class="row" style="align-items:flex-start;gap:14px"><div style="flex:1;min-width:300px"><table><tr><th>${esc(L(['клапан', 'valve']))}</th><th class="r">${esc(L(['ШИМ, %', 'duty, %']))}</th><th class="r">${esc(L(['ток уставка, А', 'current set, A']))}</th><th class="r">${esc(L(['ток шунт, А', 'shunt, A']))}</th></tr>`
    + sol.map(([n, d, c, a]) => `<tr><td style="font-family:inherit">${esc(n)}</td><td class="r">${isNum(d) ? (100 * d).toFixed(1) : DASH}</td><td class="r">${v(c, 3)}</td><td class="r">${v(a, 3)}</td></tr>`).join('') + '</table></div>'
    + `<div style="flex:1;min-width:300px">${kv(['plant.gearbox_hydraulic_bar', 'plant.tcu_outputs.pump_motor_duty', 'plant.pump_rpm', 'plant.pump_current_a', 'plant.mechatronic_oil_c'].concat(keys('plant.sub_gearbox_bar'), keys('plant.clutch_bar')).filter(k => FLAT.has(k)))}</div>`
    + `<div style="flex:1;min-width:300px">${kv(keys('plant.clutch_travel_mm').concat(keys('plant.clutch_kiss_mm'), keys('plant.fork_mm'), keys('plant.shaft_gear'), ['plant.driving_gear', 'driver.selector']).filter(k => FLAT.has(k)))}</div></div>`;
}
// SAE J1979 mode-01 PIDs shown in the live-data table: [pid, ru, en, unit, decode(bytes)].
const OBD_PIDS = [[0x04, 'Расчётная нагрузка', 'Calculated load', '%', b => (b[0] * 100) / 255], [0x05, 'Т охлаждающей жидкости', 'Coolant temp', '°C', b => b[0] - 40],
  [0x06, 'Краткосрочная коррекция топлива (STFT)', 'Short-term fuel trim (STFT)', '%', b => ((b[0] - 128) * 100) / 128], [0x07, 'Долгосрочная коррекция топлива (LTFT)', 'Long-term fuel trim (LTFT)', '%', b => ((b[0] - 128) * 100) / 128],
  [0x0B, 'Давление во впуске', 'Intake manifold pressure', 'kPa', b => b[0]], [0x0C, 'Обороты', 'Engine speed', 'rpm', b => (256 * b[0] + b[1]) / 4], [0x0D, 'Скорость', 'Vehicle speed', 'km/h', b => b[0]],
  [0x0E, 'Угол опережения зажигания (цил. 1)', 'Timing advance (cyl 1)', '°BTDC', b => b[0] / 2 - 64], [0x0F, 'Т воздуха на впуске', 'Intake air temp', '°C', b => b[0] - 40],
  [0x11, 'Положение дросселя', 'Throttle position', '%', b => (b[0] * 100) / 255], [0x1F, 'Время с пуска', 'Run time since start', 's', b => 256 * b[0] + b[1]],
  [0x23, 'Давление в рампе', 'Fuel rail pressure', 'MPa', b => ((256 * b[0] + b[1]) * 10) / 1000], [0x33, 'Атмосферное давление', 'Barometric pressure', 'kPa', b => b[0]],
  [0x42, 'Напряжение питания ЭБУ', 'ECU supply voltage', 'V', b => (256 * b[0] + b[1]) / 1000], [0x43, 'Абсолютная нагрузка', 'Absolute load', '%', b => ((256 * b[0] + b[1]) * 100) / 255],
  [0x44, 'Заданная лямбда', 'Commanded lambda', 'λ', b => ((256 * b[0] + b[1]) * 2) / 65536], [0x45, 'Относит. положение дросселя', 'Relative throttle', '%', b => (b[0] * 100) / 255],
  [0x46, 'Т окружающего воздуха', 'Ambient air temp', '°C', b => b[0] - 40], [0x5C, 'Т масла', 'Oil temp', '°C', b => b[0] - 40]];
const OBD = { on: false, sup: null, probe: 0, i: 0, last: null, busy: false, sentAt: 0, vals: new Map() };
const hexReq = bytes => bytes.map(b => b.toString(16).toUpperCase().padStart(2, '0')).join(' ');
const parseHex = s => (s || '').trim().split(/\s+/).filter(Boolean).map(h => parseInt(h, 16));
async function udsRaw(tgt, bytes) { return api('/api/uds', { target: tgt, service: 'raw', data: bytes }); }
function obdTick(s) {
  if (!OBD.on || OBD.busy) return;
  const u = s.uds_ecu || {};
  if (u.pending && performance.now() - OBD.sentAt < 2000) return;
  if (OBD.last && u.request === OBD.last && !u.pending) {
    const b = parseHex(u.response_hex), pid = parseHex(OBD.last)[1];
    if (b[0] === 0x41 && b[1] === pid) {
      if (pid % 0x20 === 0) { OBD.sup = OBD.sup || new Set(); for (let i = 0; i < 32; i++) if (b[2 + (i >> 3)] & (0x80 >> (i & 7))) OBD.sup.add(pid + i + 1); }
      else OBD.vals.set(pid, { b: b.slice(2), t: s.plant ? s.plant.sim_time_s : NaN });
    } else if (pid % 0x20 === 0) OBD.probe = 99; // no further support blocks
    OBD.last = null;
  }
  let pid;
  if (OBD.probe < 3 && (OBD.probe === 0 || (OBD.sup && OBD.sup.has(OBD.probe * 0x20)))) pid = 0x20 * OBD.probe++;
  else {
    OBD.probe = 99;
    const list = OBD_PIDS.filter(p => OBD.sup && OBD.sup.has(p[0]));
    if (!list.length) return;
    pid = list[OBD.i++ % list.length][0];
  }
  OBD.last = hexReq([0x01, pid]); OBD.busy = true; OBD.sentAt = performance.now();
  udsRaw('ecu', [0x01, pid]).catch(e => { OBD.on = false; say(e.message, true); }).finally(() => { OBD.busy = false; });
}
function obdTable() {
  const rows = OBD_PIDS.filter(p => OBD.sup && OBD.sup.has(p[0])).map(([pid, ru, en, unit, dec]) => {
    const v = OBD.vals.get(pid);
    return `<tr><td class="muted">${hexReq([pid])}</td><td style="font-family:inherit">${esc(L([ru, en]))}</td><td class="r"><b>${!v ? DASH : v.b.length && v.b.every(x => x === 0xFF) ? esc(L(['нет у ЭБУ (FF)', 'n/a (FF)'])) : esc(fmtN(dec(v.b)))}</b></td><td>${esc(unit)}</td><td class="muted">${v ? esc(hexReq(v.b)) : ''}</td><td class="r muted">${v ? f(v.t, 1) : ''}</td></tr>`;
  });
  return rows.length ? `<tr><th>PID</th><th></th><th class="r">${esc(L(['значение', 'value']))}</th><th></th><th>${esc(L(['байты', 'bytes']))}</th><th class="r">t, s</th></tr>${rows.join('')}` : '';
}
// Generic SAE J2012 descriptions (subset). Manufacturer-specific codes are shown raw.
const J2012 = {
  P0010: '"A" Camshaft Position Actuator Circuit (Bank 1)', P0011: '"A" Camshaft Position - Timing Over-Advanced or System Performance (Bank 1)', P0012: '"A" Camshaft Position - Timing Over-Retarded (Bank 1)',
  P0013: '"B" Camshaft Position - Actuator Circuit (Bank 1)', P0014: '"B" Camshaft Position - Timing Over-Advanced or System Performance (Bank 1)', P0016: 'Crankshaft Position - Camshaft Position Correlation (Bank 1 Sensor A)',
  P0017: 'Crankshaft Position - Camshaft Position Correlation (Bank 1 Sensor B)', P0030: 'HO2S Heater Control Circuit (Bank 1 Sensor 1)', P0036: 'HO2S Heater Control Circuit (Bank 1 Sensor 2)',
  P0068: 'MAP/MAF - Throttle Position Correlation', P0087: 'Fuel Rail/System Pressure - Too Low', P0088: 'Fuel Rail/System Pressure - Too High', P0089: 'Fuel Pressure Regulator 1 Performance',
  P0090: 'Fuel Pressure Regulator 1 Control Circuit', P0101: 'Mass or Volume Air Flow Circuit Range/Performance', P0102: 'Mass or Volume Air Flow Circuit Low Input', P0103: 'Mass or Volume Air Flow Circuit High Input',
  P0105: 'Manifold Absolute Pressure/Barometric Pressure Circuit', P0106: 'Manifold Absolute Pressure/Barometric Pressure Circuit Range/Performance', P0107: 'Manifold Absolute Pressure/Barometric Pressure Circuit Low Input',
  P0108: 'Manifold Absolute Pressure/Barometric Pressure Circuit High Input', P0111: 'Intake Air Temperature Sensor 1 Circuit Range/Performance', P0112: 'Intake Air Temperature Sensor 1 Circuit Low',
  P0113: 'Intake Air Temperature Sensor 1 Circuit High', P0116: 'Engine Coolant Temperature Circuit Range/Performance', P0117: 'Engine Coolant Temperature Circuit Low', P0118: 'Engine Coolant Temperature Circuit High',
  P0121: 'Throttle/Pedal Position Sensor/Switch "A" Circuit Range/Performance', P0122: 'Throttle/Pedal Position Sensor/Switch "A" Circuit Low', P0123: 'Throttle/Pedal Position Sensor/Switch "A" Circuit High',
  P0128: 'Coolant Thermostat (Coolant Temperature Below Thermostat Regulating Temperature)', P0130: 'O2 Sensor Circuit (Bank 1 Sensor 1)', P0131: 'O2 Sensor Circuit Low Voltage (Bank 1 Sensor 1)',
  P0132: 'O2 Sensor Circuit High Voltage (Bank 1 Sensor 1)', P0133: 'O2 Sensor Circuit Slow Response (Bank 1 Sensor 1)', P0134: 'O2 Sensor Circuit No Activity Detected (Bank 1 Sensor 1)',
  P0135: 'O2 Sensor Heater Circuit (Bank 1 Sensor 1)', P0136: 'O2 Sensor Circuit (Bank 1 Sensor 2)', P0137: 'O2 Sensor Circuit Low Voltage (Bank 1 Sensor 2)', P0138: 'O2 Sensor Circuit High Voltage (Bank 1 Sensor 2)',
  P0141: 'O2 Sensor Heater Circuit (Bank 1 Sensor 2)', P0171: 'System Too Lean (Bank 1)', P0172: 'System Too Rich (Bank 1)', P0190: 'Fuel Rail Pressure Sensor "A" Circuit',
  P0191: 'Fuel Rail Pressure Sensor "A" Circuit Range/Performance', P0192: 'Fuel Rail Pressure Sensor "A" Circuit Low', P0193: 'Fuel Rail Pressure Sensor "A" Circuit High',
  P0201: 'Injector Circuit/Open - Cylinder 1', P0202: 'Injector Circuit/Open - Cylinder 2', P0203: 'Injector Circuit/Open - Cylinder 3', P0204: 'Injector Circuit/Open - Cylinder 4', P0219: 'Engine Overspeed Condition',
  P0221: 'Throttle/Pedal Position Sensor/Switch "B" Circuit Range/Performance', P0222: 'Throttle/Pedal Position Sensor/Switch "B" Circuit Low', P0223: 'Throttle/Pedal Position Sensor/Switch "B" Circuit High',
  P0234: 'Turbocharger/Supercharger "A" Overboost Condition', P0236: 'Turbocharger/Supercharger Boost Sensor "A" Circuit Range/Performance', P0237: 'Turbocharger/Supercharger Boost Sensor "A" Circuit Low',
  P0238: 'Turbocharger/Supercharger Boost Sensor "A" Circuit High', P0261: 'Cylinder 1 Injector "A" Circuit Low', P0262: 'Cylinder 1 Injector "A" Circuit High', P0264: 'Cylinder 2 Injector "A" Circuit Low',
  P0265: 'Cylinder 2 Injector "A" Circuit High', P0267: 'Cylinder 3 Injector "A" Circuit Low', P0268: 'Cylinder 3 Injector "A" Circuit High', P0270: 'Cylinder 4 Injector "A" Circuit Low', P0271: 'Cylinder 4 Injector "A" Circuit High',
  P0299: 'Turbocharger/Supercharger "A" Underboost Condition', P0300: 'Random/Multiple Cylinder Misfire Detected', P0301: 'Cylinder 1 Misfire Detected', P0302: 'Cylinder 2 Misfire Detected',
  P0303: 'Cylinder 3 Misfire Detected', P0304: 'Cylinder 4 Misfire Detected', P0321: 'Ignition/Distributor Engine Speed Input Circuit Range/Performance', P0322: 'Ignition/Distributor Engine Speed Input Circuit No Signal',
  P0324: 'Knock Control System Error', P0325: 'Knock Sensor 1 Circuit (Bank 1 or Single Sensor)', P0326: 'Knock Sensor 1 Circuit Range/Performance (Bank 1 or Single Sensor)', P0327: 'Knock Sensor 1 Circuit Low (Bank 1 or Single Sensor)',
  P0328: 'Knock Sensor 1 Circuit High (Bank 1 or Single Sensor)', P0335: 'Crankshaft Position Sensor "A" Circuit', P0336: 'Crankshaft Position Sensor "A" Circuit Range/Performance',
  P0340: 'Camshaft Position Sensor "A" Circuit (Bank 1 or Single Sensor)', P0341: 'Camshaft Position Sensor "A" Circuit Range/Performance (Bank 1 or Single Sensor)',
  P0351: 'Ignition Coil "A" Primary/Secondary Circuit', P0352: 'Ignition Coil "B" Primary/Secondary Circuit', P0353: 'Ignition Coil "C" Primary/Secondary Circuit', P0354: 'Ignition Coil "D" Primary/Secondary Circuit',
  P0420: 'Catalyst System Efficiency Below Threshold (Bank 1)', P0441: 'Evaporative Emission System Incorrect Purge Flow', P0443: 'Evaporative Emission System Purge Control Valve "A" Circuit',
  P0455: 'Evaporative Emission System Leak Detected (large leak)', P0456: 'Evaporative Emission System Leak Detected (very small leak)', P0480: 'Fan 1 Control Circuit', P0500: 'Vehicle Speed Sensor "A"',
  P0501: 'Vehicle Speed Sensor "A" Range/Performance', P0505: 'Idle Air Control System', P0506: 'Idle Air Control System RPM Lower Than Expected', P0507: 'Idle Air Control System RPM Higher Than Expected',
  P0520: 'Engine Oil Pressure Sensor/Switch "A" Circuit', P0521: 'Engine Oil Pressure Sensor/Switch "A" Range/Performance', P0524: 'Engine Oil Pressure Too Low', P0562: 'System Voltage Low', P0563: 'System Voltage High',
  P0571: 'Brake Switch "A" Circuit', P0600: 'Serial Communication Link', P0601: 'Internal Control Module Memory Checksum Error', P0602: 'Control Module Programming Error',
  P0603: 'Internal Control Module Keep Alive Memory (KAM) Error', P0604: 'Internal Control Module Random Access Memory (RAM) Error', P0605: 'Internal Control Module Read Only Memory (ROM) Error',
  P0606: 'Control Module Processor', P0607: 'Control Module Performance', P0627: 'Fuel Pump "A" Control Circuit/Open', P0638: 'Throttle Actuator Control Range/Performance (Bank 1)',
  P0641: 'Sensor Reference Voltage "A" Circuit/Open', P0700: 'Transmission Control System (MIL Request)', P0705: 'Transmission Range Sensor "A" Circuit (PRNDL Input)', P0715: 'Input/Turbine Speed Sensor "A" Circuit',
  P0720: 'Output Speed Sensor Circuit', P0729: 'Gear 6 Incorrect Ratio', P0730: 'Incorrect Gear Ratio', P0731: 'Gear 1 Incorrect Ratio', P0732: 'Gear 2 Incorrect Ratio', P0733: 'Gear 3 Incorrect Ratio',
  P0734: 'Gear 4 Incorrect Ratio', P0735: 'Gear 5 Incorrect Ratio', P0736: 'Reverse Incorrect Ratio', P073A: 'Stuck in Gear 5', P073B: 'Stuck in Gear 6', P073C: 'Stuck in Gear 7',
  P074A: 'Unable to Engage Gear 1', P074B: 'Unable to Engage Gear 3', P074C: 'Unable to Engage Gear 4', P074D: 'Unable to Engage Gear 5', P074E: 'Unable to Engage Gear 6', P074F: 'Unable to Engage Reverse',
  P0810: 'Clutch Position Control Error', P0841: 'Transmission Fluid Pressure Sensor/Switch "A" Circuit Range/Performance', P0868: 'Transmission Fluid Pressure Low', P0900: 'Clutch Actuator Circuit/Open',
  P2004: 'Intake Manifold Runner Control Stuck Open (Bank 1)', P2008: 'Intake Manifold Runner Control Circuit/Open (Bank 1)', P2015: 'Intake Manifold Runner Position Sensor/Switch Circuit Range/Performance (Bank 1)',
  P2088: '"A" Camshaft Position Actuator Control Circuit Low (Bank 1)', P2096: 'Post Catalyst Fuel Trim System Too Lean (Bank 1)', P2097: 'Post Catalyst Fuel Trim System Too Rich (Bank 1)',
  P2100: 'Throttle Actuator "A" Control Motor Circuit/Open', P2101: 'Throttle Actuator "A" Control Motor Circuit Range/Performance', P2122: 'Throttle/Pedal Position Sensor/Switch "D" Circuit Low',
  P2123: 'Throttle/Pedal Position Sensor/Switch "D" Circuit High', P2127: 'Throttle/Pedal Position Sensor/Switch "E" Circuit Low', P2128: 'Throttle/Pedal Position Sensor/Switch "E" Circuit High',
  P2135: 'Throttle/Pedal Position Sensor/Switch "A" / "B" Voltage Correlation', P2138: 'Throttle/Pedal Position Sensor/Switch "D" / "E" Voltage Correlation', P2177: 'System Too Lean Off Idle (Bank 1)',
  P2181: 'Cooling System Performance', P2187: 'System Too Lean at Idle (Bank 1)', P2188: 'System Too Rich at Idle (Bank 1)', P2195: 'O2 Sensor Signal Biased/Stuck Lean (Bank 1 Sensor 1)',
  P2196: 'O2 Sensor Signal Biased/Stuck Rich (Bank 1 Sensor 1)', P2261: 'Turbocharger/Supercharger Bypass Valve "A" - Mechanical', P2279: 'Intake Air System Leak', P2293: 'Fuel Pressure Regulator 2 Performance',
  P2294: 'Fuel Pressure Regulator 2 Control Circuit', U0001: 'High Speed CAN Communication Bus', U0100: 'Lost Communication With ECM/PCM "A"', U0101: 'Lost Communication With TCM',
  U0121: 'Lost Communication With Anti-Lock Brake System (ABS) Control Module', U0140: 'Lost Communication With Body Control Module', U0155: 'Lost Communication With Instrument Panel Cluster (IPC) Control Module',
};
// SAE J2012-DA failure-type bytes (subset).
const FTB = { 0x00: 'no sub-type information', 0x01: 'general electrical failure', 0x02: 'general signal failure', 0x11: 'circuit short to ground', 0x12: 'circuit short to battery', 0x13: 'circuit open',
  0x14: 'circuit short to ground or open', 0x15: 'circuit short to battery or open', 0x16: 'circuit voltage below threshold', 0x17: 'circuit voltage above threshold', 0x1C: 'circuit voltage out of range',
  0x29: 'signal invalid', 0x62: 'signal compare failure', 0x64: 'signal plausibility failure', 0x87: 'missing message', 0x92: 'performance or incorrect operation', 0x96: 'component internal failure', 0x98: 'component or system over temperature' };
const DTC_BITS = ['testFailed', 'testFailedThisOperationCycle', 'pendingDTC', 'confirmedDTC', 'testNotCompletedSinceLastClear', 'testFailedSinceLastClear', 'testNotCompletedThisOperationCycle', 'warningIndicatorRequested'];
// "012345 status 0x2F" → P0123 / FTB 0x45. ISO 15031-6 / J2012 two-byte DTC encoding.
function decodeDtc(raw) {
  const m = /^([0-9A-F]{2})([0-9A-F]{2})([0-9A-F]{2}) status 0x([0-9A-F]{2})/i.exec(raw);
  if (!m) return { hex: '', code: raw, desc: DASH, ftb: DASH, status: DASH, fault: true, plain: DASH };
  const b0 = parseInt(m[1], 16), sys = 'PCBU'[b0 >> 6], code = `${sys}${(b0 >> 4) & 3}${(b0 & 15).toString(16).toUpperCase()}${m[2].toUpperCase()}`;
  const st = parseInt(m[4], 16), ftb = parseInt(m[3], 16);
  const d2 = (b0 >> 4) & 3, n = parseInt(code.slice(2, 4), 16);
  const generic = (sys === 'P' && (d2 === 0 || d2 === 2 || (d2 === 3 && n >= 0x40))) || (sys === 'U' && (d2 === 0 || d2 === 3)) || ((sys === 'C' || sys === 'B') && (d2 === 0 || d2 === 3));
  return {
    hex: `${m[1]} ${m[2]} ${m[3]}`.toUpperCase(), code, generic, desc: J2012[code] || (generic ? t('dtc_unknown') : t('dtc_mfr')),
    ftb: `0x${m[3].toUpperCase()}${FTB[ftb] ? ' — ' + FTB[ftb] : ''}`,
    status: `0x${m[4].toUpperCase()}: ` + DTC_BITS.filter((_, i) => st & (1 << i)).join(', '),
    fault: (st & 0x0F) !== 0,
    // current = failing now / in this operation cycle (bits 0/1); history = confirmed or failed since clear without them
    plain: st & 0x01 ? t('ds_failed') : st & 0x02 ? t('ds_cycle') : st & 0x04 ? t('ds_pending') : st & 0x08 ? t('ds_confirmed') : st & 0x20 ? t('ds_since') : st & 0x50 ? t('ds_ok') : t('ds_ok2'),
  };
}
const LAST_DTC = { ecu: null, tcu: null };
const AUTO_TP = { ecu: 0, tcu: 0 };
function udsPanel(tgt) {
  const el = document.querySelector(`[data-uds="${tgt}"]`);
  el.innerHTML = `<div class="row"><button data-svc="read_dtc" class="primary">${esc(t('read_dtc'))}</button><button data-svc="clear_dtc" class="danger">${esc(t('clear_dtc'))}</button><button data-svc="tester_present">${esc(t('tester_present'))}</button>
    <label><input type="checkbox" data-autotp${AUTO_TP[tgt] ? ' checked' : ''}> ${esc(t('auto_tp'))}</label>
    <label data-tip="tip_dtc_faults"><input type="checkbox" data-faults${LS.get('dtc_faults', true) ? ' checked' : ''}> ${esc(t('dtc_faults'))}</label></div>
    <div class="row" data-tip="tip_raw"><input data-raw type="text" style="width:170px" placeholder="${esc(t('raw_ph'))}"><button data-rawgo>${esc(t('raw_send'))}</button></div>
    <div class="kv" data-st></div><div class="scroll" style="max-height:260px"><table data-dtc></table></div><div class="note">${esc(t('dtc_fmt_note'))}</div><div class="note">${esc(t('uds_note_expl'))}</div>`
    + (tgt === 'tcu' ? wsHTML() : '')
    + (tgt === 'ecu' ? `<h4>${esc(t('obd_h'))}</h4><div class="row"><button data-obd class="${OBD.on ? 'on' : ''}">${esc(t(OBD.on ? 'obd_stop' : 'obd_go'))}</button><span data-obdst class="muted mono"></span></div><div class="scroll" style="max-height:340px"><table data-obdt></table></div><div class="note">${esc(t('obd_note'))}</div>` : '');
  const rawGo = () => act(el.querySelector('[data-rawgo]'), async () => {
    const bytes = parseHex(el.querySelector('[data-raw]').value);
    if (!bytes.length || bytes.length > 64 || bytes.some(b => !(b >= 0 && b <= 255))) throw new Error(t('raw_bad'));
    onState(await udsRaw(tgt, bytes));
    say(`UDS ${hexReq(bytes)} → ${tgt.toUpperCase()}`);
  });
  if (tgt === 'tcu') wsBind(el);
  el.querySelector('[data-rawgo]').onclick = rawGo;
  el.querySelector('[data-raw]').onkeydown = e => { if (e.key === 'Enter') rawGo(); };
  el.onclick = e => {
    const o = e.target.closest('[data-obd]');
    if (o) { OBD.on = !OBD.on; OBD.last = null; o.classList.toggle('on', OBD.on); o.textContent = t(OBD.on ? 'obd_stop' : 'obd_go'); return; }
    const b = e.target.closest('[data-svc]');
    if (!b) return;
    act(b, async () => {
      if (b.dataset.svc === 'clear_dtc' && !(await confirmBox(t('confirm_clear').replace('{t}', tgt.toUpperCase())))) return;
      onState(await api('/api/uds', { target: tgt, service: b.dataset.svc }));
      say(`UDS ${b.dataset.svc} → ${tgt.toUpperCase()}`);
    });
  };
  el.querySelector('[data-faults]').onchange = e => { LS.set('dtc_faults', e.target.checked); $$('[data-faults]').forEach(x => { x.checked = e.target.checked; }); if (S) { renderUds(S, 'ecu'); renderUds(S, 'tcu'); } };
  el.querySelector('[data-autotp]').onchange = e => {
    clearInterval(AUTO_TP[tgt]); AUTO_TP[tgt] = 0;
    if (e.target.checked) AUTO_TP[tgt] = setInterval(() => api('/api/uds', { target: tgt, service: 'tester_present' }).catch(err => say(err.message, true)), 2000);
  };
}
const UDS_SHOW = {}; // last non-poll request per unit: the OBD poller shares the tester but must not hide manual replies
function renderUds(s, tgt) {
  const el = document.querySelector(`[data-uds="${tgt}"]`), cur = s['uds_' + tgt] || {};
  if (!el.querySelector('[data-st]')) udsPanel(tgt);
  if (!(tgt === 'ecu' && OBD.on && /^01 [0-9A-F]{2}$/.test(cur.request || ''))) UDS_SHOW[tgt] = cur;
  const u = UDS_SHOW[tgt] || {};
  if (/ReadDTC/.test(u.request || '') && !u.pending && u.response_hex) LAST_DTC[tgt] = { dtcs: u.dtcs || [], sae: u.dtc_sae || [], note: u.note, at: s.plant ? s.plant.sim_time_s : NaN };
  el.querySelector('[data-st]').innerHTML = [[t('uds_req'), u.request || DASH], [t('uds_pending'), t(u.pending ? 'yes' : 'no')], [t('uds_resp'), u.response_hex || DASH], [t('uds_note'), u.note || DASH]]
    .map(([a, b], i) => `<div class="w"><span>${esc(a)}</span><b class="${i === 1 && u.pending ? 'warn' : ''}${i === 2 ? ' hexbox' : ''}">${esc(b)}</b></div>`).join('');
  const ld = LAST_DTC[tgt], onlyF = LS.get('dtc_faults', true);
  // ECU and TCU DTCs are 3-byte manufacturer numbers: use the firmware's own SAE mapping + event/DFC name when decoded
  const all = ld ? ld.dtcs.map((r, i) => {
    const d = decodeDtc(r), x = (ld.sae || [])[i];
    if (!x || x === '?') return d;
    const m = x.match(/^([PCBU][0-9A-F]{4})(?:\s*\((.+)\))?$/);
    if (!m) return Object.assign(d, { code: x, desc: t('dtc_vag'), generic: false, ftb: DASH });
    const [, code, extra] = m, j = J2012[code];
    return Object.assign(d, { code, desc: j && extra ? `${j} · ${extra}` : j || extra || t('dtc_vag'), generic: !!j, ftb: DASH });
  }) : [];
  const rows = onlyF ? all.filter(d => d.fault) : all;
  el.querySelector('[data-dtc]').innerHTML = (ld ? `<tr><td colspan="5" class="muted">${esc(t('dtc_shown'))}: ${rows.length} / ${all.length} · t = ${f(ld.at, 2)} s</td></tr>` : '')
    + (rows.length ? `<tr><th>${esc(t('dtc_hex'))}</th><th>${esc(t('dtc_code'))}</th><th>${esc(t('dtc_desc'))}</th><th>${esc(t('dtc_ftb'))}</th><th>${esc(t('dtc_status'))}</th></tr>` + rows.map(d =>
      `<tr><td class="muted">${esc(d.hex)}</td><td class="${d.fault ? 'bad' : ''}"><b>${esc(d.code)}</b></td><td class="wrap ${d.generic && J2012[d.code] ? '' : 'muted'}">${esc(d.desc)}</td><td class="wrap">${esc(d.ftb)}</td><td class="${d.fault ? 'bad' : 'muted'}" data-tipraw="${esc(d.status)}">${esc(d.plain)}</td></tr>`).join('')
      : `<tr><td class="muted">${esc(t('dtc_none'))}</td></tr>`);
  if (tgt === 'tcu') {
    wsRender(el, cur, s);
    $('uds-tcu-name').textContent = `TCU ${(s.drivetrain_profile && s.drivetrain_profile.tcu_code) || ''} ${s.tcu && s.tcu.present ? (s.tcu.sw_id || '').replace(/\s+\S+$/, '') : ''}`;
  }
}
const BOARD_ROWS = [['present', 'b_present'], ['sw_id', 'b_sw'], ['pc', 'b_pc'], ['region', 'b_region'], ['stm_ms', 'b_stm'], ['irq_count', 'b_irq'], ['warm_resets', 'b_resets'],
  ['rtos_tasks_seen', 'b_tasks'], ['rtos_task_runs', 'b_runs'], ['can_tx_total', 'b_can'], ['error', 'b_err']];
const alivePrev = { ecu: null, tcu: null };
function boardsTable(s) {
  const bv = v => (v == null || v === '' ? DASH : typeof v === 'boolean' ? t(v ? 'yes' : 'no') : isNum(v) && !Number.isInteger(v) ? v.toFixed(1) : String(v));
  const alive = b => { const p = alivePrev[b], c = s[b]; return c && c.present ? (p && c.stm_ms > p.stm_ms ? `<span class="good">${esc(t('alive_yes'))}</span>` : `<span class="warn">${esc(t('alive_no'))}</span>`) : DASH; };
  const html = `<tr><th></th><th>ECU SIMOS18.1 (TC1791)</th><th>${s.drivetrain_profile && /^0D9/.test(s.drivetrain_profile.tcu_code || '') ? 'TCU DQ250 0D9 (TC1766)' : 'TCU DQ200_G2 (TC1784)'}</th></tr><tr><th>${esc(t('b_alive'))}</th><td>${alive('ecu')}</td><td>${alive('tcu')}</td></tr>` + BOARD_ROWS.map(([k, lab]) =>
    `<tr><th data-ch="${['present', 'stm_ms', 'irq_count', 'warm_resets', 'rtos_tasks_seen', 'rtos_task_runs', 'can_tx_total'].includes(k) ? 'ecu.' + k : ''}">${esc(t(lab))}</th>${['ecu', 'tcu'].map(b => `<td class="${k === 'error' && s[b] && s[b][k] ? 'bad' : k === 'warm_resets' && s[b] && s[b][k] ? 'warn' : ''}" style="white-space:normal">${esc(bv(s[b] && s[b][k]))}</td>`).join('')}</tr>`).join('');
  return html;
}
view({
  id: 'diag',
  init() { udsPanel('ecu'); udsPanel('tcu'); },
  i18n() { udsPanel('ecu'); udsPanel('tcu'); },
  render(s) {
    renderUds(s, 'ecu'); renderUds(s, 'tcu');
    obdTick(s);
    const ot = document.querySelector('[data-obdt]');
    if (ot) { ot.innerHTML = obdTable(); document.querySelector('[data-obdst]').textContent = OBD.sup ? `${t('obd_sup')}: ${Array.from(OBD.sup).sort((a, b) => a - b).map(p => hexReq([p])).join(' ')}` : ''; }
    const im = s.immo || {};
    $('immo').innerHTML = [['immo.ecu_challenges'], ['immo.ecu_answers'], ['immo.tcu_challenges'], ['immo.tcu_answers'], ['immo.ecu_key_known']]
      .map(([k]) => `<div data-ch="${esc(k)}"><span>${esc(chName(k))}</span><b>${esc(fmtCh(k, FLAT.get(k)))}</b></div>`).join('');
    const lv = !im.ecu_key_known ? 'warn' : im.ecu_challenges ? 'ok' : 'warn';
    $('immo-verdict').innerHTML = `<div class="verdict ${lv}">${esc(!im.ecu_key_known ? L(['Ключ ЭБУ неизвестен — ответы приборки должны отвергаться прошивкой (пуск запрещён).', 'ECU key unknown — the firmware should reject the cluster answers (start blocked).'])
      : im.ecu_challenges ? L(['Обмен идёт, ключ ЭБУ известен.', 'Exchange running, ECU key known.']) : L(['ЭБУ ещё не запрашивал приборку.', 'The ECU has not queried the cluster yet.']))}</div>`
      + (im.binding ? `<div class="verdict">${esc(im.binding)}</div>` : '');
    $('boards').innerHTML = boardsTable(s);
    alivePrev.ecu = s.ecu; alivePrev.tcu = s.tcu;
    $('mech').innerHTML = mechHTML(s);
    $('mech-sub').textContent = `${(s.plant && s.plant.gearbox) || ''} · ${s.tcu && s.tcu.present ? s.tcu.sw_id : L(['ТКУ нет', 'no TCU'])} · ${benchOf(s.bench)[2]}`;
  },
});

// ---------- CAN monitor ----------
Object.assign(T, {
  p_can: ['CAN-монитор', 'CAN monitor'], can_trace: ['поток кадров', 'trace'], can_fixed: ['по идентификаторам', 'by identifier'],
  can_filter_ph: ['фильтр ID: 0AD, 100-1FF, !3C8', 'ID filter: 0AD, 100-1FF, !3C8'],
  tip_can_filter: ['Через запятую: ID (hex), диапазоны «100-1FF», исключения «!0AD». Пусто — все.', 'Comma separated: IDs (hex), ranges "100-1FF", exclusions "!0AD". Empty = all.'],
  can_note: ['Показаны кадры, которые отправили прошивки ЭБУ и ТКУ (журнал сервера). Кадры виртуального кузова, приборки и тестера сервер в журнал не пишет. Время — симуляционное.',
    'Shows frames sent by the ECU and TCU firmware (server log). Frames injected by the virtual chassis, cluster and tester are not logged by the server. Time is simulation time.'],
  can_lost: ['потеряно (переполнение буфера сервера)', 'lost (server buffer overflow)'], can_count: ['кадров', 'frames'], can_period: ['период, мс', 'period, ms'],
});
const CAN_NAMES = { '010': 'WFS5 ECU challenge', '011': 'WFS5 cluster → ECU', '012': 'WFS5 TCU challenge', '013': 'WFS5 cluster → TCU', '014': 'WFS5 TCU challenge (alt)', '015': 'WFS5 cluster → TCU (alt)',
  '0AD': 'Getriebe_11', '0AE': 'Getriebe_12', '12D': 'Getriebe_13', '3C8': 'Getriebe_14', '394': 'Getriebe_15', '128': 'Getriebe_16',
  '7E0': 'UDS tester → ECU', '7E8': 'UDS ECU → tester', '7E1': 'UDS tester → TCU', '7E9': 'UDS TCU → tester' };
let CANLOG = [], CANFIX = new Map(), canSince = -1, canPaused = false, canLost = 0, canPrevTot = null, canBusy = false;
function canFilter() {
  const toks = $('can-filter').value.split(/[,\s]+/).filter(Boolean);
  const inc = [], exc = [];
  for (const tk of toks) {
    const neg = tk.startsWith('!'), s2 = neg ? tk.slice(1) : tk, m = s2.split('-');
    const r = [parseInt(m[0], 16), parseInt(m[1] || m[0], 16)];
    if (r.every(Number.isFinite)) (neg ? exc : inc).push(r);
  }
  const src = $('can-src').value;
  return e => {
    const id = parseInt(e.id, 16);
    if (src && e.src !== src) return false;
    if (exc.some(([a, b]) => id >= a && id <= b)) return false;
    return !inc.length || inc.some(([a, b]) => id >= a && id <= b);
  };
}
const ascii = hexs => hexs.split(' ').map(h => { const c = parseInt(h, 16); return c >= 0x20 && c < 0x7f ? String.fromCharCode(c) : '.'; }).join('');
async function canPoll(s) {
  if (canPaused || canBusy) return;
  if (isNum(s.plant.sim_time_s) && s.plant.sim_time_s * 1e3 < canSince) { canSince = -1; canPrevTot = null; }
  canBusy = true;
  try {
    const r = await api('/api/ws/can?since_ms=' + canSince);
    const tot = r.ecu_tx_total + r.tcu_tx_total;
    if (canPrevTot != null && tot >= canPrevTot && canSince >= 0) canLost += Math.max(0, tot - canPrevTot - r.entries.length);
    canPrevTot = tot;
    for (const e of r.entries) {
      canSince = Math.max(canSince, e.t_ms);
      const fx = CANFIX.get(e.src + e.id) || { src: e.src, id: e.id, n: 0, t: NaN, period: NaN };
      const dt = isNum(fx.t) ? e.t_ms - fx.t : NaN;
      if (isNum(dt) && dt > 0) fx.period = isNum(fx.period) ? fx.period * 0.8 + dt * 0.2 : dt;
      Object.assign(fx, { n: fx.n + 1, t: e.t_ms, data: e.data });
      CANFIX.set(e.src + e.id, fx);
      CANLOG.push(Object.assign({ dt }, e));
    }
    if (CANLOG.length > 3000) CANLOG = CANLOG.slice(-2000);
  } finally { canBusy = false; }
  renderCan();
}
function renderCan() {
  const keep = canFilter(), mode = $('can-mode').value;
  const nm = id => CAN_NAMES[id] || '';
  if (mode === 'trace') {
    const rows = [];
    for (let i = CANLOG.length - 1; i >= 0 && rows.length < 300; i--) { const e = CANLOG[i]; if (keep(e)) rows.push(e); }
    $('can-table').innerHTML = `<tr><th class="r">t, ms</th><th class="r">Δt</th><th>src</th><th>ID</th><th>${esc(L(['имя', 'name']))}</th><th>DLC</th><th>data (hex)</th><th>ASCII</th></tr>`
      + rows.map(e => `<tr><td class="r">${f(e.t_ms, 0)}</td><td class="r">${f(e.dt, 0)}</td><td>${esc(e.src)}</td><td><b>${esc(e.id)}</b></td><td class="muted">${esc(nm(e.id))}</td><td>${e.data ? e.data.split(' ').length : 0}</td><td>${esc(e.data)}</td><td>${esc(ascii(e.data))}</td></tr>`).join('');
  } else {
    const rows = Array.from(CANFIX.values()).filter(keep).sort((a, b) => parseInt(a.id, 16) - parseInt(b.id, 16));
    $('can-table').innerHTML = `<tr><th>src</th><th>ID</th><th>${esc(L(['имя', 'name']))}</th><th class="r">${esc(t('can_count'))}</th><th class="r">${esc(t('can_period'))}</th><th class="r">t, ms</th><th>data (hex)</th><th>ASCII</th></tr>`
      + rows.map(e => `<tr><td>${esc(e.src)}</td><td><b>${esc(e.id)}</b></td><td class="muted">${esc(nm(e.id))}</td><td class="r">${e.n}</td><td class="r">${f(e.period, 1)}</td><td class="r">${f(e.t, 0)}</td><td>${esc(e.data)}</td><td>${esc(ascii(e.data))}</td></tr>`).join('');
  }
  $('can-stat').textContent = `${CANLOG.length} · IDs ${CANFIX.size} · ${t('can_lost')}: ${canLost}`;
  $('can-sub').textContent = canPaused ? t('paused') : '';
}
view({
  id: 'can',
  init() {
    $('can-pause').onclick = () => { canPaused = !canPaused; $('can-pause').classList.toggle('on', canPaused); renderCan(); };
    $('can-clear').onclick = () => { CANLOG = []; CANFIX = new Map(); canLost = 0; renderCan(); };
    for (const id of ['can-mode', 'can-src']) $(id).onchange = renderCan;
    $('can-filter').oninput = renderCan;
  },
  show() { canSince = -1; canPrevTot = null; },
  render(s) { canPoll(s).catch(e => say(e.message, true)); },
});

// ---------- firmware images & checksums ----------
Object.assign(T, {
  p_images: ['Образы прошивки ЭБУ', 'ECU firmware images'], checksums: ['Контрольные суммы образа', 'Image checksums'],
  img_note: ['Загрузка образа перезагружает ЭБУ (как замена блока) и подключает его DAMOS. Образы без описания DAMOS недоступны.', 'Loading an image power-cycles the ECU (like swapping the unit) and attaches its DAMOS. Images without a DAMOS definition are disabled.'],
  cks_note: ['«Экспорт .bin» пересчитывает все суммы, как программатор.', '"Export .bin" recomputes all checksums like a flashing tool.'],
  asw1_note: ['Несовпадение CRC блока кода означает, что образ был изменён без пересчёта контрольных сумм.', 'A code-block CRC mismatch indicates the image was modified without updating block checksums.'],
  load: ['Загрузить', 'Load'], active: ['активный', 'active'], no_damos: ['нет DAMOS', 'no DAMOS'], loaded: ['Загружен образ', 'Image loaded'],
  confirm_load: ['Загрузить образ {f}? ЭБУ будет перезагружен, текущие правки калибровок в памяти будут заменены.', 'Load image {f}? The ECU is power-cycled; the current calibration edits in memory are replaced.'],
  img_notes_h: ['Что сделано при загрузке образа', 'What was done when loading the image'], img_notes_none: ['Образ загружен как есть: маркеры и контрольные суммы не менялись.', 'Image loaded as is: no markers or checksums changed.'],
  immo_lab_b: ['Лабораторная привязка', 'Lab binding'], immo_dump_b: ['Данные дампа', 'Dump data'],
  tip_immo_lab: ['Ключи иммобилайзера генерируются для этого образа и записываются в данные DFlash ЭБУ и EEPROM ТКУ в памяти (код не меняется, файлы дампов не меняются). Работает с любым образом. Образ перезагружается.',
    'Immobiliser keys are generated for this image and written into the ECU DFlash and TCU EEPROM data in memory (no code change, dump files untouched). Works with any image. The image is reloaded.'],
  tip_immo_dump: ['Ключи WFS5 берутся напрямую из загруженных дампов DFlash ЭБУ и EEPROM ТКУ без лабораторной перепривязки (требует согласованной пары дампов ЭБУ + ТКУ и класса мощности B из списка калибровки). Образ перезагружается.',
    'Uses the raw WFS5 keys directly from the loaded ECU DFlash and TCU EEPROM dumps without lab re-binding (requires a matched ECU + TCU dump pair whose power class B is in the calibration list). The image is reloaded.'],
  confirm_immo: ['Переключить иммобилайзер на «{m}»? Образ ЭБУ будет загружен заново (перезагрузка ЭБУ).', 'Switch the immobiliser to "{m}"? The ECU image is reloaded (ECU power cycle).'],
});
const noteCls = n => (/не проверен|not verified|immo-off|не прочитаны|отвергнут/i.test(n) ? 'warn' : /пересчитан|written|дозапис|completed/i.test(n) ? 'acc' : 'muted');
let imgSig = '';
view({
  id: 'fw',
  init() {
    $('b-export2').onclick = e => act(e.target, exportBin);
    for (const [id, action] of [['immo-lab', 'immo_lab'], ['immo-dump', 'immo_dump']]) $(id).onclick = e => act(e.target, async () => {
      if (!(await confirmBox(t('confirm_immo').replace('{m}', t(id === 'immo-lab' ? 'immo_lab_b' : 'immo_dump_b')), t('apply'), false))) return;
      await afterPreset(await api('/api/preset', { action }));
      imgSig = '';
      say(`${t('immo')}: ${S.immo.binding}`);
    });
    $('img-table').onclick = e => {
      const b = e.target.closest('[data-img]');
      if (!b) return;
      const image = b.dataset.img;
      act(b, async () => {
        if (!(await confirmBox(t('confirm_load').replace('{f}', image), t('load')))) return;
        await afterPreset(await api('/api/preset', { action: 'load_image', image }));
        imgSig = '';
        say(`${t('loaded')}: ${image}`);
      });
    };
  },
  i18n() { imgSig = ''; },
  render(s) {
    const imgs = s.images || [], sig = LANG + JSON.stringify(imgs) + s.image_file + s.image_label;
    const lab = !!(s.immo && s.immo.lab);
    $('immo-lab').classList.toggle('on', lab); $('immo-dump').classList.toggle('on', !lab);
    if (sig !== imgSig) {
      imgSig = sig;
      const notes = s.image_notes || [];
      $('img-notes').innerHTML = notes.length ? notes.map(n => `<div class="note ${noteCls(n)}" style="margin-top:2px">${esc(n)}</div>`).join('') : `<div class="note">${esc(t('img_notes_none'))}</div>`;
      $('img-table').innerHTML = `<tr><th></th><th>box</th><th>${esc(L(['файл', 'file']))}</th><th>DAMOS</th><th></th></tr>` + imgs.map(i => {
        const on = i.file === s.image_file;
        return `<tr class="${on ? 'sel' : ''}"><td>${on ? '●' : ''}</td><td><b>${esc(i.box_code)}</b></td><td style="white-space:normal;word-break:break-all">${esc(i.file)}</td><td class="${i.damos ? '' : 'warn'}">${esc(i.damos ? i.damos.replace(/^.*\//, '') : t('no_damos'))}</td>`
          + `<td>${on ? `<span class="good">${esc(t('active'))}</span>` : `<button data-img="${esc(i.file)}"${i.damos ? '' : ' disabled'}>${esc(t('load'))}</button>`}</td></tr>`;
      }).join('');
    }
    const cks = (s.checksums && s.checksums.entries) || [];
    $('cks').innerHTML = '<tr><th>block</th><th>offset</th><th>stored</th><th>computed</th><th></th></tr>' + (cks.length ? cks.map(e => {
      const ok = e.stored === e.computed;
      return `<tr><td>${esc(e.name)}</td><td>${hex(e.offset)}</td><td>${hex(e.stored)}</td><td>${hex(e.computed)}</td><td class="${ok ? 'good' : 'bad'}">${ok ? 'OK' : 'MISMATCH'}</td></tr>`;
    }).join('') : `<tr><td colspan="5">${DASH}</td></tr>`);
    $('asw1-note').textContent = cks.some(e => e.name === 'ASW1' && e.stored !== e.computed) ? t('asw1_note') : '';
    $('fw-boards').innerHTML = boardsTable(s);
    alivePrev.ecu = s.ecu; alivePrev.tcu = s.tcu;
  },
});

// ---------- bench setup ----------
Object.assign(T, {
  p_cfg: ['Двигатель, трансмиссия, атмосфера, топливо', 'Engine, drivetrain, atmosphere, fuel'], p_ui: ['Интерфейс', 'Interface'],
  engine: ['Двигатель', 'Engine'], drivetrain: ['Трансмиссия', 'Drivetrain'], amb_t: ['Т воздуха, °C', 'Air temp, °C'], altitude: ['Высота, м', 'Altitude, m'], humidity: ['Влажность, %', 'Humidity, %'],
  tip_ron: ['Октановое число топлива (RON). Низкое — раньше детонация.', 'Fuel octane (RON). Lower = earlier knock.'],
  apply_config: ['Применить', 'Apply'], cold_soak: ['Холодный старт (сброс физики)', 'Cold soak (reset plant)'], power_cycle: ['Перезагрузка ЭБУ и ТКУ', 'Power-cycle ECU & TCU'],
  warm_soak: ['Тёплый старт (после поездки)', 'Warm start (after a drive)'],
  tip_warm_soak: ['Как «Холодный старт», но автомобиль только что приехал: двигатель 90 °C, масло КПП 70 °C, стоянка 5 мин (таймер ЭБУ и приборка); ЭБУ и ТКУ перезагружаются на прогретом автомобиле с сохранением их памяти (DFlash / EEPROM).',
    'Like Cold soak, but the car has just arrived: engine 90 °C, gearbox oil 70 °C, 5 min stop (ECU timer and cluster); ECU and TCU reboot on the warm car while keeping their NVM (DFlash / EEPROM).'],
  tip_cold_soak: ['Мотор остывает до температуры воздуха (стоянка 12 ч), повреждения сбрасываются; ЭБУ и ТКУ перезагружаются на холодном автомобиле с сохранением их памяти (DFlash / EEPROM).',
    'The engine cools to ambient (12 h soak) and damage is reset; ECU and TCU reboot on the cold car while keeping their NVM (DFlash / EEPROM).'],
  tip_power_cycle: ['Снять и подать питание блоков: физика сбрасывается (холодный мотор, стоянка 12 ч), затем прошивки ЭБУ и ТКУ загружаются заново на сброшенной модели.',
    'Remove and restore power to the units: the plant resets (cold engine, 12 h soak), then ECU and TCU firmware reboot on the reset plant.'],
  config_hint: ['Смена двигателя или трансмиссии сбрасывает физическую модель. Атмосфера влияет на плотность воздуха и на коэффициенты приведения мощности.', 'Changing engine or drivetrain resets the plant. The atmosphere sets air density and the power correction factors.'],
  confirm_power: ['Перезагрузить ЭБУ и ТКУ? Физика будет сброшена.', 'Power-cycle the ECU and TCU? The plant will be reset.'],
  ui_alarms: ['Пределы тревог…', 'Alarm limits…'], ui_reset: ['Сбросить настройки интерфейса', 'Reset UI settings'],
  ui_note: ['Раскладки приборов, наборы каналов, пределы тревог и история замеров хранятся в этом браузере (localStorage).', 'Gauge layouts, channel sets, alarm limits and run history are stored in this browser (localStorage).'],
  confirm_ui_reset: ['Удалить все настройки интерфейса, раскладки и историю замеров в этом браузере?', 'Delete all UI settings, layouts and run history in this browser?'],
  prof_now: ['Сейчас', 'Current'],
});
const ENGINES = [['1.8t_is12', '1.8 TSI CJSA + IS12'], ['1.8t_is38_hybrid', '1.8 TSI + IS38 hybrid'], ['2.0t_is20', '2.0 TSI CHHB + IS20'], ['2.0t_is38', '2.0 TSI CJXC + IS38']];
const DRIVES = [['dq200_fwd', 'DQ200 7-sp FWD (dry)'], ['dq250_4wd', 'DQ250 6-sp 4WD (wet)'], ['dq381_4wd', 'DQ381 7-sp 4WD (wet)']];
function syncSetup(s) {
  const set = (id, v) => { if (v != null) $(id).value = v; };
  set('cfg-engine', s.engine_profile && s.engine_profile.slug);
  set('cfg-dt', s.drivetrain_profile && s.drivetrain_profile.slug);
  set('cfg-ron', s.fuel_ron);
  set('cfg-temp', s.ambient && s.ambient.temp_c);
  set('cfg-alt', s.ambient && s.ambient.altitude_m);
  set('cfg-hum', s.ambient && s.ambient.humidity_pct);
}
function alarmsModal() {
  const ks = numKeys().filter(k => { const l = limitsOf(k); return l.on || ALIM[k]; });
  modal(t('ui_alarms'), `<table><tr><th></th><th></th><th class="r">&lt;</th><th class="r">&gt;</th><th>${esc(L(['звук', 'sound']))}</th><th></th></tr>` + ks.map(k => {
    const l = limitsOf(k);
    return `<tr><td data-ch="${esc(k)}">${esc(chName(k))}</td><td class="muted">${esc(meta(k).unit || '')}</td><td class="r">${fmtN(l.lo)}</td><td class="r">${fmtN(l.hi)}</td><td>${l.snd ? '🔔' : ''}</td><td><button data-lim="${esc(k)}">${esc(t('edit_limits'))}</button></td></tr>`;
  }).join('') + `</table><div class="note">${esc(L(['Любой канал можно настроить из «Все каналы» (кнопка ⚠).', 'Any channel can be configured from All channels (⚠ button).']))}</div>`, [{ label: t('close') }]);
  $('m-body').onclick = e => { const b = e.target.closest('[data-lim]'); if (b) editLimits(b.dataset.lim); };
}
view({
  id: 'setup',
  init() {
    $('cfg-engine').innerHTML = ENGINES.map(([v, n]) => `<option value="${v}">${esc(n)}</option>`).join('');
    $('cfg-dt').innerHTML = DRIVES.map(([v, n]) => `<option value="${v}">${esc(n)}</option>`).join('');
    const numIn = id => { const v = $(id).value.trim(); return v === '' || !Number.isFinite(+v) ? undefined : +v; };
    $('b-config').onclick = e => act(e.target, async () => {
      const body = { action: 'config', fuel_ron: numIn('cfg-ron'), ambient_temp_c: numIn('cfg-temp'), altitude_m: numIn('cfg-alt'), humidity_pct: numIn('cfg-hum') };
      if (S && $('cfg-engine').value !== S.engine_profile.slug) body.engine_slug = $('cfg-engine').value;
      if (S && $('cfg-dt').value !== S.drivetrain_profile.slug) body.drivetrain_slug = $('cfg-dt').value;
      await afterPreset(await api('/api/preset', body));
      say(t('done'));
    });
    $$('[data-preset]').forEach(b => {
      b.onclick = () => act(b, async () => {
        if (b.dataset.preset === 'power_cycle' && !(await confirmBox(t('confirm_power')))) return;
        await afterPreset(await api('/api/preset', { action: b.dataset.preset }));
        say(`${t(b.dataset.preset)} — ${t('done')}`);
      });
    });
    $('ui-alarms').onclick = alarmsModal;
    $('ui-reset').onclick = async () => {
      if (!(await confirmBox(t('confirm_ui_reset')))) return;
      try { Object.keys(localStorage).filter(k => k.startsWith('simemu.')).forEach(k => localStorage.removeItem(k)); } catch (e) { /* storage disabled */ }
      location.reload();
    };
  },
  first: syncSetup,
  show() { renderLog(); },
  render(s) {
    const e = s.engine_profile || {}, d = s.drivetrain_profile || {};
    const disp = e.bore_m && e.stroke_m && e.cylinders ? (Math.PI / 4) * e.bore_m * e.bore_m * e.stroke_m * e.cylinders * 1e6 : NaN;
    const rows = [[t('engine'), e.name], ['code', e.engine_code], ['cm³', isNum(disp) ? disp.toFixed(0) : DASH], ['ε', e.compression_ratio], ['n max (mech.)', e.mechanical_overspeed_rpm],
      ['turbo', e.turbo && e.turbo.model], [t('drivetrain'), d.name], ['TCU', d.tcu_code], [L(['прошивка КПП', 'TCU firmware']), d.has_tcu_firmware ? t('yes') : t('no')], ['AWD', d.awd ? t('yes') : t('no')], [L(['передач', 'gears']), d.gears]];
    const html = rows.map(([a, b]) => `<div class="${String(b).length > 24 ? 'w' : ''}"><span>${esc(a)}</span><b>${esc(b ?? DASH)}</b></div>`).join('');
    if ($('cfg-now').dataset.h !== html) { $('cfg-now').dataset.h = html; $('cfg-now').innerHTML = html; }
  },
});

boot();
