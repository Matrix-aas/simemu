// ---------- pre-flash tune check («Проверка прошивки перед заливкой», studio::tune_check) ----------
Object.assign(T, {
  v_check: ['Проверка перед заливкой', 'Pre-flash check'],
  p_ck: ['Проверка прошивки перед заливкой', 'Pre-flash firmware check'],
  ck_note: ['Стандартная программа стенда для выбранного образа ЭБУ (и ТКУ сборки) на текущей сборке стенда: развёртка на моторном стенде, разгон в передачах, старт с места, жара с прогревом, АИ-92. Каждая фаза — отдельная безголовая симуляция той же машины (живой стенд не трогается); решает только прошивка, ломает только физика. Вердикт: OK / риск / сломает — с причинами, моментом и цилиндром; рядом — эталон (сток того же номера).',
    'The bench\'s standard program for the chosen ECU image (and the build\'s TCU) on the current bench build: engine-dyno sweep, pull in gears, standing launch, heat soak, 92 RON. Each phase is a separate headless simulation of the same car (the live bench is untouched); only the firmware decides, only physics breaks. Verdict: OK / risk / breaks — with reasons, moment and cylinder; next to it the reference (stock of the same part number).'],
  ck_image: ['Образ ЭБУ', 'ECU image'], ck_ref: ['Эталон (сток)', 'Reference (stock)'], ck_noref: ['без эталона', 'no reference'],
  ck_slot: ['Слот MultiMap (патч O30)', 'MultiMap slot (O30 patch)'], ck_slot_ecu: ['как в ЭБУ', 'as stored in the ECU'],
  tip_ck_slot: ['Для образов с патчем переключения карт O30 Switch: водитель выбирает слот педалью и тормозом перед пуском (как в автомобиле).',
    'For images with the O30 Switch MultiMap patch: the driver selects the slot with pedal and brake before starting the engine (as in the car).'],
  ck_run: ['Проверить', 'Check'], ck_cancel: ['Остановить', 'Stop'], ck_running: ['идёт проверка', 'checking'],
  ck_result: ['Результат', 'Result'], ck_html: ['Отчёт HTML', 'HTML report'], ck_json: ['Отчёт JSON', 'JSON report'],
  ck_none: ['Проверка ещё не запускалась.', 'No check run yet.'], ck_reasons: ['Причины', 'Reasons'],
  ck_level: ['уровень', 'level'], ck_phase: ['фаза', 'phase'], ck_cyl: ['цил.', 'cyl.'], ck_why: ['причина', 'reason'],
  tip_ck_run: ['Запустить программу проверки (≈ 3–8 мин на native, обе прошивки параллельно)', 'Run the check program (≈ 3–8 min on native, both images in parallel)'],
});
let ckSig = '';
view({
  id: 'check',
  init() {
    $('ck-run').onclick = e => act(e.target, async () => {
      const slot = $('ck-slot').value;
      await api('/api/check', { action: 'start', image: $('ck-image').value, reference: $('ck-ref').value || null, mm_slot: slot ? +slot : null });
      ckSig = '';
      say(t('ck_running'));
    });
    $('ck-cancel').onclick = e => act(e.target, async () => { await api('/api/check', { action: 'cancel' }); });
  },
  i18n() { ckSig = ''; },
  render(s) {
    const c = s.tune_check || {}, imgs = s.images || [];
    const opts = (sel, none) => (none ? `<option value="">${esc(t('ck_noref'))}</option>` : '') + imgs.map(i => `<option value="${esc(i.file)}"${i.file === sel ? ' selected' : ''}>${esc(i.box_code)} — ${esc(i.file)}</option>`).join('');
    if ($('ck-image').dataset.n !== String(imgs.length)) {
      $('ck-image').dataset.n = String(imgs.length);
      $('ck-image').innerHTML = opts(s.image_file, false);
      $('ck-ref').innerHTML = opts(c.default_reference, true);
      $('ck-slot').innerHTML = `<option value="">${esc(t('ck_slot_ecu'))}</option>` + [1, 2, 3, 4].map(n => `<option value="${n}"${n === 3 ? ' selected' : ''}>${n}</option>`).join('');
    }
    $('ck-run').disabled = !!c.running;
    $('ck-cancel').disabled = !c.running;
    const pg = c.progress;
    $('ck-prog').innerHTML = c.running && pg ? `<b>${esc(pg.step || t('ck_running'))}</b>` + (pg.parts || []).map(([n, f]) => `<div class="row"><span style="flex:1">${esc(n)}</span><span class="mono">${(100 * f).toFixed(0)} %</span></div>`).join('') : '';
    const sig = LANG + (c.verdict || '') + (c.summary || '') + (c.running ? 1 : 0);
    if (sig === ckSig) return;
    ckSig = sig;
    if (!c.verdict) { $('ck-res').innerHTML = `<div class="note">${esc(t('ck_none'))}</div>`; return; }
    api('/api/check').then(r => {
      const rep = r.report;
      if (!rep) return;
      const cls = v => (v === 'breaks' ? 'bad' : v === 'risk' ? 'warn' : 'good');
      const ru = v => ({ ok: 'OK', risk: L(['риск', 'risk']), breaks: L(['сломает', 'breaks']) }[v] || v);
      const rows = (rep.tested.reasons || []).map(x => `<tr><td class="${cls(x.level)}">${esc(ru(x.level))}</td><td>${esc(x.phase)}</td><td class="r">${(+x.t_s).toFixed(1)}</td><td>${x.cyl || DASH}</td><td style="white-space:normal">${esc(x.text)}</td></tr>`).join('');
      $('ck-res').innerHTML = `<div class="${cls(rep.verdict)}" style="font-size:1.6em;font-weight:bold">${esc(ru(rep.verdict))}</div><p style="white-space:normal">${esc(rep.summary)}</p>`
        + `<div class="row"><a href="/api/check/report.html" target="_blank"><button>${esc(t('ck_html'))}</button></a><a href="/api/check/report.json" download="tune_check.json"><button>${esc(t('ck_json'))}</button></a></div>`
        + (rows ? `<h4>${esc(t('ck_reasons'))}</h4><table><tr><th>${esc(t('ck_level'))}</th><th>${esc(t('ck_phase'))}</th><th>t, s</th><th>${esc(t('ck_cyl'))}</th><th>${esc(t('ck_why'))}</th></tr>${rows}</table>` : '')
        + `<iframe src="/api/check/report.html?ts=${Date.now()}" style="width:100%;height:70vh;border:1px solid var(--line);margin-top:8px;background:#fff"></iframe>`;
    }).catch(e => say(e.message, true));
  },
});
