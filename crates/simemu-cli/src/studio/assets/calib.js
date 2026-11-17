// ---------- calibration: map browser, table / diff / 3D editor, selection math, undo/redo ----------
Object.assign(T, {
  p_calbar: ['Калибровка образа ЭБУ', 'ECU image calibration'], presets: ['Пресеты', 'Presets'], export: ['⇩ Экспорт .bin', '⇩ Export .bin'],
  tip_export: ['Скачать текущий образ 4 МБ со всеми пересчитанными контрольными суммами (как после программатора).', 'Download the current 4 MB image with all checksums recomputed (as a flashing tool would).'],
  p_maps: ['Карты', 'Maps'], mb_cur: ['Ключевые', 'Key maps'], mb_damos: ['Весь DAMOS', 'Full DAMOS'], search: ['Поиск: имя или описание…', 'Search: name or description…'],
  kind_all: ['все типы', 'all kinds'], cat_all: ['все категории', 'all categories'], more: ['ещё…', 'more…'],
  p_editor: ['Редактор', 'Editor'], ev_table: ['Таблица', 'Table'], ev_diff: ['Разница со стоком', 'Diff vs stock'], ev_3d: ['3D / график', '3D / plot'],
  tip_ed_val: ['Число для операций над выделением.', 'Operand for operations on the selection.'],
  tip_op_set: ['Присвоить число всем выделенным ячейкам', 'Set the selected cells to the number'], tip_op_add: ['Прибавить число', 'Add the number'], tip_op_sub: ['Вычесть число', 'Subtract the number'],
  tip_op_mul: ['Умножить на число', 'Multiply by the number'], tip_op_pct: ['Изменить на число процентов (+5 = ×1,05)', 'Change by the number of percent (+5 = ×1.05)'],
  tip_op_ih: ['Линейная интерполяция по строке между крайними выделенными ячейками (по значениям оси X)', 'Linear interpolation along each row between the outer selected cells (by X-axis values)'],
  tip_op_iv: ['Линейная интерполяция по столбцу (по значениям оси Y)', 'Linear interpolation along each column (by Y-axis values)'],
  tip_op_i2: ['Билинейная интерполяция по четырём углам выделения', 'Bilinear interpolation from the four selection corners'],
  tip_op_smooth: ['Сглаживание: среднее 3×3 по соседям внутри выделения', 'Smoothing: 3×3 average over neighbours inside the selection'],
  tip_op_stock: ['Вернуть выделенным ячейкам заводские значения', 'Restore the stock values of the selected cells'],
  tip_undo: ['Отменить (Ctrl+Z)', 'Undo (Ctrl+Z)'], tip_redo: ['Повторить (Ctrl+Y)', 'Redo (Ctrl+Y)'],
  write: ['⇪ Записать в ЭБУ', '⇪ Write to ECU'], tip_write: ['Записать правки в образ, пересчитать контрольные суммы CAL и загрузить CAL в работающий ЭБУ (как программатор).', 'Write the edits into the image, recompute CAL checksums and push CAL into the running ECU (as a flashing tool does).'],
  discard: ['Отменить правки', 'Discard edits'], to_stock: ['Карта → сток', 'Map → stock'], stock: ['сток', 'stock'],
  show_stock_wire: ['сетка стока', 'stock wireframe'], ed_3d_help: ['Перетаскивание — вращение.', 'Drag to rotate.'],
  map_note: ['Выделение: мышь (протяжка) или стрелки с Shift; двойной клик или Enter — правка ячейки; Ctrl+C / Ctrl+V — обмен таблицами с Excel. Жёлтая рамка — отличается от стока, синяя — несохранённая правка. Значения округлены только для показа; при записи сервер кодирует их в формат ячейки (шаг = LSB).',
    'Select with the mouse (drag) or Shift+arrows; double-click or Enter edits a cell; Ctrl+C / Ctrl+V exchange tables with Excel. Amber = differs from stock, blue = unsaved edit. Values are rounded for display only; on write the server encodes them into the cell format (step = LSB).'],
  pick_map: ['Выберите карту слева.', 'Pick a map on the left.'], edits: ['несохранённых правок', 'unsaved edits'], no_edits: ['Нет правок для записи.', 'No edits to write.'],
  written: ['Карта записана, контрольные суммы пересчитаны, CAL загружен в ЭБУ.', 'Map written, checksums recomputed, CAL pushed to the ECU.'],
  dropped: ['Несохранённые правки предыдущей карты отброшены.', 'Unsaved edits of the previous map were discarded.'],
  clamped: ['ячеек ограничено пределами DAMOS', 'cells clamped to the DAMOS limits'], need_num: ['Введите число в поле операнда.', 'Enter a number in the operand field.'],
  trace_na: ['Трассировка рабочей точки скрыта: сервер не отдаёт входные переменные осей этой карты из ОЗУ ЭБУ. Физические значения модели (обороты, наполнение) не равны тому, что вычисляет прошивка, поэтому интерфейс их не подставляет.',
    'Operating-point tracing hidden: the server does not expose this map\'s axis input variables from ECU RAM. Plant values (rpm, air charge) are not what the firmware computes, so the UI does not substitute them.'],
  confirm_preset: ['Пресет восстановит область калибровок (CAL) из стока и применит свои правки — ваши записанные изменения карт будут потеряны. Продолжить?', 'The preset restores CAL from stock and applies its own edits — your written map changes will be lost. Continue?'],
  confirm_tostock: ['Записать в ЭБУ заводские значения всей карты?', 'Write the stock values of the whole map to the ECU?'],
  cks_ok: ['Контр. суммы: OK', 'Checksums: OK'], cks_bad: ['Контр. суммы: несовпадений', 'Checksums: mismatches'],
  m_cat: ['категория', 'category'], m_type: ['тип ячейки', 'cell type'], m_lsb: ['шаг (LSB)', 'step (LSB)'], m_lim: ['пределы', 'limits'],
  pr_stock: ['Вернуть заводские калибровки (для чужого образа — загрузить заводской образ S50 и КПП).', 'Restore factory calibration (for another image — reload the factory S50 image and gearbox).'],
  pr_stage1_dq200: ['Набор правок поверх стока: наддув, лимиты момента, УОЗ, лямбда защиты — в пределах ресурса сцеплений DQ200. Точный список — apply_preset в session.rs.', 'Edits on top of stock: boost, torque limits, spark, protection lambda — within the DQ200 clutch capacity. Exact list: apply_preset in session.rs.'],
  pr_stage2: ['Более сильный набор правок (наддув, момент, обороты турбины, УОЗ, лямбда).', 'A stronger edit set (boost, torque, turbo speed, spark, lambda).'],
  pr_cold_thermostat: ['Снижает уставку температуры охлаждающей жидкости.', 'Lowers the coolant temperature set-point.'],
  pr_pops_bangs: ['Задержка отсечки топлива при сбросе газа и поздний УОЗ — «выстрелы» в выпуске.', 'Delayed overrun fuel cut and retarded spark — exhaust "pops".'],
  pr_unlock_stationary_limiter: ['Поднимает ограничение оборотов на месте.', 'Raises the stationary rev limit.'],
});
const PRESETS = ['stock', 'stage1_dq200', 'stage2', 'cold_thermostat', 'pops_bangs', 'unlock_stationary_limiter'];
let MB_MODE = 'cur', CATS = null, DLIST = null, CAT_IMG = null;
let MAP = null, MMETA = null, EV = 'table';
let EDITS = new Map(), UNDO = [], REDO = [];
let SEL = null; // {ar, ac, r, c} anchor + focus
const cellVal = (r, c) => (EDITS.has(r + ',' + c) ? EDITS.get(r + ',' + c) : MAP.values[r][c]);
const selBox = () => (SEL ? { r0: Math.min(SEL.ar, SEL.r), r1: Math.max(SEL.ar, SEL.r), c0: Math.min(SEL.ac, SEL.c), c1: Math.max(SEL.ac, SEL.c) } : null);
const heat = x => { const k = clamp(x, 0, 1); return `rgba(${Math.round(20 + k * 175)},${Math.round(45 + (1 - Math.abs(k - 0.5) * 2) * 95)},${Math.round(130 * (1 - k) + 35)},0.45)`; };
const diffHeat = (d, span) => { const k = clamp(d / (span || 1), -1, 1); return k > 0 ? `rgba(255,107,107,${0.15 + 0.5 * k})` : k < 0 ? `rgba(54,181,255,${0.15 - 0.5 * k})` : 'transparent'; };

function renderPresets() {
  $('cal-presets').innerHTML = PRESETS.map(p => `<button data-preset2="${esc(p)}" data-tip="pr_${esc(p)}" class="${S && S.active_preset === p ? 'on' : ''}">${esc(p)}</button>`).join(' ');
  const cks = (S && S.checksums && S.checksums.entries) || [];
  const bad = cks.filter(e => e.stored !== e.computed).length;
  $('cal-cks').innerHTML = cks.length ? `<span class="lamp ${bad ? 'bad' : 'ok'}"></span><b>${esc(bad ? `${t('cks_bad')}: ${bad}` : t('cks_ok'))}</b>` : `<b>${DASH}</b>`;
}
async function loadCats() {
  CAT_IMG = S && S.image_file;
  CATS = await api('/api/ws/damos/categories');
  $('mb-cat').innerHTML = `<option value="">${esc(t('cat_all'))} (${CATS.total})</option>` + CATS.categories.map(c => `<option value="${esc(c.name)}">${esc(c.name)} (${c.count})</option>`).join('');
}
async function damosQuery(append) {
  const q = new URLSearchParams({ q: $('mb-q').value.trim(), category: $('mb-cat').value, kind: $('mb-kind').value, offset: append && DLIST ? DLIST.items.length : 0, limit: 200 });
  const r = await api('/api/ws/damos/list?' + q);
  DLIST = append && DLIST ? { total: r.total, items: DLIST.items.concat(r.items) } : r;
  renderMapList();
}
function renderMapList() {
  const sel = MAP && MAP.id;
  const item = (id, title, sub) => `<div class="it${id === sel ? ' sel' : ''}" data-id="${esc(id)}">${esc(title)}<small>${esc(sub)}</small></div>`;
  let html = '';
  if (MB_MODE === 'cur') {
    let g = null;
    for (const m of (S && S.curated_maps) || []) {
      if (m.group !== g) { g = m.group; html += `<div class="grp">${esc(g)}</div>`; }
      html += item(m.id, L([m.title_ru, m.title_en]), m.id);
    }
  } else if (DLIST) {
    html += `<div class="note">${DLIST.items.length} / ${DLIST.total}</div>` + DLIST.items.map(h => item(h.id, h.title || h.id, `${h.id} · ${h.kind} ${h.rows}×${h.cols} ${h.unit || ''} · ${h.category || ''} · ${h.address}`)).join('');
    if (DLIST.items.length < DLIST.total) html += `<div class="row"><button id="mb-more">${esc(t('more'))}</button></div>`;
  }
  $('maplist').innerHTML = html || `<div class="muted">${DASH}</div>`;
  const more = $('mb-more');
  if (more) more.onclick = () => act(more, () => damosQuery(true));
}
async function loadMap(id) {
  const [m, meta2] = await Promise.all([api('/api/map?id=' + encodeURIComponent(id)), api('/api/ws/damos/meta?id=' + encodeURIComponent(id)).catch(() => null)]);
  if (EDITS.size) say(t('dropped'));
  EDITS = new Map(); UNDO = []; REDO = [];
  MAP = m; MMETA = meta2;
  SEL = { ar: 0, ac: 0, r: 0, c: 0 };
  renderMap(); renderMapList();
}
function mapStats() {
  let lo = Infinity, hi = -Infinity, dmax = 0;
  MAP.values.forEach((rv, r) => rv.forEach((_, c) => { const v = cellVal(r, c); lo = Math.min(lo, v, MAP.stock_values[r][c]); hi = Math.max(hi, v, MAP.stock_values[r][c]); dmax = Math.max(dmax, Math.abs(v - MAP.stock_values[r][c])); }));
  return { lo, hi, dmax };
}
function renderMap() {
  $('map-edits').textContent = EDITS.size ? `${EDITS.size} ${t('edits')}` : '';
  $('ed-undo').disabled = !UNDO.length; $('ed-redo').disabled = !REDO.length;
  if (!MAP) { $('map-head').innerHTML = `<div class="muted">${esc(t('pick_map'))}</div>`; $('matrix').innerHTML = ''; $('ed-sub').textContent = ''; $('ed-trace').textContent = ''; return; }
  const m = MAP, mm = MMETA || {};
  const cur = S && (S.curated_maps || []).find(c => c.id === m.id);
  $('ed-sub').textContent = m.id;
  $('map-head').innerHTML = `<div><b style="font-size:14px">${esc(cur ? L([cur.title_ru, cur.title_en]) : m.title || m.id)}</b></div>`
    + `<div class="mono muted">${esc(m.id)} @ ${hex(m.cpu_address)} · ${esc(m.kind)} · ${m.rows}×${m.cols} · ${esc(m.unit || '')}`
    + (mm.category ? ` · ${esc(t('m_cat'))}: ${esc(mm.category)}` : '') + (mm.data_type ? ` · ${esc(t('m_type'))}: ${esc(mm.data_type)}` : '')
    + (isNum(mm.scale) ? ` · ${esc(t('m_lsb'))}: ${fmtN(mm.scale)}` : '') + (mm.min != null || mm.max != null ? ` · ${esc(t('m_lim'))}: ${fmtN(mm.min)} … ${fmtN(mm.max)}` : '') + '</div>'
    + (cur ? `<div class="hint">${esc(L([cur.desc_ru, cur.desc_en]))} <i>${esc(cur.fr_chapter)}</i></div>` : m.title ? `<div class="note">${esc(m.title)}</div>` : '');
  $('ed-trace').textContent = m.x_channel || m.y_channel ? '' : t('trace_na');
  $('ed-table').hidden = EV === '3d'; $('ed-3d').hidden = EV !== '3d';
  $$('#ed-tools [data-ev]').forEach(b => b.classList.toggle('on', b.dataset.ev === EV));
  if (EV === '3d') { draw3D(); renderSelInfo(); return; }
  const { lo, hi, dmax } = mapStats(), span = hi - lo || 1, b = selBox();
  let html = `<tr><th>${esc(m.y_unit || '')} \\ ${esc(m.x_unit || '')}</th>` + (m.values[0] || []).map((_, c) => `<th class="ax">${fmtN(m.x_axis[c])}</th>`).join('') + '</tr>';
  m.values.forEach((rv, r) => {
    html += `<tr><th class="ax">${fmtN(m.y_axis[r])}</th>` + rv.map((_, c) => {
      const v = cellVal(r, c), st = m.stock_values[r][c], d = v - st;
      const cls = (EDITS.has(r + ',' + c) ? 'edit' : v !== st ? 'mod' : '') + (b && r >= b.r0 && r <= b.r1 && c >= b.c0 && c <= b.c1 ? ' s' : '');
      const txt = EV === 'diff' ? (d ? (d > 0 ? '+' : '') + fmtN(d) : '·') : fmtN(v);
      return `<td data-r="${r}" data-c="${c}" class="${cls}" style="background:${EV === 'diff' ? diffHeat(d, dmax) : heat((v - lo) / span)}">${txt}</td>`;
    }).join('') + '</tr>';
  });
  $('matrix').innerHTML = html;
  renderSelInfo();
}
function renderSelInfo() {
  const b = selBox();
  if (!MAP || !b) { $('ed-sel').textContent = ''; return; }
  const vals = [];
  for (let r = b.r0; r <= b.r1; r++) for (let c = b.c0; c <= b.c1; c++) vals.push(cellVal(r, c));
  const st = MAP.stock_values[SEL.r][SEL.c], v = cellVal(SEL.r, SEL.c);
  $('ed-sel').textContent = `[${SEL.r},${SEL.c}] x=${fmtN(MAP.x_axis[SEL.c])} y=${fmtN(MAP.y_axis[SEL.r])} · ${fmtN(v)} (${t('stock')} ${fmtN(st)}, Δ ${fmtN(v - st)}${st ? `, ${(((v / st) - 1) * 100).toFixed(2)} %` : ''})`
    + (vals.length > 1 ? ` · ${vals.length} cells: min ${fmtN(Math.min(...vals))} max ${fmtN(Math.max(...vals))} avg ${fmtN(vals.reduce((a, x) => a + x, 0) / vals.length)}` : '');
}
// Applies new values {key: v} as one undoable step; clamps to the DAMOS limits.
function commit(newVals) {
  if (!newVals.size) return;
  UNDO.push(new Map(EDITS)); REDO = [];
  if (UNDO.length > 200) UNDO.shift();
  const lo = MMETA && isNum(MMETA.min) ? MMETA.min : -Infinity, hi = MMETA && isNum(MMETA.max) ? MMETA.max : Infinity;
  let clamped = 0;
  for (const [k, v0] of newVals) {
    if (!Number.isFinite(v0)) continue;
    const v = clamp(v0, lo, hi);
    if (v !== v0) clamped++;
    const [r, c] = k.split(',').map(Number);
    if (v === MAP.values[r][c]) EDITS.delete(k); else EDITS.set(k, v);
  }
  if (clamped) say(`${clamped} ${t('clamped')}`, true);
  renderMap();
}
function axisPos(axis, i, n) { const a = axis || []; const mono = a.length === n && a.every((x, j) => j === 0 || x > a[j - 1]); return mono ? a[i] : i; }
function applyOp(op) {
  const b = selBox();
  if (!MAP || !b) return;
  const x = parseFloat($('ed-val').value);
  if (['set', 'add', 'sub', 'mul', 'pct'].includes(op) && !Number.isFinite(x)) return say(t('need_num'), true);
  const out = new Map(), nr = MAP.rows, nc = MAP.cols;
  const X = c => axisPos(MAP.x_axis, c, nc), Y = r => axisPos(MAP.y_axis, r, nr);
  for (let r = b.r0; r <= b.r1; r++) for (let c = b.c0; c <= b.c1; c++) {
    const v = cellVal(r, c), k = r + ',' + c;
    let n = v;
    if (op === 'set') n = x; else if (op === 'add') n = v + x; else if (op === 'sub') n = v - x; else if (op === 'mul') n = v * x; else if (op === 'pct') n = v * (1 + x / 100);
    else if (op === 'stock') n = MAP.stock_values[r][c];
    else if (op === 'ih' && b.c1 > b.c0) { const u = (X(c) - X(b.c0)) / (X(b.c1) - X(b.c0) || 1); n = cellVal(r, b.c0) + u * (cellVal(r, b.c1) - cellVal(r, b.c0)); }
    else if (op === 'iv' && b.r1 > b.r0) { const u = (Y(r) - Y(b.r0)) / (Y(b.r1) - Y(b.r0) || 1); n = cellVal(b.r0, c) + u * (cellVal(b.r1, c) - cellVal(b.r0, c)); }
    else if (op === 'i2' && b.r1 > b.r0 && b.c1 > b.c0) {
      const u = (X(c) - X(b.c0)) / (X(b.c1) - X(b.c0) || 1), w = (Y(r) - Y(b.r0)) / (Y(b.r1) - Y(b.r0) || 1);
      const top = cellVal(b.r0, b.c0) + u * (cellVal(b.r0, b.c1) - cellVal(b.r0, b.c0)), bot = cellVal(b.r1, b.c0) + u * (cellVal(b.r1, b.c1) - cellVal(b.r1, b.c0));
      n = top + w * (bot - top);
    } else if (op === 'smooth') {
      let s = 0, cnt = 0;
      for (let dr = -1; dr <= 1; dr++) for (let dc = -1; dc <= 1; dc++) { const rr = r + dr, cc = c + dc; if (rr >= b.r0 && rr <= b.r1 && cc >= b.c0 && cc <= b.c1) { s += cellVal(rr, cc); cnt++; } }
      n = s / cnt;
    }
    if (n !== v) out.set(k, n);
  }
  commit(out);
}
function undo() { if (!UNDO.length) return; REDO.push(new Map(EDITS)); EDITS = UNDO.pop(); renderMap(); }
function redo() { if (!REDO.length) return; UNDO.push(new Map(EDITS)); EDITS = REDO.pop(); renderMap(); }
function editCell(td) {
  if (!td || td.querySelector('input')) return;
  const r = +td.dataset.r, c = +td.dataset.c;
  const inp = document.createElement('input');
  inp.type = 'number'; inp.step = 'any'; inp.value = String(cellVal(r, c)); // full precision
  td.textContent = ''; td.appendChild(inp); inp.focus(); inp.select();
  let done = false;
  const finish = ok => {
    if (done) return;
    done = true;
    const v = parseFloat(inp.value);
    if (ok && Number.isFinite(v) && v !== cellVal(r, c)) commit(new Map([[r + ',' + c, v]])); else renderMap();
  };
  inp.onblur = () => finish(true);
  inp.onkeydown = e => { e.stopPropagation(); if (e.key === 'Enter') finish(true); else if (e.key === 'Escape') finish(false); };
}
async function writeMap(values) {
  const res = await api('/api/map/write', { id: MAP.id, values });
  if (!res || !res.map) throw new Error('/api/map/write: no map in response');
  MAP = res.map; EDITS = new Map(); UNDO = []; REDO = [];
  renderMap(); say(t('written'));
}
async function exportBin() {
  const res = await fetch('/api/export_bin');
  if (!res.ok) {
    let msg = res.status + ' ' + res.statusText;
    try { msg = (await res.json()).error || msg; } catch (e) { /* keep status */ }
    throw new Error('/api/export_bin: ' + msg);
  }
  download(`simemu_${(S && S.active_preset) || 'image'}_${stamp()}.bin`, await res.blob());
  say(t('done'));
}
// After a preset / image change: re-sync inputs, reload the open map (or drop it if the DAMOS changed).
async function afterPreset(s) {
  const imageChanged = S && s.image_file !== S.image_file;
  onState(s);
  syncDock(s);
  if (typeof syncSetup === 'function') syncSetup(s);
  if (imageChanged) { MAP = null; MMETA = null; EDITS = new Map(); DLIST = null; CATS = null; renderMap(); if (MB_MODE === 'damos') await loadCats(); }
  else if (MAP) await loadMap(MAP.id).catch(e => { MAP = null; renderMap(); throw e; });
  renderMapList(); renderPresets();
}
// 3D surface (maps) or line plot (curves); stock as a wireframe.
const v3 = { yaw: -0.75, pitch: 0.55 };
function draw3D() {
  const cv = $('ed-canvas');
  if (!MAP) return;
  if (MAP.rows === 1 || MAP.cols === 1) {
    const n = Math.max(MAP.rows, MAP.cols), get = (arr, i) => (MAP.rows === 1 ? arr[0][i] : arr[i][0]);
    const ax = MAP.rows === 1 ? MAP.x_axis : MAP.y_axis, xs = Array.from({ length: n }, (_, i) => axisPos(ax, i, n));
    xyChart(cv, { series: [{ xs, ys: xs.map((_, i) => (MAP.rows === 1 ? cellVal(0, i) : cellVal(i, 0))), color: '#36b5ff', axis: 0, label: L(['текущее', 'current']), points: true },
      { xs, ys: xs.map((_, i) => get(MAP.stock_values, i)), color: '#9fb0c8', dashed: true, axis: 0, label: t('stock') }], xLabel: (MAP.rows === 1 ? MAP.x_unit : MAP.y_unit) || '' });
    return;
  }
  const g = fitCanvas(cv);
  if (!g) return;
  const { c, w, h } = g, { lo, hi } = mapStats(), span = hi - lo || 1, R = MAP.rows, Cn = MAP.cols;
  const cy = Math.cos(v3.yaw), sy = Math.sin(v3.yaw), cp = Math.cos(v3.pitch), sp = Math.sin(v3.pitch), sc = Math.min(w, h) * 0.36;
  const P = (r, col, v) => {
    const x = (col / (Cn - 1)) * 2 - 1, y = (r / (R - 1)) * 2 - 1, z = ((v - lo) / span) * 1.5 - 0.75;
    const xr = x * cy - y * sy, yr = x * sy + y * cy;
    return [w / 2 + xr * sc, h / 2 + 20 - (z * cp - yr * sp) * sc, yr * cp + z * sp];
  };
  const quads = [];
  for (let r = 0; r < R - 1; r++) for (let col = 0; col < Cn - 1; col++) {
    const pts = [[r, col], [r, col + 1], [r + 1, col + 1], [r + 1, col]].map(([a, b]) => P(a, b, cellVal(a, b)));
    const avg = (cellVal(r, col) + cellVal(r, col + 1) + cellVal(r + 1, col + 1) + cellVal(r + 1, col)) / 4;
    quads.push({ pts, d: pts.reduce((s, p) => s + p[2], 0) / 4, avg, ed: [[r, col], [r, col + 1], [r + 1, col + 1], [r + 1, col]].some(([a, b]) => EDITS.has(a + ',' + b)) });
  }
  quads.sort((a, b) => b.d - a.d);
  for (const q of quads) {
    c.beginPath(); q.pts.forEach((p, i) => (i ? c.lineTo(p[0], p[1]) : c.moveTo(p[0], p[1]))); c.closePath();
    c.fillStyle = heat((q.avg - lo) / span).replace('0.45)', '0.92)'); c.fill();
    c.strokeStyle = q.ed ? '#36b5ff' : '#08101c'; c.lineWidth = q.ed ? 1.5 : 0.6; c.stroke();
  }
  if ($('ed-3dstock').checked) {
    c.strokeStyle = 'rgba(255,255,255,0.35)'; c.lineWidth = 0.8;
    for (let r = 0; r < R; r++) { c.beginPath(); for (let col = 0; col < Cn; col++) { const p = P(r, col, MAP.stock_values[r][col]); col ? c.lineTo(p[0], p[1]) : c.moveTo(p[0], p[1]); } c.stroke(); }
    for (let col = 0; col < Cn; col++) { c.beginPath(); for (let r = 0; r < R; r++) { const p = P(r, col, MAP.stock_values[r][col]); r ? c.lineTo(p[0], p[1]) : c.moveTo(p[0], p[1]); } c.stroke(); }
  }
  c.fillStyle = '#9fb0c8'; c.textAlign = 'center';
  const a = P(0, Cn - 1, lo), b2 = P(R - 1, 0, lo);
  c.fillText(`X: ${MAP.x_unit || ''} ${fmtN(MAP.x_axis[0])} → ${fmtN(MAP.x_axis[Cn - 1])}`, a[0], a[1] + 18);
  c.fillText(`Y: ${MAP.y_unit || ''} ${fmtN(MAP.y_axis[0])} → ${fmtN(MAP.y_axis[R - 1])}`, b2[0], b2[1] + 18);
  c.textAlign = 'left'; c.fillText(`Z: ${MAP.unit || ''} ${fmtN(lo)} … ${fmtN(hi)}`, 10, 16);
}
function moveSel(dr, dc, extend) {
  if (!MAP) return;
  if (!SEL) SEL = { ar: 0, ac: 0, r: 0, c: 0 };
  const r = clamp(SEL.r + dr, 0, MAP.rows - 1), c = clamp(SEL.c + dc, 0, MAP.cols - 1);
  SEL = extend ? Object.assign(SEL, { r, c }) : { ar: r, ac: c, r, c };
  renderMap();
  const td = document.querySelector(`#matrix td[data-r="${r}"][data-c="${c}"]`);
  if (td) td.scrollIntoView({ block: 'nearest', inline: 'nearest' });
}
view({
  id: 'cal',
  init() {
    $('cal-presets').onclick = e => {
      const b = e.target.closest('[data-preset2]');
      if (!b) return;
      act(b, async () => {
        if (!(await confirmBox(t('confirm_preset'), b.dataset.preset2, false))) return;
        await afterPreset(await api('/api/preset', { action: b.dataset.preset2 }));
        say(`${t('preset')}: ${b.dataset.preset2} — ${t('done')}`);
      });
    };
    $('b-export').onclick = e => act(e.target, exportBin);
    $('mb-cur').onclick = () => { MB_MODE = 'cur'; $('mb-cur').classList.add('on'); $('mb-damos').classList.remove('on'); $('mb-damos-ctl').hidden = true; renderMapList(); };
    $('mb-damos').onclick = () => act(null, async () => {
      MB_MODE = 'damos'; $('mb-damos').classList.add('on'); $('mb-cur').classList.remove('on'); $('mb-damos-ctl').hidden = false;
      if (!CATS || CAT_IMG !== (S && S.image_file)) await loadCats();
      await damosQuery(false);
    });
    let qt = 0;
    $('mb-q').oninput = () => { clearTimeout(qt); qt = setTimeout(() => act(null, () => damosQuery(false)), 250); };
    $('mb-cat').onchange = $('mb-kind').onchange = () => act(null, () => damosQuery(false));
    $('maplist').onclick = e => { const d = e.target.closest('[data-id]'); if (d) act(null, () => loadMap(d.dataset.id)); };
    $('ed-tools').onclick = e => {
      const ev = e.target.closest('[data-ev]'), op = e.target.closest('[data-op]');
      if (ev) { EV = ev.dataset.ev; renderMap(); }
      if (op) applyOp(op.dataset.op);
    };
    $('ed-undo').onclick = undo; $('ed-redo').onclick = redo;
    $('b-write').onclick = e => act(e.target, async () => {
      if (!MAP) return say(t('pick_map'));
      if (!EDITS.size) return say(t('no_edits'));
      await writeMap(MAP.values.map((rv, r) => rv.map((_, c) => cellVal(r, c)))); // untouched cells: exactly as received
    });
    $('b-discard').onclick = () => { if (EDITS.size) { UNDO.push(new Map(EDITS)); EDITS = new Map(); renderMap(); } };
    $('b-tostock').onclick = e => act(e.target, async () => { if (MAP && await confirmBox(t('confirm_tostock'))) await writeMap(MAP.stock_values); });
    let dragSel = false;
    $('matrix').onmousedown = e => {
      const td = e.target.closest('td[data-r]');
      if (!td || td.querySelector('input')) return;
      const r = +td.dataset.r, c = +td.dataset.c;
      SEL = e.shiftKey && SEL ? Object.assign(SEL, { r, c }) : { ar: r, ac: c, r, c };
      dragSel = true; renderMap(); e.preventDefault();
    };
    $('matrix').onmouseover = e => { if (!dragSel) return; const td = e.target.closest('td[data-r]'); if (td && SEL) { SEL.r = +td.dataset.r; SEL.c = +td.dataset.c; renderMap(); } };
    addEventListener('mouseup', () => { dragSel = false; });
    $('matrix').ondblclick = e => editCell(e.target.closest('td[data-r]'));
    const cv = $('ed-canvas');
    let rot = null;
    cv.onmousedown = e => { rot = { x: e.clientX, y: e.clientY, yaw: v3.yaw, pitch: v3.pitch }; };
    addEventListener('mousemove', e => { if (!rot) return; v3.yaw = rot.yaw + (e.clientX - rot.x) * 0.01; v3.pitch = clamp(rot.pitch + (e.clientY - rot.y) * 0.01, 0.05, 1.5); draw3D(); });
    addEventListener('mouseup', () => { rot = null; });
    $('ed-3dstock').onchange = draw3D;
    // Ctrl/Cmd shortcuts and clipboard (TSV, Excel-compatible).
    addEventListener('keydown', e => {
      if (CUR_VIEW !== 'cal' || modalClose || inField(e) || !(e.ctrlKey || e.metaKey)) return;
      const k = e.key.toLowerCase();
      if (k === 'z' && !e.shiftKey) { undo(); e.preventDefault(); } else if (k === 'y' || (k === 'z' && e.shiftKey)) { redo(); e.preventDefault(); }
      else if (k === 'a' && MAP) { SEL = { ar: 0, ac: 0, r: MAP.rows - 1, c: MAP.cols - 1 }; renderMap(); e.preventDefault(); }
    });
    document.addEventListener('copy', e => {
      const b = selBox();
      if (CUR_VIEW !== 'cal' || !MAP || !b || inField(e)) return;
      const rows = [];
      for (let r = b.r0; r <= b.r1; r++) { const row = []; for (let c = b.c0; c <= b.c1; c++) row.push(cellVal(r, c)); rows.push(row.join('\t')); }
      e.clipboardData.setData('text/plain', rows.join('\n'));
      e.preventDefault();
    });
    document.addEventListener('paste', e => {
      const b = selBox();
      if (CUR_VIEW !== 'cal' || !MAP || !b || inField(e)) return;
      const txt = e.clipboardData.getData('text/plain');
      const out = new Map();
      txt.trim().split(/\r?\n/).forEach((ln, i) => ln.split(/\t|;/).forEach((cell, j) => {
        const r = b.r0 + i, c = b.c0 + j, v = parseFloat(cell.replace(',', '.'));
        if (r < MAP.rows && c < MAP.cols && Number.isFinite(v)) out.set(r + ',' + c, v);
      }));
      commit(out);
      e.preventDefault();
    });
  },
  key(e) {
    if (!MAP || EV === '3d') return false;
    const mv = { ArrowUp: [-1, 0], ArrowDown: [1, 0], ArrowLeft: [0, -1], ArrowRight: [0, 1] }[e.key];
    if (mv) { moveSel(mv[0], mv[1], e.shiftKey); return true; }
    if (e.key === 'Enter' && SEL) { editCell(document.querySelector(`#matrix td[data-r="${SEL.r}"][data-c="${SEL.c}"]`)); return true; }
    return false;
  },
  show() { renderPresets(); renderMapList(); renderMap(); },
  i18n() { renderPresets(); renderMapList(); renderMap(); if (CATS && MB_MODE === 'damos') loadCats().catch(() => {}); },
  render(s, prev) {
    const sig = s.active_preset + JSON.stringify(s.checksums || null, (k, v) => (typeof v === 'bigint' ? v.toString() : v));
    if (sig !== $('cal-presets').dataset.sig) { $('cal-presets').dataset.sig = sig; renderPresets(); }
    if (!$('maplist').children.length) renderMapList();
  },
});
