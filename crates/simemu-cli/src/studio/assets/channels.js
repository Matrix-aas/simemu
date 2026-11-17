// ---------- channel registry ----------
// Metadata only (names, units, descriptions for beginners, typical ranges, default alarm limits).
// Values always come from the backend. Fields not listed here are rendered automatically with a
// guessed name/unit (see autoMeta), so new physics telemetry shows up without UI changes.
// Entry options: r = typical range [min,max] (display units), d = decimals, k = display multiplier,
// il = index labels for arrays ({i} in names), alarm = {lo, hi, when(S), snd} (display units, may be functions of S),
// na = reason shown when the value is null/NaN, bool = on/off channel.
const GROUPS = {
  eng: ['Двигатель', 'Engine'], air: ['Воздух и наддув', 'Air & boost'], fuel: ['Топливо и смесь', 'Fuel & mixture'],
  ign: ['Зажигание и детонация', 'Ignition & knock'], temp: ['Температуры', 'Temperatures'], elec: ['Электрика', 'Electrical'],
  dl: ['Коробка передач', 'Gearbox'], veh: ['Автомобиль', 'Vehicle'], hlth: ['Ресурс узлов', 'Component health'],
  comb: ['Сгорание по цилиндрам', 'Combustion per cylinder'], mech: ['Механика: клапаны, шатуны, подшипники', 'Mechanics: valves, rods, bearings'],
  ecuo: ['Команды прошивки ЭБУ → механизмы', 'ECU firmware → actuators'], tcuo: ['Команды прошивки ТКУ → мехатроник', 'TCU firmware → mechatronic'],
  drv: ['Водитель', 'Driver'], brd: ['Платы ЭБУ / ТКУ', 'ECU / TCU boards'], imm: ['Иммобилайзер', 'Immobiliser'], sim: ['Симуляция', 'Simulation'],
  new: ['Новые каналы (без описания)', 'New channels (no description)'],
};
const GROUP_ORDER = Object.keys(GROUPS);
const REG = {};
function C(key, unit, group, ru, en, dru, den, o) { REG[key] = Object.assign({ key, unit, group, name: [ru, en], desc: [dru, den] }, o || {}); }
const NA_OUT = ['выход прошивки не декодирован эмулятором — механизм обесточен (см. «Обзор»)', 'firmware output not decoded by the emulator — actuator de-energised (see Overview)'];
const KK = { il: ['K1', 'K2'] };
const overspeed = s => s.engine_profile && s.engine_profile.mechanical_overspeed_rpm;
const turboBurst = s => s.engine_profile && s.engine_profile.turbo && s.engine_profile.turbo.burst_margin_rpm;
const running = s => s.plant && s.plant.rpm > 1500;

// engine
C('plant.rpm', 'rpm', 'eng', 'Обороты двигателя', 'Engine speed', 'Сколько оборотов в минуту делает коленвал. Холостой ход ≈ 700–800, отсечка ≈ 6500–7000. Выше механического предела (из профиля двигателя) детали разрушаются.',
  'Crankshaft revolutions per minute. Idle ≈ 700–800, limiter ≈ 6500–7000. Above the mechanical limit (engine profile) parts fail.', { r: [0, 7500], d: 0, alarm: { hi: overspeed, snd: true } });
C('plant.crank_deg', '°', 'eng', 'Угол коленвала', 'Crank angle', 'Положение коленвала в 4-тактном цикле 0–720° (0° — ВМТ сжатия 1-го цилиндра).', 'Crank position in the 4-stroke cycle 0–720° (0° = cylinder 1 firing TDC).', { r: [0, 720], d: 0 });
C('plant.torque_brake_nm', 'Nm', 'eng', 'Крутящий момент на валу', 'Brake torque', 'То, что измеряет тормоз стенда: момент от газов минус трение, насосные потери и навесное оборудование.', 'What a dyno measures: gas torque minus friction, pumping and accessory losses.', { r: [-50, 500], d: 1 });
C('plant.torque_indicated_nm', 'Nm', 'eng', 'Индикаторный момент', 'Indicated torque', 'Момент, который создают газы в цилиндрах, до вычета потерь.', 'Torque produced by the gas in the cylinders before losses.', { r: [0, 600], d: 1 });
C('plant.torque_friction_nm', 'Nm', 'eng', 'Момент трения', 'Friction torque', 'Потери на трение поршней, подшипников, ГРМ; растут с оборотами и на холодном масле.', 'Friction of pistons, bearings and valvetrain; grows with speed and cold oil.', { r: [0, 80], d: 1 });
C('plant.torque_pumping_nm', 'Nm', 'eng', 'Насосные потери', 'Pumping torque', 'Работа на прокачку воздуха через мотор; больше всего при прикрытом дросселе.', 'Work to pump air through the engine; largest at part throttle.', { r: [-20, 60], d: 1 });
C('plant.power_kw', 'kW', 'eng', 'Мощность', 'Power', 'Эффективная мощность = момент на валу × угловая скорость.', 'Brake power = brake torque × angular speed.', { r: [0, 300], d: 1 });
C('plant.power_ps', 'PS', 'eng', 'Мощность (л.с.)', 'Power (PS)', 'То же в метрических лошадиных силах (1 кВт = 1,36 PS).', 'Same in metric horsepower (1 kW = 1.36 PS).', { r: [0, 400], d: 1 });
C('plant.dyno_absorber_nm', 'Nm', 'eng', 'Тормозной момент стенда', 'Dyno absorber torque', 'Сколько тормозит вихретоковый тормоз, чтобы удержать заданные обороты (только в режиме моторного стенда).', 'Braking torque of the eddy-current absorber holding the target speed (engine-dyno mode only).', { r: [0, 800], d: 1 });
C('plant.oil_bar', 'bar', 'eng', 'Давление масла', 'Oil pressure', 'Без давления масла подшипники коленвала задираются за секунды. Холостой ≥ 1 бар, высокие обороты 3–4 бар.', 'Without oil pressure the crank bearings seize within seconds. Idle ≥ 1 bar, high rpm 3–4 bar.', { r: [0, 6], d: 2, alarm: { lo: 0.8, when: running, snd: true } });
// air
C('plant.baro_kpa', 'kPa', 'air', 'Атмосферное давление', 'Barometric pressure', 'Давление окружающего воздуха; зависит от высоты над уровнем моря.', 'Ambient air pressure; depends on altitude.', { r: [80, 105], d: 1 });
C('plant.map_kpa', 'kPa', 'air', 'Давление во впуске (MAP)', 'Manifold pressure (MAP)', 'Абсолютное давление после дросселя: ~30 кПа на холостом, ~100 без наддува, больше 100 — наддув.', 'Absolute pressure after the throttle: ~30 kPa idle, ~100 unboosted, above 100 = boost.', { r: [0, 300], d: 1 });
C('plant.put_kpa', 'kPa', 'air', 'Давление до дросселя (PUT)', 'Pre-throttle pressure (PUT)', 'Давление, которое создаёт турбина перед дроссельной заслонкой.', 'Pressure created by the turbo upstream of the throttle.', { r: [0, 300], d: 1 });
C('plant.exhaust_manifold_kpa', 'kPa', 'air', 'Давление в выпускном коллекторе', 'Exhaust manifold pressure', 'Противодавление перед турбиной: высокое — мотору тяжело «выдыхать».', 'Back-pressure before the turbine: high = the engine struggles to exhale.', { r: [0, 400], d: 1 });
C('plant.throttle_pct', '%', 'air', 'Открытие дросселя', 'Throttle opening', 'Положение заслонки (её двигает прошивка через электропривод). Без питания пружина держит ~6 %.', 'Throttle plate position (driven by the firmware through the ETC motor). Unpowered, a spring holds ~6 %.', { r: [0, 100], d: 1 });
C('plant.wastegate_open_pct', '%', 'air', 'Открытие вестгейта', 'Wastegate opening', 'Перепуск газов мимо турбины: открыт — турбина не раскручивается, закрыт — наддув растёт.', 'Turbine bypass: open = turbo does not spool, closed = boost rises.', { r: [0, 100], d: 1 });
C('plant.turbo_rpm', 'rpm', 'air', 'Обороты турбины', 'Turbo speed', 'Скорость ротора турбокомпрессора. Выше предела разрушения (из профиля) ротор разрушается.', 'Turbocharger rotor speed. Above the burst limit (profile) the rotor fails.', { r: [0, 250000], d: 0, alarm: { hi: turboBurst, snd: true } });
C('plant.maf_kg_h', 'kg/h', 'air', 'Расход воздуха', 'Air mass flow', 'Сколько воздуха проходит через дроссель за час.', 'Air mass passing the throttle per hour.', { r: [0, 1000], d: 1 });
C('plant.air_mg_stroke', 'mg', 'air', 'Наполнение цилиндра', 'Cylinder air charge', 'Масса воздуха, попадающая в цилиндр за такт впуска — главная величина для топлива и момента.', 'Air mass trapped per intake stroke — the key quantity for fuel and torque.', { r: [0, 1500], d: 1 });
// fuel
C('plant.fuel_mg_stroke[]', 'mg', 'fuel', 'Топливо, цил. {i}', 'Fuel, cyl {i}', 'Масса впрыснутого за цикл топлива (из команд впрыска прошивки).', 'Fuel mass injected per cycle (from the firmware injection commands).', { r: [0, 100], d: 2 });
C('plant.lambda_sensor', 'λ', 'fuel', 'Лямбда (датчик)', 'Lambda (sensor)', '1,00 — идеальная смесь; < 1 богатая (охлаждает); > 1 бедная (горячее сгорание, под нагрузкой — риск прогара). При отсечке топлива датчик видит воздух (большие значения).',
  '1.00 = ideal mixture; < 1 rich (cools); > 1 lean (hot combustion, risky under load). With fuel cut the sensor sees air (large values).', { r: [0.6, 1.5], d: 3 });
C('plant.lambda_cyl[]', 'λ', 'fuel', 'Лямбда, цил. {i}', 'Lambda, cyl {i}', 'Состав смеси в отдельном цилиндре.', 'Mixture of one cylinder.', { r: [0.6, 1.5], d: 3 });
C('plant.rail_mpa', 'MPa', 'fuel', 'Давление в рампе (ТНВД)', 'Fuel rail pressure (HPFP)', 'Давление непосредственного впрыска от насоса высокого давления. Без него мотор не заведётся.', 'Direct-injection pressure from the high-pressure pump. Without it the engine will not start.', { r: [0, 25], d: 2 });
C('plant.lp_fuel_bar', 'bar', 'fuel', 'Давление подкачки', 'Low-pressure fuel', 'Давление электробензонасоса в баке, питающего ТНВД.', 'In-tank electric pump pressure feeding the HPFP.', { r: [0, 7], d: 2 });
// ignition
C('plant.firing[]', '', 'ign', 'Вспышка, цил. {i}', 'Firing, cyl {i}', '1 — в цилиндре идёт сгорание; 0 — нет (нет искры или топлива).', '1 = combustion in the cylinder; 0 = none (no spark or fuel).', { bool: true });
C('plant.spark_efficiency[]', '%', 'ign', 'Эффективность УОЗ, цил. {i}', 'Spark efficiency, cyl {i}', 'Насколько угол зажигания, выбранный прошивкой, близок к оптимальному (100 % = максимум момента).', 'How close the firmware spark timing is to optimum (100 % = maximum torque).', { r: [0, 100], d: 1, k: 100 });
C('plant.knock_integral[]', '', 'ign', 'Интеграл детонации, цил. {i}', 'Knock integral, cyl {i}', 'Мера приближения смеси к самовоспламенению. ≥ 1 — детонация; долгая сильная детонация разрушает поршень.', 'How close the charge is to auto-ignition. ≥ 1 = knock; sustained heavy knock destroys the piston.', { r: [0, 2], d: 3, alarm: { hi: 1.0, snd: true } });
C('plant.knock_integral', '', 'ign', 'Интеграл детонации (макс. по цилиндрам)', 'Knock integral (max over cylinders)', 'Так его проверяет защита бокса: наибольший из четырёх цилиндров.', 'As the test-cell interlock checks it: the largest of the four cylinders.', { d: 3 });
C('plant.mbt_btdc_deg', '°BTDC', 'ign', 'УОЗ максимального момента (MBT)', 'MBT spark advance', 'Угол опережения, при котором физика даёт максимум момента в текущей точке. Свойство физики, не решение прошивки.', 'Spark advance giving maximum torque at the current point. A property of the physics, not a firmware decision.', { r: [0, 50], d: 1 });
// temperatures
C('plant.ambient_c', '°C', 'temp', 'Т окружающего воздуха', 'Ambient temperature', 'Температура воздуха в боксе / на улице (задаётся в «Настройке стенда»).', 'Air temperature of the cell / outside (set in Bench setup).', { r: [-40, 50], d: 1 });
C('plant.iat_c', '°C', 'temp', 'Т воздуха во впуске', 'Intake air temperature', 'После интеркулера. Горячий воздух — меньше мощности и больше детонации.', 'After the intercooler. Hot air = less power and more knock.', { r: [-40, 120], d: 1, alarm: { hi: 70 } });
C('plant.charge_air_c', '°C', 'temp', 'Т наддувочного воздуха', 'Charge-air temperature', 'Температура воздуха перед дросселем.', 'Air temperature upstream of the throttle.', { r: [-40, 150], d: 1 });
C('plant.compressor_out_c', '°C', 'temp', 'Т после компрессора', 'Compressor outlet temperature', 'Воздух нагревается при сжатии в компрессоре турбины.', 'Air heats up when compressed by the turbo.', { r: [-40, 250], d: 1, alarm: { hi: 200 } });
C('plant.egt_c', '°C', 'temp', 'Т выхлопных газов (EGT)', 'Exhaust gas temperature (EGT)', 'Перед турбиной. Выше ~980–1000 °C разрушаются турбина и выпускные клапаны.', 'Before the turbine. Above ~980–1000 °C the turbo and exhaust valves fail.', { r: [0, 1100], d: 0, alarm: { hi: 980, snd: true } });
C('plant.turbine_out_c', '°C', 'temp', 'Т после турбины', 'Turbine outlet temperature', 'Газы отдают энергию турбине и остывают.', 'Gas cools as it gives energy to the turbine.', { r: [0, 1000], d: 0 });
C('plant.cat_c', '°C', 'temp', 'Т катализатора', 'Catalyst temperature', 'Работает от ~300 °C; выше ~1000 °C керамика разрушается.', 'Lights off around 300 °C; above ~1000 °C the substrate is damaged.', { r: [0, 1100], d: 0, alarm: { hi: 1000 } });
C('plant.coolant_c', '°C', 'temp', 'Т охлаждающей жидкости', 'Coolant temperature', 'Рабочая 90–105 °C. Выше ~115 °C — перегрев.', 'Normal 90–105 °C. Above ~115 °C = overheating.', { r: [-40, 130], d: 1, alarm: { hi: 115, snd: true } });
C('plant.radiator_c', '°C', 'temp', 'Т радиатора', 'Radiator temperature', 'Температура жидкости в радиаторе.', 'Coolant temperature in the radiator.', { r: [-40, 130], d: 1 });
C('plant.radiator_valve_pct', '%', 'temp', 'Клапан радиатора (ITM)', 'Radiator valve (ITM)', 'Поворотный клапан N493 направляет жидкость через радиатор; им управляет прошивка.', 'Rotary valve N493 routes coolant through the radiator; driven by the firmware.', { r: [0, 100], d: 1 });
C('plant.oil_c', '°C', 'temp', 'Т масла', 'Oil temperature', 'Масло разжижается при нагреве; выше ~140–150 °C теряет несущую способность.', 'Oil thins as it heats; above ~140–150 °C it loses load capacity.', { r: [-40, 160], d: 1, alarm: { hi: 140, snd: true } });
// electrical
C('plant.battery_v', 'V', 'elec', 'Напряжение бортсети', 'System voltage', '12,6 В — АКБ без зарядки; 13,8–14,5 В — работает генератор. Ниже ~11 В электроника может сбрасываться.', '12.6 V = battery only; 13.8–14.5 V = alternator charging. Below ~11 V electronics may reset.', { r: [9, 16], d: 2, alarm: { lo: 11, hi: 15.5 } });
C('plant.starter_current_a', 'A', 'elec', 'Ток стартера', 'Starter current', 'Ток, потребляемый стартером при прокрутке (включает его реле, которым управляет прошивка).', 'Starter motor current while cranking (its relay is driven by the firmware).', { r: [0, 400], d: 0 });
// gearbox / vehicle
C('plant.shaft_rpm[]', 'rpm', 'dl', 'Обороты первичного вала {i}', 'Input shaft {i} speed', 'Вал 1 — нечётные передачи (сцепление K1), вал 2 — чётные и задняя (K2).', 'Shaft 1 = odd gears (clutch K1), shaft 2 = even gears and reverse (K2).', { r: [0, 7500], d: 0 });
C('plant.shaft_gear[]', '', 'dl', 'Передача на валу {i}', 'Gear on shaft {i}', 'Какая передача включена на валу (0 — нейтраль, −1 — задняя). Следствие положения вилок.', 'Gear engaged on the shaft (0 = neutral, −1 = reverse). A consequence of the fork positions.', { d: 0 });
C('plant.driving_gear', '', 'dl', 'Передача под нагрузкой', 'Driving gear', 'Передача, через которую сейчас идёт момент (0 — никакая, −1 — задняя).', 'Gear currently carrying torque (0 = none, −1 = reverse).', { d: 0 });
C('plant.fork_mm[]', 'mm', 'dl', 'Положение вилки {i}', 'Fork {i} position', 'Ход вилки переключения (датчики G487–G490). Отклонение от нуля = включение передачи.', 'Shift-fork travel (sensors G487–G490). Away from zero = a gear engages.', { r: [-9, 9], d: 2 });
C('plant.clutch_travel_mm[]', 'mm', 'dl', 'Ход сцепления {i}', 'Clutch {i} travel', 'Ход выжимного штока (датчики G617/G618). Точка касания — начало передачи момента.', 'Actuator rod travel (sensors G617/G618). Kiss point = torque begins to transfer.', Object.assign({ r: [0, 17], d: 2 }, KK));
C('plant.clutch_capacity_nm[]', 'Nm', 'dl', 'Предельный момент сцепления {i}', 'Clutch {i} capacity', 'Сколько момента сцепление может передать при текущем прижатии.', 'Torque the clutch can carry at the current clamp load.', Object.assign({ r: [0, 450], d: 0 }, KK));
C('plant.clutch_torque_nm[]', 'Nm', 'dl', 'Момент через сцепление {i}', 'Clutch {i} torque', 'Момент, фактически передаваемый сцеплением.', 'Torque actually transmitted by the clutch.', Object.assign({ r: [-200, 450], d: 0 }, KK));
C('plant.clutch_slip_rpm[]', 'rpm', 'dl', 'Проскальзывание сцепления {i}', 'Clutch {i} slip', 'Разница скоростей мотора и вала. Долгое проскальзывание под нагрузкой перегревает сухое сцепление.', 'Engine minus shaft speed. Long slip under load overheats a dry clutch.', Object.assign({ r: [-3000, 3000], d: 0 }, KK));
C('plant.clutch_temp_c[]', '°C', 'dl', 'Т сцепления {i}', 'Clutch {i} temperature', 'Температура дисков сцепления. Сухое сцепление DQ200 боится длительной пробуксовки.', 'Clutch disc temperature. The DQ200 dry clutch dislikes sustained slip.', Object.assign({ r: [0, 350], d: 0, alarm: { hi: 300 } }, KK));
C('plant.gearbox_hydraulic_bar', 'bar', 'dl', 'Давление гидравлики КПП', 'Gearbox hydraulic pressure', 'Давление в гидроаккумуляторе мехатроника (насос V401).', 'Mechatronic accumulator pressure (pump V401).', { r: [0, 70], d: 1 });
C('plant.vehicle_kmh', 'km/h', 'veh', 'Скорость автомобиля', 'Vehicle speed', 'Скорость кузова.', 'Body speed.', { r: [0, 250], d: 1 });
C('plant.wheel_kmh_driven', 'km/h', 'veh', 'Скорость ведущих колёс', 'Driven wheel speed', 'Если заметно больше скорости автомобиля — колёса буксуют.', 'Noticeably above vehicle speed = wheelspin.', { r: [0, 250], d: 1 });
C('plant.accel_ms2', 'm/s²', 'veh', 'Ускорение', 'Acceleration', 'Продольное ускорение кузова (9,81 м/с² = 1 g).', 'Longitudinal body acceleration (9.81 m/s² = 1 g).', { r: [-10, 10], d: 2 });
C('plant.brake_bar', 'bar', 'veh', 'Давление в тормозах', 'Brake line pressure', 'Давление в магистрали от педали тормоза.', 'Brake line pressure from the pedal.', { r: [0, 120], d: 1 });
// health
C('plant.cylinder_health[]', '%', 'hlth', 'Целостность цилиндра {i}', 'Cylinder {i} integrity', 'Поршень, кольца и клапаны цилиндра (100 % — исправно). Падает от сильной детонации, перекрута, перегрева. Не восстанавливается до «Холодного старта».',
  'Piston, rings and valves of the cylinder (100 % = intact). Drops from heavy knock, overspeed, overheating. Permanent until Cold soak.', { r: [0, 100], d: 1, k: 100, alarm: { lo: 99.5, snd: true } });
C('plant.bearing_health', '%', 'hlth', 'Целостность подшипников', 'Bearing integrity', 'Коренные и шатунные подшипники. Падает при масляном голодании и перегреве масла; 0 % — мотор заклинило.', 'Main and rod bearings. Drops with oil starvation and oil overheating; 0 % = engine seized.', { r: [0, 100], d: 1, k: 100, alarm: { lo: 99.5, snd: true } });
C('plant.turbo_health', '%', 'hlth', 'Целостность турбины', 'Turbo integrity', 'Ротор турбокомпрессора. Падает при превышении предельных оборотов.', 'Turbocharger rotor. Drops when overspeeding.', { r: [0, 100], d: 1, k: 100, alarm: { lo: 99.5, snd: true } });
// firmware outputs
C('plant.ecu_outputs_connected', '', 'ecuo', 'Выходы ЭБУ подключены к физике', 'ECU outputs connected', '1 — эмулятор переводит сигналы с выводов процессора ЭБУ в действия механизмов. 0 — физика не управляется прошивкой, мотор завестись не может.',
  '1 = the emulator turns ECU pin signals into actuator actions. 0 = the physics is not driven by the firmware; the engine cannot start.', { bool: true });
C('plant.tcu_outputs_connected', '', 'tcuo', 'Выходы ТКУ подключены к физике', 'TCU outputs connected', '1 — команды мехатроника (сцепления, вилки) доходят до физики. 0 — коробка не управляется.', '1 = mechatronic commands (clutches, forks) reach the physics. 0 = gearbox not driven.', { bool: true });
const O = (k, u, ru, en, dru, den, o) => C('plant.ecu_outputs.' + k, u, 'ecuo', ru, en, dru, den, Object.assign({ na: NA_OUT }, o));
O('main_relay', '', 'Главное реле J271', 'Main relay J271', 'Питает катушки, форсунки, привод дросселя и насосы.', 'Powers coils, injectors, throttle motor and pumps.', { bool: true });
O('starter_relay', '', 'Реле стартера J906/J907', 'Starter relays J906/J907', 'Прошивка включает стартер, когда разрешает пуск.', 'The firmware engages the starter when it permits a start.', { bool: true });
O('etc_duty', '', 'Привод дросселя (H-мост)', 'Throttle motor (H-bridge)', 'Скважность мотора заслонки: −1 закрывать … +1 открывать.', 'Throttle motor duty: −1 close … +1 open.', { r: [-1, 1], d: 3 });
O('ignition[].spark_btdc_deg', '°BTDC', 'УОЗ, катушка {i}', 'Spark advance, coil {i}', 'Угол опережения зажигания (момент разрыва тока катушки), выданный прошивкой.', 'Spark advance (coil current break) issued by the firmware.', { r: [-30, 60], d: 1 });
O('ignition[].dwell_ms', 'ms', 'Время накопления, катушка {i}', 'Dwell, coil {i}', 'Сколько прошивка держит ток в первичной обмотке перед искрой — от этого зависит энергия искры.', 'How long the firmware charges the primary before the spark — sets spark energy.', { r: [0, 8], d: 2 });
O('injection[][].soi_btdc_deg', '°BTDC', 'Начало впрыска (цил., импульс)', 'Start of injection (cyl, pulse)', 'Угол начала открытия форсунки для импульса; до 3 импульсов за цикл.', 'Crank angle where the injector opens for this pulse; up to 3 pulses per cycle.', { r: [-60, 400], d: 1 });
O('injection[][].width_ms', 'ms', 'Длительность впрыска (цил., импульс)', 'Injection width (cyl, pulse)', 'Время открытия форсунки для импульса.', 'Injector opening time of this pulse.', { r: [0, 20], d: 3 });
O('hpfp_close_deg', '°CA', 'ТНВД N276: угол закрытия клапана', 'HPFP N276 valve close angle', 'Угол коленвала, при котором прошивка закрывает клапан дозирования ТНВД (повторяется каждые 180°): раньше — больше подача.', 'Crank angle at which the firmware closes the HPFP metering valve (repeats every 180°): earlier = more delivery.', { r: [0, 180], d: 1 });
O('cam_phaser_duty[]', '', 'Фазовращатель {i} (впуск/выпуск): ШИМ', 'Cam phaser {i} (intake/exhaust) duty', 'ШИМ клапана фазовращателя распредвала N205/N318.', 'PWM of the cam phaser valve N205/N318.', { r: [0, 1], d: 3 });
O('piston_jets_off', '', 'Форсунки охлаждения поршней выкл.', 'Piston cooling jets off', '1 — прошивка перекрыла масляные форсунки охлаждения поршней.', '1 = the firmware shut the piston-cooling oil jets.', { bool: true });
O('tumble_flaps_closed', '', 'Заслонки впуска (tumble) закрыты', 'Tumble flaps closed', '1 — прошивка закрыла заслонки впускного коллектора.', '1 = the firmware closed the intake tumble flaps.', { bool: true });
O('lpfp_duty', '', 'Насос подкачки J538', 'In-tank pump J538', 'Скважность управления насосом в баке (0…1).', 'In-tank pump control duty (0…1).', { r: [0, 1], d: 3 });
O('wastegate_duty', '', 'Вестгейт V465', 'Wastegate V465', 'Привод перепуска турбины: −1 открыть … +1 закрыть.', 'Wastegate actuator: −1 open … +1 close.', { r: [-1, 1], d: 3 });
O('diverter_open', '', 'Перепускной клапан N249', 'Diverter valve N249', 'Сбрасывает наддув при закрытии дросселя.', 'Vents boost when the throttle closes.', { bool: true });
O('oil_pump_low_stage', '', 'Маслонасос N428 (низкая ступень)', 'Oil pump N428 (low stage)', '1 — насос в режиме низкого давления; 0 — высокое давление.', '1 = low-pressure stage; 0 = high-pressure stage.', { bool: true });
O('rotary_valve_duty', '', 'Клапан ITM N493', 'ITM valve N493', 'Скважность моста ATIC111 №1 (−1…+1, + = к большему углу).', 'ATIC111 bridge 1 duty (−1…+1, + = towards a larger angle).', { r: [-1, 1], d: 3 });
C('plant.rotary_valve_deg', '°', 'temp', 'Угол поворотного клапана', 'Rotary valve angle', 'Угол золотника N493 от нижнего упора (датчик SENT).', 'N493 slide angle from the lower stop (SENT sensor).', { r: [0, 263], d: 1 });
O('fan_duty', '', 'Вентилятор J293', 'Radiator fan J293', 'ШИМ вентилятора радиатора (0…1).', 'Radiator fan PWM (0…1).', { r: [0, 1], d: 3 });
O('alternator_setpoint_v', 'V', 'Уставка генератора', 'Alternator set-point', 'Напряжение, которое прошивка заказывает регулятору генератора по LIN.', 'Voltage the firmware requests from the alternator regulator over LIN.', { r: [12, 15.5], d: 2 });
const TO = (k, u, ru, en, dru, den, o) => C('plant.tcu_outputs.' + k, u, 'tcuo', ru, en, dru, den, Object.assign({ na: NA_OUT }, o));
TO('solenoid_duty[]', '%', 'Соленоид {i}: ШИМ', 'Solenoid {i} PWM duty', 'Скважность ШИМ, которую прошивка ТКУ подаёт на клапан {i} (DQ200: 0 N433, 1 N434, 2 N435 K1, 3 N436, 4 N437, 5 N438, 6 N439 K2, 7 N440; мокрые: N88–N91, N92, N215 K1, N216 K2, N217, N218, N233, N371).', 'PWM duty the TCU firmware drives on valve {i} (DQ200: 0 N433, 1 N434, 2 N435 K1, 3 N436, 4 N437, 5 N438, 6 N439 K2, 7 N440; wet: N88–N91, N92, N215 K1, N216 K2, N217, N218, N233, N371).', { r: [0, 100], d: 1, k: 100 });
TO('solenoid_current_a[]', 'A', 'Соленоид {i}: ток (уставка)', 'Solenoid {i}: current set-point', 'Ток катушки, заданный прошивкой ТКУ (токовые клапаны).', 'Coil current commanded by the TCU firmware (current-controlled valves).', { r: [0, 1.5], d: 3 });
TO('pump_motor_duty', '%', 'Гидронасос: ШИМ', 'Hydraulic pump duty', 'Скважность мотора гидронасоса мехатроника (DQ200 V401), заданная прошивкой.', 'Mechatronic pump motor duty commanded by the firmware (DQ200 V401).', { r: [0, 100], d: 1, k: 100 });
TO('pump_shaft_rad_s', 'rad/s', 'Насос (мокрые КПП): вал', 'Pump shaft (wet gearboxes)', 'Скорость вала гидронасоса по данным ТКУ.', 'Hydraulic pump shaft speed as seen by the TCU.', { d: 1 });
// ---- channels added by the physics after the first registry (names from simemu-physics/src/telemetry.rs) ----
const CYL = '{i}';
C('plant.engine_state', '', 'eng', 'Состояние двигателя', 'Engine state', 'stopped / cranking / spinning / running / seized — по физической модели.', 'stopped / cranking / spinning / running / seized — from the physics.');
C('plant.dmf_secondary_rpm', 'rpm', 'eng', 'Обороты вторичного маховика (ДМФ)', 'DMF secondary speed', 'Сторона КПП двухмассового маховика.', 'Gearbox side of the dual-mass flywheel.', { r: [0, 7500], d: 0 });
C('plant.crank_speed_ripple_rpm', 'rpm', 'eng', 'Неравномерность вращения', 'Crank speed ripple', 'Размах скорости коленвала за 720° — признак пропусков и неровной работы (это читает диагностика пропусков прошивки).', 'Peak-to-peak crank speed over 720° — misfire / roughness signature (what the firmware misfire detection reads).', { r: [0, 300], d: 1 });
C('plant.blowby_l_min', 'l/min', 'eng', 'Прорыв газов в картер', 'Blow-by', 'Расход картерных газов при плотности окружающего воздуха.', 'Crankcase blow-by at ambient density.', { r: [0, 100], d: 1 });
C('plant.oil_level_l', 'l', 'eng', 'Уровень масла', 'Oil level', 'Объём масла в поддоне.', 'Oil volume in the sump.', { r: [0, 6], d: 2 });
C('plant.oil_aeration', '%', 'eng', 'Аэрация масла', 'Oil aeration', 'Доля воздуха, захваченного маслоприёмником.', 'Air fraction drawn by the oil pickup.', { r: [0, 100], d: 1, k: 100 });
C('plant.lambda_down', 'λ', 'fuel', 'Лямбда после катализатора', 'Lambda after catalyst', 'Состав газа у второго (диагностического) зонда.', 'Gas mixture at the downstream (diagnostic) sensor.', { r: [0.6, 1.5], d: 3 });
C('plant.lsu_tip_c', '°C', 'fuel', 'Т керамики зонда 1 (ШДК)', 'Upstream sensor tip temp', 'Рабочая температура широкополосного зонда ≈ 780 °C (греет нагреватель по команде прошивки).', 'Wide-band sensor operating temp ≈ 780 °C (heater driven by the firmware).', { r: [0, 1000], d: 0 });
C('plant.lsf_tip_c', '°C', 'fuel', 'Т керамики зонда 2', 'Downstream sensor tip temp', 'Температура второго зонда.', 'Downstream sensor temperature.', { r: [0, 1000], d: 0 });
C('plant.lsu_heater_w', 'W', 'fuel', 'Мощность нагревателя зонда 1', 'Upstream heater power', 'Электрическая мощность нагревателя ШДК.', 'Wide-band heater electrical power.', { r: [0, 30], d: 1 });
C('plant.residual_frac[]', '%', 'comb', `Остаточные газы, цил. ${CYL}`, `Residual gas, cyl ${CYL}`, 'Доля отработавших газов в цилиндре на закрытии впускного клапана.', 'Burned-gas fraction at intake valve closing.', { r: [0, 30], d: 1, k: 100 });
C('plant.cam_phase_deg[]', '°CA', 'eng', 'Фазовращатель {i}', 'Cam phaser {i}', 'Положение распредвала от упора: впуск — опережение, выпуск — запаздывание (что видит датчик Холла распредвала).', 'Cam position from the locked rest: intake advance / exhaust retard (what the cam Hall sensor sees).', { il: ['впуск', 'выпуск'], r: [0, 60], d: 1 });
C('plant.cyl_peak_bar[]', 'bar', 'comb', `Пиковое давление, цил. ${CYL}`, `Peak pressure, cyl ${CYL}`, 'Максимум давления в цилиндре за цикл (абс.).', 'Peak in-cylinder pressure of the cycle (abs).', { r: [0, 150], d: 1 });
C('plant.cyl_peak_deg[]', '°ATDC', 'comb', `Угол пика давления, цил. ${CYL}`, `Peak pressure angle, cyl ${CYL}`, 'Где по углу коленвала достигнут пик давления (оптимум ≈ 12–16° после ВМТ).', 'Crank angle of the pressure peak (optimum ≈ 12–16° ATDC).', { r: [-10, 60], d: 1 });
C('plant.imep_bar[]', 'bar', 'comb', `IMEP, цил. ${CYL}`, `IMEP, cyl ${CYL}`, 'Среднее индикаторное давление (нетто) — работа цикла на литр объёма.', 'Net indicated mean effective pressure.', { r: [-2, 30], d: 2 });
C('plant.ca50_deg[]', '°ATDC', 'comb', `CA50, цил. ${CYL}`, `CA50, cyl ${CYL}`, 'Угол сгорания 50 % топлива; ≈ 8° после ВМТ — максимум КПД.', '50 % mass-burned angle; ≈ 8° ATDC = best efficiency.', { r: [-10, 60], d: 1 });
C('plant.knock_amp_bar[]', 'bar', 'ign', `Амплитуда детонации, цил. ${CYL}`, `Knock amplitude, cyl ${CYL}`, 'Пик первой окружной моды колебаний давления.', 'Peak of the first circumferential pressure mode.', { r: [0, 20], d: 2 });
C('plant.knock_sensor_mv[]', 'mV', 'ign', `Датчик детонации, окно цил. ${CYL}`, `Knock sensor, cyl ${CYL} window`, 'СКЗ полосы датчика детонации в окне цилиндра (5–70° после ВМТ) — то, что оцифровывает ЭБУ.', 'Knock-sensor band RMS in the cylinder window (5–70° ATDC) — what the ECU digitises.', { r: [0, 500], d: 0 });
C('plant.piston_crown_c[]', '°C', 'temp', `Т днища поршня, цил. ${CYL}`, `Piston crown temp, cyl ${CYL}`, 'Выше ~350 °C алюминий теряет прочность.', 'Above ~350 °C the aluminium loses strength.', { r: [0, 450], d: 0 });
C('plant.exhaust_valve_c[]', '°C', 'temp', `Т выпускного клапана, цил. ${CYL}`, `Exhaust valve temp, cyl ${CYL}`, 'Тарелка выпускного клапана.', 'Exhaust valve head.', { r: [0, 1000], d: 0 });
C('plant.bearing_c[]', '°C', 'temp', 'Т вкладыша {i}', 'Bearing {i} temp', 'Коренные 1–5, шатунные 6–9.', 'Mains 1–5, rods 6–9.', { r: [0, 200], d: 0 });
C('plant.turbo_bearing_c', '°C', 'temp', 'Т подшипника турбины', 'Turbo bearing temp', 'Корпус подшипников турбокомпрессора.', 'Turbocharger bearing housing.', { r: [0, 350], d: 0 });
C('plant.ring_land_failed[]', '', 'hlth', `Перемычка поршня разрушена, цил. ${CYL}`, `Ring land broken, cyl ${CYL}`, '1 — перемычка между канавками колец разрушена (усталость).', '1 = the ring land has failed (fatigue).', { bool: true });
C('plant.valve_float_mm[]', 'mm', 'mech', `Зависание клапана, цил. ${CYL}`, `Valve float, cyl ${CYL}`, 'Отрыв клапана от кулачка за последний цикл.', 'Valve/follower separation in the last cycle.', { r: [0, 2], d: 3 });
C('plant.hla_pumpup_mm[]', 'mm', 'mech', `Подкачка гидрокомпенсатора, цил. ${CYL}`, `HLA pump-up, cyl ${CYL}`, 'Выдвижение гидрокомпенсатора сверх нулевого зазора.', 'Hydraulic lash adjuster extension beyond zero lash.', { r: [0, 1], d: 3 });
C('plant.valve_bend_mm[][]', 'mm', 'mech', 'Изгиб стержня клапана (цил., впуск/выпуск)', 'Valve stem bend (cyl, intake/exhaust)', 'Остаточный изгиб стержня клапана.', 'Permanent valve stem bend.', { r: [0, 1], d: 3 });
C('plant.rod_bend_mm[]', 'mm', 'mech', `Укорочение шатуна, цил. ${CYL}`, `Rod shortening, cyl ${CYL}`, 'Пластическая деформация шатуна.', 'Plastic connecting-rod shortening.', { r: [0, 1], d: 3 });
C('plant.head_gasket_leak_mm2[]', 'mm²', 'mech', `Пробой прокладки ГБЦ, цил. ${CYL}`, `Head gasket breach, cyl ${CYL}`, 'Площадь прорыва прокладки.', 'Breach area of the head gasket.', { r: [0, 10], d: 2 });
C('plant.cylinder_liquid_ml[]', 'ml', 'mech', `Жидкость в цилиндре, цил. ${CYL}`, `Liquid in cylinder, cyl ${CYL}`, 'ОЖ и плёнка топлива в камере сгорания (гидроудар).', 'Coolant and fuel film in the chamber (hydrolock).', { r: [0, 10], d: 2 });
C('plant.bearing_min_film_um[]', 'µm', 'mech', 'Мин. масляная плёнка вкладыша {i}', 'Min oil film, bearing {i}', 'Коренные 1–5, шатунные 6–9. Меньше ~1 мкм — контакт металла.', 'Mains 1–5, rods 6–9. Below ~1 µm = metal contact.', { r: [0, 20], d: 2 });
C('plant.bearing_oil_fill[]', '%', 'mech', 'Заполнение маслом вкладыша {i}', 'Bearing {i} oil fill', 'Заполнение зазора вкладыша маслом.', 'Oil fill of the bearing clearance.', { r: [0, 100], d: 0, k: 100 });
C('plant.turbo_stress_ratio[]', '', 'mech', 'Напряжение колеса турбины {i} / предел', 'Turbo wheel {i} stress / UTS', 'Отношение напряжения в ступице к пределу прочности (компрессор, турбина). 1 — разрушение.', 'Hub stress over UTS (compressor, turbine). 1 = failure.', { il: ['компрессор', 'турбина'], r: [0, 1], d: 3 });
// damage / wear: damage 0..1 is shown as % of life used; wear in physical units is not a percentage
const DMG = (k, ru, en, dru, den) => C(k, '%', 'hlth', ru, en, dru, den, { r: [0, 100], d: 2, k: 100, damage: true });
DMG('plant.piston_damage[]', `Усталость перемычки поршня, цил. ${CYL}`, `Ring-land fatigue, cyl ${CYL}`, 'Накопленная усталость (правило Майнера), % ресурса; 100 % — разрушение.', 'Accumulated fatigue (Miner), % of life; 100 % = failure.');
DMG('plant.rod_damage[]', `Усталость шатуна, цил. ${CYL}`, `Rod fatigue, cyl ${CYL}`, 'Накопленная усталость шатуна, % ресурса.', 'Accumulated rod fatigue, % of life.');
DMG('plant.head_gasket_damage[]', `Повреждение прокладки ГБЦ, цил. ${CYL}`, `Head gasket damage, cyl ${CYL}`, 'Повреждение окантовки / огневого кольца, %.', 'Bead / fire-ring damage, %.');
DMG('plant.exhaust_valve_creep[]', `Ползучесть выпускного клапана, цил. ${CYL}`, `Exhaust valve creep, cyl ${CYL}`, 'Израсходованный ресурс по ползучести, %.', 'Creep life consumed, %.');
DMG('plant.turbo_damage', 'Ресурс турбины израсходован', 'Turbo life consumed', 'Максимум усталости компрессора и ползучести турбины, %.', 'Max of compressor LCF and turbine creep, %.');
C('plant.bearing_wear_um[]', 'µm', 'hlth', 'Износ вкладыша {i}', 'Bearing {i} wear', 'Износ антифрикционного слоя, мкм (коренные 1–5, шатунные 6–9).', 'Lining wear, µm (mains 1–5, rods 6–9).', { r: [0, 50], d: 2 });
C('plant.clutch_wear_mm[]', 'mm', 'hlth', 'Износ накладок сцепления {i}', 'Clutch {i} lining wear', 'Износ фрикционных накладок, мм.', 'Friction lining wear, mm.', Object.assign({ r: [0, 3], d: 3 }, KK));
C('plant.clutch_health[]', '%', 'hlth', 'Ресурс сцепления {i}', 'Clutch {i} health', 'Остаток ресурса накладок (100 % — новое).', 'Remaining lining life (100 % = new).', Object.assign({ r: [0, 100], d: 1, k: 100 }, KK));
C('plant.rotary_valve_sensor', '', 'temp', 'Датчик клапана N493 (SENT, код)', 'N493 sensor (SENT code)', '12-битный код датчика положения поворотного клапана.', '12-bit SENT value of the rotary valve position sensor.', { d: 0 });
C('plant.gearbox', '', 'dl', 'Тип КПП', 'Gearbox type', 'Модель коробки в физике.', 'Gearbox model in the physics.');
C('plant.wheel_kmh[]', 'km/h', 'veh', 'Датчик скорости колеса {i}', 'Wheel speed {i}', 'ПЛ, ПП, ЗЛ, ЗП.', 'FL, FR, RL, RR.', { il: ['ПЛ', 'ПП', 'ЗЛ', 'ЗП'], r: [0, 250], d: 1 });
C('plant.pitch_deg', '°', 'veh', 'Тангаж кузова', 'Body pitch', 'Наклон кузова при разгоне / торможении.', 'Body pitch under acceleration / braking.', { r: [-5, 5], d: 2 });
C('plant.axle_load_n[]', 'N', 'veh', 'Нагрузка на ось {i}', 'Axle {i} load', 'Передняя / задняя.', 'Front / rear.', { il: ['перед', 'зад'], r: [0, 12000], d: 0 });
C('plant.tyre_force_n[]', 'N', 'veh', 'Тяговое усилие шин, ось {i}', 'Tyre force, axle {i}', 'Продольная сила в пятне контакта.', 'Longitudinal contact-patch force.', { il: ['перед', 'зад'], r: [-10000, 10000], d: 0 });
C('plant.tyre_tread_c[]', '°C', 'veh', 'Т протектора, ось {i}', 'Tread temp, axle {i}', 'Температура протектора.', 'Tread temperature.', { il: ['перед', 'зад'], r: [0, 150], d: 0 });
C('plant.clutch_kiss_mm[]', 'mm', 'dl', 'Точка касания сцепления {i}', 'Clutch {i} kiss point', 'Физическая точка начала передачи момента (сравните с адаптацией прошивки).', 'Physical torque-onset point (compare with the firmware adaptation).', Object.assign({ r: [0, 17], d: 2 }, KK));
C('plant.clutch_bar[]', 'bar', 'dl', 'Давление сцепления {i}', 'Clutch {i} pressure', 'Давление в цилиндре / поршне сцепления.', 'Clutch actuator / piston pressure.', Object.assign({ r: [0, 70], d: 1 }, KK));
C('plant.clutch_surface_c[]', '°C', 'dl', 'Т поверхности трения {i}', 'Clutch {i} surface temp', 'Мгновенная («вспышка») температура фрикционной поверхности.', 'Friction-interface (flash) temperature.', Object.assign({ r: [0, 500], d: 0 }, KK));
C('plant.sub_gearbox_bar[]', 'bar', 'dl', 'Давление подсистемы {i}', 'Sub-gearbox {i} pressure', 'Давление в контуре частичной КПП (клапаны N436 / N440).', 'Sub-gearbox circuit pressure (valves N436 / N440).', { r: [0, 70], d: 1 });
C('plant.solenoid_current_a[]', 'A', 'dl', 'Соленоид {i}: ток (шунт ТКУ)', 'Solenoid {i}: current (TCU shunt)', 'Ток катушки, измеренный шунтом ТКУ.', 'Coil current measured by the TCU shunt.', { r: [0, 1.5], d: 3 });
C('plant.solenoid_names[]', '', 'dl', 'Соленоид {i}: обозначение', 'Solenoid {i}: designation', 'Обозначение клапана в этой КПП.', 'Valve designation in this gearbox.');
C('plant.pump_rpm', 'rpm', 'dl', 'Гидронасос: обороты', 'Hydraulic pump speed', 'Обороты электронасоса мехатроника (DQ200 V401).', 'Mechatronic e-pump speed (DQ200 V401).', { r: [0, 6000], d: 0 });
C('plant.pump_current_a', 'A', 'dl', 'Гидронасос: ток', 'Hydraulic pump current', 'Ток мотора насоса.', 'Pump motor current.', { r: [0, 30], d: 1 });
C('plant.pump_flow_lpm', 'l/min', 'dl', 'Гидронасос: подача', 'Hydraulic pump flow', 'Подача насоса.', 'Pump delivery.', { r: [0, 20], d: 2 });
C('plant.cooling_lpm', 'l/min', 'dl', 'Охлаждение сцеплений: расход', 'Clutch cooling flow', 'Расход охлаждающего масла сцеплений (мокрые КПП).', 'Clutch cooling oil flow (wet gearboxes).', { r: [0, 30], d: 1 });
C('plant.mechatronic_oil_c', '°C', 'dl', 'Т гидравлики КПП', 'Gearbox hydraulic fluid temp', 'DQ200 — датчик G510, мокрые — ATF в поддоне.', 'DQ200 sensor G510; wet — ATF sump.', { r: [-40, 150], d: 1 });
C('plant.gearbox_case_c', '°C', 'dl', 'Т картера КПП', 'Gearbox case temp', 'Температура картера коробки.', 'Gearbox case temperature.', { r: [-40, 150], d: 1 });
C('plant.driveshaft_torque_nm', 'Nm', 'dl', 'Момент на карданном валу', 'Prop-shaft torque', 'Полный привод: момент к задней оси.', 'AWD: torque to the rear axle.', { r: [-500, 2000], d: 0 });
C('plant.haldex_bar', 'bar', 'dl', 'Давление муфты Haldex', 'Haldex pressure', 'Давление в муфте полного привода.', 'AWD coupling pressure.', { r: [0, 40], d: 1 });
C('plant.haldex_torque_nm', 'Nm', 'dl', 'Момент через муфту Haldex', 'Haldex torque', 'Момент, передаваемый на заднюю ось.', 'Torque sent to the rear axle.', { r: [0, 2000], d: 0 });
C('plant.brake_disc_c[]', '°C', 'veh', 'Т тормозного диска, ось {i}', 'Brake disc temp, axle {i}', 'Температура тормозных дисков.', 'Brake disc temperature.', { il: ['перед', 'зад'], r: [0, 700], d: 0 });
C('plant.roller_kmh', 'km/h', 'veh', 'Скорость барабана стенда', 'Roller speed', 'Скорость барабанов шасси-стенда.', 'Chassis-dyno roller speed.', { r: [0, 300], d: 1 });
C('plant.roller_absorber_n', 'N', 'veh', 'Сила тормоза барабанного стенда', 'Roller absorber force', 'Окружная сила, которую тормоз стенда прикладывает к барабанам.', 'Tangential force of the roller absorber.', { r: [0, 10000], d: 0 });
C('plant.wastegate_pos_pct', '%', 'air', 'Положение заслонки вестгейта', 'Wastegate flap position', '0 — закрыта (седло), 100 — открыта до упора. Электропривод V465.', '0 = seated, 100 = open stop. E-actuator V465.', { r: [0, 100], d: 1 });
C('plant.wastegate_sensor_v', 'V', 'air', 'Датчик положения вестгейта', 'Wastegate position sensor', 'Выход датчика положения привода (читает ЭБУ).', 'Actuator position-sensor output (read by the ECU).', { r: [0, 5], d: 3 });
C('plant.wastegate_motor_a', 'A', 'air', 'Ток мотора вестгейта', 'Wastegate motor current', '+ — момент на закрытие.', '+ = closing torque.', { r: [-10, 10], d: 2 });
C('plant.port_flap_sensor_v', 'V', 'air', 'Датчик заслонок впуска', 'Port-flap sensor', 'Выход датчика положения заслонок впускного коллектора.', 'Intake port-flap position-sensor output.', { r: [0, 5], d: 3 });
O('hpfp_off_deg', '°CA', 'ТНВД N276: угол открытия клапана', 'HPFP N276 valve open angle', 'Угол, где прошивка снимает ток с клапана дозирования ТНВД.', 'Crank angle where the firmware drops the HPFP metering valve current.', { r: [0, 180], d: 1 });
O('port_injection[][].soi_btdc_deg', '°BTDC', 'MPI: начало впрыска (цил., импульс)', 'MPI start of injection (cyl, pulse)', 'Угол начала впрыска форсунки во впускной канал.', 'Port injector start angle.', { r: [-60, 720], d: 1 });
O('port_injection[][].width_ms', 'ms', 'MPI: длительность впрыска (цил., импульс)', 'MPI injection width (cyl, pulse)', 'Время открытия форсунки во впускной канал.', 'Port injector opening time.', { r: [0, 20], d: 3 });
// A null command (coil not fired / pulse not issued in the last cycle) arrives without sub-fields.
const NA_PULSE = ['в последнем цикле прошивка этот импульс не выдала (или выход не декодирован)', 'the firmware issued no such pulse in the last cycle (or the output is not decoded)'];
O('ignition[]', '', 'Катушка {i}: команды нет', 'Coil {i}: no command', 'Катушка не получила команду искры.', 'The coil received no spark command.', { na: NA_PULSE });
O('injection[][]', '', 'DI: импульса нет (цил., №)', 'DI: no pulse (cyl, #)', 'Форсунка непосредственного впрыска: импульса с этим номером нет.', 'Direct injector: no pulse with this number.', { na: NA_PULSE });
O('port_injection[][]', '', 'MPI: импульса нет (цил., №)', 'MPI: no pulse (cyl, #)', 'Форсунка впрыска во впускной канал: импульса с этим номером нет.', 'Port injector: no pulse with this number.', { na: NA_PULSE });
O('lambda_heater_duty[]', '%', 'Нагреватель лямбда-зонда {i}: ШИМ', 'Lambda heater {i} duty', 'Скважность нагревателя зонда (1 — до, 2 — после катализатора).', 'Sensor heater duty (1 = upstream, 2 = downstream).', { r: [0, 100], d: 1, k: 100 });
const CB = (k, u, ru, en, dru, den, o) => C('plant.combustion.' + k, u, 'comb', ru, en, dru, den, o);
CB('ign_deg[]', '°BTDC', `Момент искры (физика), цил. ${CYL}`, `Spark timing (physics), cyl ${CYL}`, 'Фактический угол искры в цилиндре.', 'Actual spark angle in the cylinder.', { r: [-30, 60], d: 1 });
CB('ca10_deg[]', '°ATDC', `CA10, цил. ${CYL}`, `CA10, cyl ${CYL}`, 'Угол сгорания 10 % топлива.', '10 % mass-burned angle.', { r: [-30, 60], d: 1 });
CB('ca90_deg[]', '°ATDC', `CA90, цил. ${CYL}`, `CA90, cyl ${CYL}`, 'Угол сгорания 90 % топлива.', '90 % mass-burned angle.', { r: [-10, 120], d: 1 });
CB('u_prime_ms[]', 'm/s', `Интенсивность турбулентности, цил. ${CYL}`, `Turbulence u', cyl ${CYL}`, 'Пульсационная скорость в камере.', 'Turbulent velocity fluctuation.', { r: [0, 20], d: 2 });
CB('s_l_ms[]', 'm/s', `Ламинарная скорость пламени, цил. ${CYL}`, `Laminar flame speed, cyl ${CYL}`, 'Скорость фронта пламени без турбулентности.', 'Flame-front speed without turbulence.', { r: [0, 2], d: 3 });
CB('stratification[]', '', `Расслоение заряда, цил. ${CYL}`, `Charge stratification, cyl ${CYL}`, 'Неоднородность смеси (0 — гомогенная).', 'Mixture inhomogeneity (0 = homogeneous).', { r: [0, 1], d: 3 });
CB('t_end_gas_c[]', '°C', `Т несгоревшей смеси, цил. ${CYL}`, `End-gas temp, cyl ${CYL}`, 'Температура «концевых» газов — главный фактор детонации.', 'End-gas temperature — the main knock driver.', { r: [0, 1200], d: 0 });
CB('t_burned_c[]', '°C', `Т продуктов сгорания, цил. ${CYL}`, `Burned gas temp, cyl ${CYL}`, 'Пиковая температура сгоревших газов.', 'Peak burned-gas temperature.', { r: [0, 3000], d: 0 });
CB('knock_unburned[]', '%', `Несгоревшее при детонации, цил. ${CYL}`, `Unburned at knock onset, cyl ${CYL}`, 'Доля несгоревшей смеси в момент самовоспламенения.', 'Unburned fraction at autoignition.', { r: [0, 100], d: 1, k: 100 });
CB('p_local_bar[]', 'bar', `Локальное давление детонации, цил. ${CYL}`, `Local knock pressure, cyl ${CYL}`, 'Пик локального давления при детонации.', 'Local pressure peak at knock.', { r: [0, 300], d: 1 });
CB('no_ppm[]', 'ppm', `NO, цил. ${CYL}`, `NO, cyl ${CYL}`, 'Оксид азота в отработавших газах цилиндра.', 'Nitric oxide in the cylinder exhaust.', { r: [0, 5000], d: 0 });
CB('co_pct[]', '%', `CO, цил. ${CYL}`, `CO, cyl ${CYL}`, 'Монооксид углерода.', 'Carbon monoxide.', { r: [0, 10], d: 2 });
CB('hc_ppm[]', 'ppm', `HC, цил. ${CYL}`, `HC, cyl ${CYL}`, 'Несгоревшие углеводороды.', 'Unburned hydrocarbons.', { r: [0, 5000], d: 0 });
CB('soot_mg[]', 'mg', `Сажа за цикл, цил. ${CYL}`, `Soot per cycle, cyl ${CYL}`, 'Масса сажи за цикл.', 'Soot mass per cycle.', { r: [0, 1], d: 4 });
CB('piston_land_c[]', '°C', `Т перемычки поршня, цил. ${CYL}`, `Piston land temp, cyl ${CYL}`, 'Температура верхней перемычки.', 'Top land temperature.', { r: [0, 450], d: 0 });
CB('piston_groove_c[]', '°C', `Т канавки кольца, цил. ${CYL}`, `Ring groove temp, cyl ${CYL}`, 'Выше ~250 °C масло коксуется.', 'Above ~250 °C the oil cokes.', { r: [0, 400], d: 0 });
CB('exhaust_seat_c[]', '°C', `Т седла выпуска, цил. ${CYL}`, `Exhaust seat temp, cyl ${CYL}`, 'Седло выпускного клапана.', 'Exhaust valve seat.', { r: [0, 800], d: 0 });
CB('plug_tip_c[]', '°C', `Т центрального электрода, цил. ${CYL}`, `Plug centre electrode, cyl ${CYL}`, 'Выше ~850 °C — калильное зажигание.', 'Above ~850 °C = hot-surface ignition.', { r: [0, 1100], d: 0, alarm: { hi: 850 } });
CB('plug_ground_c[]', '°C', `Т бокового электрода, цил. ${CYL}`, `Plug ground electrode, cyl ${CYL}`, 'Температура бокового электрода.', 'Ground electrode temperature.', { r: [0, 1100], d: 0 });
CB('deposit_c[]', '°C', `Т нагара, цил. ${CYL}`, `Deposit temp, cyl ${CYL}`, 'Температура отложений в камере.', 'Combustion-chamber deposit temperature.', { r: [0, 1000], d: 0 });
CB('plug_fouling[]', '', `Загрязнение свечи, цил. ${CYL}`, `Plug fouling, cyl ${CYL}`, 'Нагар на изоляторе (0 — чистая).', 'Insulator fouling (0 = clean).', { r: [0, 1], d: 3 });
CB('land_stress_ratio[]', '', `Напряжение перемычки / предел, цил. ${CYL}`, `Land stress / UTS, cyl ${CYL}`, 'Пик напряжения перемычки за цикл к пределу прочности.', 'Ring-land peak stress over UTS in the cycle.', { r: [0, 1.2], d: 3 });
CB('land_stress_max[]', '', `Макс. напряжение перемычки / предел, цил. ${CYL}`, `Max land stress / UTS, cyl ${CYL}`, 'Наибольшее значение с начала работы.', 'Highest seen since start.', { r: [0, 1.2], d: 3 });
CB('preign_events[]', '', `Предвоспламенений, цил. ${CYL}`, `Pre-ignitions, cyl ${CYL}`, 'Счётчик (калильное + LSPI) с начала.', 'Count (hot surface + LSPI) since start.', { d: 0 });
CB('lspi_events[]', '', `LSPI, цил. ${CYL}`, `LSPI, cyl ${CYL}`, 'Низкооборотное предвоспламенение, счётчик.', 'Low-speed pre-ignition count.', { d: 0 });
CB('knock_events[]', '', `Циклов с детонацией, цил. ${CYL}`, `Knocking cycles, cyl ${CYL}`, 'Счётчик с начала.', 'Count since start.', { d: 0 });
CB('misfire_events[]', '', `Пропусков, цил. ${CYL}`, `Misfires, cyl ${CYL}`, 'Счётчик с начала.', 'Count since start.', { d: 0 });
CB('preignition[]', '', `Предвоспламенение сейчас, цил. ${CYL}`, `Pre-ignition now, cyl ${CYL}`, '1 — в последнем цикле.', '1 = in the last cycle.', { bool: true });
CB('preign_deg[]', '°ATDC', `Угол предвоспламенения, цил. ${CYL}`, `Pre-ignition angle, cyl ${CYL}`, 'Где началось предвоспламенение.', 'Where the pre-ignition started.', { r: [-60, 30], d: 1 });
CB('oil_fuel_pct', '%', 'Топливо в масле', 'Fuel in oil', 'Разжижение масла топливом, % массы.', 'Oil dilution by fuel, mass %.', { r: [0, 10], d: 2 });
CB('afterburn_kw', 'kW', 'Догорание в коллекторе', 'Manifold afterburning', 'Тепловыделение догорания в выпускном коллекторе.', 'Heat release of afterburning in the exhaust manifold.', { r: [0, 100], d: 1 });
C('driver.selector', '', 'drv', 'Селектор КПП', 'Gear selector', 'Положение рычага (P/R/N/D/S/M).', 'Lever position (P/R/N/D/S/M).');
const PF = (k, u, ru, en, dru, den, o) => C('frame.' + k, u, 'sim', ru, en, dru, den, o);
PF('pacing.speed', '×', 'Заданная скорость симуляции', 'Requested sim speed', '1 — реальное время.', '1 = real time.', { d: 2 });
PF('pacing.rtf', '×', 'Фактическая скорость (по часам)', 'Achieved speed (wall clock)', 'Время симуляции / настенное время.', 'Simulated time / wall time.', { d: 2 });
PF('pacing.lag_s', 's', 'Отставание от реального времени', 'Lag behind real time', 'Сколько симуляция отстаёт от настенных часов.', 'How far the simulation is behind the wall clock.', { d: 3 });
PF('pacing.slip_s', 's', 'Сброшенное отставание', 'Dropped lag', 'Отставание, которое планировщик перестал догонять (время не пропускается, просто идёт медленнее).', 'Lag the pacer stopped catching up (no simulation is skipped, it just runs slower).', { d: 2 });
PF('pacing.overruns', '', 'Перегрузок шага', 'Step overruns', 'Сколько раз шаг считался дольше реального времени.', 'Steps that took longer than real time.', { d: 0 });
for (const [k, ru, en] of [['windows', 'окон обмена', 'exchange windows'], ['plant_s', 'физика, с', 'plant, s'], ['ecu_s', 'ЭБУ, с', 'ECU, s'], ['tcu_s', 'ТКУ, с', 'TCU, s'], ['wait_tcu_s', 'ожидание ТКУ, с', 'TCU wait, s'],
  ['tcu_idle_s', 'простой ТКУ, с', 'TCU idle, s'], ['glue_s', 'обвязка, с', 'glue, s'], ['max_window_s', 'макс. окно, с', 'max window, s'], ['main_cpu_s', 'CPU основного потока, с', 'main thread CPU, s'], ['tcu_cpu_s', 'CPU потока ТКУ, с', 'TCU thread CPU, s']])
  PF('perf.' + k, k === 'windows' ? '' : 's', `Шаг: ${ru}`, `Step: ${en}`, 'Профиль последнего шага расчёта (docs/PERFORMANCE.md).', 'Profile of the last compute step (docs/PERFORMANCE.md).', { d: k === 'windows' ? 0 : 4 });
for (const [b, ru, en] of [['ecu', 'ЭБУ', 'ECU'], ['tcu', 'ТКУ', 'TCU']]) for (const [k, r2, e2] of [['sw_id', 'ПО', 'software'], ['pc', 'счётчик команд', 'program counter'], ['region', 'область памяти', 'memory region'], ['error', 'ошибка', 'error']])
  C(`${b}.${k}`, '', 'brd', `${ru}: ${r2}`, `${en}: ${e2}`, 'Состояние платы.', 'Board state.');
C('immo.binding', '', 'imm', 'Привязка иммобилайзера', 'Immobiliser binding', 'Как привязан иммобилайзер при загрузке образа.', 'How the immobiliser was bound at image load.');
C('immo.lab', '', 'imm', 'Лабораторная привязка', 'Lab binding', '1 — лабораторные ключи, 0 — данные дампа.', '1 = lab keys, 0 = dump data.', { bool: true });
// driver / sim / boards
C('driver.pedal_pct', '%', 'drv', 'Педаль газа', 'Accelerator pedal', 'Положение педали (запрос водителя); прошивка решает, что делать с дросселем.', 'Pedal position (driver request); the firmware decides the throttle.', { r: [0, 100], d: 0 });
C('driver.brake_pct', '%', 'drv', 'Педаль тормоза', 'Brake pedal', 'Положение педали тормоза.', 'Brake pedal position.', { r: [0, 100], d: 0 });
C('driver.ignition_on', '', 'drv', 'Зажигание (КЛ15)', 'Ignition (KL15)', 'Клемма 15 включена.', 'Terminal 15 on.', { bool: true });
C('driver.start_button', '', 'drv', 'Запрос пуска (КЛ50)', 'Start request (KL50)', 'Кнопка «Старт» нажата.', 'Start button held.', { bool: true });
C('driver.selector', '', 'drv', 'Селектор', 'Selector', 'Положение рычага селектора (вход водителя; по CAN Waehlhebel_03).', 'Selector lever position (driver input; on CAN as Waehlhebel_03).');
// gearbox: what the TCU firmware itself reports on the bus (studio GearboxBus)
C('gearbox_bus.gear', '', 'dl', 'Передача по ТКУ (WBA_03)', 'Gear per TCU (WBA_03)', 'WBA_03.WBA_eing_Gang_02 (0x394): включённая передача, как её сообщает прошивка ТКУ.', 'WBA_03.WBA_eing_Gang_02 (0x394): the engaged gear as the TCU firmware reports it.', { d: 0 });
C('gearbox_bus.fahrstufe', '', 'dl', 'Режим КПП по ТКУ (GE_Fahrstufe)', 'Gearbox mode per TCU (GE_Fahrstufe)', 'Getriebe_11.GE_Fahrstufe: позиция, которую использует ТКУ (5 P, 6 R, 7 N, 8 D, 9 S, 14 типтроник — таблица DQ200 0x801382BC).', 'Getriebe_11.GE_Fahrstufe: the position the TCU uses (5 P, 6 R, 7 N, 8 D, 9 S, 14 tiptronic — DQ200 table 0x801382BC).', { d: 0 });
C('gearbox_bus.zielgang', '', 'dl', 'Целевая передача ТКУ (GE_Zielgang)', 'TCU target gear (GE_Zielgang)', 'Getriebe_11.GE_Zielgang: передача, которую прошивка ТКУ готовит.', 'Getriebe_11.GE_Zielgang: the gear the TCU firmware is preparing.', { d: 0 });
C('plant.sim_time_s', 's', 'sim', 'Время симуляции', 'Simulation time', 'Сколько секунд прошло в виртуальном мире.', 'Seconds elapsed in the virtual world.', { d: 2 });
C('frame.realtime_factor', '×', 'sim', 'Скорость расчёта (RTF)', 'Compute speed (RTF)', 'Время симуляции / время расчёта шага на сервере. 1 = реальное время, < 1 — компьютер не успевает.', 'Simulated time / server compute time of the step. 1 = real time, < 1 = the host cannot keep up.', { r: [0, 3], d: 2 });
C('frame.wall_s', 's', 'sim', 'Время расчёта шага', 'Step compute time', 'Сколько сервер считал последний шаг.', 'Server compute time of the last step.', { d: 3 });
C('frame.sim_dt_s', 's', 'sim', 'Длина шага', 'Step length', 'Сколько времени симуляции добавил последний шаг.', 'Simulated time advanced by the last step.', { d: 3 });
for (const [b, ru, en] of [['ecu', 'ЭБУ', 'ECU'], ['tcu', 'ТКУ', 'TCU']]) {
  C(b + '.present', '', 'brd', `${ru}: плата есть`, `${en}: board present`, 'Эмулируется ли эта плата.', 'Whether this board is emulated.', { bool: true });
  C(b + '.stm_ms', 'ms', 'brd', `${ru}: системный таймер STM`, `${en}: system timer STM`, 'Время, которое «прожил» процессор. Растёт — прошивка работает.', 'Time the CPU has lived. Increasing = firmware running.', { d: 1 });
  C(b + '.irq_count', '', 'brd', `${ru}: прерываний`, `${en}: interrupts`, 'Количество обработанных аппаратных прерываний.', 'Hardware interrupts dispatched.', { d: 0 });
  C(b + '.warm_resets', '', 'brd', `${ru}: перезапусков`, `${en}: warm resets`, 'Сколько раз прошивка перезапустила процессор (сторожевой таймер, ошибка). Больше 0 — повод разобраться.', 'Times the firmware reset the CPU (watchdog, fault). Above 0 deserves attention.', { d: 0, alarm: { hi: 0 } });
  C(b + '.rtos_tasks_seen', '', 'brd', `${ru}: задач ОС`, `${en}: RTOS tasks seen`, 'Сколько разных задач операционной системы прошивки уже запускалось.', 'Distinct firmware RTOS tasks seen running.', { d: 0 });
  C(b + '.rtos_task_runs', '', 'brd', `${ru}: запусков задач`, `${en}: RTOS task runs`, 'Сколько раз запускались задачи ОС.', 'Total RTOS task activations.', { d: 0 });
  C(b + '.can_tx_total', '', 'brd', `${ru}: CAN-кадров отправлено`, `${en}: CAN frames sent`, 'Сколько кадров прошивка отправила на шину.', 'Frames the firmware sent to the bus.', { d: 0 });
}
C('immo.ecu_challenges', '', 'imm', 'ЭБУ: запросов иммобилайзера', 'ECU immobiliser challenges', 'Кадры 0x010 от ЭБУ к приборке.', 'Frames 0x010 from the ECU to the cluster.', { d: 0 });
C('immo.ecu_answers', '', 'imm', 'ЭБУ: ответов приборки', 'ECU cluster answers', 'Ответы виртуальной приборки 0x011.', 'Virtual cluster answers 0x011.', { d: 0 });
C('immo.tcu_challenges', '', 'imm', 'ТКУ: запросов иммобилайзера', 'TCU immobiliser challenges', 'Кадры 0x012 от коробки.', 'Frames 0x012 from the gearbox.', { d: 0 });
C('immo.tcu_answers', '', 'imm', 'ТКУ: ответов приборки', 'TCU cluster answers', 'Ответы приборки 0x013.', 'Cluster answers 0x013.', { d: 0 });
C('immo.ecu_key_known', '', 'imm', 'Ключ ЭБУ известен', 'ECU key known', '1 — виртуальная приборка знает ключ WFS5 моторного ЭБУ и может ответить правильно.', '1 = the virtual cluster knows the engine ECU WFS5 key and can answer correctly.', { bool: true });

// ---------- automatic metadata for fields the registry does not know ----------
const SUFFIX = [[/_kpa$/, 'kPa'], [/_mpa$/, 'MPa'], [/_bar$/, 'bar'], [/_pa$/, 'Pa'], [/_c$/, '°C'], [/_k$/, 'K'], [/_rpm$/, 'rpm'], [/_nm$/, 'Nm'], [/_pct$/, '%'],
  [/_mm$/, 'mm'], [/_kw$/, 'kW'], [/_ps$/, 'PS'], [/_w$/, 'W'], [/_ms2$/, 'm/s²'], [/_ms$/, 'ms'], [/_us$/, 'µs'], [/_s$/, 's'], [/_v$/, 'V'], [/_a$/, 'A'],
  [/_deg$/, '°'], [/_btdc_deg$/, '°BTDC'], [/_kmh$/, 'km/h'], [/_kg_h$/, 'kg/h'], [/_kg_s$/, 'kg/s'], [/_g_s$/, 'g/s'], [/_mg(_stroke)?$/, 'mg'], [/_n$/, 'N'], [/_j$/, 'J'], [/_hz$/, 'Hz'], [/_m$/, 'm']];
function autoMeta(key, idx) {
  const parts = key.replace(/\[\d+\]/g, '').split('.');
  const leaf = parts[parts.length - 1];
  let unit = '';
  for (const [re, u] of SUFFIX) if (re.test(leaf)) unit = u;
  const healthy = /health|integrity/i.test(leaf), worn = /wear|damage/i.test(leaf);
  const nice = (parts.length > 2 ? parts.slice(1, -1).join(' › ') + ' › ' : '') + leaf.replace(/_/g, ' ') + (idx.length ? ' ' + idx.map(i => `[${i + 1}]`).join('') : '');
  const group = healthy || worn ? 'hlth' : parts[0] === 'plant' && parts.length > 2 ? 'auto:' + parts[1] : 'new';
  const m = { key, unit, group, name: [nice, nice], desc: [`Поле телеметрии ${key}.`, `Telemetry field ${key}.`], auto: true };
  if (healthy) Object.assign(m, { unit: '%', k: 100, r: [0, 100], alarm: { lo: 99.5 } });
  return m;
}
const MCACHE = new Map();
// Solenoid labels come from the gearbox model (plant.solenoid_names), not from a fixed table.
let solSig = null;
HOOKS.push(s => {
  const n = s.plant && s.plant.solenoid_names, sig = n ? n.join('|') : '';
  if (sig === solSig) return;
  solSig = sig;
  const il = n ? n.map((x, i) => (x && x !== '-' ? x : String(i + 1))) : undefined;
  for (const k of ['plant.tcu_outputs.solenoid_duty[]', 'plant.tcu_outputs.solenoid_current_a[]', 'plant.solenoid_current_a[]', 'plant.solenoid_names[]']) REG[k].il = il;
  MCACHE.clear();
});
function meta(key) {
  let m = MCACHE.get(key);
  if (m) return m;
  const idx = Array.from(key.matchAll(/\[(\d+)\]/g), x => +x[1]);
  const base = REG[key] || REG[key.replace(/\[\d+\]/g, '[]')];
  if (base) {
    const i = idx.length ? idx[idx.length - 1] : 0;
    const lab = base.il && base.il[i] != null ? base.il[i] : String(i + 1);
    m = Object.assign({}, base, { key, name: base.name.map(s => s.replace('{i}', lab)), desc: base.desc.map(s => s.replace('{i}', lab)) });
  } else m = autoMeta(key, idx);
  MCACHE.set(key, m);
  return m;
}
const chName = k => L(meta(k).name);
const groupName = g => (GROUPS[g] ? L(GROUPS[g]) : g.startsWith('auto:') ? `${L(['Новое', 'New'])}: ${g.slice(5)}` : g);
const dv = (k, v) => (isNum(v) ? v * (meta(k).k || 1) : v);
function fmtCh(k, v) {
  if (typeof v === 'string') return v || DASH;
  if (!isNum(v)) return DASH;
  const m = meta(k);
  if (m.bool) return v ? '1 ●' : '0 ○';
  const x = v * (m.k || 1);
  if (m.d != null) return x.toFixed(m.d);
  const a = Math.abs(x);
  return x.toFixed(a >= 1000 ? 0 : a >= 100 ? 1 : a >= 10 ? 2 : 3);
}
function groupKeys(keys) {
  const by = new Map();
  for (const k of keys) { const g = meta(k).group; if (!by.has(g)) by.set(g, []); by.get(g).push(k); }
  return Array.from(by.entries()).sort((a, b) => {
    const ia = GROUP_ORDER.indexOf(a[0]), ib = GROUP_ORDER.indexOf(b[0]);
    return (ia < 0 ? 99 : ia) - (ib < 0 ? 99 : ib) || a[0].localeCompare(b[0]);
  });
}
const COLORS = ['#36b5ff', '#ff6b6b', '#ffd23f', '#5ee27a', '#c48cff', '#ff9f43', '#2ee6d6', '#ff5fd2', '#a3e635', '#9aa9ff', '#f4f4f4', '#ffb3a7'];
