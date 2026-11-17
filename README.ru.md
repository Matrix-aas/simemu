# SimEmu

**[English](README.md) | Русский**

Аппаратный эмулятор ЭБУ двигателя **Continental SIMOS18.1** (Infineon SAK-TC1791, TriCore 1.6) и мехатроников
**DSG7 DQ200_G2** (`0CW`, Infineon SAK-TC1784, TriCore 1.3.1) и **DSG6 DQ250 MQB** (`0D9`, Infineon SAK-TC1766, TriCore 1.3)
в замкнутом контуре с физической моделью двигателя **EA888 Gen3** (1.8 / 2.0 TSI), трансмиссии, автомобиля и акустики.

Заводские и модифицированные прошивки исполняются **побайтно без изменений** (`Zero-Patch`) в нативном ядре
`tricore-core` (по умолчанию в CLI и Studio, скорость расчёта полной co-simulation **~1.4–1.7× реального времени**)
или в референсном бэкенде Unicorn. Rust эмулирует исключительно архитектуру процессора, периферию MCU, внешние микросхемы
плат и законы физики (`Zero-Fake-Logic`): все решения управления двигателем и КПП принимают сами прошивки.

- **Путеводитель по памяти (`rom/sboot.bin`, `rom/dflash.bin`), ключам WFS5 и полному циклу запуска ECU/TCU:** [`docs/FIRMWARE_AND_BOOT_GUIDE.ru.md`](docs/FIRMWARE_AND_BOOT_GUIDE.ru.md) ([English](docs/FIRMWARE_AND_BOOT_GUIDE.md))
- Инженерные правила и исходные данные: [`AGENTS.md`](AGENTS.md)
- Архитектура, граф крейтов и Black-Box контракты: [`ARCHITECTURE.md`](ARCHITECTURE.md)
- Проверенные результаты и дорожная карта: [`MILESTONE.md`](MILESTONE.md) (все 37 сценариев `scripts/run_scenarios.sh` — PASS)
- Руководство для разработчиков и AI-агентов: [`CONTRIBUTING.md`](CONTRIBUTING.md)
- Юридический текст лицензии: [`LICENSE`](LICENSE)

---

## Структура репозитория

```text
simemu/
├── Cargo.toml             # Манифест Cargo workspace (8 крейтов)
├── LICENSE                # PolyForm Noncommercial 1.0.0 + Additional Terms (© 2026 Matrix)
├── README.md              # Документация на английском языке (по умолчанию)
├── README.ru.md           # Документация на русском языке
├── ARCHITECTURE.md        # Схема крейтов, память MCU, периферия и потоки данных
├── MILESTONE.md           # Верифицированные результаты, бенчмарки и известные ограничения
├── CONTRIBUTING.md        # Инварианты Zero-Patch / Zero-Fake-Logic и чек-лист PR
├── AGENTS.md              # Правила работы и доказательная база для ИИ-агентов
├── rom/
│   ├── sboot.bin          # Штатный BootROM SIMOS18.1 (SBOOT 112 КБ для .frf и OBD-дампов)
│   └── dflash.bin         # Чистый обезличенный DFlash / FEE SIMOS18.1 (256 КБ, лаб. ключи WFS5)
├── crates/
│   ├── tricore-core/      # Нативное ядро Infineon TriCore 1.3 / 1.3.1 / 1.6 на чистом Rust
│   ├── unicorn-tricore-sys/ # Опциональные FFI-привязки к референсному libunicorn 2.1.x
│   ├── simemu-tc1791/     # Платы SIMOS18.1 (TC1791), DQ200_G2 (TC1784), DQ250 (TC1766) и ASIC
│   ├── simemu-physics/    # Поугловая физика EA888 Gen3, турбокомпрессора, КПП и шасси
│   ├── simemu-acoustics/  # Физический синтез звука двигателя, выпуска, ТНВД и детонации
│   ├── simemu-damos/      # Парсер карт DAMOS/A2L, распаковка .frf/.odx и контрольные суммы CRC / ECM3
│   ├── simemu-bridge/     # Кодировщик CAN-шины шасси MQB и UDP-шлюз
│   └── simemu-cli/        # CLI `simemu`, веб-стенд Studio и TCP-сервер Lawicel/SLCAN
├── definitions/           # JSON-схемы калибровок DAMOS (SC8O40, SC8S50, SC8O30, SC8H83, SC8O20)
├── docs/                  # Чистые технические спецификации плат, пинауты и отчёты реверса
├── scripts/               # Утилиты распаковки FRF, сборки ELF, профилирования и прогона сценариев
└── vault/                 # [вне Git] Единая папка для пользовательских прошивок, FRF и закрытых PDF
```

---

## Где взять и куда положить прошивки (`vault/firmwares/`)

Репозиторий **не содержит** сторонних OEM-прошивок и закрытых даташитов производителей: каталог `vault/` добавлен в
`.gitignore`. При отсутствии прошивок проект полностью собирается, а все 5 чисто-Rust крейтов проходят модульные тесты
сразу из коробки (`cargo test --release -p tricore-core -p simemu-physics -p simemu-acoustics -p simemu-damos -p simemu-bridge`);
интеграционные тесты с прошивками корректно выводят `SKIP`.

Для полноценного запуска виртуального моторного стенда Studio и E2E-сценариев разложите прошивки в каталог `vault/firmwares/`
(в архивах релизов этот каркас папок уже создан; также можно указать свой путь через `$SIMEMU_IMAGE_DIR`):

```text
rom/
├── sboot.bin              # Штатный BootROM SIMOS18.1 (112 КБ, уже включён для .frf и OBD-дампов)
└── dflash.bin             # Чистый обезличенный DFlash / FEE SIMOS18.1 (256 КБ, уже включён в репозиторий)
vault/
└── firmwares/
    ├── ecu/
    │   ├── frf/           # Официальные контейнеры обновлений VAG ODIS (.frf)
    │   ├── stock/         # Опционально: распакованные 4 МБ заводские образы SIMOS18.1 (.bin)
    │   ├── bench/         # Опционально: собственный стендовый DFlash 256 КБ / PFlash 4 МБ / sboot.bin
    │   └── custom/        # Тюнинговые / пользовательские прошивки SIMOS18.1 (.bin или .frf)
    └── tcu/
        ├── dq200/         # Полный стендовый дамп мехатроника DSG7 DQ200_G2 (micro + eeprom)
        └── dq250/         # Полный вычит Flash мехатроника DSG6 DQ250 MQB (1.5 МБ .bin)
```

### Что именно нужно пользователю

1. **Чистая заводская (Stock) или пользовательская прошивка ЭБУ SIMOS18.1 (`vault/firmwares/ecu/frf/` или `custom/`)**:
   - **SimEmu читает, расшифровывает и исполняет официальные контейнеры VAG ODIS `.frf` (и `.odx`) напрямую на чистом Rust** (`simemu_damos::frf`) — никакие внешние распаковщики или Python-скрипты не требуются.
   - Положите контейнер `.frf` из марочных дисков **VAG Flashdaten** в `vault/firmwares/ecu/frf/`:
     - для **1.8 TSI EA888 Gen3 (`CJSA` / `CJSB`, Octavia A7)**: **`FL_8V0906264H__0005.frf`** (используется по умолчанию в CLI и Studio) или `FL_8V0906264J__0003.frf` (софт `SC8O40`);
     - для **2.0 TSI EA888 Gen3**: `FL_8V0906264K__0003.frf` (`SC8S50`) или `FL_8V0906264F__0003.frf` (`SC8H83`).
   - **BootROM ЭБУ (`SBOOT`, `0x80000000..0x8001BFFF`) уже встроен в проект (`rom/sboot.bin`, 112 КБ):** внутри `.frf` и виртуальных/OBD-чтений никогда нет первичного загрузчика `SBOOT` и общей криптотаблицы `0x80014088`, вызываемой из `CBOOT`. При загрузке действует строгий приоритет:
     1. **Пользовательский `SBOOT` внутри самого дампа:** если загружаемый `.bin` (`4 МБ` или `3 МБ` BSL) уже содержит непустой `SBOOT`, **всегда используется `SBOOT` пользователя из самого дампа без изменений**;
     2. **Пользовательский донор:** для `.frf`/`.odx` и неполных OBD-чтений (где область `SBOOT` заполнена `0x00`/`0xFF`) сначала проверяется наличие пользовательского `sboot.bin` или полного стендового дампа в `vault/firmwares/ecu/bench/` (и `$SIMEMU_IMAGE_DIR`);
     3. **Дефолтный `rom/sboot.bin`:** если пользователь не положил свой донор, эмулятор автоматически подставляет в ОЗУ штатный `rom/sboot.bin` из репозитория (файл `rom/sboot.bin` при желании можно заменить без перекомпиляции).
   - Если нужно сохранить распакованный из `.frf` 4 МБ `.bin` на диск, используйте `simemu unpack-frf <файл.frf> [выход.bin]`.
   - Любые кастомные/тюнинговые `.bin` или `.frf` файлы кладите в `vault/firmwares/ecu/custom/`.

2. **Энергонезависимая память ЭБУ (`DFlash / EEPROM`, `rom/dflash.bin` 256 КБ уже в комплекте!)**:
   - **Отдельно искать дамп `DFlash` пользователю не нужно:** в репозитории и релизных архивах уже лежит полностью обезличенный образ `rom/dflash.bin` (`256 КБ`, банк `DF0` = первые `96 КБ`) с валидной разметкой FEE и лабораторным комплектом ключей WFS5 (`LABEMU…`). Достаточно положить **один только `.frf`-файл** в `vault/firmwares/ecu/frf/`, чтобы сразу запустить двигатель в Studio.
   - Если положить собственный стендовый вычит `DFlash` (`256 КБ` или `96 КБ` `DF0`) в `vault/firmwares/ecu/bench/` (либо в `$SIMEMU_IMAGE_DIR`), эмулятор автоматически отдаст приоритет вашему файлу. Подробное устройство `rom/sboot.bin`, `rom/dflash.bin`, ключей WFS5 и процесса запуска блоков описано в [`docs/FIRMWARE_AND_BOOT_GUIDE.ru.md`](docs/FIRMWARE_AND_BOOT_GUIDE.ru.md).

3. **Мехатроник DSG7 DQ200 Gen2 (`0CW`, `vault/firmwares/tcu/dq200/`)**:
   - Полный стендовый вычит мехатроника `J743` (`GSG DQ200G2`, MCU `SAK-TC1784`, например `0CW300043H_0002`, PCMFlash Модуль 58):
     - **Micro (PFlash TC1784)** — `2.625 МБ` (`0x2A0000` байт);
     - **EEPROM (SPI ST95640)** — `8 КБ` (`8192` байта).

4. **Мехатроник DSG6 DQ250 MQB (`0D9`, `vault/firmwares/tcu/dq250/`)**:
   - Достаточно **одного файла полного чтения Flash** мехатроника на `SAK-TC1766` размером `1.5 МБ` (`0x180000` байт, или `0x178000` без пустого хвоста — например `0D9300011R` SW 4319 или `0D9300012F` SW 4930, PCMFlash Модуль 58).
   - **Дамп EEPROM для DQ250 искать не нужно:** при первом запуске Studio сама создаёт чистую 8 КБ микросхему EEPROM, привязывает лабораторный иммобилайзер WFS5, а сама прошивка ТКУ по UDS выполняет кодирование и базовые установки вилок/сцеплений (`0x0380` / `0x04D4`) и сохраняет готовый EEPROM в `build/studio_state/`.

---

## Требования и сборка

- **Rust 1.88+** (Cargo workspace, edition 2021).
- **Никаких внешних C/C++ библиотек для сборки и запуска не требуется:** по умолчанию `simemu` собирается на **100% чистом Rust** с собственным ядром процессора **`tricore-core` (`CpuBackend::Native`)** — единый автономный исполняемый файл на Windows, Linux и macOS.

```sh
cargo build --release                          # полная сборка (features studio + bridge)
cargo build --release --no-default-features    # минимальный CLI без веб-стенда и моста
```

### Опционально (только для разработчиков): дифференциальная сверка против Unicorn 2.1.4

Референсный C-бэкенд `libunicorn` 2.1.x вынесен за опциональный Cargo-флаг **`--features unicorn`** и используется при разработке для дифференциального тестирования детерминизма ядра `tricore-core`:

```sh
python scripts/build_unicorn.py       # собирает патченый Unicorn 2.1.4 -> target/unicorn-simemu/build/
cargo test --release --workspace --all-features
```

---

## Запуск и выбор языка (`--lang en|ru`)

По умолчанию консоль и веб-интерфейс Studio запускаются на **английском языке (`en`)**. Для переключения на **русский язык** передайте флаг **`--lang ru`** (или коротко **`--ru`**, `-L ru`, либо переменную окружения `SIMEMU_LANG=ru`):

```sh
simemu --help                                   # справка на английском (по умолчанию)
simemu --lang ru --help                         # справка на русском (или: simemu --ru)

simemu studio                                   # веб-стенд http://127.0.0.1:8080 + SLCAN 127.0.0.1:28750 (EN)
simemu --ru studio [порт] [--slcan порт|--no-slcan] # веб-стенд и логи консоли на русском языке (RU)

simemu can-server [порт] [image.bin|frf] [eeprom]   # автономный стенд SIMOS18.1 с TCP SLCAN (:28750)
simemu boot [инструкций] [image.bin|image.frf]      # холодная загрузка SIMOS18.1 (.bin или напрямую .frf)
simemu unpack-frf <input.frf> [output.bin]          # нативная распаковка контейнера ODIS .frf/.odx в 4 МБ .bin
simemu damos [запрос] [--image <bin|frf>]           # поиск параметров в DAMOS JSON со значениями из .bin/.frf
simemu tcu-info                                     # паспорт образа DQ200_G2 и короткий прогон
simemu can-table                                    # дамп таблиц CAN-дескрипторов из прошивки SIMOS18.1
simemu render-audio [start|rev|misfire|knock|all|rtf] [каталог]
simemu log-report <лог.csv>                         # форензика лога SimosTools → build/log_report/
simemu cal-diff [A.bin|frf A.json B.bin|frf B.json] # сравнение калибровок по DAMOS → build/caldiff/
simemu replay <лог.csv> [--image …] [--profile …]   # воспроизведение лога через физику → build/replay/
```

Studio слушает только локальный интерфейс `127.0.0.1`:
- `http://127.0.0.1:8080` — веб-интерфейс моторного/шасси-стенда, редактор калибровок по DAMOS, осциллограф, проверка прошивок перед заливкой и UDS-диагностика (переключатель `EN / RU` также доступен в верхней панели);
- `socket://127.0.0.1:28750` — живой виртуальный CAN-адаптер **Lawicel / SLCAN поверх TCP** для внешних диагностических клиентов, логгеров и флешеров (`python-can`, `udsoncan`, `SavvyCAN`, `SimosTools`, `VW_Flash`).

---

## Тестирование

Все эмуляционные тесты запускаются с флагом `--release`:

```sh
# Модульные тесты всех крейтов (работают и без прошивок в vault/)
cargo test --release --lib

# Полный прогон workspace (подхватывает прошивки из vault/firmwares/ автоматически)
cargo test --release

# Сторож реального времени полной co-simulation (ECU + TCU + физика, цель RTF > 1.0×)
cargo test --release -p simemu-cli --lib cosim_realtime_guard -- --ignored --nocapture

# Регрессия загрузки и аппаратных контуров всех образов в vault/firmwares/
cargo test --release -p simemu-tc1791 --test boot_regression --test power_stages_e2e --test perf_determinism -- --nocapture
```

---

## Лицензия

SimEmu © 2026 Matrix распространяется под PolyForm Noncommercial 1.0.0 с дополнительными условиями. Коротко (юридически действует только текст [`LICENSE`](LICENSE)):

- код открыт — его можно читать, изучать и дорабатывать;
- бесплатно для личного и некоммерческого использования, а также для платного ремонта и диагностики: автосервису можно работать программой с клиентами;
- продавать нельзя — ни саму программу, ни копии, ни доступ к ней, ни в составе платных сборок;
- доработанные версии можно распространять только с открытым исходным кодом на тех же условиях и с сохранением строки об авторстве (`Required Notice`).
