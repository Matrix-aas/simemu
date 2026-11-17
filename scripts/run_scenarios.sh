#!/bin/sh
# Ignored co-simulation scenario tests (they need the firmware images, see README "Тесты") on one CPU backend,
# one PASS / FAIL / SKIP line each — the merge check for what `cargo test` does not run.
#
#   scripts/run_scenarios.sh [native|unicorn]        default: $SIMEMU_CPU, else unicorn
#
# Images are found as by the tests themselves ($SIMEMU_IMAGE_DIR, then the workspace). Full output of every
# scenario: build/scenarios/<backend>/<name>.log. Exit status = number of failed scenarios.
set -u
cpu=${1:-${SIMEMU_CPU:-unicorn}}
cd "$(dirname "$0")/.." || exit 99
logs=build/scenarios/$cpu
mkdir -p "$logs"
{ cargo test --release -q -p simemu-cli --no-run && cargo test --release -q -p simemu-tc1791 --test dq250_boot --no-run; } 2>"$logs/build.log" || { echo "BUILD FAILED (see $logs/build.log)"; exit 99; }
failed=0

# scenario <name> <test filter> [VAR=value ...]   (PKG=<package args> before the call: another test target)
scenario() {
    name=$1 filter=$2 pkg=${PKG:--p simemu-cli}
    shift 2
    log=$logs/$name.log
    start=$(date +%s)
    exact=
    case $filter in *::*) exact=--exact ;; esac   # full test path = exactly that test
    # shellcheck disable=SC2086 # $pkg is a word list
    env SIMEMU_CPU="$cpu" "$@" cargo test --release -q $pkg "$filter" -- --ignored --nocapture $exact >"$log" 2>&1
    rc=$?
    secs=$(($(date +%s) - start))
    if [ $rc -ne 0 ]; then
        failed=$((failed + 1))
        why=$(grep -m1 -A1 "panicked at" "$log" | tail -n 1 | cut -c1-120)
        echo "FAIL  $name (${secs} s) $why"
    elif ! grep -q "1 passed" "$log"; then
        failed=$((failed + 1))
        echo "FAIL  $name: no test matched '$filter'"
    elif grep -q "SKIP" "$log"; then
        echo "SKIP  $name ($(grep -m1 SKIP "$log" | cut -c1-100))"
    else
        echo "PASS  $name (${secs} s)"
    fi
}

echo "scenarios on SIMEMU_CPU=$cpu (logs: $logs)"
scenario engine_start_cjxc engine_start_scenario SIMEMU_ENGINE=cjxc
scenario engine_start_cjsa_264H engine_start_scenario SIMEMU_ENGINE=cjsa SIMEMU_IMAGE=8V0906264H__0005.bin
scenario dyno_wot_boost_knock engine_dyno_wot_boost_and_knock
scenario dyno_cam_phasing studio::dyno_run::tests::engine_dyno_cam_phasing
[ -n "${SIMEMU_SCENARIOS_LONG:-}" ] && scenario dyno_cam_phasing_hot studio::dyno_run::tests::engine_dyno_cam_phasing_hot  # 240 s plant
scenario dq200_bench dq200_bench_scenario
scenario vehicle_drive vehicle_drive_scenario
# DQ200 clutch slip check: road, octavia_fl + 264H, 1 -> 4 at 25 %, kickdown; steady-gear slip and DTCs
scenario dq200_slip dq200_slip_scenario
# DQ250 in the loop (FTOU in its speed computation: isa_fix on Unicorn)
scenario dq250_drive dq250_drive_scenario
scenario dq250_drive_4319 dq250_drive_scenario SIMEMU_DQ250_IMAGE=0D9300011R
# coding 000024 + basic settings 0x0380 / 0x04D4 over UDS, ignition cycles, short drive (MILESTONE §2.3)
scenario dq250_workshop dq250_workshop_scenario
# workshop, then a whole drive: launch, 1..6, kickdown, coast, brake to a stop, creep, R, P (no stall)
scenario dq250_full_drive dq250_full_drive_scenario
scenario dq250_full_drive_4319 dq250_full_drive_scenario SIMEMU_DQ250_IMAGE=0D9300011R
# the studio's own path (fresh state, build chosen like the UI, studio preparation, warm start, road): stock 264H +
# default DQ250; and stock SC8S50 (264K) + 0D9300012F
scenario dq250_studio_default dq250_studio_default_drive_scenario
scenario dq250_studio_s50_repro dq250_studio_default_drive_scenario SIMEMU_IMAGE=8V0906264K__0003.bin SIMEMU_DQ250_IMAGE=0D9300012F
# DQ250 AWD build: warm idle in P / D / R, load steps, stop after an overrun cut-off (no stall, no lean collapse)
scenario dq250_idle dq250_idle_scenario
# DQ250 AWD build: cold start after an overnight soak at 20 °C, 30 s in P without stalling
scenario cold_start_dq250 cold_start_dq250_scenario
# the same at 5 °C (studio's default ambient) and −10 °C: fuel wall films at cold start (docs/SIMOS18_COMBUSTION.md §15)
scenario cold_start_dq250_5 cold_start_dq250_scenario SIMEMU_COLD_AMB=5
scenario cold_start_dq250_m10 cold_start_dq250_scenario SIMEMU_COLD_AMB=-10
# −20 °C: it cranks ≈ 4.5 s before it runs on its own (open, §15.4), so ≥ 600 rpm is asserted from 8 s, not 3 s
scenario cold_start_dq250_m20 cold_start_dq250_scenario SIMEMU_COLD_AMB=-20 SIMEMU_COLD_FROM_S=8
# engine ECU coding written to its DFlash by the firmware, kept over KL15 and power cycles
scenario ecu_coding_nvm ecu_coding_survives_power_cycle
# studio prepares an erased DQ250 by itself (headless workshop run), saved state reloads, D engages (studio::dq250_prep)
scenario dq250_studio_prepare dq250_studio_prepare
scenario engine_restart engine_restart_scenario
# real time is a property of the native core (the default); Unicorn is the cross-check backend
if [ "$cpu" = native ]; then scenario cosim_realtime_guard cosim_realtime_guard; else echo "SKIP  cosim_realtime_guard (Unicorn is not the real-time backend)"; fi
# MQB rest-bus (simemu_bridge::mqb): no lost-communication / invalid-data DTC in ECU and TCU after a drive, outside air via CAN
scenario network_dtc_default network_dtc_scenario
scenario network_dtc_dq200 network_dtc_scenario SIMEMU_RESTBUS_BUILD=octavia_fl
scenario network_dtc_dq250 network_dtc_scenario SIMEMU_RESTBUS_BUILD=a3_8v_quattro
# full fault audit across reference builds (idle P, creep D/R, 30 %, kickdown to 110 km/h, coast, 40 %, stop), warm and 5 °C cold
scenario fault_audit_octavia_fl fault_audit_scenario SIMEMU_AUDIT_BUILD=octavia_fl
scenario fault_audit_octavia_fl_cold fault_audit_scenario SIMEMU_AUDIT_BUILD=octavia_fl SIMEMU_AUDIT_COLD=5
scenario fault_audit_a3_quattro fault_audit_scenario SIMEMU_AUDIT_BUILD=a3_8v_quattro
scenario fault_audit_a3_quattro_cold fault_audit_scenario SIMEMU_AUDIT_BUILD=a3_8v_quattro SIMEMU_AUDIT_COLD=5
scenario fault_audit_cjsa_is38 fault_audit_scenario SIMEMU_AUDIT_BUILD=cjsa_is38_dq250
scenario fault_audit_cjsa_is38_cold fault_audit_scenario SIMEMU_AUDIT_BUILD=cjsa_is38_dq250 SIMEMU_AUDIT_COLD=5
# studio tester clears ECU, DQ200 and DQ250 (OBD 04 on 0x7DF, MILESTONE.md §2.11); history from the NVM dumps is gone
scenario clear_dtc clear_dtc_scenario
scenario immo_lab_board board_level_lab_binding
scenario immo_lab_e2e lab_binding_e2e
scenario immo_lab_dq250 dq250_lab_binding
PKG="-p simemu-tc1791 --test dq250_boot" scenario dq250_bench dq250_bench_scenario
exit $failed
