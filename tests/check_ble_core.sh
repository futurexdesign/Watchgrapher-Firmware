#!/usr/bin/env bash
# Static checks for the radio core selection (plan section 5.8, task A9).
#
#   CORE-02  No function outside an allow-list calls the BT lock, BTstack, the
#            BLE library or cyw43_arch_gpio_put; BLE.startAdvertising() appears
#            exactly once, in ble_control_begin().
#   CORE-04  variants/rpipico2w_tg is a faithful copy of the framework variant
#            (skipped with a notice when the framework package is not installed).
#   CORE-01  (optional) ELF checks of both radio placements. Core 1 is the
#            default of every W env (pico2w_synthetic is the sample); core 0 is
#            the TG_BLE_CORE=0 override with the stock framework variant, which
#            no env in platformio.ini builds. Needs arm-none-eabi-nm / -objdump
#            (found via PATH or the PlatformIO toolchain). Also the evidence for
#            BLD-06.
#              --build [DIR]        builds env pico2w_synthetic and a temporary
#                                   core-0 env (extra config in DIR, stock
#                                   variant, -DTG_BLE_CORE=0) with PlatformIO,
#                                   then checks both ELFs. DIR defaults to a
#                                   mktemp directory. Takes a few minutes.
#              --elf CORE1.elf CORE0.elf   checks ELFs built earlier.
#
# Usage:   tests/check_ble_core.sh [--build [DIR] | --elf CORE1.elf CORE0.elf]
# Env:     PYTHON (default python3), PLATFORMIO_CORE_DIR (default ~/.platformio)
# Exit:    0 when every check passed or was skipped, 1 on any failure.
#
# CORE-02 approach: strip comments and strings from every src/**/*.cpp, find the
# outermost function around each forbidden token and fail unless that function
# is in the allow-list below. This is stricter than "reachable from loop()":
# loop-side code can never use these calls, because a function that uses one
# must be allow-listed, and the functions in LOOP_SIDE may not be allow-listed.
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
py="${PYTHON:-python3}"
fail=0

# ---------------------------------------------------------------------------
# CORE-02
# ---------------------------------------------------------------------------
"$py" - "$root" <<'PYEOF' || fail=1
import re, sys, pathlib

root = pathlib.Path(sys.argv[1])

# Tokens that must not appear in loop()-side code (CORE-02).
FORBIDDEN = re.compile(
    r"\b(__lockBluetooth|BluetoothLock|async_context_acquire_lock_blocking|"
    r"(?:att_server|gap|hci|l2cap|btstack)_\w+(?=\s*\()|"
    r"cyw43_arch_gpio_put|cyw43_arch_gpio_get|BLE\s*\.\s*\w+)\b")

# "file::function": allowed to use the forbidden tokens, with the reason.
ALLOW = {
    # Setup time, radio core: the only user of BluetoothLock, BLE.begin() /
    # startAdvertising(), hci_add_event_handler, att_server_register_packet_handler.
    "src/ble_control.cpp::ble_control_begin": "setup time on the radio core",
    # BT context: callbacks and workers that BTstack / the async context invoke.
    "src/ble_control.cpp::hci_handler": "BT context, HCI event callback",
    "src/ble_control.cpp::att_handler": "BT context, ATT event callback",
    "src/ble_control.cpp::l2cap_handler": "BT context, L2CAP event callback",
    "src/ble_control.cpp::on_timer": "BT context, housekeeping timer",
    "src/ble_control.cpp::tx_do_work": "BT context, TX worker",
    "src/ble_control.cpp::pacer_request": "BT context, pacer callback",
    "src/ble_control.cpp::pacer_notify": "BT context, pacer callback",
    "src/ble_control.cpp::arm_timer": "BT context, housekeeping timer helper",
    "src/ble_control.cpp::on_connected": "BT context, called by hci_handler",
    "src/ble_control.cpp::on_disconnected": "BT context, called by hci_handler",
    "src/ble_control.cpp::apply_power": "BT context, called by the TX worker (ble on/off)",
    "src/ble_core.cpp::led_work": "BT context, LED worker (plan 5.8 rule 4)",
}

# Functions that run in loop() / setup() on the main thread: never allow-listed
# and must exist (so a rename cannot silently drop the check).
LOOP_SIDE = {
    "src/ble_control.cpp::ble_control_task",
    "src/ble_control.cpp::ble_control_status",
    "src/ble_control.cpp::ble_control_set_enabled",
    "src/ble_control.cpp::ble_control_reset_counters",
    "src/ble_core.cpp::ble_core_led_set",
    "src/TimegrapherStand_main.cpp::led_set",
    "src/TimegrapherStand_main.cpp::loop",
}

def strip(text):
    """Blank out comments, string and char literals, keeping newlines."""
    out, i, n = [], 0, len(text)
    while i < n:
        c = text[i]
        two = text[i:i + 2]
        if two == "//":
            while i < n and text[i] != "\n":
                i += 1
        elif two == "/*":
            j = text.find("*/", i + 2)
            j = n if j < 0 else j + 2
            out.append("".join(ch if ch == "\n" else " " for ch in text[i:j]))
            i = j
        elif c in "\"'":
            q, j = c, i + 1
            while j < n and text[j] != q:
                j += 2 if text[j] == "\\" else 1
            out.append(q + q)
            i = j + 1
        else:
            out.append(c)
            i += 1
    return "".join(out)

SCOPE = re.compile(r"\b(namespace|class|struct|union|enum)\b|^\s*extern\s*\"\"\s*$")

def functions(text):
    """Yield (name, start_offset, end_offset) of every outermost function body."""
    stack, head_start, res = [], 0, []
    for i, c in enumerate(text):
        if c == "{":
            head = text[head_start:i]
            kind, name = "block", None
            if SCOPE.search(head) or head.strip() in ("", "="):
                kind = "scope" if SCOPE.search(head) else "block"
            elif not any(k == "func" for k, _, _ in stack) and "(" in head:
                # name: identifier before the first '(' (strip qualifiers)
                m = re.search(r"([~\w:]+)\s*\(", head)
                if m and m.group(1) not in ("if", "for", "while", "switch"):
                    kind, name = "func", m.group(1).split("::")[-1]
            stack.append((kind, name, i))
            head_start = i + 1
        elif c == "}":
            if stack:
                kind, name, start = stack.pop()
                if kind == "func":
                    res.append((name, start, i))
            head_start = i + 1
        elif c == ";":
            head_start = i + 1
    return res

errors, found, used_allow = [], set(), set()
start_adv = []
for path in sorted((root / "src").rglob("*.cpp")):
    rel = path.relative_to(root).as_posix()
    text = strip(path.read_text())
    funcs = functions(text)
    for name, s, e in funcs:
        found.add(f"{rel}::{name}")
    for m in FORBIDDEN.finditer(text):
        owner = next((f for f in funcs if f[1] <= m.start() <= f[2]), None)
        if owner is None:
            continue  # file-scope declaration, not code that runs
        key = f"{rel}::{owner[0]}"
        if "startAdvertising" in m.group(0):
            start_adv.append(key)
        if key in LOOP_SIDE or key not in ALLOW:
            line = text.count("\n", 0, m.start()) + 1
            errors.append(f"{rel}:{line}: '{m.group(0)}' in {owner[0]}() is not allow-listed"
                          + (" (loop-side function)" if key in LOOP_SIDE else ""))
        else:
            used_allow.add(key)

for key in sorted(LOOP_SIDE):
    if key not in found:
        errors.append(f"loop-side function {key} not found: update LOOP_SIDE")
    if key in ALLOW:
        errors.append(f"{key} is loop-side and must not be allow-listed")

if start_adv != ["src/ble_control.cpp::ble_control_begin"]:
    errors.append("BLE.startAdvertising must appear exactly once, in ble_control_begin(); "
                  f"found in: {start_adv}")

for key in sorted(set(ALLOW) - used_allow):
    print(f"note: allow-list entry {key} has no forbidden token (stale or unused)")

if errors:
    print("FAIL CORE-02")
    for e in errors:
        print("  " + e)
    sys.exit(1)
print("PASS CORE-02 (BT lock / BTstack / cyw43 gpio only in allow-listed functions)")
PYEOF

# ---------------------------------------------------------------------------
# CORE-04
# ---------------------------------------------------------------------------
pio_dir="${PLATFORMIO_CORE_DIR:-$HOME/.platformio}"
fw="$pio_dir/packages/framework-arduinopico"
mine="$root/variants/rpipico2w_tg"
if [[ ! -d "$fw/variants/rpipico2w" ]]; then
  echo "SKIP CORE-04 (framework-arduinopico not installed at $fw)"
else
  ok=1
  cmp -s "$mine/pins_arduino.h" "$fw/variants/rpipico2w/pins_arduino.h" ||
    { echo "FAIL CORE-04: pins_arduino.h differs from the framework copy"; ok=0; }
  cmp -s "$mine/digital.cpp" "$fw/variants/rpipico2w/digital.cpp" ||
    { echo "FAIL CORE-04: digital.cpp differs from the framework copy"; ok=0; }
  cmp -s "$root/variants/generic/common.h" "$fw/variants/generic/common.h" ||
    { echo "FAIL CORE-04: variants/generic/common.h differs from the framework copy"; ok=0; }
  if ! diff -q <(sed '/TG-VARIANT-BEGIN/,/TG-VARIANT-END/d' "$mine/init.cpp") \
       "$fw/variants/rpipico2w/init.cpp" >/dev/null; then
    echo "FAIL CORE-04: init.cpp differs outside the TG-VARIANT blocks"; ok=0
  fi
  grep -q "5\.5\.1" "$mine/init.cpp" ||
    { echo "FAIL CORE-04: init.cpp header comment must name arduino-pico 5.5.1"; ok=0; }
  if grep -q '"version": "1.50501.0"' "$fw/package.json"; then
    :
  else
    echo "note: framework version is not 1.50501.0; refresh the variant copy and its header comment"
    ok=0
  fi
  if [[ $ok == 1 ]]; then echo "PASS CORE-04 (variant matches framework 1.50501.0)"; else fail=1; fi
fi

# ---------------------------------------------------------------------------
# CORE-01 / BLD-06 (optional, needs built ELFs)
# ---------------------------------------------------------------------------
elf1=""; elf0=""
if [[ "${1:-}" == "--build" ]]; then
  work="${2:-$(mktemp -d)}"
  mkdir -p "$work"
  # The core-0 override: stock variant, -DTG_BLE_CORE=0. It spells out the [ble]
  # flags instead of inheriting them: PlatformIO moves -U options behind -D
  # ones, so -UTG_BLE_CORE cannot cancel the inherited -DTG_BLE_CORE=1.
  cat >"$work/core0.ini" <<INI
[platformio]
extra_configs = $root/platformio.ini

[env:core0_check]
extends = env:pico2w_synthetic
board_build.variants_dir = $pio_dir/packages/framework-arduinopico/variants
board_build.variant = rpipico2w
build_flags =
	\${board_pico2w.build_flags}
	-DPIO_FRAMEWORK_ARDUINO_ENABLE_BLUETOOTH
	-DTG_BLE=1
	-DTG_BLE_CORE=0
	-DAUDIO_SOURCE=SYNTHETIC
INI
  PLATFORMIO_BUILD_DIR="$work/pio" pio run -d "$root" -c "$work/core0.ini" \
    -e pico2w_synthetic -e core0_check >"$work/build.log" 2>&1 ||
    { echo "FAIL CORE-01: build failed, see $work/build.log"; exit 1; }
  elf1="$work/pio/pico2w_synthetic/firmware.elf"
  elf0="$work/pio/core0_check/firmware.elf"
elif [[ "${1:-}" == "--elf" ]]; then
  elf1="${2:?core-1 ELF}"; elf0="${3:?core-0 ELF}"
fi
if [[ -n "$elf1" ]]; then
  tc="$pio_dir/packages/toolchain-rp2040-earlephilhower/bin"
  nm="$tc/arm-none-eabi-nm"; od="$tc/arm-none-eabi-objdump"
  [[ -x "$nm" ]] || nm="$(command -v arm-none-eabi-nm)"
  [[ -x "$od" ]] || od="$(command -v arm-none-eabi-objdump)"
  ok=1
  # Capture first: grep -q closing the pipe early would make pipefail misreport.
  syms0="$("$nm" "$elf0")"; syms1="$("$nm" "$elf1")"
  dis0="$("$od" -d "$elf0" | awk '/<initVariant>:/,/^$/')"
  dis1="$("$od" -d "$elf1" | awk '/<initVariant>:/,/^$/')"
  if grep -q ble_core_setup1 <<<"$syms0"; then
    echo "FAIL CORE-01: ble_core_setup1 present in the core-0 ELF"; ok=0; fi
  grep -q init_cyw43_wifi <<<"$dis0" ||
    { echo "FAIL CORE-01: core-0 initVariant does not call init_cyw43_wifi"; ok=0; }
  if grep -q init_cyw43_wifi <<<"$dis1"; then
    echo "FAIL CORE-01/BLD-06: core-1 initVariant calls init_cyw43_wifi"; ok=0; fi
  size="$("$nm" -S "$elf1" | awk '$4=="initVariant"{print $2}')"
  [[ "$size" =~ ^0*2$ ]] ||
    { echo "FAIL BLD-06: core-1 initVariant is $size bytes, expected the empty body (2)"; ok=0; }
  grep -q ble_core_setup1 <<<"$syms1" ||
    { echo "FAIL CORE-01: ble_core_setup1 missing from the core-1 ELF"; ok=0; }
  if [[ $ok == 1 ]]; then echo "PASS CORE-01/BLD-06 (ELF checks)"; else fail=1; fi
fi

exit "$fail"
