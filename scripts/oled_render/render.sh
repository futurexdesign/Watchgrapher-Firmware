#!/usr/bin/env bash
# Renders every OLED screen state from the real src/display_ui.cpp.
#
# Builds scripts/oled_render/render_main.cpp together with display_ui.cpp and
# the pinned Adafruit GFX library (host shims in shim/), runs each scenario,
# and writes PNGs plus contact_sheet.png to docs/images/oled/ (or $1).
#
# Needs a C++17 compiler, Python 3 with Pillow, and the PlatformIO library
# checkout (run `pio run -e pico2_pcm1808` once so .pio/libdeps exists).
set -euo pipefail

root="$(cd "$(dirname "$0")/../.." && pwd)"
out="${1:-$root/docs/images/oled}"
gfx="$(ls -d "$root"/.pio/libdeps/*/"Adafruit GFX Library" 2>/dev/null | head -1)"
if [[ -z "$gfx" ]]; then
  echo "Adafruit GFX Library not found under .pio/libdeps; run pio run -e pico2_pcm1808 first" >&2
  exit 1
fi

work="$(mktemp -d)"
trap 'rm -rf "$work"' EXIT

# Rotation 0: the firmware default (3) turns the image 90 degrees CCW to cancel
# the way the panel is mounted in the case, so the renders, which show what the
# user sees in the case, are drawn unrotated.
"${CXX:-c++}" -std=c++17 -Wall -Wextra -O1 \
  -DARDUINO=100 -DBOARD_PICO2 -DDISPLAY_SH1107=1 -DDISPLAY_ROTATION=0 \
  -DDISPLAY_SDA_GPIO=4 -DDISPLAY_SCL_GPIO=5 -DADC_GPIO=26 -DADC_CHANNEL=0 \
  -I"$root/scripts/oled_render/shim" -I"$root/include" -I"$gfx" \
  "$root/scripts/oled_render/render_main.cpp" "$root/src/display_ui.cpp" \
  "$gfx/Adafruit_GFX.cpp" -o "$work/oled_render" 2> "$work/build.log" || {
  cat "$work/build.log" >&2
  exit 1
}

"$work/oled_render" --list > "$work/captions.tsv"
while IFS=$'\t' read -r name _; do
  "$work/oled_render" "$name" "$work/$name.pbm"
done < "$work/captions.tsv"

python3 "$root/scripts/oled_render/make_png.py" "$work" "$out" "$work/captions.tsv"
echo "wrote $(wc -l < "$work/captions.tsv" | tr -d ' ') screens to $out"
