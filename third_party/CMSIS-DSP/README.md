# Vendored CMSIS-DSP subset (RP2350 / Cortex-M33 only)

* DSP: https://github.com/ARM-software/CMSIS-DSP v1.16.2 (Apache-2.0, see LICENSE).
  * `Include/`: upstream headers, unmodified.
  * `Source/`: only `arm_biquad_cascade_df1_f32`, `arm_biquad_cascade_df1_init_f32`,
    `arm_rms_f32`, `arm_power_f32`, `arm_max_f32`, `arm_max_no_idx_f32`.
* Core (CMSIS 6.1, Apache-2.0): `cmsis_compiler.h`, `cmsis_gcc.h`, `m-profile/cmsis_gcc_m.h`
  copied into `Include/` (intrinsics needed by `arm_math_types.h`).

Compiled from source with the project's own toolchain flags (no prebuilt `.a`), so the
ABI always matches Arduino-Pico. Wired only into RP2350 envs via `[rp2350_dsp]` in
`platformio.ini` (`lib_extra_dirs = third_party`, `lib_deps = CMSIS-DSP`).
To add a function: copy its `.c` from `CMSIS-DSP/Source/<group>/` into `Source/`.
