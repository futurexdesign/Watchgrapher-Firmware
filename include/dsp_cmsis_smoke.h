// CMSIS-DSP link/behaviour smoke test (RP2350 / ARM_MATH_CM33 builds only).
#pragma once

// Returns true if CMSIS-DSP linked and produced the expected results.
// Always returns false on targets without CMSIS-DSP (e.g. RP2040 envs).
bool dsp_cmsis_smoke();
