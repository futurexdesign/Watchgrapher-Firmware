/**
 * @file dsp_blocks.cpp
 * @brief DSP block / parameter name table.
 *
 * @see dsp_blocks.h
 */

#include "dsp_blocks.h"

namespace {

constexpr dsp_param_info_t kGainParams[] = {{"db", "db", true}};
constexpr dsp_param_info_t kPresetParams[] = {{"preset", "preset", true}};
constexpr dsp_param_info_t kEventParams[] = {
    {"threshold", "threshold", true},
    {"preroll_ms", "prerollMs", true},
    {"tail_ms", "tailMs", true},
};
constexpr dsp_param_info_t kBeatParams[] = {
    {"lift_angle", "liftAngle", true},
    {"vph", "vph", true},
    /*
     * Read-only over patchState: it reports the correction in effect (PPS
     * while locked), and a write would relabel the stored calibration as
     * manual. Hosts use setTimebase; the CLI 'dsp set' still writes it.
     */
    {"cal_ppm", "calPpm", false},
    {"window_s", "windowS", true},
    {"avg_s", "avgS", true},
};
constexpr dsp_param_info_t kGateParams[] = {
    {"floor_db", "floorDb", true},
    {"attack_ms", "attackMs", true},
    {"release_ms", "releaseMs", true},
};
constexpr dsp_param_info_t kNotchParams[] = {{"hz", "hz", true}};
constexpr dsp_param_info_t kAgcParams[] = {
    {"target", "target", true},
    {"tau_s", "tauS", true},
    {"gain_db", "gainDb", false},
};
constexpr dsp_param_info_t kMatchParams[] = {
    {"threshold", "threshold", true},
    {"learn", "learn", true},
    {"score", "score", false},
};
constexpr dsp_param_info_t kSpectrumParams[] = {{"stride", "stride", true}};

template <size_t N> constexpr uint8_t count_of(const dsp_param_info_t (&)[N]) {
  return static_cast<uint8_t>(N);
}

/** Indexed by dsp_block_id_t; order must match the enum (checked below). */
constexpr dsp_block_info_t kBlocks[] = {
    {DSP_BLOCK_BYPASS, "bypass", "bypass", false, nullptr, 0},
    {DSP_BLOCK_LEVEL_METER, "level_meter", "levelMeter", false, nullptr, 0},
    {DSP_BLOCK_GAIN, "gain", "gain", true, kGainParams, count_of(kGainParams)},
    {DSP_BLOCK_LIMITER, "limiter", "limiter", true, nullptr, 0},
    {DSP_BLOCK_FILTER_BANK, "filter_bank", "filterBank", true, kPresetParams,
     count_of(kPresetParams)},
    {DSP_BLOCK_EVENT_DETECT, "event_detect", "eventDetect", false, kEventParams,
     count_of(kEventParams)},
    {DSP_BLOCK_BEAT_RATE, "beat_rate", "beatRate", false, kBeatParams,
     count_of(kBeatParams)},
    {DSP_BLOCK_NOISE_FLOOR, "noise_floor", "noiseFloor", false, nullptr, 0},
    {DSP_BLOCK_EVENT_LOG, "event_log", "eventLog", false, nullptr, 0},
    {DSP_BLOCK_SOFT_GATE, "soft_gate", "softGate", true, kGateParams,
     count_of(kGateParams)},
    {DSP_BLOCK_NOTCH, "notch", "notch", true, kNotchParams,
     count_of(kNotchParams)},
    {DSP_BLOCK_AGC, "agc", "agc", true, kAgcParams, count_of(kAgcParams)},
    {DSP_BLOCK_MATCHED_FILTER, "matched_filter", "matchedFilter", false,
     kMatchParams, count_of(kMatchParams)},
    {DSP_BLOCK_FIR_BANK, "fir_bank", "firBank", true, kPresetParams,
     count_of(kPresetParams)},
    {DSP_BLOCK_SPECTRUM, "spectrum", "spectrum", false, kSpectrumParams,
     count_of(kSpectrumParams)},
};

static_assert(sizeof(kBlocks) / sizeof(kBlocks[0]) == DSP_BLOCK_COUNT,
              "kBlocks must have one entry per dsp_block_id_t");

constexpr bool ids_in_order(size_t i = 0) {
  return i == DSP_BLOCK_COUNT ||
         (kBlocks[i].id == static_cast<dsp_block_id_t>(i) &&
          ids_in_order(i + 1));
}
static_assert(ids_in_order(), "kBlocks must be ordered by dsp_block_id_t");

} // namespace

const dsp_block_info_t *dsp_block_info(dsp_block_id_t id) {
  if (static_cast<uint32_t>(id) >= DSP_BLOCK_COUNT) {
    return nullptr;
  }
  return &kBlocks[id];
}

const char *dsp_block_name(dsp_block_id_t id) {
  const auto *b = dsp_block_info(id);
  return b ? b->name : "unknown";
}

const char *dsp_block_json_name(dsp_block_id_t id) {
  const auto *b = dsp_block_info(id);
  return b ? b->json_name : "unknown";
}

bool dsp_block_is_process(dsp_block_id_t id) {
  const auto *b = dsp_block_info(id);
  return b != nullptr && b->is_process;
}
