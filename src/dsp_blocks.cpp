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
    {DSP_BLOCK_BYPASS, "bypass", "bypass", false, nullptr, 0, DSP_GROUP_PROCESS,
     0},
    {DSP_BLOCK_LEVEL_METER, "level_meter", "levelMeter", false, nullptr, 0,
     DSP_GROUP_ANALYSIS, 0},
    {DSP_BLOCK_GAIN, "gain", "gain", true, kGainParams, count_of(kGainParams),
     DSP_GROUP_PROCESS, 5},
    {DSP_BLOCK_LIMITER, "limiter", "limiter", true, nullptr, 0,
     DSP_GROUP_PROCESS, 7},
    {DSP_BLOCK_FILTER_BANK, "filter_bank", "filterBank", true, kPresetParams,
     count_of(kPresetParams), DSP_GROUP_PROCESS, 2},
    {DSP_BLOCK_EVENT_DETECT, "event_detect", "eventDetect", false, kEventParams,
     count_of(kEventParams), DSP_GROUP_ANALYSIS, 4},
    {DSP_BLOCK_BEAT_RATE, "beat_rate", "beatRate", false, kBeatParams,
     count_of(kBeatParams), DSP_GROUP_ANALYSIS, 6},
    {DSP_BLOCK_NOISE_FLOOR, "noise_floor", "noiseFloor", false, nullptr, 0,
     DSP_GROUP_ANALYSIS, 3},
    {DSP_BLOCK_EVENT_LOG, "event_log", "eventLog", false, nullptr, 0,
     DSP_GROUP_ANALYSIS, 5},
    {DSP_BLOCK_SOFT_GATE, "soft_gate", "softGate", true, kGateParams,
     count_of(kGateParams), DSP_GROUP_PROCESS, 4},
    {DSP_BLOCK_NOTCH, "notch", "notch", true, kNotchParams,
     count_of(kNotchParams), DSP_GROUP_PROCESS, 1},
    {DSP_BLOCK_AGC, "agc", "agc", true, kAgcParams, count_of(kAgcParams),
     DSP_GROUP_PROCESS, 6},
    {DSP_BLOCK_MATCHED_FILTER, "matched_filter", "matchedFilter", false,
     kMatchParams, count_of(kMatchParams), DSP_GROUP_ANALYSIS, 1},
    {DSP_BLOCK_FIR_BANK, "fir_bank", "firBank", true, kPresetParams,
     count_of(kPresetParams), DSP_GROUP_PROCESS, 3},
    {DSP_BLOCK_SPECTRUM, "spectrum", "spectrum", false, kSpectrumParams,
     count_of(kSpectrumParams), DSP_GROUP_ANALYSIS, 2},
};

static_assert(sizeof(kBlocks) / sizeof(kBlocks[0]) == DSP_BLOCK_COUNT,
              "kBlocks must have one entry per dsp_block_id_t");

constexpr bool ids_in_order(size_t i = 0) {
  return i == DSP_BLOCK_COUNT ||
         (kBlocks[i].id == static_cast<dsp_block_id_t>(i) &&
          ids_in_order(i + 1));
}
static_assert(ids_in_order(), "kBlocks must be ordered by dsp_block_id_t");

/*
 * Pipeline graph. Derived from dsp_graph_process (dsp_graph.cpp):
 *
 *   analyze(buf)       -- taps the RAW input block, never modifies it:
 *     levelMeter(pre) -> matchedFilter -> spectrum -> noiseFloor
 *       -> eventDetect (tick_detect_block; reads noiseFloor and, when a
 *          template is learned, gates on the matchedFilter score)
 *       eventDetect close callback -> eventLog (ring) and beatRate (beat
 *          analysis), in that order. They are siblings, both fed by
 *          eventDetect; beatRate does NOT read eventLog.
 *   process_path(buf)  -- skipped under master bypass; in place, in order:
 *     notch -> filterBank -> firBank -> softGate -> gain -> agc -> limiter
 *   quantize           -- limiter output -> pcm24; levelMeter's POST
 *                         peak/RMS is measured here on the same samples.
 *   pcm1808_capture    -- pcm24 is upsampled/pushed to audio_ring -> usb.
 *
 * levelMeter is one block that measures both ends: pre on "input" and post
 * on the end of the chain ("limiter"), so its inputs are {input, limiter}.
 * That is not a cycle: the post tap only reads, and levelMeter feeds nothing.
 * The matchedFilter also learns from the closed event's wave (feedback from
 * eventDetect to matchedFilter, one event late); that is state feedback,
 * not sample flow, and is omitted to keep the graph acyclic.
 *
 * quartz: the engine reads the raw LEFT/RIGHT codes of the same DMA block as
 * "input" (pcm1808_capture.cpp), after the block is queued for USB, and
 * feeds nothing in the audio path.
 */
constexpr const char *kInInput[] = {"input"};
constexpr const char *kInLevel[] = {"input", "limiter"};
constexpr const char *kInEvent[] = {"input", "noiseFloor", "matchedFilter"};
constexpr const char *kInFromEvent[] = {"eventDetect"};
constexpr const char *kInNotch[] = {"input"};
constexpr const char *kInFilter[] = {"notch"};
constexpr const char *kInFir[] = {"filterBank"};
constexpr const char *kInGate[] = {"firBank"};
constexpr const char *kInGain[] = {"softGate"};
constexpr const char *kInAgc[] = {"gain"};
constexpr const char *kInLimiter[] = {"agc"};
constexpr const char *kInPcm[] = {"limiter"};
constexpr const char *kInUsb[] = {"pcm24"};

template <size_t N>
constexpr dsp_pipeline_node_t node(const char *key, dsp_node_kind_t kind,
                                   uint8_t order, const char *const (&in)[N]) {
  return {key, static_cast<uint8_t>(kind), order, in, static_cast<uint8_t>(N)};
}

constexpr dsp_pipeline_node_t kNodes[] = {
    {"input", DSP_NODE_SOURCE, 0, nullptr, 0}, /* raw capture, no inputs */
    node("levelMeter", DSP_NODE_ANALYSIS, 0, kInLevel),
    node("matchedFilter", DSP_NODE_ANALYSIS, 1, kInInput),
    node("spectrum", DSP_NODE_ANALYSIS, 2, kInInput),
    node("noiseFloor", DSP_NODE_ANALYSIS, 3, kInInput),
    node("eventDetect", DSP_NODE_ANALYSIS, 4, kInEvent),
    node("eventLog", DSP_NODE_ANALYSIS, 5, kInFromEvent),
    node("beatRate", DSP_NODE_ANALYSIS, 6, kInFromEvent),
    node("notch", DSP_NODE_PROCESS, 1, kInNotch),
    node("filterBank", DSP_NODE_PROCESS, 2, kInFilter),
    node("firBank", DSP_NODE_PROCESS, 3, kInFir),
    node("softGate", DSP_NODE_PROCESS, 4, kInGate),
    node("gain", DSP_NODE_PROCESS, 5, kInGain),
    node("agc", DSP_NODE_PROCESS, 6, kInAgc),
    node("limiter", DSP_NODE_PROCESS, 7, kInLimiter),
    node("pcm24", DSP_NODE_SINK, 8, kInPcm),
    node("usb", DSP_NODE_SINK, 9, kInUsb),
    node("quartz", DSP_NODE_QUARTZ, 0, kInInput),
};

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

const dsp_pipeline_node_t *dsp_pipeline_nodes(uint32_t *count) {
  if (count) {
    *count = static_cast<uint32_t>(sizeof(kNodes) / sizeof(kNodes[0]));
  }
  return kNodes;
}
