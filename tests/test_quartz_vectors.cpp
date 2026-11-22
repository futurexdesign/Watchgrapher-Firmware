/**
 * @file test_quartz_vectors.cpp
 * @brief Acceptance harness: streams every handoff test vector through the
 *        quartz library in 240-frame blocks and checks every row of the
 *        handoff §9 table.
 *
 * For each entry of {@code vectors/manifest.json}:
 *  - {@code ind_*} pulse vectors (LEFT): run {@link quartz::MotorTiming}
 *    twice, once per timestamp method; score exactly as
 *    {@code ref/run_tests.py} does: pulse count, polarity pattern, and
 *    jitter = RMS of (timestamp - truth onset) after removing the mean
 *    (the constant delay); rates through the segmented fit;
 *  - {@code ind_tuning_*} (LEFT): {@link quartz::ToneTiming}, 0.25 s blocks;
 *  - {@code lcd_*} (RIGHT): {@link quartz::LcdTiming};
 *  - {@code xtal_*} (RIGHT, 96 kHz): {@link quartz::ToneTiming} at
 *    32 768 Hz, 0.05 s blocks.
 *
 * Prints the port's values next to {@code results.json} (the Python
 * reference) and the ground truth, then the §9 pass/fail table. Exits
 * non-zero if any row fails.
 *
 * Build and run (from the repository root): {@code
 * tests/build_quartz_tests.sh [BUILD_DIR]} compiles the vendored CMSIS-DSP
 * kernels for the host, builds this test and its sibling with
 * {@code -std=c++17 -Wall -Wextra}, and runs both. The binary is
 * {@code BUILD_DIR/test_quartz_vectors} (it takes the vector directory as its
 * argument, default docs/handoff/vectors).
 */

#include "quartz/lcd_timing.h"
#include "quartz/motor_timing.h"
#include "quartz/tone_timing.h"
#include "wav24.h"

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

namespace {

// ------------------------------------------------------------- tiny JSON

/**
 * A view of one JSON value inside a document: just enough to pull fields
 * out of the manifest, the truth files and results.json. No validation
 * beyond what navigation needs.
 */
class Json {
public:
  /** @param text Whole document (must outlive the view). */
  explicit Json(const std::string &text) : s_(&text), b_(skip_ws(0)) {
    e_ = value_end(b_);
  }

  /**
   * @param key Member name.
   * @return The member's value if this is an object that has it.
   */
  [[nodiscard]] std::optional<Json> get(const std::string &key) const {
    if (b_ >= e_ || (*s_)[b_] != '{') {
      return std::nullopt;
    }
    std::size_t i = skip_ws(b_ + 1u);
    while (i < e_ && (*s_)[i] != '}') {
      const std::size_t kend = value_end(i);
      const std::string k = s_->substr(i + 1u, kend - i - 2u);
      std::size_t v = skip_ws(kend);
      v = skip_ws(v + 1u); // ':'
      const std::size_t vend = value_end(v);
      if (k == key) {
        return Json(s_, v, vend);
      }
      i = skip_ws(vend);
      if (i < e_ && (*s_)[i] == ',') {
        i = skip_ws(i + 1u);
      }
    }
    return std::nullopt;
  }

  /** @return The elements if this is an array, else empty. */
  [[nodiscard]] std::vector<Json> items() const {
    std::vector<Json> out;
    if (b_ >= e_ || (*s_)[b_] != '[') {
      return out;
    }
    std::size_t i = skip_ws(b_ + 1u);
    while (i < e_ && (*s_)[i] != ']') {
      const std::size_t vend = value_end(i);
      out.emplace_back(Json(s_, i, vend));
      i = skip_ws(vend);
      if (i < e_ && (*s_)[i] == ',') {
        i = skip_ws(i + 1u);
      }
    }
    return out;
  }

  /** @return The number, or nullopt for anything else (incl. null). */
  [[nodiscard]] std::optional<double> num() const {
    const std::string t = raw();
    if (t.empty() || t == "null" || t[0] == '"' || t[0] == '{' || t[0] == '[' ||
        t == "true" || t == "false") {
      return std::nullopt;
    }
    return std::stod(t);
  }

  /** @return The boolean, or nullopt. */
  [[nodiscard]] std::optional<bool> boolean() const {
    const std::string t = raw();
    if (t == "true") {
      return true;
    }
    if (t == "false") {
      return false;
    }
    return std::nullopt;
  }

  /** @return The string contents (no escape handling), or nullopt. */
  [[nodiscard]] std::optional<std::string> str() const {
    if (b_ >= e_ || (*s_)[b_] != '"') {
      return std::nullopt;
    }
    return s_->substr(b_ + 1u, e_ - b_ - 2u);
  }

  /** @return {@code true} if the value is JSON null. */
  [[nodiscard]] bool is_null() const { return raw() == "null"; }

  /** @return The raw text of the value. */
  [[nodiscard]] std::string raw() const { return s_->substr(b_, e_ - b_); }

private:
  /** View of {@code [b, e)} inside {@code *s}. */
  Json(const std::string *s, std::size_t b, std::size_t e)
      : s_(s), b_(b), e_(e) {}

  /** @return First non-space index at or after {@code i}. */
  [[nodiscard]] std::size_t skip_ws(std::size_t i) const {
    while (i < s_->size() &&
           (std::isspace(static_cast<unsigned char>((*s_)[i])) != 0)) {
      i++;
    }
    return i;
  }

  /** @return One past the end of the value starting at {@code i}. */
  [[nodiscard]] std::size_t value_end(std::size_t i) const {
    const std::string &s = *s_;
    if (i >= s.size()) {
      return i;
    }
    if (s[i] == '"') {
      for (std::size_t j = i + 1u; j < s.size(); j++) {
        if (s[j] == '\\') {
          j++;
        } else if (s[j] == '"') {
          return j + 1u;
        }
      }
      return s.size();
    }
    if (s[i] == '{' || s[i] == '[') {
      int depth = 0;
      for (std::size_t j = i; j < s.size(); j++) {
        const char c = s[j];
        if (c == '"') {
          j = value_end(j) - 1u;
        } else if (c == '{' || c == '[') {
          depth++;
        } else if (c == '}' || c == ']') {
          if (--depth == 0) {
            return j + 1u;
          }
        }
      }
      return s.size();
    }
    std::size_t j = i;
    while (j < s.size() && s[j] != ',' && s[j] != '}' && s[j] != ']' &&
           std::isspace(static_cast<unsigned char>(s[j])) == 0) {
      j++;
    }
    return j;
  }

  const std::string *s_; /**< Document (not owned). */
  std::size_t b_ = 0;    /**< First character of the value. */
  std::size_t e_ = 0;    /**< One past its last character. */
};

/** @return File contents, or empty on failure. */
std::string slurp(const std::string &path) {
  std::ifstream f(path, std::ios::binary);
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

/** Reads a numeric field along a path of member names. */
std::optional<double> num_at(const Json &root,
                             std::initializer_list<const char *> path) {
  std::optional<Json> j = root;
  for (const char *k : path) {
    if (!j) {
      return std::nullopt;
    }
    j = j->get(k);
  }
  return j ? j->num() : std::nullopt;
}

// --------------------------------------------------------------- results

/** Frames per block, as the firmware's DMA delivers them. */
constexpr std::size_t kBlockFrames = 240u;

/** One printed acceptance row. */
struct Row {
  std::string vector;    /**< Vector name. */
  std::string criterion; /**< §9 "pass if". */
  std::string port;      /**< The port's values. */
  std::string ref;       /**< results.json values. */
  std::string truth;     /**< Ground truth. */
  bool pass = true;      /**< Row passes. */
};

/** @return printf-formatted string. */
template <typename... Args> std::string fmt(const char *f, Args... args) {
  std::array<char, 256> b{};
  std::snprintf(b.data(), b.size(), f, args...);
  return std::string(b.data());
}

/** @return "%.4f" or "-" when absent. */
std::string fnum(std::optional<double> v, const char *f = "%+.4f") {
  return v ? fmt(f, *v) : std::string("-");
}

/** Which channel a vector is read from. */
enum class Channel { kLeft, kRight };

/** Streams one channel of a vector in 240-frame blocks. */
class VectorStream {
public:
  /**
   * @param path WAV file.
   * @return {@code true} if opened.
   */
  bool open(const std::string &path) {
    std::string err;
    if (!reader_->open(path, err)) {
      std::fprintf(stderr, "%s: %s\n", path.c_str(), err.c_str());
      return false;
    }
    return true;
  }

  /** @return Sample rate (Hz). */
  [[nodiscard]] uint32_t rate() const { return reader_->rate(); }

  /**
   * Calls {@code fn(codes, n)} for each block of one channel.
   *
   * @param ch Channel.
   * @param fn Sink.
   */
  template <typename Fn> void for_each_block(Channel ch, Fn &&fn) {
    std::array<int32_t, kBlockFrames> l{};
    std::array<int32_t, kBlockFrames> r{};
    for (;;) {
      const std::size_t n = reader_->read(l.data(), r.data(), kBlockFrames);
      if (n == 0u) {
        break;
      }
      fn(ch == Channel::kLeft ? l.data() : r.data(), n);
    }
  }

private:
  /** The reader holds a 24 KB buffer; keep it off the stack. */
  std::unique_ptr<wav24::Reader> reader_ = std::make_unique<wav24::Reader>();
};

/** Ground-truth pulse events. */
struct Events {
  std::vector<double> t_s; /**< Onset times (s). */
  std::vector<int> pol;    /**< Polarities. */
};

/** Port output for one motor run. */
struct MotorRun {
  uint32_t pulses = 0;      /**< Pulses detected. */
  bool polarity_ok = false; /**< Polarity pattern matches the truth. */
  std::optional<double> jitter_rms_us;   /**< RMS edge error, mean removed. */
  std::optional<double> jitter_max_us;   /**< Max edge error, mean removed. */
  std::optional<quartz::MotorRate> rate; /**< Fit result. */
  bool clipped = false;                  /**< Whole-stream clip flag. */
  uint32_t clipped_pulses = 0;           /**< Pulses whose group clipped. */
  uint32_t extras = 0;                   /**< Correction/detection bursts. */
  float sigma = 0.f;                     /**< Final noise sigma (codes). */
  std::vector<float> jumps_us;           /**< Per-event jump sizes (µs). */
  /** The capture-path (incremental) result, solved as the engine does. */
  std::optional<quartz::MotorRate> inc_rate;
};

/** @return results.json inhibition_jumps_us for a method, as text. */
std::string ref_jumps(const std::optional<Json> &ref, const std::string &key) {
  std::string out;
  if (!ref) {
    return "-";
  }
  const auto rate = ref->get("rate_" + key);
  const auto jumps = rate ? rate->get("inhibition_jumps_us") : std::nullopt;
  if (!jumps) {
    return "-";
  }
  for (const Json &j : jumps->items()) {
    out += fmt("%s%.2f", out.empty() ? "" : " ", j.num().value_or(0.0));
  }
  return out.empty() ? "none" : out;
}

/**
 * Runs the motor pipeline on a vector with one timestamp method and scores
 * it as run_tests.py does.
 */
std::optional<MotorRun> run_motor(const std::string &wav,
                                  quartz::TimestampMethod method,
                                  const Events &truth) {
  static quartz::MotorTiming mt; // ~40 KB; static, as in the firmware
  VectorStream vs;
  if (!vs.open(wav)) {
    return std::nullopt;
  }
  quartz::MotorTiming::Config cfg{};
  cfg.method = method;
  if (!mt.configure(vs.rate(), cfg)) {
    std::fprintf(stderr, "%s: unsupported rate %u\n", wav.c_str(), vs.rate());
    return std::nullopt;
  }
  // Solve incrementally the way the engine does: after every block that
  // brought a new pulse, and once per second of signal otherwise.
  MotorRun out{};
  uint32_t solved_pulses = 0;
  std::size_t since = 0;
  vs.for_each_block(Channel::kLeft, [&](const int32_t *c, std::size_t n) {
    mt.process(c, n);
    since += n;
    if (mt.pulse_count() != solved_pulses || since >= vs.rate()) {
      out.inc_rate = mt.solve_incremental();
      solved_pulses = mt.pulse_count();
      since = 0;
    }
  });
  out.inc_rate = mt.solve_incremental();
  out.pulses = mt.pulse_count();
  out.rate = mt.solve();
  out.clipped = mt.clipped();
  out.sigma = mt.detector().noise_sigma();
  if (out.rate) {
    out.clipped_pulses = out.rate->clipped_pulses;
    out.extras = out.rate->extra_pulses;
    for (std::size_t i = 0; i < mt.fit().jump_count(); i++) {
      out.jumps_us.push_back(mt.fit().jump_us(i));
    }
  }
  const std::size_t n = truth.t_s.size();
  if (out.pulses == n && n > 0u && mt.fit().size() == n) {
    const auto fs = static_cast<double>(vs.rate());
    std::vector<double> d(n);
    double mean = 0.0;
    out.polarity_ok = true;
    for (std::size_t i = 0; i < n; i++) {
      d[i] = mt.fit().time_samples(i) / fs - truth.t_s[i];
      mean += d[i];
      out.polarity_ok =
          out.polarity_ok && mt.fit().polarity(i) * mt.fit().polarity(0) ==
                                 truth.pol[i] * truth.pol[0];
    }
    mean /= static_cast<double>(n);
    double ss = 0.0;
    double mx = 0.0;
    for (double v : d) {
      const double e = v - mean; // constant delay removed
      ss += e * e;
      mx = std::max(mx, std::fabs(e));
    }
    out.jitter_rms_us = std::sqrt(ss / static_cast<double>(n)) * 1e6;
    out.jitter_max_us = mx * 1e6;
  }
  return out;
}

/** Maps a nominal frequency from JSON to an exact fraction. */
quartz::Rational exact_frequency(double hz) {
  for (const auto &f : quartz::kForkFrequencies) {
    if (std::fabs(f.hz() - hz) < 0.01) {
      return f;
    }
  }
  return quartz::Rational{static_cast<uint32_t>(std::lround(hz)), 1u};
}

/** Runs the tone pipeline on one channel of a vector. */
std::optional<quartz::PhaseRate> run_tone(const std::string &wav, Channel ch,
                                          quartz::Rational f, double block_s) {
  static quartz::ToneTiming tt;
  VectorStream vs;
  if (!vs.open(wav) || !tt.configure(vs.rate(), {f, block_s})) {
    return std::nullopt;
  }
  vs.for_each_block(ch,
                    [&](const int32_t *c, std::size_t n) { tt.process(c, n); });
  return tt.result();
}

/** Runs the LCD pipeline on the RIGHT channel of a vector. */
std::optional<quartz::LcdResult> run_lcd(const std::string &wav) {
  static quartz::LcdTiming lt;
  VectorStream vs;
  if (!vs.open(wav) || !lt.configure(vs.rate())) {
    return std::nullopt;
  }
  vs.for_each_block(Channel::kRight,
                    [&](const int32_t *c, std::size_t n) { lt.process(c, n); });
  return lt.result();
}

/** @return {@code true} if {@code s} starts with {@code p}. */
bool starts_with(const std::string &s, const char *p) {
  return s.rfind(p, 0) == 0;
}

/** Per-vector motor acceptance criteria (handoff §9). */
struct MotorCriteria {
  std::optional<double> jitter_max_us; /**< Jitter limit. */
  bool need_clip = false;              /**< Clip flag must be set. */
  std::optional<double> raw_tol;       /**< Raw-rate tolerance (s/d). */
  std::optional<double> corr_tol;      /**< Corrected-rate tolerance. */
  bool period_exact = false;           /**< Inhibition period exact. */
  bool step_exact = false;             /**< Step exact. */
};

/** @return The §9 criteria for an ind_* pulse vector. */
MotorCriteria motor_criteria(const std::string &name) {
  MotorCriteria c{};
  if (name == "ind_pulses_20uV") {
    c.jitter_max_us = 4.0;
  } else if (name == "ind_pulses_100uV") {
    c.jitter_max_us = 0.5;
  } else if (name == "ind_pulses_1mV") {
    c.jitter_max_us = 0.1;
  } else if (name == "ind_pulses_2p5mV_clipping") {
    c.need_clip = true;
  } else if (name == "ind_duty_sweep") {
    c.jitter_max_us = 0.6;
  } else if (name == "ind_rate_no_inhibition") {
    c.raw_tol = 0.01;
  } else if (name == "ind_inhibition_10s" || name == "ind_inhibition_60s") {
    c.raw_tol = 0.01;
    c.corr_tol = 0.02;
    c.period_exact = true;
  } else if (name == "ind_step_20s") {
    c.raw_tol = 0.01;
    c.step_exact = true;
  }
  return c;
}

/** @return The §9 "pass if" text for a criteria set. */
std::string criteria_text(const MotorCriteria &c) {
  std::string s = "count exact, polarity";
  if (c.jitter_max_us) {
    s += fmt("; jitter <= %.1f us", *c.jitter_max_us);
  }
  if (c.need_clip) {
    s += "; clip flag";
  }
  if (c.raw_tol) {
    s += fmt("; raw +-%.2f", *c.raw_tol);
  }
  if (c.corr_tol) {
    s += fmt("; corr +-%.2f", *c.corr_tol);
  }
  if (c.period_exact) {
    s += "; period exact";
  }
  if (c.step_exact) {
    s += "; step exact";
  }
  return s;
}

/** Evaluates one motor vector, prints details, returns its row. */
Row motor_vector(const std::string &name, const std::string &wav,
                 const Json &truth, const std::optional<Json> &ref) {
  Events ev{};
  if (auto events = truth.get("events")) {
    for (const Json &e : events->items()) {
      ev.t_s.push_back(e.get("t_onset_s")->num().value_or(0.0));
      ev.pol.push_back(static_cast<int>(e.get("polarity")->num().value_or(0)));
    }
  }
  const MotorCriteria crit = motor_criteria(name);
  const auto exp_raw =
      num_at(truth, {"expected", "rate_without_inhibition_s_per_day"});
  const auto exp_corr =
      num_at(truth, {"expected", "rate_with_inhibition_s_per_day"});
  const auto exp_period = num_at(truth, {"expected", "inhibition_period_s"});
  const auto exp_step = num_at(truth, {"nominal_step_interval_s"});

  Row row{};
  row.vector = name;
  row.criterion = criteria_text(crit);
  std::printf("%s  (%zu true pulses)\n", name.c_str(), ev.t_s.size());
  const std::array<std::pair<quartz::TimestampMethod, const char *>, 2>
      methods = {{{quartz::TimestampMethod::kThreshold, "threshold"},
                  {quartz::TimestampMethod::kXcorr, "xcorr"}}};
  for (const auto &[method, label] : methods) {
    const auto run = run_motor(wav, method, ev);
    if (!run) {
      row.pass = false;
      continue;
    }
    const std::string key = label;
    const auto ref_rms =
        ref ? num_at(*ref, {("timing_error_rms_us_" + key).c_str()})
            : std::nullopt;
    const auto ref_raw =
        ref ? num_at(*ref, {("rate_" + key).c_str(),
                            "rate_without_inhibition_s_per_day"})
            : std::nullopt;
    const auto ref_corr = ref ? num_at(*ref, {("rate_" + key).c_str(),
                                              "rate_with_inhibition_s_per_day"})
                              : std::nullopt;
    const auto ref_sigma =
        ref ? num_at(*ref, {"noise_sigma_codes"}) : std::nullopt;
    const std::optional<quartz::MotorRate> &r = run->rate;

    bool ok = run->pulses == ev.t_s.size() && run->polarity_ok;
    if (crit.jitter_max_us) {
      ok = ok && run->jitter_rms_us &&
           *run->jitter_rms_us <= *crit.jitter_max_us;
    }
    if (crit.need_clip) {
      ok = ok && run->clipped;
    }
    if (crit.raw_tol) {
      ok = ok && r && exp_raw &&
           std::fabs(r->rate_without_inhibition_s_per_day - *exp_raw) <=
               *crit.raw_tol;
    }
    if (crit.corr_tol) {
      ok = ok && r && exp_corr &&
           r->status == quartz::InhibitionStatus::kCorrected &&
           std::fabs(r->rate_with_inhibition_s_per_day - *exp_corr) <=
               *crit.corr_tol;
    }
    if (crit.period_exact) {
      ok = ok && r && exp_period && r->inhibition_period_s &&
           *r->inhibition_period_s == *exp_period;
    }
    if (crit.step_exact) {
      ok = ok && r && exp_step && r->step_s == *exp_step;
    }
    // Capture-path solver against the batch reference (not a §9 row, but
    // the firmware reports the incremental result).
    const auto &ir = run->inc_rate;
    const bool inc_ok =
        ir.has_value() == r.has_value() &&
        (!r || (std::fabs(ir->rate_without_inhibition_s_per_day -
                          r->rate_without_inhibition_s_per_day) <= 1e-4 &&
                std::fabs(ir->rate_with_inhibition_s_per_day -
                          r->rate_with_inhibition_s_per_day) <= 1e-4 &&
                ir->inhibition_events == r->inhibition_events &&
                ir->inhibition_period_s == r->inhibition_period_s &&
                ir->step_s == r->step_s && ir->status == r->status));
    if (ir && r) {
      std::printf("            incremental: raw %+.6f corr %+.6f (batch "
                  "%+.6f %+.6f)  events %u  %s\n",
                  ir->rate_without_inhibition_s_per_day,
                  ir->rate_with_inhibition_s_per_day,
                  r->rate_without_inhibition_s_per_day,
                  r->rate_with_inhibition_s_per_day, ir->inhibition_events,
                  inc_ok ? "same" : "DIFFERENT");
    }
    ok = ok && inc_ok;
    row.pass = row.pass && ok;

    std::printf(
        "  %-9s pulses %u pol %s  jitter rms %s us (ref %s) max %s"
        "  sigma %.0f (ref %s)  clip %s/%u  extras %u\n",
        label, run->pulses, run->polarity_ok ? "ok" : "BAD",
        fnum(run->jitter_rms_us, "%.3f").c_str(), fnum(ref_rms, "%.3f").c_str(),
        fnum(run->jitter_max_us, "%.3f").c_str(),
        static_cast<double>(run->sigma), fnum(ref_sigma, "%.0f").c_str(),
        run->clipped ? "yes" : "no", run->clipped_pulses, run->extras);
    if (r) {
      std::printf("            step %.1f s  raw %+.4f (ref %s, truth %s)"
                  "  corr %+.4f (ref %s, truth %s)  period %s (truth %s)\n",
                  r->step_s, r->rate_without_inhibition_s_per_day,
                  fnum(ref_raw).c_str(), fnum(exp_raw).c_str(),
                  r->rate_with_inhibition_s_per_day, fnum(ref_corr).c_str(),
                  fnum(exp_corr).c_str(),
                  fnum(r->inhibition_period_s, "%.0f").c_str(),
                  fnum(exp_period, "%.0f").c_str());
      std::printf("            status %s  events %u  jumps",
                  to_string(r->status), r->inhibition_events);
      for (float j : run->jumps_us) {
        std::printf(" %.2f", static_cast<double>(j));
      }
      std::printf(" us (ref %s)  n@8192 %u  resid %.3f us  c %+.3f us\n",
                  ref_jumps(ref, key).c_str(), r->n_at_8192, r->residual_rms_us,
                  r->polarity_offset_us);
    }
    row.port += fmt("%s%s: n=%u", row.port.empty() ? "" : " | ",
                    key == "threshold" ? "thr" : "xc", run->pulses);
    row.ref += fmt("%s%s:", row.ref.empty() ? "" : " | ",
                   key == "threshold" ? "thr" : "xc");
    if (crit.jitter_max_us || crit.need_clip) {
      row.port += " j=" + fnum(run->jitter_rms_us, "%.3f");
      row.ref += " j=" + fnum(ref_rms, "%.3f");
    }
    if (crit.need_clip) {
      row.port += run->clipped ? " clip" : " NOCLIP";
      row.ref += (ref && ref->get("clipped") &&
                  ref->get("clipped")->boolean().value_or(false))
                     ? " clip"
                     : " noclip";
    }
    if (crit.raw_tol && r) {
      row.port += " raw=" + fnum(r->rate_without_inhibition_s_per_day);
      row.ref += " raw=" + fnum(ref_raw);
    }
    if (crit.corr_tol && r) {
      row.port += " corr=" + fnum(r->rate_with_inhibition_s_per_day) +
                  " P=" + fnum(r->inhibition_period_s, "%.0f");
      row.ref += " corr=" + fnum(ref_corr) + " P=" +
                 fnum(ref ? num_at(*ref, {("rate_" + key).c_str(),
                                          "inhibition_period_s"})
                          : std::nullopt,
                      "%.0f");
    }
    if (crit.step_exact && r) {
      row.port += fmt(" step=%.0f", r->step_s);
      row.ref += " step=" +
                 fnum(ref ? num_at(*ref, {("rate_" + key).c_str(), "step_s"})
                          : std::nullopt,
                      "%.0f");
    }
  }
  row.truth = fmt("n=%zu", ev.t_s.size());
  if (crit.raw_tol) {
    row.truth += " raw=" + fnum(exp_raw);
  }
  if (crit.corr_tol) {
    row.truth += " corr=" + fnum(exp_corr) + " P=" + fnum(exp_period, "%.0f");
  }
  if (crit.step_exact) {
    row.truth += " step=" + fnum(exp_step, "%.0f");
  }
  return row;
}

/** Evaluates a tone vector (fork on LEFT, crystal on RIGHT). */
Row tone_vector(const std::string &name, const std::string &wav,
                const Json &truth, const std::optional<Json> &ref,
                bool crystal) {
  const auto exp = num_at(truth, {"expected", "rate_s_per_day"});
  const auto ref_rate = ref ? num_at(*ref, {"rate_s_per_day"}) : std::nullopt;
  const quartz::Rational f =
      crystal ? quartz::kCrystalFrequency
              : exact_frequency(
                    num_at(truth, {"nominal_frequency_hz"}).value_or(0.0));
  const auto r = run_tone(wav, crystal ? Channel::kRight : Channel::kLeft, f,
                          crystal ? quartz::ToneTiming::kCrystalBlockSeconds
                                  : quartz::ToneTiming::kForkBlockSeconds);
  Row row{};
  row.vector = name;
  row.criterion = "+-0.005 s/d";
  row.pass = r && exp && std::fabs(r->rate_s_per_day - *exp) <= 0.005;
  row.port = r ? fnum(r->rate_s_per_day) : "-";
  row.ref = fnum(ref_rate);
  row.truth = fnum(exp);
  std::printf("%s\n  f %u/%u Hz  blocks %u  rate %s (ref %s, truth %s)\n",
              name.c_str(), f.num, f.den, r ? r->blocks : 0u, row.port.c_str(),
              row.ref.c_str(), row.truth.c_str());
  return row;
}

/** Evaluates an LCD vector. */
Row lcd_vector(const std::string &name, const std::string &wav,
               const Json &truth, const std::optional<Json> &ref) {
  const auto exp = num_at(truth, {"expected", "rate_s_per_day"});
  const auto exp_f0 =
      num_at(truth, {"expected", "pattern_repetition_hz_nominal"});
  const auto ref_rate = ref ? num_at(*ref, {"rate_s_per_day"}) : std::nullopt;
  const auto ref_f0 =
      ref ? num_at(*ref, {"pattern_repetition_hz"}) : std::nullopt;
  const auto res = run_lcd(wav);
  Row row{};
  row.vector = name;
  row.criterion = "+-0.005 s/d; f0 exact";
  const auto &pr = res ? res->rate : std::nullopt;
  row.pass = res && pr && exp && exp_f0 &&
             static_cast<double>(res->pattern_hz) == *exp_f0 &&
             std::fabs(pr->rate_s_per_day - *exp) <= 0.005;
  row.port = fmt("%s f0=%u", pr ? fnum(pr->rate_s_per_day).c_str() : "-",
                 res ? res->pattern_hz : 0u);
  row.ref = fnum(ref_rate) + " f0=" + fnum(ref_f0, "%.0f");
  row.truth = fnum(exp) + " f0=" + fnum(exp_f0, "%.0f");
  std::printf("%s\n  status %s  autocorr 128/64/32/16: %.3f %.3f %.3f %.3f\n",
              name.c_str(), res ? to_string(res->status) : "-",
              res ? static_cast<double>(res->autocorr[0]) : 0.0,
              res ? static_cast<double>(res->autocorr[1]) : 0.0,
              res ? static_cast<double>(res->autocorr[2]) : 0.0,
              res ? static_cast<double>(res->autocorr[3]) : 0.0);
  if (pr) {
    std::printf("  harmonics");
    for (std::size_t i = 0; i < pr->harmonic_count; i++) {
      std::printf(" %u", pr->harmonics[i]);
    }
    std::string ref_h = "-";
    if (ref && ref->get("harmonics_used")) {
      ref_h.clear();
      for (const Json &h : ref->get("harmonics_used")->items()) {
        ref_h += fmt("%s%.0f", ref_h.empty() ? "" : " ", h.num().value_or(0));
      }
    }
    std::printf(" (ref %s)  blocks %u  rate %s (ref %s, truth %s)\n",
                ref_h.c_str(), pr->blocks, fnum(pr->rate_s_per_day).c_str(),
                fnum(ref_rate).c_str(), fnum(exp).c_str());
  }
  return row;
}

} // namespace

int main(int argc, char **argv) {
  const std::string dir = argc > 1 ? argv[1] : "docs/handoff/vectors";
  const std::string manifest_text = slurp(dir + "/manifest.json");
  std::string results_text = slurp(dir + "/../results.json");
  if (manifest_text.empty()) {
    std::fprintf(stderr, "cannot read %s/manifest.json\n", dir.c_str());
    return 2;
  }
  const Json manifest(manifest_text);
  if (results_text.empty()) {
    results_text = "{}"; // no baseline: the ref column prints "-"
  }
  const Json results(results_text); // views must not outlive the text

  std::vector<Row> rows;
  for (const Json &item : manifest.items()) {
    const std::string file = item.get("file")->str().value_or("");
    const std::string truth_name = item.get("truth")->str().value_or("");
    const std::string name = file.substr(0, file.size() - 4u);
    const std::string truth_text = slurp(dir + "/" + truth_name);
    const Json truth(truth_text);
    const std::optional<Json> ref = results.get(name);
    const std::string wav = dir + "/" + file;
    if (starts_with(name, "ind_") && truth.get("events")) {
      rows.push_back(motor_vector(name, wav, truth, ref));
    } else if (starts_with(name, "ind_tuning")) {
      rows.push_back(tone_vector(name, wav, truth, ref, false));
    } else if (starts_with(name, "lcd_")) {
      rows.push_back(lcd_vector(name, wav, truth, ref));
    } else if (starts_with(name, "xtal_")) {
      rows.push_back(tone_vector(name, wav, truth, ref, true));
    } else {
      Row r{};
      r.vector = name;
      r.criterion = "unknown vector kind";
      r.pass = false;
      rows.push_back(r);
    }
  }

  std::printf("\nAcceptance (FIRMWARE_HANDOFF.md section 9)\n");
  int failures = 0;
  for (const Row &r : rows) {
    std::printf("%-4s %-26s %s\n", r.pass ? "PASS" : "FAIL", r.vector.c_str(),
                r.criterion.c_str());
    std::printf("       port : %s\n       ref  : %s\n       truth: %s\n",
                r.port.c_str(), r.ref.c_str(), r.truth.c_str());
    failures += r.pass ? 0 : 1;
  }
  std::printf("\n%zu rows, %d failed\n", rows.size(), failures);
  std::printf("\nsizeof: MotorTiming %zu, PulseDetector %zu, "
              "XcorrTimestamper %zu, MotorRateFit %zu, LcdTiming %zu, "
              "ToneTiming %zu, HarmonicPhaseTracker %zu, Highpass4 %zu\n",
              sizeof(quartz::MotorTiming), sizeof(quartz::PulseDetector),
              sizeof(quartz::XcorrTimestamper), sizeof(quartz::MotorRateFit),
              sizeof(quartz::LcdTiming), sizeof(quartz::ToneTiming),
              sizeof(quartz::HarmonicPhaseTracker), sizeof(quartz::Highpass4));
  return (failures == 0 && rows.size() == 14u) ? 0 : 1;
}
