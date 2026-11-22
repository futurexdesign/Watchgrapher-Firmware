/**
 * @file beat_fit.cpp
 * @brief Least-squares beat timing fit.
 *
 * @see mechanical/beat_fit.h
 */

#include "mechanical/beat_fit.h"

#include <math.h>

namespace {

uint32_t ring_index(const beat_fit_t *f, uint32_t i) {
  return (f->head + i) % BEAT_FIT_CAP;
}

void start_at(beat_fit_t *f, double t, double nominal) {
  f->origin = t;
  f->grid_origin = t;
  f->k_base = 0;
  f->nominal = nominal;
  f->head = 0;
  f->n = 1;
  f->k[0] = 0;
  f->u[0] = 0.f;
  f->last_k = 0;
  f->last_t = t;
  f->rejects = 0;
}

/** Drops the oldest point and rebases the rest on the new oldest point. */
void drop_oldest(beat_fit_t *f) {
  f->head = (f->head + 1u) % BEAT_FIT_CAP;
  f->n--;
  const int32_t k0 = f->k[f->head];
  const float u0 = f->u[f->head];
  f->origin += static_cast<double>(k0) * f->nominal + static_cast<double>(u0);
  for (uint32_t i = 0; i < f->n; i++) {
    const uint32_t j = ring_index(f, i);
    f->k[j] -= k0;
    f->u[j] -= u0;
  }
  f->last_k -= k0;
  f->k_base += k0;
}

/**
 * Fills {@code pt} (if any) for a point at grid-absolute index {@code k_abs}.
 */
void set_point(const beat_fit_t *f, beat_fit_point_t *pt, int32_t k_abs,
               double t) {
  if (!pt) {
    return;
  }
  pt->valid = true;
  pt->k = k_abs;
  pt->rel = t - f->grid_origin;
  pt->nominal = f->nominal;
  pt->u = pt->rel - static_cast<double>(k_abs) * f->nominal;
}

/** {@code (-1)^k} as +1 / -1. */
float parity(int32_t k) { return (k & 1) ? -1.f : 1.f; }

/**
 * Fits u = a + d*k + c*s over points where {@code use[i]} is set (all if
 * {@code use} is null). Centred two-pass sums keep float error small.
 *
 * @return Number of points used.
 */
uint32_t fit(const beat_fit_t *f, const bool *use, float *d, float *c,
             bool *alt_valid) {
  uint32_t m = 0;
  float kbar = 0.f, sbar = 0.f, ubar = 0.f;
  for (uint32_t i = 0; i < f->n; i++) {
    if (use && !use[i]) {
      continue;
    }
    const uint32_t j = ring_index(f, i);
    kbar += static_cast<float>(f->k[j]);
    sbar += parity(f->k[j]);
    ubar += f->u[j];
    m++;
  }
  if (m < 2u) {
    *d = 0.f;
    *c = 0.f;
    *alt_valid = false;
    return m;
  }
  const float inv_m = 1.f / static_cast<float>(m);
  kbar *= inv_m;
  sbar *= inv_m;
  ubar *= inv_m;

  float skk = 0.f, sss = 0.f, sks = 0.f, sku = 0.f, ssu = 0.f;
  for (uint32_t i = 0; i < f->n; i++) {
    if (use && !use[i]) {
      continue;
    }
    const uint32_t j = ring_index(f, i);
    const float dk = static_cast<float>(f->k[j]) - kbar;
    const float ds = parity(f->k[j]) - sbar;
    const float du = f->u[j] - ubar;
    skk += dk * dk;
    sss += ds * ds;
    sks += dk * ds;
    sku += dk * du;
    ssu += ds * du;
  }

  const float det = skk * sss - sks * sks;
  /* Both parities present (and not perfectly confounded with k) → 2-param. */
  if (sss > 0.1f * static_cast<float>(m) && det > 1e-6f * skk * sss) {
    *d = (sku * sss - ssu * sks) / det;
    *c = (ssu * skk - sku * sks) / det;
    *alt_valid = true;
  } else {
    *d = skk > 0.f ? sku / skk : 0.f;
    *c = 0.f;
    *alt_valid = false;
  }
  return m;
}

} // namespace

void beat_fit_reset(beat_fit_t *f) {
  f->n = 0;
  f->head = 0;
  f->rejects = 0;
}

beat_fit_push_result_t beat_fit_push(beat_fit_t *f, double t,
                                     double nominal_half_period,
                                     double max_span, beat_fit_point_t *pt) {
  if (pt) {
    *pt = {};
  }
  if (nominal_half_period <= 0.0) {
    return BEAT_FIT_REJECTED;
  }
  if (f->n == 0u || nominal_half_period != f->nominal) {
    start_at(f, t, nominal_half_period);
    set_point(f, pt, 0, t);
    return BEAT_FIT_STARTED;
  }

  const double steps = (t - f->last_t) / f->nominal;
  const double whole = floor(steps + 0.5);
  const auto grid_err = static_cast<float>(fabs(steps - whole));
  if (whole > static_cast<double>(BEAT_FIT_MAX_GAP)) {
    start_at(f, t, nominal_half_period);
    set_point(f, pt, 0, t);
    return BEAT_FIT_STARTED;
  }
  if (whole < 1.0 || grid_err > BEAT_FIT_MAX_GRID_ERR) {
    if (++f->rejects >= BEAT_FIT_MAX_REJECTS) {
      start_at(f, t, nominal_half_period);
      set_point(f, pt, 0, t);
      return BEAT_FIT_STARTED;
    }
    set_point(f, pt, f->last_k + static_cast<int32_t>(whole) + f->k_base, t);
    return BEAT_FIT_REJECTED;
  }
  f->rejects = 0;

  if (f->n == BEAT_FIT_CAP) {
    drop_oldest(f);
  }
  const int32_t k = f->last_k + static_cast<int32_t>(whole);
  const uint32_t j = ring_index(f, f->n);
  f->k[j] = k;
  f->u[j] =
      static_cast<float>(t - (f->origin + static_cast<double>(k) * f->nominal));
  f->n++;
  f->last_k = k;
  f->last_t = t;
  const int32_t k_abs = k + f->k_base; /* before any rebase below */
  set_point(f, pt, k_abs, t);

  /* The oldest point is always rebased to k = 0, u = 0, i.e. at origin. */
  while (f->n > 1u && t - f->origin > max_span) {
    drop_oldest(f);
  }
  return BEAT_FIT_ACCEPTED;
}

bool beat_fit_solve(const beat_fit_t *f, beat_fit_result_t *out) {
  *out = {};
  if (f->n < BEAT_FIT_MIN_POINTS) {
    return false;
  }

  float d = 0.f, c = 0.f;
  bool alt = false;
  fit(f, nullptr, &d, &c, &alt);

  /* Residuals of the first fit; intercept from the means. */
  auto residuals = [&](const bool *use, float dd, float cc, float *r,
                       float *rms) {
    float kbar = 0.f, sbar = 0.f, ubar = 0.f;
    uint32_t m = 0;
    for (uint32_t i = 0; i < f->n; i++) {
      if (use && !use[i]) {
        continue;
      }
      const uint32_t j = ring_index(f, i);
      kbar += static_cast<float>(f->k[j]);
      sbar += parity(f->k[j]);
      ubar += f->u[j];
      m++;
    }
    const float inv_m = m ? 1.f / static_cast<float>(m) : 0.f;
    const float a = (ubar - dd * kbar - cc * sbar) * inv_m;
    float ss = 0.f;
    for (uint32_t i = 0; i < f->n; i++) {
      const uint32_t j = ring_index(f, i);
      r[i] = f->u[j] -
             (a + dd * static_cast<float>(f->k[j]) + cc * parity(f->k[j]));
      if (!use || use[i]) {
        ss += r[i] * r[i];
      }
    }
    *rms = m ? sqrtf(ss * inv_m) : 0.f;
  };

  static float r[BEAT_FIT_CAP];
  static bool use[BEAT_FIT_CAP];
  float rms = 0.f;
  residuals(nullptr, d, c, r, &rms);

  const float gate = fmaxf(3.5f * rms, 1.f);
  uint32_t kept = 0;
  for (uint32_t i = 0; i < f->n; i++) {
    use[i] = fabsf(r[i]) <= gate;
    kept += use[i] ? 1u : 0u;
  }
  uint32_t m = f->n;
  if (kept < f->n && kept >= BEAT_FIT_MIN_POINTS) {
    m = fit(f, use, &d, &c, &alt);
    residuals(use, d, c, r, &rms);
  }

  out->half_period = f->nominal + static_cast<double>(d);
  out->half_period_delta = d;
  out->alt_offset = c;
  out->resid_rms = rms;
  out->points = m;
  out->alt_valid = alt;
  return true;
}
