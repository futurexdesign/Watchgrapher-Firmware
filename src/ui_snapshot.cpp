/**
 * @file ui_snapshot.cpp
 * @brief Sequence-locked snapshot shared between core 0 and core 1.
 *
 * Writer: seq becomes odd, data is written, seq becomes even again. Reader:
 * copies only between two equal, even reads of seq. {@code __dmb()} orders
 * the accesses across cores and acts as a compiler barrier.
 *
 * @see ui_snapshot.h
 */

#include "ui_snapshot.h"

#include <hardware/sync.h>

#include <cstring>

namespace {

volatile uint32_t g_seq = 0; /* even = stable, odd = write in progress */
ui_snapshot_t g_snap;

} // namespace

void ui_snapshot_publish(const ui_snapshot_t *s) {
  g_seq = g_seq + 1u;
  __dmb();
  memcpy(&g_snap, s, sizeof(g_snap));
  __dmb();
  g_seq = g_seq + 1u;
}

bool ui_snapshot_read(ui_snapshot_t *out) {
  /* A publish copies ~100 bytes, so a few retries always suffice. */
  for (int attempt = 0; attempt < 64; attempt++) {
    const uint32_t before = g_seq;
    if (before == 0u) {
      return false; /* nothing published yet */
    }
    if (before & 1u) {
      continue;
    }
    __dmb();
    memcpy(out, &g_snap, sizeof(*out));
    __dmb();
    if (g_seq == before) {
      return true;
    }
  }
  return false;
}
