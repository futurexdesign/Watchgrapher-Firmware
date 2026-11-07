/* Host-side ring buffer behavior test (simplified mirror of audio_ring). */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define CAP 16

static int16_t ring[CAP];
static uint32_t wr, rd;

static uint32_t occ(void) { return wr - rd; }
static uint32_t free_slots(void) {
  uint32_t o = occ();
  return o >= CAP ? 0 : (CAP - 1 - o);
}

static uint32_t push(const int16_t *s, uint32_t n) {
  uint32_t w = 0;
  for (uint32_t i = 0; i < n; i++) {
    if (free_slots() == 0) {
      break;
    }
    ring[wr % CAP] = s[i];
    wr++;
    w++;
  }
  return w;
}

static uint32_t pop(int16_t *o, uint32_t n, bool pad) {
  uint32_t r = 0;
  for (uint32_t i = 0; i < n; i++) {
    if (occ() == 0) {
      if (pad) {
        o[i] = 0;
        r++;
        continue;
      }
      break;
    }
    o[i] = ring[rd % CAP];
    rd++;
    r++;
  }
  return r;
}

int main(void) {
  int16_t in[8] = {1, 2, 3, 4, 5, 6, 7, 8};
  int16_t out[8];
  assert(push(in, 8) == 8);
  assert(occ() == 8);
  assert(pop(out, 8, false) == 8);
  assert(memcmp(in, out, sizeof(in)) == 0);
  assert(occ() == 0);

  /* Fill to capacity-1 */
  int16_t bulk[CAP];
  for (int i = 0; i < CAP; i++) {
    bulk[i] = (int16_t)i;
  }
  assert(push(bulk, CAP) == CAP - 1);
  assert(free_slots() == 0);

  /* Order preserved across wrap */
  wr = rd = 0;
  assert(push(in, 8) == 8);
  assert(pop(out, 3, false) == 3);
  assert(out[0] == 1 && out[1] == 2 && out[2] == 3);
  assert(push(in, 8) == 8);
  assert(pop(out, 5, false) == 5);
  assert(out[0] == 4 && out[4] == 8);

  printf("test_ring_buffer: OK\n");
  return 0;
}
