/**
 * @file test_dsp_blocks.cpp
 * @brief Host test for the DSP block table and pipeline graph.
 *
 * Build and run:
 *   c++ -std=c++17 -Wall -Wextra -Werror -Iinclude tests/test_dsp_blocks.cpp \
 *       src/dsp_blocks.cpp -o /tmp/t && /tmp/t
 */

#include "dsp_blocks.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace {

int g_fail = 0;

#define CHECK(c)                                                               \
  do {                                                                         \
    if (!(c)) {                                                                \
      std::printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c);                  \
      g_fail++;                                                                \
    }                                                                          \
  } while (0)

/** @return Index of the node named {@code key}, or -1. */
int find(const dsp_pipeline_node_t *n, uint32_t count, const char *key) {
  for (uint32_t i = 0; i < count; i++) {
    if (std::strcmp(n[i].key, key) == 0) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

void test_blocks_are_nodes() {
  uint32_t count = 0;
  const auto *n = dsp_pipeline_nodes(&count);
  CHECK(n != nullptr && count > 0);
  for (uint32_t id = 0; id < DSP_BLOCK_COUNT; id++) {
    const auto *b = dsp_block_info(static_cast<dsp_block_id_t>(id));
    CHECK(b != nullptr);
    if (id == DSP_BLOCK_BYPASS) {
      CHECK(find(n, count, b->json_name) < 0);
      CHECK(b->group == DSP_GROUP_PROCESS && b->order == 0);
      continue;
    }
    int idx = find(n, count, b->json_name);
    CHECK(idx >= 0);
    if (idx < 0) {
      continue;
    }
    /* Exactly once. */
    for (uint32_t j = static_cast<uint32_t>(idx) + 1; j < count; j++) {
      CHECK(std::strcmp(n[j].key, b->json_name) != 0);
    }
    CHECK(n[idx].kind == (b->group == DSP_GROUP_PROCESS ? DSP_NODE_PROCESS
                                                        : DSP_NODE_ANALYSIS));
    CHECK(n[idx].order == b->order);
    CHECK(b->is_process == (b->group == DSP_GROUP_PROCESS));
  }
  /* Orders unique within each group. */
  for (uint32_t a = 0; a < DSP_BLOCK_COUNT; a++) {
    for (uint32_t b = a + 1; b < DSP_BLOCK_COUNT; b++) {
      const auto *x = dsp_block_info(static_cast<dsp_block_id_t>(a));
      const auto *y = dsp_block_info(static_cast<dsp_block_id_t>(b));
      CHECK(!(x->group == y->group && x->order == y->order));
    }
  }
  /* Unique keys overall; pseudo-nodes present. */
  for (uint32_t i = 0; i < count; i++) {
    for (uint32_t j = i + 1; j < count; j++) {
      CHECK(std::strcmp(n[i].key, n[j].key) != 0);
    }
  }
  CHECK(find(n, count, "input") >= 0 && find(n, count, "pcm24") >= 0 &&
        find(n, count, "usb") >= 0 && find(n, count, "quartz") >= 0);
  CHECK(count == DSP_BLOCK_COUNT - 1 + 4);
}

void test_inputs_exist_and_acyclic() {
  uint32_t count = 0;
  const auto *n = dsp_pipeline_nodes(&count);
  for (uint32_t i = 0; i < count; i++) {
    for (uint32_t k = 0; k < n[i].n_inputs; k++) {
      CHECK(find(n, count, n[i].inputs[k]) >= 0);
    }
  }
  CHECK(n[find(n, count, "input")].n_inputs == 0);
  /* Kahn's algorithm: all nodes must be removable. */
  int indeg[64] = {};
  bool done[64] = {};
  CHECK(count <= 64);
  for (uint32_t i = 0; i < count; i++) {
    indeg[i] = n[i].n_inputs;
  }
  uint32_t removed = 0;
  for (bool progress = true; progress;) {
    progress = false;
    for (uint32_t i = 0; i < count; i++) {
      if (done[i] || indeg[i] != 0) {
        continue;
      }
      done[i] = true;
      removed++;
      progress = true;
      for (uint32_t j = 0; j < count; j++) {
        for (uint32_t k = 0; k < n[j].n_inputs; k++) {
          if (std::strcmp(n[j].inputs[k], n[i].key) == 0) {
            indeg[j]--;
          }
        }
      }
    }
  }
  CHECK(removed == count);
}

void test_process_chain_single_path() {
  uint32_t count = 0;
  const auto *n = dsp_pipeline_nodes(&count);
  /* Walk backwards from usb: each node must have exactly one input. */
  static const char *const expect[] = {
      "usb",      "pcm24",   "limiter",    "agc",   "gain",
      "softGate", "firBank", "filterBank", "notch", "input"};
  int idx = find(n, count, "usb");
  CHECK(idx >= 0);
  for (const char *key : expect) {
    CHECK(idx >= 0 && std::strcmp(n[idx].key, key) == 0);
    if (idx < 0 || std::strcmp(key, "input") == 0) {
      break;
    }
    CHECK(n[idx].n_inputs == 1);
    idx = find(n, count, n[idx].inputs[0]);
  }
  /* Process nodes ascend in order along the chain. */
  const char *const chain[] = {"notch", "filterBank", "firBank", "softGate",
                               "gain",  "agc",        "limiter"};
  for (size_t i = 1; i < sizeof(chain) / sizeof(chain[0]); i++) {
    CHECK(n[find(n, count, chain[i])].order >
          n[find(n, count, chain[i - 1])].order);
  }
  /* Analysis dependencies. */
  const int ev = find(n, count, "eventDetect");
  CHECK(std::strcmp(n[find(n, count, "beatRate")].inputs[0], n[ev].key) == 0);
  CHECK(std::strcmp(n[find(n, count, "eventLog")].inputs[0], n[ev].key) == 0);
}

} // namespace

int main() {
  test_blocks_are_nodes();
  test_inputs_exist_and_acyclic();
  test_process_chain_single_path();
  if (g_fail) {
    std::printf("%d failure(s)\n", g_fail);
    return 1;
  }
  std::printf("test_dsp_blocks: all passed\n");
  return 0;
}
