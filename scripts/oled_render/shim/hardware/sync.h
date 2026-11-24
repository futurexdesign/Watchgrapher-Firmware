/** @file sync.h @brief Host stand-in for the Pico SDK barrier. */
#pragma once
#include <atomic>
inline void __dmb() { std::atomic_thread_fence(std::memory_order_seq_cst); }
