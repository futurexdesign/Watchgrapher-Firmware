/**
 * @file ble_control.h
 * @brief BLE (GATT) transport of the control plane on the Pico 2 W: link
 *        setup, status and counters.
 *
 * Specification: docs/plans/BLE_TRANSPORT_PLAN.md sections 4 and 5.4. The
 * implementation is {@code src/ble_control.cpp}, which compiles to nothing
 * unless {@code TG_BLE=1}; callers guard their calls with {@code #if TG_BLE}.
 *
 * Threading (plan section 5.8): the BTstack and CYW43 work runs in "BT
 * context" (the async context of the radio, on whichever core initialised
 * it). {@code loop()} code only calls the functions below. None of them
 * calls BTstack or takes {@code BluetoothLock}; they reach BT context through
 * lock-free rings, atomics and {@code async_context_set_work_pending()}.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Size of {@link ble_status_t::name}, including the terminating NUL. */
#define BLE_STATUS_NAME_LEN    8
/** Size of {@link ble_status_t::address}, including the terminating NUL. */
#define BLE_STATUS_ADDRESS_LEN 18

/**
 * Snapshot of the BLE link for the CLI, the display and {@code getLink}.
 *
 * All counters count since boot or since the last {@link
 * ble_control_reset_counters} and wrap at 2^32. The snapshot is not atomic as
 * a whole: fields are read one by one while BT context may update them.
 */
typedef struct {
  /** Radio brought up by {@link ble_control_begin}; false until then. */
  bool started;
  /** BLE is enabled (advertising allowed). {@code ble off} clears it. */
  bool enabled;
  /** Connectable advertising is running. */
  bool advertising;
  /** A central is connected (link layer). */
  bool connected;
  /** The central enabled notifications on TX: the link carries frames. */
  bool cccd;
  /** Negotiated ATT MTU in bytes; 23 until the central exchanges it. */
  uint16_t mtu;
  /** Connection interval in microseconds; 0 when not connected. */
  uint32_t conn_interval_us;
  /**
   * Connection-parameter request progress (plan section 14.4): 0 none yet, 1
   * asked 15 ms, 2 asked 15 .. 30 ms after a rejection, 3 gave up after a
   * second rejection. Reset at each connect.
   */
  uint8_t conn_param_step;
  /** Connection-parameter requests the central rejected. */
  uint32_t conn_param_rejects;
  /** Frames accepted from the central into the RX ring. */
  uint32_t rx_frames;
  /** Notifications handed to BTstack successfully. */
  uint32_t tx_frames;
  /** Received frames lost: RX ring full, or written right after a disconnect.
   */
  uint32_t rx_dropped;
  /** Writes longer than 63 bytes, discarded as invalid. */
  uint32_t rx_invalid;
  /** Notify attempts that failed and were retried (ACL buffers full). */
  uint32_t tx_retries;
  /** Frames dropped because they exceeded MTU - 3 bytes. */
  uint32_t tx_oversize;
  /** Connections that ended since boot or reset. */
  uint32_t disconnects;
  /** HCI reason code of the last disconnect; 0 before the first. */
  uint8_t last_disconnect_reason;
  /** {@code get_core_num()} of the core that runs BT context (0 or 1). */
  uint8_t core;
  /** Advertised name {@code TG-XXXX}, NUL-terminated. */
  char name[BLE_STATUS_NAME_LEN];
  /** Bluetooth device address {@code AA:BB:CC:DD:EE:FF}, NUL-terminated. */
  char address[BLE_STATUS_ADDRESS_LEN];
} ble_status_t;

/**
 * Brings up BLE: GATT service, characteristics, advertising and the TX
 * worker. Call once from {@code setup()} after {@code usb_audio_init()}, on
 * the radio core. Blocks until the controller reports its address.
 */
void ble_control_begin(void);

/**
 * Main-loop service: moves received frames out of the RX ring and handles
 * link-down events set by BT context. Makes no BTstack call. Cheap when idle.
 */
void ble_control_task(void);

/**
 * Fills {@code out} with a snapshot of the link. Safe from the main loop.
 *
 * @param out destination, must not be null
 */
void ble_control_status(ble_status_t *out);

/**
 * Requests {@code ble off} ({@code false}: stop advertising and disconnect) or
 * {@code ble on}. Runtime only, not persisted. The request is applied in BT
 * context shortly after; {@link ble_control_status} shows the result.
 */
void ble_control_set_enabled(bool enabled);

/** Requests that BT context clears the link counters. */
void ble_control_reset_counters(void);

#ifdef __cplusplus
}
#endif
