/**
 * @file ble_control.cpp
 * @brief BLE GATT transport of the control plane (Pico 2 W): service, RX and
 *        TX rings, notify pacer, INFO characteristic, advertising, status.
 *
 * Implements docs/plans/BLE_TRANSPORT_PLAN.md sections 4 and 5.4 under the
 * threading rules of section 5.8. Compiles to nothing unless {@code TG_BLE=1}.
 *
 * Received frames go to a {@code control::Session} over {@code BleLink} and
 * the BLE arbiter port (milestone M2, plan section 5.4), so the BLE link
 * serves the same JSON protocol as USB HID. With {@code -DTG_BLE_ECHO=1}
 * (milestone M1 test build) every received frame is sent straight back
 * instead.
 *
 * <b>Contexts.</b>
 * - Main loop: {@link ble_control_task}, {@link ble_control_status},
 *   {@link ble_control_set_enabled}, {@link ble_control_reset_counters},
 *   {@code BleLink}-style pushes into the TX ring. It never calls BTstack and
 *   never takes {@code BluetoothLock}. It reaches BT context through the two
 *   rings, atomics and {@code async_context_set_work_pending()}.
 * - BT context: BTstack callbacks (HCI and ATT events, GATT write, can-send
 *   callback, housekeeping timer) and the TX worker. It touches only the
 *   rings, atomics, the pacer and the INFO value. It never calls
 *   {@code Session}, {@code control_api}, {@code Serial} or TinyUSB. The code
 *   does not assume which core BT context runs on; {@code get_core_num()} is
 *   recorded once for the status.
 * - {@link ble_control_begin}: setup time, on the radio core. The only place
 *   that uses {@code BluetoothLock}, the heap-allocating library calls and
 *   the one {@code BLE.startAdvertising()} (plan F-27).
 *
 * Plan F-7: {@code setValue()} is never called on the TX characteristic. The
 * INFO characteristic uses it, but only from BT context and it has no notify
 * property, so there is no deferred read of the value buffer.
 */

#include "ble_control.h"

#include "board_config.h"

#if TG_BLE

#include "control/arbiter_port.h"
#include "control/ble_pacer.h"
#include "control/frame.h"
#include "control/frame_ring.h"
#include "control/link.h"
#include "control/session.h"

#include <Arduino.h>
#include <BLE.h>
#include <BluetoothLock.h>
#include <btstack.h>
#include <pico/async_context.h>
#include <pico/cyw43_arch.h>

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstring>

#ifndef TG_BLE_ECHO
/** 1 = send every received frame back (milestone M1 test build). */
#define TG_BLE_ECHO 0
#endif

namespace {

/** GATT UUIDs, plan section 4.1 (lowercase for Web Bluetooth). */
constexpr char kServiceUuid[] = "c137e7c0-92b4-4d9e-87e0-9ba757169016";
constexpr char kRxUuid[] = "c137e7c1-92b4-4d9e-87e0-9ba757169016";
constexpr char kTxUuid[] = "c137e7c2-92b4-4d9e-87e0-9ba757169016";
constexpr char kInfoUuid[] = "c137e7c3-92b4-4d9e-87e0-9ba757169016";

/** RX ring slots: a full 35-fragment request plus margin. */
constexpr std::size_t kRxRingSlots = 40;
/** TX ring slots. */
constexpr std::size_t kTxRingSlots = 16;
/**
 * Largest TX frame in bytes on this link, advertised as {@code txMax} in INFO
 * and the bound of every {@code frameMax} opt-in (plan section 14.3).
 *
 * 244, below the protocol's 255: measured on a Pico 2 W (2026-11-23), a
 * notification whose HCI ACL payload exceeds 255 bytes (value > 248 B, plus
 * 3 B ATT and 4 B L2CAP) gets no reply and leaves the radio unable to
 * advertise until reset. 244 gives a 251-byte ACL payload, which is also the
 * most one LE data PDU carries if data length extension is ever enabled.
 */
constexpr std::size_t kTxFrameMax = 244;
static_assert(kTxFrameMax <= control::kFrameMaxTxBytes,
              "BLE TX frames must fit the protocol's frame limit");
/** Largest INFO value in bytes (plan section 4.1). */
constexpr std::size_t kInfoMaxBytes = 64;
/** ATT MTU before the central exchanges it. */
constexpr std::uint16_t kDefaultMtu = 23;
/** ATT notification header: opcode (1) + handle (2). */
constexpr std::uint16_t kAttNotifyOverhead = 3;
/** Period of the BT-context housekeeping timer while connected. */
constexpr std::uint32_t kHousekeepingMs = 20;
/** Delay after connect before the connection-parameter request. */
constexpr std::uint32_t kConnParamDelayMs = 1000;
/** First request: 15 ms exactly, 1.25 ms units (plan section 14.4). */
constexpr std::uint16_t kConnIntervalFirst = 12;
/** Fallback request after a rejection: 15 ms .. 30 ms, 1.25 ms units. */
constexpr std::uint16_t kConnIntervalFallbackMax = 24;
/** Requested peripheral latency (connection events). */
constexpr std::uint16_t kConnLatency = 0;
/** Requested supervision timeout, 10 ms units (4 s). */
constexpr std::uint16_t kConnTimeout = 400;
/** Frames moved out of the RX ring per {@link ble_control_task} call. */
constexpr int kFramesPerTask = 8;
/** Frames the BLE session sends per pump (plan section 5.4). */
constexpr std::uint8_t kSessionFramesPerTask = 4;
/** Microseconds per 1.25 ms connection-interval unit. */
constexpr std::uint32_t kIntervalUnitUs = 1250;

/** Values of {@link g_conn_param_step} ({@code getLink.ble.connParamStep}). */
constexpr std::uint8_t kConnParamNone = 0;      /**< Nothing requested yet. */
constexpr std::uint8_t kConnParamAsked15 = 1;   /**< Asked 15 ms exactly. */
constexpr std::uint8_t kConnParamAsked1530 = 2; /**< Asked 15 .. 30 ms. */
constexpr std::uint8_t kConnParamGaveUp = 3;    /**< Second rejection. */

/** Values of {@link g_req_power}. */
constexpr std::uint8_t kPowerNone = 0;
constexpr std::uint8_t kPowerOff = 1;
constexpr std::uint8_t kPowerOn = 2;

// ---------------------------------------------------------------------------
// Rings and pacer. RX: producer BT context, consumer main loop. TX: producer
// main loop, consumer BT context (the pacer).
// ---------------------------------------------------------------------------
control::FrameRing<kRxRingSlots> g_rx;
control::FrameRing<kTxRingSlots, kTxFrameMax> g_tx;

int pacer_request(void *ctx, std::uint16_t con_handle);
int pacer_notify(void *ctx, std::uint16_t con_handle, const std::uint8_t *data,
                 std::uint16_t len);

control::BlePacer<kTxRingSlots, kTxFrameMax>
    g_pacer(g_tx, control::BlePacerOps{pacer_request, pacer_notify, nullptr});

/** BTstack can-send registration; reused, never queued twice (pacer guard). */
btstack_context_callback_registration_t g_can_send_reg;
/** Registration of the HCI event handler. */
btstack_packet_callback_registration_t g_hci_reg;
/** Registration of the L2CAP event handler (parameter-update response). */
btstack_packet_callback_registration_t g_l2cap_reg;
/** Housekeeping timer (CCCD sampling, connection-parameter request). */
btstack_timer_source_t g_timer;

/**
 * TX characteristic: notify only. Overrides the library's connection hooks so
 * the CCCD does not outlive a connection (the library keeps the old value).
 */
class TgTxCharacteristic : public BLECharacteristic {
public:
  TgTxCharacteristic()
      : BLECharacteristic(BLEUUID(String(kTxUuid)),
                          static_cast<std::uint16_t>(BLENotify)) {}

  /** ATT value handle, valid after {@code BLE.startAdvertising()}. */
  std::uint16_t value_handle() const { return _valueHandle; }

  /** True when the central enabled notifications. BT context only. */
  bool cccd_notify() const { return (_configData & 0x0001U) != 0; }

protected:
  /** Library hook, BT context: a central connected; notifications are off. */
  void setConHandle(std::uint16_t h) override {
    BLECharacteristic::setConHandle(h);
    _configData = 0;
  }
  /** Library hook, BT context: the library leaves the CCCD value stale. */
  void disconnected() override {
    BLECharacteristic::disconnected();
    _configData = 0;
  }
};

TgTxCharacteristic g_tx_char;
BLECharacteristic
    g_rx_char(BLEUUID(String(kRxUuid)),
              static_cast<std::uint16_t>(BLEWrite | BLEWriteWithoutResponse));
BLECharacteristic g_info_char(BLEUUID(String(kInfoUuid)),
                              static_cast<std::uint16_t>(BLERead));
BLEService g_service{BLEUUID(String(kServiceUuid))};

// ---------------------------------------------------------------------------
// State shared between BT context and the main loop. "BT" = written by BT
// context only; "loop" = written by the main loop only.
// ---------------------------------------------------------------------------
/** BT: begin() finished. Release store publishes the globals below. */
std::atomic<bool> g_started{false};
/** BT (worker): BLE enabled. */
std::atomic<bool> g_enabled{true};
/** BT: connectable advertising is on. */
std::atomic<bool> g_advertising{false};
/** BT: a central is connected. */
std::atomic<bool> g_connected{false};
/** BT: notifications enabled on TX. */
std::atomic<bool> g_cccd{false};
/** BT: negotiated ATT MTU. */
std::atomic<std::uint16_t> g_mtu{kDefaultMtu};
/** BT: connection interval in microseconds. */
std::atomic<std::uint32_t> g_conn_interval_us{0};
/** BT: connection-parameter request progress, a {@code kConnParam*} value. */
std::atomic<std::uint8_t> g_conn_param_step{0};
/** BT: connection-parameter requests the central rejected. */
std::atomic<std::uint32_t> g_conn_param_rejects{0};
/** BT: frames accepted into the RX ring. */
std::atomic<std::uint32_t> g_rx_frames{0};
/** BT: writes above 63 bytes. */
std::atomic<std::uint32_t> g_rx_invalid{0};
/** BT: ended connections. */
std::atomic<std::uint32_t> g_disconnects{0};
/** BT: HCI reason of the last disconnect. */
std::atomic<std::uint8_t> g_last_reason{0};
/** BT: set on every disconnect and CCCD-off; consumed by the main loop. */
std::atomic<bool> g_evt_link_down{false};
/**
 * RX gate. BT context (the RX ring's producer) closes it on every disconnect,
 * before raising {@link g_evt_link_down}; while closed, {@code onWrite} drops
 * writes. The main loop (the RX ring's consumer) clears the ring on the link
 * event and then reopens the gate. Neither side ever calls the other's ring
 * operations, so each ring index has exactly one writer.
 *
 * Ordering: BT stores the gate (release) then the event (release); the loop
 * exchanges the event (acquire), so it sees the closed gate and every push
 * that preceded it, clears, and only then stores {@code false} (release).
 * Frames of the next connection that arrive before the gate reopens are
 * counted in {@link g_rx_gated} and dropped; the window is one loop
 * iteration and hosts write only after reading INFO.
 */
std::atomic<bool> g_rx_gate_closed{false};
/** BT: writes dropped while the gate was closed. */
std::atomic<std::uint32_t> g_rx_gated{0};
/** loop: pending power request (kPower*); consumed by the worker. */
std::atomic<std::uint8_t> g_req_power{kPowerNone};
/** loop: counter reset request; consumed by the worker. */
std::atomic<bool> g_req_reset{false};
/** BT: resume advertising request; consumed by the worker. */
std::atomic<bool> g_req_adv_resume{false};

// Written once in ble_control_begin() before g_started is released; read-only
// afterwards, so plain variables are safe.
/** Async context of the radio; set once in begin(). */
async_context_t *g_ctx = nullptr;
/** {@code get_core_num()} of the core that ran begin(). */
std::uint8_t g_core = 0;
/** Advertised name. */
char g_name[BLE_STATUS_NAME_LEN] = {};
/** Device address text. */
char g_address[BLE_STATUS_ADDRESS_LEN] = {};

// BT-context-only state (plain variables; never read from the main loop).
/** Connection handle, 0 when none. */
std::uint16_t g_con = 0;
/** CCCD state last published. */
bool g_cccd_seen = false;
/** The fallback request is due; sent by the next timer tick. */
bool g_params_retry = false;
/** {@code btstack_run_loop_get_time_ms()} at connect. */
std::uint32_t g_connected_at_ms = 0;

void tx_do_work(async_context_t *context, async_when_pending_worker_t *worker);

/** TX worker; runs in BT context whenever it has been marked pending. */
async_when_pending_worker_t g_tx_worker = {nullptr, tx_do_work, false, nullptr};

/** Wakes the TX worker. Lock-free, callable from any core or context. */
void wake_worker() { async_context_set_work_pending(g_ctx, &g_tx_worker); }

/**
 * Rebuilds the INFO value for {@code mtu}. BT context only (plan F-7): the
 * library resizes the value buffer in {@code setValue()}, which is safe
 * because the ATT read handler runs in the same context and INFO has no
 * notify property (no deferred read).
 */
void rebuild_info(std::uint16_t mtu) {
  // fmax is the host-to-device limit (unchanged); txMax is the largest
  // hello.frameMax the build accepts (plan section 14.3).
  const unsigned fmax = mtu > kAttNotifyOverhead + control::kFrameMaxBytes
                            ? control::kFrameMaxBytes
                            : static_cast<unsigned>(mtu - kAttNotifyOverhead);
  char buf[kInfoMaxBytes];
  const int n = std::snprintf(buf, sizeof(buf),
                              "{\"p\":1,\"mtu\":%u,\"fmax\":%u,\"seg\":0,"
                              "\"txMax\":%u}",
                              static_cast<unsigned>(mtu), fmax,
                              static_cast<unsigned>(kTxFrameMax));
  if (n > 0 && static_cast<std::size_t>(n) < sizeof(buf)) {
    g_info_char.setValue(reinterpret_cast<const std::uint8_t *>(buf),
                         static_cast<std::size_t>(n));
  }
}

/** Publishes a CCCD change. BT context. */
void sample_cccd() {
  if (g_con == 0) {
    return;
  }
  const bool on = g_tx_char.cccd_notify();
  if (on == g_cccd_seen) {
    return;
  }
  g_cccd_seen = on;
  g_pacer.set_cccd(on);
  g_cccd.store(on, std::memory_order_release);
  if (!on) {
    g_evt_link_down.store(true, std::memory_order_release);
  }
}

/** Starts (or restarts) the housekeeping timer. BT context. */
void arm_timer() {
  btstack_run_loop_remove_timer(&g_timer);
  btstack_run_loop_set_timer(&g_timer, kHousekeepingMs);
  btstack_run_loop_add_timer(&g_timer);
}

/**
 * Housekeeping timer: CCCD sampling and the connection-parameter requests.
 * One second after connect it asks for 15 ms exactly; after a rejection (see
 * {@link l2cap_handler}) the next tick asks for 15 .. 30 ms (plan section
 * 14.4). The request is deferred to this timer instead of being sent from the
 * L2CAP event callback, so no BTstack call re-enters the event dispatch.
 */
void on_timer(btstack_timer_source_t *) {
  if (g_con == 0) {
    return;
  }
  sample_cccd();
  const std::uint8_t step = g_conn_param_step.load(std::memory_order_relaxed);
  if (step == kConnParamNone &&
      btstack_run_loop_get_time_ms() - g_connected_at_ms >= kConnParamDelayMs) {
    g_conn_param_step.store(kConnParamAsked15, std::memory_order_relaxed);
    (void)gap_request_connection_parameter_update(g_con, kConnIntervalFirst,
                                                  kConnIntervalFirst,
                                                  kConnLatency, kConnTimeout);
  } else if (g_params_retry) {
    g_params_retry = false;
    g_conn_param_step.store(kConnParamAsked1530, std::memory_order_relaxed);
    (void)gap_request_connection_parameter_update(g_con, kConnIntervalFirst,
                                                  kConnIntervalFallbackMax,
                                                  kConnLatency, kConnTimeout);
  }
  arm_timer();
}

/** A central connected. BT context. */
void on_connected(std::uint16_t handle, std::uint16_t interval_units) {
  g_con = handle;
  g_cccd_seen = false;
  g_params_retry = false;
  g_conn_param_step.store(kConnParamNone, std::memory_order_relaxed);
  g_connected_at_ms = btstack_run_loop_get_time_ms();
  g_pacer.on_connect(handle);
  g_pacer.set_max_value_len(kDefaultMtu - kAttNotifyOverhead);
  rebuild_info(kDefaultMtu);
  g_mtu.store(kDefaultMtu, std::memory_order_relaxed);
  g_conn_interval_us.store(static_cast<std::uint32_t>(interval_units) *
                               kIntervalUnitUs,
                           std::memory_order_relaxed);
  g_advertising.store(false, std::memory_order_relaxed);
  g_cccd.store(false, std::memory_order_relaxed);
  g_connected.store(true, std::memory_order_release);
  arm_timer();
}

/** The connection ended. BT context. */
void on_disconnected(std::uint8_t reason) {
  g_con = 0;
  g_cccd_seen = false;
  btstack_run_loop_remove_timer(&g_timer);
  g_pacer.on_disconnect();
  g_cccd.store(false, std::memory_order_relaxed);
  g_connected.store(false, std::memory_order_release);
  g_conn_interval_us.store(0, std::memory_order_relaxed);
  g_mtu.store(kDefaultMtu, std::memory_order_relaxed);
  g_last_reason.store(reason, std::memory_order_relaxed);
  g_disconnects.fetch_add(1, std::memory_order_relaxed);
  // Frames queued by the old link must not reach the next session. The RX
  // ring is cleared by its consumer, the main loop (see g_rx_gate_closed).
  g_rx_gate_closed.store(true, std::memory_order_release);
  g_evt_link_down.store(true, std::memory_order_release);
  // Advertising stops with the connection. Resume it from the worker with
  // gap_advertisements_enable(1); never BLE.startAdvertising() (plan F-27).
  g_req_adv_resume.store(true, std::memory_order_release);
  wake_worker();
}

/** HCI events: connect, disconnect, connection update. BT context. */
void hci_handler(std::uint8_t type, std::uint16_t, std::uint8_t *packet,
                 std::uint16_t) {
  if (type != HCI_EVENT_PACKET) {
    return;
  }
  switch (hci_event_packet_get_type(packet)) {
  case HCI_EVENT_META_GAP:
    if (hci_event_gap_meta_get_subevent_code(packet) ==
            GAP_SUBEVENT_LE_CONNECTION_COMPLETE &&
        gap_subevent_le_connection_complete_get_status(packet) == 0) {
      on_connected(
          gap_subevent_le_connection_complete_get_connection_handle(packet),
          gap_subevent_le_connection_complete_get_conn_interval(packet));
    }
    break;
  case HCI_EVENT_LE_META:
    if (hci_event_le_meta_get_subevent_code(packet) ==
            HCI_SUBEVENT_LE_CONNECTION_UPDATE_COMPLETE &&
        hci_subevent_le_connection_update_complete_get_status(packet) == 0 &&
        hci_subevent_le_connection_update_complete_get_connection_handle(
            packet) == g_con) {
      g_conn_interval_us.store(
          static_cast<std::uint32_t>(
              hci_subevent_le_connection_update_complete_get_conn_interval(
                  packet)) *
              kIntervalUnitUs,
          std::memory_order_relaxed);
    }
    break;
  case HCI_EVENT_DISCONNECTION_COMPLETE:
    if (g_con != 0 && hci_event_disconnection_complete_get_connection_handle(
                          packet) == g_con) {
      on_disconnected(hci_event_disconnection_complete_get_reason(packet));
    }
    break;
  default:
    break;
  }
}

/**
 * L2CAP events: the central's answer to the connection-parameter request.
 * BT context.
 *
 * BTstack sends {@code gap_request_connection_parameter_update()} as an L2CAP
 * LE signalling request and reports the central's answer with
 * {@code L2CAP_EVENT_CONNECTION_PARAMETER_UPDATE_RESPONSE}. {@code l2cap.c}
 * emits it through {@code l2cap_emit_event()}, which walks only the handlers
 * registered with {@code l2cap_add_event_handler()}; a handler added with
 * {@code hci_add_event_handler()} never sees it. {@code result} 0 means
 * accepted, anything else rejected. The interval the central actually uses
 * arrives separately as {@code HCI_SUBEVENT_LE_CONNECTION_UPDATE_COMPLETE}
 * ({@link hci_handler}). First rejection: fall back once; second: give up.
 */
void l2cap_handler(std::uint8_t type, std::uint16_t, std::uint8_t *packet,
                   std::uint16_t) {
  if (type != HCI_EVENT_PACKET ||
      hci_event_packet_get_type(packet) !=
          L2CAP_EVENT_CONNECTION_PARAMETER_UPDATE_RESPONSE ||
      g_con == 0 ||
      l2cap_event_connection_parameter_update_response_get_handle(packet) !=
          g_con ||
      l2cap_event_connection_parameter_update_response_get_result(packet) ==
          0) {
    return;
  }
  g_conn_param_rejects.fetch_add(1, std::memory_order_relaxed);
  if (g_conn_param_step.load(std::memory_order_relaxed) == kConnParamAsked15) {
    g_params_retry = true;
  } else {
    g_conn_param_step.store(kConnParamGaveUp, std::memory_order_relaxed);
  }
}

/**
 * ATT events: MTU exchange. BT context. Registered after
 * {@code BLE.startAdvertising()} and replaces the library's ATT packet
 * handler, which ignores ATT events (arduino-pico 5.5.1, BLE.cpp).
 */
void att_handler(std::uint8_t type, std::uint16_t, std::uint8_t *packet,
                 std::uint16_t) {
  if (type != HCI_EVENT_PACKET ||
      hci_event_packet_get_type(packet) != ATT_EVENT_MTU_EXCHANGE_COMPLETE ||
      att_event_mtu_exchange_complete_get_handle(packet) != g_con) {
    return;
  }
  const std::uint16_t mtu = att_event_mtu_exchange_complete_get_MTU(packet);
  g_mtu.store(mtu, std::memory_order_relaxed);
  g_pacer.set_max_value_len(mtu > kAttNotifyOverhead
                                ? static_cast<std::size_t>(mtu) -
                                      kAttNotifyOverhead
                                : 0);
  rebuild_info(mtu);
}

/** GATT write on RX: queue the frame for the main loop. BT context. */
class TgRxCallbacks : public BLECharacteristicCallbacks {
public:
  void onWrite(BLECharacteristic *c) override {
    // A host that writes RX has normally just written the CCCD, which has no
    // library hook: sample it here so the link state is current.
    sample_cccd();
    if (g_rx_gate_closed.load(std::memory_order_acquire)) {
      g_rx_gated.fetch_add(1, std::memory_order_relaxed);
      return;
    }
    const std::size_t len = c->valueLen();
    if (len > control::kFrameMaxBytes) {
      g_rx_invalid.fetch_add(1, std::memory_order_relaxed);
      return;
    }
    if (g_rx.push(static_cast<const std::uint8_t *>(c->valueData()), len)) {
      g_rx_frames.fetch_add(1, std::memory_order_relaxed);
    }
  }
};

TgRxCallbacks g_rx_callbacks;

/** BTstack can-send callback: forwards to the pacer. BT context. */
void can_send_cb(void *) { g_pacer.on_can_send(); }

/** Pacer operation: register the can-send callback. BT context. */
int pacer_request(void *, std::uint16_t con_handle) {
  return att_server_register_can_send_now_callback(&g_can_send_reg, con_handle);
}

/** Pacer operation: send one notification on TX. BT context. */
int pacer_notify(void *, std::uint16_t con_handle, const std::uint8_t *data,
                 std::uint16_t len) {
  return att_server_notify(con_handle, g_tx_char.value_handle(), data, len);
}

/** Applies a pending power request. BT context. */
void apply_power(std::uint8_t request) {
  if (request == kPowerOff) {
    g_enabled.store(false, std::memory_order_relaxed);
    gap_advertisements_enable(0);
    g_advertising.store(false, std::memory_order_relaxed);
    if (g_con != 0) {
      (void)gap_disconnect(g_con);
    }
  } else if (request == kPowerOn) {
    g_enabled.store(true, std::memory_order_relaxed);
    if (g_con == 0) {
      gap_advertisements_enable(1);
      g_advertising.store(true, std::memory_order_relaxed);
    }
  }
}

/**
 * TX worker, BT context. Handles the loop's requests (power, counter reset,
 * advertising resume) and starts pending notifications. The loop makes no
 * BTstack call itself (plan section 5.8, rule 1).
 */
void tx_do_work(async_context_t *, async_when_pending_worker_t *) {
  if (g_req_reset.exchange(false, std::memory_order_acq_rel)) {
    g_rx_frames.store(0, std::memory_order_relaxed);
    g_rx_invalid.store(0, std::memory_order_relaxed);
    g_disconnects.store(0, std::memory_order_relaxed);
    g_rx_gated.store(0, std::memory_order_relaxed);
    g_conn_param_rejects.store(0, std::memory_order_relaxed);
    g_rx.reset_overflows();
    g_pacer.reset_counters();
  }
  const std::uint8_t power =
      g_req_power.exchange(kPowerNone, std::memory_order_acq_rel);
  if (power != kPowerNone) {
    apply_power(power);
  }
  if (g_req_adv_resume.exchange(false, std::memory_order_acq_rel) &&
      g_enabled.load(std::memory_order_relaxed) && g_con == 0) {
    gap_advertisements_enable(1);
    g_advertising.store(true, std::memory_order_relaxed);
  }
  g_pacer.kick();
}

// ---------------------------------------------------------------------------
// Session side (main loop only).
// ---------------------------------------------------------------------------

/**
 * The BLE end of the control plane as the {@code Session} sees it. Every
 * member runs in the main loop and only touches the pacer's atomics, the TX
 * ring (as its producer) and the worker wake-up.
 */
class BleLink final : public control::Link {
public:
  [[nodiscard]] const char *kind() const noexcept override { return "ble"; }
  /** Connected with notifications on: replies can reach the central. */
  [[nodiscard]] bool connected() const noexcept override {
    return g_pacer.connected();
  }
  [[nodiscard]] bool can_send() const noexcept override {
    return g_pacer.can_send();
  }
  /** Queues the frame for the pacer and wakes the TX worker. */
  [[nodiscard]] bool send_frame(const std::uint8_t *frame,
                                std::size_t len) noexcept override {
    if (!g_tx.push(frame, len)) {
      return false;
    }
    wake_worker();
    return true;
  }
  /** BLE frames are {@code 5 + payloadLen} bytes, not padded. */
  [[nodiscard]] bool pad_frames() const noexcept override { return false; }
  [[nodiscard]] std::uint16_t mtu() const noexcept override {
    return g_mtu.load(std::memory_order_relaxed);
  }
  /**
   * {@code min(244, mtu - 3)} ({@link kTxFrameMax}): a notification value
   * must fit the ATT MTU and the radio's measured ACL limit. The
   * session never goes below 63 (plan section 14.3); a frame above the
   * pacer's limit is dropped and counted ({@code txOversize}).
   */
  [[nodiscard]] std::size_t max_tx_frame() const noexcept override {
    const std::size_t mtu = g_mtu.load(std::memory_order_relaxed);
    const std::size_t room =
        mtu > kAttNotifyOverhead ? mtu - kAttNotifyOverhead : 0;
    return room < kTxFrameMax ? room : kTxFrameMax;
  }
};

BleLink g_link;
control::Session g_session(g_link, control::ble_port(), kSessionFramesPerTask);
/** {@link control::attach_ble_link} has run (first main-loop task). */
bool g_link_attached = false;

} // namespace

void ble_control_begin(void) {
  // Name from the Bluetooth address, which is the Wi-Fi MAC with the last
  // byte incremented (btstack_hci_transport_cyw43.c). The CYW43 is up before
  // setup(), so the MAC is known before BLE.begin() needs the name.
  std::uint8_t mac[6] = {};
  cyw43_hal_get_mac(0, mac);
  mac[5] = static_cast<std::uint8_t>(mac[5] + 1U);
  std::snprintf(g_name, sizeof(g_name), "TG-%02X%02X", mac[4], mac[5]);

  BLE.begin(g_name); // Blocks (delay) until the controller reports its address.
  const String address = BLE.address().toString();
  std::snprintf(g_address, sizeof(g_address), "%s", address.c_str());

  g_rx_char.setCallbacks(&g_rx_callbacks);
  rebuild_info(kDefaultMtu);
  g_service.addCharacteristic(&g_rx_char);
  g_service.addCharacteristic(&g_tx_char);
  g_service.addCharacteristic(&g_info_char);
  BLE.server()->setName(g_name);
  BLE.server()->addService(&g_service);
  // Exactly once: rebuilds the ATT database (plan F-27). Prints the service
  // UUID on Serial (plan F-12).
  BLE.startAdvertising();

  g_ctx = cyw43_arch_async_context();
  g_core = static_cast<std::uint8_t>(get_core_num());
  g_can_send_reg.callback = can_send_cb;
  g_can_send_reg.context = nullptr;
  g_hci_reg.callback = hci_handler;
  g_l2cap_reg.callback = l2cap_handler;
  btstack_run_loop_set_timer_handler(&g_timer, on_timer);
  {
    BluetoothLock lock;
    hci_add_event_handler(&g_hci_reg);
    l2cap_add_event_handler(&g_l2cap_reg);
    att_server_register_packet_handler(att_handler);
  }
  (void)async_context_add_when_pending_worker(g_ctx, &g_tx_worker);
  g_advertising.store(true, std::memory_order_relaxed);
  g_started.store(true, std::memory_order_release);
}

void ble_control_task(void) {
  if (!g_started.load(std::memory_order_acquire)) {
    return;
  }
  if (!g_link_attached) {
    // Here rather than in ble_control_begin(), which may run on core 1.
    control::attach_ble_link(g_link);
    g_link_attached = true;
  }
  if (g_evt_link_down.exchange(false, std::memory_order_acq_rel)) {
    // The link ended: frames of the old link must not reach the next session,
    // and a half-received request or reply must not survive it.
    g_rx.clear();
    g_rx_gate_closed.store(false, std::memory_order_release);
#if !TG_BLE_ECHO
    g_session.on_link_down();
#endif
  }
  [[maybe_unused]] const std::uint32_t now = millis();
  std::uint8_t frame[control::kRingSlotBytes];
  std::size_t len = 0;
  for (int i = 0; i < kFramesPerTask; i++) {
#if TG_BLE_ECHO
    // Echo: leave the frame in the RX ring until the TX ring has room, so
    // nothing is lost (backpressure).
    if (!g_pacer.can_send()) {
      break;
    }
#endif
    if (!g_rx.pop(frame, len)) {
      break;
    }
#if TG_BLE_ECHO
    if (g_tx.push(frame, len)) {
      wake_worker();
    }
#else
    g_session.on_frame(frame, len, now);
#endif
  }
#if !TG_BLE_ECHO
  g_session.task(now);
#endif
}

void ble_control_status(ble_status_t *out) {
  *out = ble_status_t{};
  out->started = g_started.load(std::memory_order_acquire);
  if (!out->started) {
    return;
  }
  out->enabled = g_enabled.load(std::memory_order_relaxed);
  out->advertising = g_advertising.load(std::memory_order_relaxed);
  out->connected = g_connected.load(std::memory_order_acquire);
  out->cccd = g_cccd.load(std::memory_order_acquire);
  out->mtu = g_mtu.load(std::memory_order_relaxed);
  out->conn_interval_us = g_conn_interval_us.load(std::memory_order_relaxed);
  out->conn_param_step = g_conn_param_step.load(std::memory_order_relaxed);
  out->conn_param_rejects =
      g_conn_param_rejects.load(std::memory_order_relaxed);
  out->rx_frames = g_rx_frames.load(std::memory_order_relaxed);
  out->tx_frames = g_pacer.tx_frames();
  out->rx_dropped =
      g_rx.overflows() + g_rx_gated.load(std::memory_order_relaxed);
  out->rx_invalid = g_rx_invalid.load(std::memory_order_relaxed);
  out->tx_retries = g_pacer.tx_retries();
  out->tx_oversize = g_pacer.tx_oversize();
  out->disconnects = g_disconnects.load(std::memory_order_relaxed);
  out->last_disconnect_reason = g_last_reason.load(std::memory_order_relaxed);
  out->core = g_core;
  std::memcpy(out->name, g_name, sizeof(out->name));
  std::memcpy(out->address, g_address, sizeof(out->address));
}

void ble_control_set_enabled(bool enabled) {
  if (!g_started.load(std::memory_order_acquire)) {
    return;
  }
  g_req_power.store(enabled ? kPowerOn : kPowerOff, std::memory_order_release);
  wake_worker();
}

void ble_control_reset_counters(void) {
  if (!g_started.load(std::memory_order_acquire)) {
    return;
  }
  g_req_reset.store(true, std::memory_order_release);
  wake_worker();
}

#endif // TG_BLE
