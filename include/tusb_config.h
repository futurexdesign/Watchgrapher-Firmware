/**
 * @file tusb_config.h
 * @brief TinyUSB device configuration for Timegrapher Stand (reference copy).
 *
 * PlatformIO + Adafruit TinyUSB does not reliably honor
 * {@code CFG_TUSB_CONFIG_FILE} (project {@code include/} is not first on the
 * library compile path). Live settings are applied as {@code -DCFG_TUD_*}
 * flags in {@code platformio.ini}. Keep this file in sync with those flags
 * for documentation and future CMake ports.
 */
#ifndef TUSB_CONFIG_TIMEGRAPHER_H_
#define TUSB_CONFIG_TIMEGRAPHER_H_

#ifdef __cplusplus
extern "C" {
#endif

//--------------------------------------------------------------------
// COMMON
//--------------------------------------------------------------------

#ifndef CFG_TUSB_MCU
#define CFG_TUSB_MCU OPT_MCU_RP2040
#endif

#ifndef CFG_TUSB_OS
#define CFG_TUSB_OS OPT_OS_PICO
#endif

#ifndef CFG_TUSB_DEBUG
#define CFG_TUSB_DEBUG 0
#endif

#define CFG_TUD_ENABLED 1

/* Device only — host stack off to avoid MAX3421/PIO-USB deps. */
#define CFG_TUH_ENABLED 0

#define CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_ALIGN __attribute__((aligned(4)))

//--------------------------------------------------------------------
// DEVICE
//--------------------------------------------------------------------

#ifndef CFG_TUD_ENDPOINT0_SIZE
#define CFG_TUD_ENDPOINT0_SIZE 64
#endif

/* Class drivers: UAC2 + CDC (Arduino) + vendor HID control. */
#define CFG_TUD_AUDIO   1
#define CFG_TUD_CDC     1
#define CFG_TUD_MSC     0
#define CFG_TUD_HID     1
#define CFG_TUD_MIDI    0
#define CFG_TUD_VENDOR  0
#define CFG_TUD_VIDEO   0
#define CFG_TUD_VIDEO_STREAMING 0

#ifndef CFG_TUD_HID_EP_BUFSIZE
#define CFG_TUD_HID_EP_BUFSIZE 64
#endif

//--------------------------------------------------------------------
// AUDIO (UAC2 mono mic @ 48 kHz)
//--------------------------------------------------------------------

#define CFG_TUD_AUDIO_FUNC_1_SAMPLE_RATE            48000

#define CFG_TUD_AUDIO_ENABLE_EP_IN                  1
#define CFG_TUD_AUDIO_ENABLE_EP_OUT                 0
#define CFG_TUD_AUDIO_ENABLE_FEEDBACK_EP            0
#define CFG_TUD_AUDIO_ENABLE_INTERRUPT_EP           0

#define CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_TX  3
#define CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX          1

/*
 * Max packet = 49 samples * 3 bytes (24-bit; allows 47/48/49 scheduling).
 * Hardcoded: TUD_AUDIO_EP_SIZE is not available when this header is parsed.
 */
#define CFG_TUD_AUDIO_EP_SZ_IN                      147
#define CFG_TUD_AUDIO_FUNC_1_EP_IN_SZ_MAX           CFG_TUD_AUDIO_EP_SZ_IN
#define CFG_TUD_AUDIO_FUNC_1_EP_IN_SW_BUF_SZ        (4 * CFG_TUD_AUDIO_EP_SZ_IN)

/* Equals TUD_AUDIO_MIC_ONE_CH_DESC_LEN (hardcoded; usbd.h not visible here). */
#define CFG_TUD_AUDIO_FUNC_1_DESC_LEN               132
#define CFG_TUD_AUDIO_FUNC_1_N_AS_INT               1
#define CFG_TUD_AUDIO_FUNC_1_CTRL_BUF_SZ            64

/* TinyUSB sizes ISO IN packets (47/48/49) from FIFO level + nominal rate. */
#define CFG_TUD_AUDIO_EP_IN_FLOW_CONTROL            1

#ifdef __cplusplus
}
#endif

#endif /* TUSB_CONFIG_TIMEGRAPHER_H_ */
