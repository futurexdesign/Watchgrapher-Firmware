# Porting to PCM1809 (I²S)

The USB transport and PCM ring are intentionally independent of the sample producer.

## Current boundary

| Module | Role |
|---|---|
| `adc_capture.*` | Produces raw → PCM24 into `audio_ring` (ADC path) |
| `pcm1808_capture.*` | Produces PCM24 (left slot, through `dsp_graph_process`) into `audio_ring` and the raw stereo codes for the quartz engine; the closest template for a PCM1809 backend |
| `audio_source.*` | Selects SYNTHETIC / ADC / PCM1808 at compile time; owns the stream state and the run latch |
| `usb_audio.*` / descriptors | Unchanged for a new source |

## Suggested steps

1. Add `AUDIO_SOURCE=PCM1809` (new identifier in `app_config.h`; today `SYNTHETIC=1`, `ADC=2`, `PCM1808=3`).
2. Implement `pcm1809_capture.cpp` using PIO I²S + DMA into ping-pong buffers, modelled on `pcm1808_capture.cpp` (sequence-numbered blocks taken oldest first, lost-block index advance, measured frame rate, `dsp_graph_process`, quartz hooks). The ring holds signed PCM24 in `int32_t`.
3. From the main loop, not the ISR: `audio_ring_push(pcm, n)`, only when `audio_source_feeds_ring()` is true (it is false in standalone device mode).
4. Provide `*_capture_init/start/stop/is_running/process` and call them from `audio_source.cpp` (`capture_service_request`, `backend_running`, `audio_source_capture_pause/resume`). Capture is started by the stream open **or** the run latch, and must survive being stopped and restarted around a flash write.
5. Keep USB descriptors identical (mono 48 kHz S24LE).
6. Leave `adc_capture` in the tree for A/B comparison builds.

## Clocking note

Match `APP_SAMPLE_RATE_HZ` (48000). The app feeds 48 samples per millisecond and TinyUSB's IN flow control (47/48/49 samples per packet) absorbs crystal vs USB SOF ppm drift; do not resample in firmware. The PCM1808 backend measures its real frame rate instead of assuming 48 kHz (`pcm1808_rate.h`); a PCM1809 backend that is strapped differently needs the same treatment, and the DSP timebase must stay unverified until a rate is confirmed.
