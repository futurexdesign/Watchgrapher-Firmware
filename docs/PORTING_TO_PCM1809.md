# Porting to PCM1809 (I²S)

The USB transport and PCM ring are intentionally independent of the sample producer.

## Current boundary

| Module | Role |
|---|---|
| `adc_capture.*` | Produces raw → PCM16 into `audio_ring` (ADC path) |
| `audio_source.*` | Selects SYNTHETIC vs ADC at compile time; owns stream state |
| `usb_audio.*` / descriptors | Unchanged for a new source |

## Suggested steps

1. Add `AUDIO_SOURCE=PCM1809` (new identifier in `app_config.h`).
2. Implement `pcm1809_capture.cpp` using PIO I²S + DMA into ping-pong buffers of `int16_t` (or 24-bit packed → saturate to int16).
3. From the DMA completion path (main loop, not ISR): `audio_ring_push(pcm, n)`.
4. Wire `audio_source_on_stream_open/close` to start/stop the I²S receiver the same way ADC is started today.
5. Keep USB descriptors identical (still mono 48 kHz S16).
6. Leave `adc_capture` in the tree for A/B comparison builds.

## Clocking note

Match `APP_SAMPLE_RATE_HZ` (48000). The ring’s 47/48/49 packet scheduler already absorbs crystal vs USB SOF ppm drift; do not resample in firmware.
