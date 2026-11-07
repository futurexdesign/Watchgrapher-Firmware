# Host capture notes

## Linux

```bash
arecord -l
arecord -D hw:CARD=WatchTiming,DEV=0 -c 1 -f S16_LE -r 48000 -d 60 watch_test.wav
# Card name varies — use arecord -l after plugging in.
python3 tests/host_analyze.py watch_test.wav
```

## macOS / Windows

Record uncompressed 16-bit mono 48 kHz WAV. Disable enhancements, noise suppression, and automatic mic level.

## Synthetic expect

997 Hz tone near −12 dBFS, stable amplitude, no periodic dropouts.

## ADC expect

Biased mid-scale DC mean near `ADC_DC_OFFSET_COUNTS` (default 2048) when quiet; watch ticks as short transients without clipping if the analog frontend is correct.
