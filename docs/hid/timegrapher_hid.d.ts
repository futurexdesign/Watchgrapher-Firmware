/**
 * Timegrapher Stand — HID control plane types (WebHID / JS).
 *
 * Normative protocol: ../HID_CONTROL_PLANE.md
 * Protocol version: 1
 */

/** Current wire protocol version (`v` field). */
export type HidProtocolVersion = 1;

/** Vendor HID identity for navigator.hid filters. */
export declare const TIMEGRAPHER_HID_USAGE_PAGE = 0xffa0;
export declare const TIMEGRAPHER_HID_USAGE = 0x0001;
export declare const TIMEGRAPHER_HID_REPORT_ID = 0x01;

/**
 * Max reassembled JSON payload (bytes). The device sends replies up to 2047
 * bytes but accepts requests of at most 2030 bytes (35 fragments of 58);
 * longer requests are dropped silently, without a reply.
 */
export declare const TIMEGRAPHER_HID_MAX_JSON_BYTES = 2048;

/**
 * Error codes returned in HidErrorResponse. `notFound` and `internal` are
 * reserved: the current firmware never sends them. `tooLarge` is sent only
 * for a reply that exceeds 2047 bytes (never for an oversized request).
 */
export type HidErrorCode =
  | 'unknownOp'
  | 'badRequest'
  | 'badParam'
  | 'notFound'
  | 'tooLarge'
  | 'busy'
  | 'flashError'
  | 'internal';

/** CamelCase block keys in getState / patchState. */
export type DspBlockKey =
  | 'levelMeter'
  | 'gain'
  | 'limiter'
  | 'filterBank'
  | 'eventDetect'
  | 'beatRate'
  | 'noiseFloor'
  | 'eventLog'
  | 'softGate'
  | 'notch'
  | 'agc'
  | 'matchedFilter'
  | 'firBank'
  | 'spectrum';

/** filterBank.preset: 0=off, 1=wide, 2=medium, 3=narrow. */
export type FilterPreset = 0 | 1 | 2 | 3;

/** notch.hz */
export type NotchHz = 50 | 60;

export type DspFaultReason = 'none' | 'underrun' | 'usbStall' | 'overload';

/** uint64 sample indices as decimal strings (JS Number is unsafe above 2^53). */
export type SampleIndexString = string;

export interface BlockState<P extends object = Record<string, never>> {
  enabled: boolean;
  params: P;
}

export interface GainParams {
  db: number;
}

export interface FilterBankParams {
  preset: FilterPreset;
}

export interface EventDetectParams {
  threshold: number;
  prerollMs: number;
  tailMs: number;
}

export interface SoftGateParams {
  floorDb: number;
  attackMs: number;
  releaseMs: number;
}

export interface NotchParams {
  hz: NotchHz;
}

export interface AgcParams {
  target: number;
  tauS: number;
  /** Live AGC gain; read-only — ignored on patchState. */
  gainDb?: number;
}

export interface MatchedFilterParams {
  threshold: number;
  learn: number;
  /** Latest score; read-only. */
  score?: number;
}

export interface SpectrumParams {
  stride: number;
}

export interface BeatRateParams {
  /** Lift angle λ in degrees (10..90). Default 51. */
  liftAngle: number;
  /** Nominal A/h; 0 = auto nearest common rate. */
  vph: number;
  /**
   * Sample-clock correction in effect (ppm, + = board fast; PPS while locked).
   * Read-only: patchState ignores it. Use setTimebase to change it.
   */
  readonly calPpm?: number;
  /** Rate / beat-error fit window in seconds, 2..30. Default 6. */
  windowS?: number;
  /**
   * Integration time of {@link DspBeatRate.avgLong} in seconds, 1..60.
   * Default 10. Also the averaging shown on the OLED. Not persisted.
   */
  avgS?: number;
}

export interface DspBlocksState {
  levelMeter?: BlockState;
  gain?: BlockState<GainParams>;
  limiter?: BlockState;
  filterBank?: BlockState<FilterBankParams>;
  eventDetect?: BlockState<EventDetectParams>;
  beatRate?: BlockState<BeatRateParams>;
  noiseFloor?: BlockState;
  eventLog?: BlockState;
  softGate?: BlockState<SoftGateParams>;
  notch?: BlockState<NotchParams>;
  agc?: BlockState<AgcParams>;
  matchedFilter?: BlockState<MatchedFilterParams>;
  firBank?: BlockState<FilterBankParams>;
  spectrum?: BlockState<SpectrumParams>;
}

/** Quartz measurement mode (HID_CONTROL_PLANE.md §5.3). */
export type QuartzMode = 'mechanical' | 'motor' | 'lcd' | 'fork';

/** Motor leading-edge timestamp method (handoff §4). */
export type QuartzTsMethod = 'threshold' | 'xcorr';

/** Selectable tuning-fork frequencies (Hz); 341.333 = 1024/3. */
export type QuartzForkHz = 300 | 341.333 | 360 | 480 | 720;

/**
 * Quartz settings: `getState.quartz`, written with `patchState.quartz`.
 * Present only when {@link HidHelloCaps.quartz} is true. Not persisted.
 */
export interface QuartzState {
  /** Active mode. Selecting a different mode starts a new measurement. */
  mode: QuartzMode;
  /**
   * Motor gate in seconds: 0 = run until two inhibition events, else
   * 3..960 (10 and 60 are the usual choices). Default 0.
   */
  gateS: number;
  /**
   * Fork frequency (Hz): 300, 341.333 (any value within 0.01 Hz of 1024/3),
   * 360, 480 or 720. Default 360.
   */
  forkHz: number;
  /**
   * Motor timestamp method. Default 'threshold'. patchState also accepts
   * 0 (threshold) / 1 (xcorr).
   */
  tsMethod: QuartzTsMethod;
  /** Motor detection threshold in noise sigmas, 4..20. Default 8. */
  kSigma: number;
}

/** Partial update of {@link QuartzState} (patchState). */
export type QuartzStatePatch = Partial<Omit<QuartzState, 'tsMethod'>> & {
  /** 'threshold' | 'xcorr', or 0 / 1. */
  tsMethod?: QuartzTsMethod | 0 | 1;
};

/** Full DSP snapshot from getState / patchState result. */
export interface DspState {
  bypass: boolean;
  blocks: DspBlocksState;
  /** Quartz settings; only when `hello.caps.quartz` is true. */
  quartz?: QuartzState;
}

/** Partial update for patchState (JSON Merge Patch style). */
export type DspStatePatch = {
  bypass?: boolean;
  /**
   * Quartz settings. Numeric settings and tsMethod are applied before
   * mode. Rejected values reply `badParam` naming the field.
   */
  quartz?: QuartzStatePatch;
  blocks?: {
    [K in keyof DspBlocksState]?: {
      enabled?: boolean;
      params?: Partial<NonNullable<DspBlocksState[K]> extends BlockState<infer P> ? P : never>;
    };
  };
};

export interface DspMetrics {
  prePeak: number;
  postPeak: number;
  preRms: number;
  postRms: number;
  noiseFloor: number;
  agcGainDb: number;
  matchScore: number;
  clipCount: number;
  limiterCount: number;
}

/** Core-0 DSP block + main-loop timing (getTiming / timing notify). */
export interface DspTiming {
  dspBlockUsLast: number;
  dspBlockUsMean: number;
  dspBlockUsMax: number;
  dspBlockBudgetUs: number;
  dspBlockSamples: number;
  dspBlocks: number;
  loopUsLast: number;
  loopUsMean: number;
  loopUsMax: number;
  loopCount: number;
  idleUsPerS: number;
  loopBusyPct: number;
  underruns: number;
  shortWrites: number;
  ringMin: number;
  ringMax: number;
  /** Quartz engine time per capture block (µs); 0 without PCM1808. */
  quartzBlockUsLast: number;
  quartzBlockUsMean: number;
  quartzBlockUsMax: number;
  /** Quartz engine calls since reset. */
  quartzBlocks: number;
}

export interface DspSpectrumPeak {
  hz: number;
  mag: number;
}

export interface DspSpectrum {
  valid: boolean;
  fftN: number;
  sourceHz: number;
  nPeaks: number;
  peaks: DspSpectrumPeak[];
}

export interface DspMatched {
  score: number;
  templatePeak: number;
  templateLen: number;
  learned: boolean;
}

/** Instant or rolling-average view of rate / beat error / amplitude. */
export interface DspBeatTimingWindow {
  rateSDay: number;
  beatErrorMs: number;
  amplitudeDeg: number;
  /** Instant: 0 or 1. Rolling: ticks contributing in the window. */
  samples: number;
  rateValid: boolean;
  beatErrorValid: boolean;
  amplitudeValid: boolean;
}

export interface DspBeatRate {
  vphRaw: number;
  vphNearest: number;
  confidence: number;
  locked: boolean;
  intervalSamples: number;
  eventsSeen: number;
  intervalsHeld: number;
  /** Analysis sample rate used for all timing math (Hz). */
  sourceHz: number;
  /** Measured capture frame rate (PCM1808), 0 where fixed by design. */
  sourceHzMeasured: number;
  /** False while the capture rate is unverified; no lock or timing then. */
  timebaseOk: boolean;
  /** Ticks in the least-squares rate fit (0 = no fit yet). */
  ratePoints: number;
  /** RMS unlock-time scatter about the fit (µs). */
  jitterUs: number;
  /** Sample-clock correction applied to all timing (ppm, + = board fast). */
  calPpm: number;
  /** Where calPpm came from. */
  calSource: TimebaseSource;
  /** Instant daily rate (s/d); same as instant.rateSDay. */
  rateSDay: number;
  /** Instant beat error (ms); same as instant.beatErrorMs. */
  beatErrorMs: number;
  /** Instant amplitude (degrees); same as instant.amplitudeDeg. */
  amplitudeDeg: number;
  /** Configured lift angle λ (degrees). */
  liftAngleDeg: number;
  /** Latest unlock→drop lift time (ms). */
  liftTimeMs: number;
  rateValid: boolean;
  beatErrorValid: boolean;
  amplitudeValid: boolean;
  /** Last computed tick set (mirrors top-level rate/BE/amp fields). */
  instant: DspBeatTimingWindow;
  /** Rolling ~1 s arithmetic mean of instant values. */
  avg1s: DspBeatTimingWindow;
  /** Integration time of {@link avgLong} in seconds (beatRate param avgS). */
  avgS: number;
  /**
   * Rolling arithmetic mean of instant values over the last {@link avgS}
   * seconds. (Replaces the fixed 10 s `avg10s` of earlier firmware.)
   */
  avgLong: DspBeatTimingWindow;
}

/** Origin of a sample-clock correction. */
export type TimebaseSource = 'none' | 'manual' | 'ntp' | 'pps';

/** getTime result: device µs timer, sampled first (calibration probe). */
export interface HidTimeResult {
  tUs: number;
  uptimeMs: number;
}

/** getTimebase / setTimebase / saveTimebase / clearTimebase result. */
export interface Timebase {
  /** Correction in effect (PPS while locked, else stored), ppm, + = board fast. */
  ppm: number;
  source: TimebaseSource;
  /** Stored calibration (RAM; `saved` says whether flash matches). */
  calPpm: number;
  calSource: Exclude<TimebaseSource, 'pps'>;
  /** UNIX time the stored value was measured, 0 = unknown. */
  dateUnix: number;
  /** Measurement length (s), 0 = unknown / manual. */
  durationS: number;
  saved: boolean;
  /**
   * Minimum |ppm| change versus the last log entry that appends a
   * 'delta' timebase-log entry (0.05..10). Set via setTimebase.logDeltaPpm.
   */
  logDeltaPpm: number;
  /** A timebase-log flash write is deferred (streaming or quartz measuring). */
  logPending: boolean;
  pps: {
    enabled: boolean;
    locked: boolean;
    ppm: number;
    /** Seconds in the current average (≤ 64). */
    window: number;
    good: number;
    rejects: number;
    /** Pulses ignored because GPS NMEA reported no fix (GPS_UART_RX builds). */
    noFix: number;
  };
  /** GPS NMEA status; `enabled` false unless built with GPS_UART_RX. */
  gps: {
    enabled: boolean;
    /** Valid fix now: RMC 'A', and GGA quality > 0 with ≥ GPS_MIN_SATS (default 4). */
    fix: boolean;
    /** GGA fix quality, 0 = none / no recent GGA. */
    quality: number;
    /** Satellites in use (GGA). */
    sats: number;
    /** HDOP, 0 = unknown. */
    hdop: number;
    /** Current UTC from GPS (UNIX s), 0 = never received. */
    utcUnix: number;
    /** Since the last RMC sentence (ms), -1 = never. */
    ageMs: number;
    sentences: number;
    /** Bad checksum / overlong / UART framing errors. */
    errors: number;
    /** UART FIFO overruns (characters lost). */
    overruns: number;
  };
}

/** Quartz measurement state (HID_CONTROL_PLANE.md §5.3). */
export type QuartzStateName =
  | 'idle'
  | 'waitTimebase'
  | 'unsupportedRate'
  | 'acquiring'
  | 'measuring'
  | 'done';

/**
 * What the motor result can claim about inhibition (handoff §5).
 * - `none`: fewer than three pulses, no rate.
 * - `no_jump_long_gate`: no jump, gate ≥ 2 × the longest expected period:
 *   "Rate (no inhibition detected)".
 * - `may_include_inhibition`: no jump in a short gate: raw rate, "may
 *   include inhibition"; offer a longer gate.
 * - `need_one_more_cycle`: one jump: raw rate, "need one more cycle".
 * - `correction_pending`: two jumps, waiting for the next same-polarity
 *   pulse (transient).
 * - `corrected`: ≥ 2 jumps: the corrected rate is the headline.
 */
export type QuartzInhibitionStatus =
  | 'none'
  | 'no_jump_long_gate'
  | 'may_include_inhibition'
  | 'need_one_more_cycle'
  | 'correction_pending'
  | 'corrected';

/** LCD measurement progress (handoff §6). */
export type QuartzLcdStatus =
  | 'detecting_period'
  | 'no_pattern'
  | 'unsupported_rate'
  | 'tracking';

/**
 * Stepper-motor detail (handoff §4/§5). Rates are s/d, positive = gaining;
 * they are 0 while `valid` is false. "Without inhibition" = the raw
 * oscillator rate, NOT "uncalibrated"; `*Uncal*` = board clock, no
 * sample-clock correction.
 */
export interface DspQuartzMotor {
  /** A fit exists (three or more pulses). */
  valid: boolean;
  /** Pulses detected since the measurement started. */
  pulses: number;
  /** Pulses in the fit (≤ 2048, sliding window). */
  fitPulses: number;
  /** Step interval snapped to 1/2/2.5/5/10/20/30/60 s (s). */
  stepS: number;
  /** First to last fitted pulse (s). */
  spanS: number;
  /**
   * Rate WITH inhibition, calibrated (s/d): the motor headline (what the
   * wearer sees). Equals {@link rateWithoutSDay} unless `inhibition` is
   * 'corrected'.
   */
  rateWithSDay: number;
  /** Rate with inhibition against the board clock (s/d). */
  rateWithUncalSDay: number;
  /** Raw rate WITHOUT inhibition (the oscillator), calibrated (s/d). */
  rateWithoutSDay: number;
  /** Raw rate without inhibition against the board clock (s/d). */
  rateWithoutUncalSDay: number;
  /** Handoff §5 status; always surface it to the user. */
  inhibition: QuartzInhibitionStatus;
  /** Two or more inhibition events seen. */
  inhibitionPeriodValid: boolean;
  /** Median spacing of inhibition events (s); 0 until valid. */
  inhibitionPeriodS: number;
  /** Inhibition jumps (> 20 µs) seen. */
  events: number;
  /** Whole inhibition cycles in the corrected rate. */
  cyclesUsed: number;
  /** Size of the most recent jump (µs); 0 if none. */
  jumpLastUs: number;
  /** jumpLastUs / 122.07 µs, rounded: the "n" deleted at 8192 Hz. */
  jumpLastN: number;
  /** Mean jump size (µs); 0 if none. */
  jumpMeanUs: number;
  /** Mean jump in 8192 Hz pulses, rounded. */
  jumpMeanN: number;
  /** RMS fit residual (µs): per-pulse timing jitter. */
  residualRmsUs: number;
  /** Constant edge offset between the two coil halves (µs). */
  polarityOffsetUs: number;
  /** A LEFT sample hit the ADC rail since the start. */
  clipped: boolean;
  /** Pulses whose group hit the rail. */
  clippedPulses: number;
  /** Detection / correction bursts logged (fault indication). */
  extraPulses: number;
  /** Noise sigma after the 400 Hz high-pass (ADC codes); 0 until calibrated. */
  noiseSigma: number;
  /** Detection threshold, kSigma × noiseSigma (ADC codes). */
  threshold: number;
  /** Median peak / noiseSigma over the recent pulses (≤ 16); 0 with none. */
  peakSnr: number;
  /** Weak signal (≈ < 30 µV EMF): tell the user to reposition the watch. */
  hintWeakSignal: boolean;
  /** Clipping: tell the user to open JP1. */
  hintClipped: boolean;
  /**
   * Fitted span (s) a jump-free measurement must reach before `inhibition`
   * becomes 'no_jump_long_gate' (2 × the longest expected period). Engine
   * constant, filled in every motor state.
   */
  noJumpSpanS: number;
  /** Interval excess (µs) that counts as an inhibition jump. Engine constant. */
  jumpThresholdUs: number;
  /**
   * The fit is replaying its pulse store; the rate fields are the last good
   * result. Transient; show a "refitting" note and keep the rates.
   */
  rebuilding: boolean;
}

/** LCD detail (handoff §6). Rates are s/d, positive = gaining. */
export interface DspQuartzLcd {
  /** A phase-slope rate exists. */
  valid: boolean;
  /** Progress. */
  status: QuartzLcdStatus;
  /** Pattern rate 16 / 32 / 64 / 128 Hz; 0 until known. */
  f0Hz: number;
  /** Harmonic numbers k tracked (≤ 4). */
  harmonics: number[];
  /** 0.5 s phase blocks in the fit. */
  blocks: number;
  /**
   * Rate, calibrated (s/d). The LCD runs before the inhibition stage, so
   * this is the oscillator rate and may differ from a motor rate.
   */
  rateSDay: number;
  /** Rate against the board clock (s/d). */
  rateUncalSDay: number;
  /** Pattern searches restarted after 'no_pattern'. */
  retries: number;
  /** A RIGHT sample hit the ADC rail since the start. */
  clipped: boolean;
}

/** Tuning-fork detail (handoff §7). Rates are s/d, positive = gaining. */
export interface DspQuartzFork {
  /** A phase-slope rate exists (≥ 2 blocks). */
  valid: boolean;
  /** Nominal fork frequency (Hz). */
  fNomHz: number;
  /** 0.25 s phase blocks in the fit. */
  blocks: number;
  /** Rate, calibrated (s/d). */
  rateSDay: number;
  /** Rate against the board clock (s/d). */
  rateUncalSDay: number;
  /** A LEFT sample hit the ADC rail since the start. */
  clipped: boolean;
}

/**
 * Quartz snapshot (getQuartz, quartzRestart, `quartz` notify). Exactly one
 * of motor / lcd / fork is an object (the active mode's); the others are
 * null, and all three are null in mechanical mode. Non-finite values are
 * sent as 0: check the valid flags.
 */
export interface DspQuartz {
  /** Active mode. */
  mode: QuartzMode;
  /** Measurement state. */
  state: QuartzStateName;
  /**
   * Current measurement (uint32). +1 on every (re)start, never repeats while
   * the firmware runs (wraps after 2^32); a change means drop history.
   */
  measurementId: number;
  /** Why the state is what it is; '' when nothing to add. Display only. */
  reason: string;
  /** Analysis sample rate (Hz); 48000 is required to measure. */
  sourceHz: number;
  /** Capture rate verified. */
  timebaseOk: boolean;
  /** Signal time since the measurement started (s); frozen in 'done'. */
  elapsedS: number;
  /** Motor gate in effect (s); 0 = until two inhibition events. */
  gateS: number;
  /** Fork frequency in effect (Hz). */
  forkHz: number;
  /** Motor timestamp method in effect. */
  tsMethod: QuartzTsMethod;
  /** Motor detection threshold in effect (noise sigmas). */
  kSigma: number;
  /** Sample-loss restarts since the mode was selected. */
  sampleLossRestarts: number;
  /**
   * Headline rate, calibrated (s/d, positive = gaining): motor = rate with
   * inhibition, LCD / fork = the phase-slope rate.
   */
  rateSDay: number;
  /** rateSDay is meaningful. */
  rateValid: boolean;
  /**
   * Sample-clock correction applied (ppm, + = board fast); in 'done' the
   * value the result was frozen with.
   * calibrated ≈ uncal + 0.0864 × calPpm (s/d).
   */
  calPpm: number;
  /** Where calPpm came from. */
  calSource: TimebaseSource;
  /** False when calSource is 'none': show every rate as UNCALIBRATED. */
  calibrated: boolean;
  /** Motor detail, or null outside motor mode. */
  motor: DspQuartzMotor | null;
  /** LCD detail, or null outside LCD mode. */
  lcd: DspQuartzLcd | null;
  /** Tuning-fork detail, or null outside fork mode. */
  fork: DspQuartzFork | null;
}

/** One motor pulse (getQuartzPulses element, `quartzPulse` notify). */
export interface DspQuartzPulse {
  /** {@link DspQuartz.measurementId} of the pulse's measurement. */
  measurementId: number;
  /** 0-based pulse number within the measurement; key by (measurementId, sequence). */
  sequence: number;
  /** First sample over threshold (absolute source index, uint64 string). */
  sampleIndex: SampleIndexString;
  /**
   * Leading-edge timestamp, s since the measurement started (0.1 µs
   * resolution). xcorr timestamps carry a constant offset that cancels in
   * rates.
   */
  tS: number;
  /** |y| at the leading-spike peak after the 400 Hz high-pass (codes). */
  peak: number;
  /** peak / noise sigma at detection. */
  snr: number;
  /** Sign of the leading spike; alternates every step. */
  polarity: 1 | -1;
  /** Detection / correction bursts in its group. */
  extras: number;
  /** A raw sample reached the ADC rail in its group. */
  clipped: boolean;
  /** Its 50 ms quiet gap has passed (always true in notifies). */
  closed: boolean;
}

/** getQuartzPulses result: newest first, at most caps.quartzPulsesMax. */
export interface DspQuartzPulses {
  /** Recent pulses, newest first; empty outside motor mode. */
  pulses: DspQuartzPulse[];
}

export interface DspEvent {
  startSample: SampleIndexString;
  peakSample: SampleIndexString;
  /** First pulse (unlock) landmark for rate / beat error. */
  unlockSample: SampleIndexString;
  /** Loudest pulse (drop) landmark for amplitude lift. */
  dropSample: SampleIndexString;
  durationSamples: number;
  riseSamples: number;
  /** dropSample - unlockSample when ordered; else 0. */
  liftSamples: number;
  peakAbs: number;
  noiseFloor: number;
  snrDb: number;
  energy: number;
  confidence: number;
}

export interface DspError {
  block: string;
  reason: DspFaultReason;
  uptimeMs: number;
  sampleIndex: SampleIndexString;
}

/** Capture mode of the device (A1). */
export type DeviceMode = 'usbMic' | 'standalone';

/** USB audio stream state reported in {@link DeviceStatus.streamState}. */
export type DeviceStreamState =
  | 'unconfigured'
  | 'idle'
  | 'prefill'
  | 'active'
  | 'suspended'
  | 'error';

/**
 * Device mode / run-control status (getDevice, setDevice, start, stop).
 * Capture runs while `usbStreaming || runRequested`.
 */
export interface DeviceStatus {
  /** 'usbMic': a host streams the audio. 'standalone': device analyses alone. */
  mode: DeviceMode;
  /** Capture + analysis currently running (for any reason). */
  running: boolean;
  /** Host / CLI start latch; cleared by `stop`. Set at boot in standalone. */
  runRequested: boolean;
  /** The USB host has the audio stream open. */
  usbStreaming: boolean;
  /** USB audio stream state. */
  streamState: DeviceStreamState;
  /** Flash holds this mode. */
  saved: boolean;
}

/** Outcome of one beat in {@link DspBeat.status}. */
export type DspBeatStatus = 'accepted' | 'started' | 'rejected';

/**
 * One tick as seen by the beat-timing fit (getBeats, `beats` notify): the
 * raw material of a tick-tock (paper-strip) chart. One record per tick once
 * the A/h lock is established; none while unlocked. Ring of 64 on the device.
 *
 * Each `gridId` has a fixed origin (the `'started'` tick: k = 0, uUs = 0)
 * and the nominal half-period `nominalUs`. `uUs` is the arrival time minus
 * the nominal-grid time, with the sample-clock (timebase) correction already
 * applied; it is NOT relative to the fitted line. A gaining watch has ticks
 * arriving early, so `uUs` DECREASES over time:
 * `rate_s_day = -(du/dt) * 86400` with u and t in seconds. The separation of
 * the even-k and odd-k clouds is the beat error. If the timebase ppm changes
 * mid-grid, u steps: treat that as a new grid. `'rejected'` ticks are
 * recorded too (k = nearest grid slot, uUs = distance to it, up to about
 * half a beat) but are not in the fit.
 */
export interface DspBeat {
  /**
   * Monotonic per stream; restarts at 0 on a stream reset (a `seq` that goes
   * backwards, or the ring being re-seeded, means a new stream).
   */
  seq: number;
  /** Bumps whenever the beat fit restarts its grid (`status` 'started'). */
  gridId: number;
  /** Integer part of the unlock sample time (uint64 string). */
  sampleIndex: SampleIndexString;
  /** Unlock time (s of signal = sample / sourceHz), sub-sample precision. */
  tS: number;
  /** Grid index in half-period units since the grid origin. */
  k: number;
  /** k & 1: 0 = "tick", 1 = "tock" (arbitrary but stable within a grid). */
  parity: 0 | 1;
  /** Residual t - (origin + k * nominal) in µs, timebase-corrected; see the interface comment. */
  uUs: number;
  /** Nominal half-period (µs). */
  nominalUs: number;
  /** Per-beat amplitude (degrees); 0 if invalid. */
  amplitudeDeg: number;
  amplitudeValid: boolean;
  /** Unlock-to-drop lift time (ms); 0 if unavailable. */
  liftMs: number;
  status: DspBeatStatus;
}

/** getBeats result: newest first. */
export interface DspBeats {
  beats: DspBeat[];
}

/**
 * `beats` notify payload: oldest first, at most 8 per notify (fewer when
 * they do not fit one message; the rest follow in the next notify).
 */
export type BeatsNotifyData = DspBeats;

/** Pipeline node group (A3). */
export type PipelineGroup = 'analysis' | 'process' | 'source' | 'sink' | 'quartz';

/**
 * One node of the DSP data-flow graph advertised in `hello.caps.pipeline`.
 * Includes pseudo-nodes `input` (source), `pcm24` and `usb` (sinks) and
 * `quartz` (only when caps.quartz), plus every block key in caps.blocks
 * except `bypass`.
 */
export interface PipelineNode {
  /** Node key: a {@link DspBlockKey} or one of the pseudo-nodes. */
  key: string;
  group: PipelineGroup;
  /** Position in the real processing order. */
  order: number;
  /** Keys of the nodes this one reads from. */
  inputs: string[];
}

/** GNSS constellation code in a {@link GpsSat} tuple (index 0). */
export type GpsSystem = 'GP' | 'GL' | 'GA' | 'BD' | 'QZ' | '??';

/**
 * One tracked satellite as a compact tuple (so that 32 fit one 2 KB
 * message), in the column order of {@link GpsInfo.satCols}:
 * `[sys, prn, el, az, snr, used]`.
 * - `sys`: constellation code (`'??'` unknown).
 * - `prn`: normalised PRN.
 * - `el`: elevation (deg), -1 unknown.
 * - `az`: azimuth (deg), -1 unknown.
 * - `snr`: C/N0 (dB-Hz), -1 = not tracked.
 * - `used`: 1 when the satellite is in the position solution, else 0
 *   (a number, not a boolean).
 */
export type GpsSat = [sys: GpsSystem, prn: number, el: number, az: number, snr: number, used: 0 | 1];

/**
 * GPS detail (getGps). `badRequest` "GPS not built in" when caps.gps is
 * false. At most 32 satellites; entries older than 5 s are dropped.
 */
export interface GpsInfo {
  fix: boolean;
  /** GGA fix quality. */
  quality: number;
  /** UTC (UNIX s), 0 = never received. */
  utcUnix: number;
  /** Since the last RMC sentence (ms), -1 = never. */
  ageMs: number;
  satsUsed: number;
  hdop: number;
  pdop: number;
  vdop: number;
  lat: number;
  lon: number;
  /** Altitude (m). */
  altM: number;
  /** lat / lon / altM are meaningful. */
  posValid: boolean;
  /** Column names of every {@link GpsSat} tuple, always `sys, prn, el, az, snr, used`. */
  satCols: ['sys', 'prn', 'el', 'az', 'snr', 'used'];
  /** Up to 32 satellites; those not heard for 5 s are dropped. */
  sats: GpsSat[];
}

/** Why a timebase-log entry was appended. */
export type TimebaseLogReason = 'save' | 'delta' | 'monthly';

/** Origin of a log entry (a subset of {@link TimebaseSource}). */
export type TimebaseLogSource = 'manual' | 'ntp' | 'pps';

/** One persisted timebase measurement (flash ring of 24). */
export interface TimebaseLogEntry {
  /** UNIX time of the measurement. */
  dateUnix: number;
  /** Correction (ppm, + = board fast). */
  ppm: number;
  source: TimebaseLogSource;
  /** Measurement length (s). */
  durationS: number;
  reason: TimebaseLogReason;
}

/**
 * getTimebaseLog / clearTimebaseLog result: one page of the log. A full ring
 * of 24 needs at most 2 pages; repeat with `offset += entries.length` until
 * `offset + entries.length >= total`.
 */
export interface TimebaseLog {
  /** Entries `[offset, offset + entries.length)`, oldest first. */
  entries: TimebaseLogEntry[];
  /** Entries in the whole log (0..caps.timebaseLogMax, 24). */
  total: number;
  /** Index of the first entry of this page (the request's `offset`, clamped to `total`). */
  offset: number;
  /** A log flash write is deferred. */
  pending: boolean;
  /** Current append threshold (ppm). */
  logDeltaPpm: number;
  /** UNIX time of the last log write, 0 = never. */
  lastWriteUnix: number;
}

export interface HidHelloCaps {
  maxJsonBytes: number;
  metricsHzMax: number;
  beatHzMax: number;
  timingHzMax: number;
  timing: boolean;
  /**
   * Quartz timing available (PCM1808 builds). When false only mechanical
   * mode exists and getState has no `quartz`.
   */
  quartz: boolean;
  /** Max subscribe.quartzHz. */
  quartzHzMax: number;
  /** Max getQuartzPulses.max (the device logs 16; replies carry ≤ 8). */
  quartzPulsesMax: number;
  blocks: DspBlockKey[];
  /** getDevice / setDevice / start / stop exist. Absent = false. */
  deviceMode?: boolean;
  /** getBeats + subscribe.beats exist. Absent = false. */
  beats?: boolean;
  /** Max getBeats.max (8). */
  beatsMax?: number;
  /** getGps exists AND the build has GPS_UART_RX (false: getGps replies badRequest). */
  gps?: boolean;
  /** The build has PPS_GPIO. */
  pps?: boolean;
  /** getTimebaseLog / clearTimebaseLog exist. */
  timebaseLog?: boolean;
  /** Timebase-log ring size (24). */
  timebaseLogMax?: number;
  /** DSP data-flow graph. Absent on older firmware. */
  pipeline?: PipelineNode[];
}

export interface HidHelloResult {
  protocolVersion: HidProtocolVersion;
  fwVersion: string;
  board: string;
  source: string;
  vid: number;
  pid: number;
  caps: HidHelloCaps;
}

export interface HidSubscription {
  metricsHz: number;
  beatHz: number;
  timingHz: number;
  /** Periodic `quartz` notify (Hz); 0 = off, max caps.quartzHzMax (5). */
  quartzHz: number;
  events: boolean;
  errors: boolean;
  /** Push each new motor pulse as a `quartzPulse` notify. */
  quartzPulses: boolean;
  /**
   * Push new beats as `beats` notifies (at most 8 per notify, oldest first).
   * Current firmware always sends it; typed optional so that a host can
   * still talk to older firmware, which omits it.
   */
  beats?: boolean;
}

/** Partial subscribe body; omitted fields leave prior values unchanged. */
export type HidSubscribePatch = Partial<HidSubscription>;

export type HidOp =
  | 'hello'
  | 'ping'
  | 'getState'
  | 'patchState'
  | 'getMetrics'
  | 'getTiming'
  | 'resetTiming'
  | 'getBeat'
  | 'getSpectrum'
  | 'getMatched'
  | 'getEvents'
  | 'getErrors'
  | 'clearErrors'
  | 'subscribe'
  | 'getTime'
  | 'getTimebase'
  | 'setTimebase'
  | 'saveTimebase'
  | 'clearTimebase'
  | 'getQuartz'
  | 'getQuartzPulses'
  | 'quartzRestart'
  | 'getDevice'
  | 'setDevice'
  | 'start'
  | 'stop'
  | 'getBeats'
  | 'getGps'
  | 'getTimebaseLog'
  | 'clearTimebaseLog';

interface HidEnvelopeBase {
  v: HidProtocolVersion;
}

export interface HidRequestBase extends HidEnvelopeBase {
  id: number;
  op: HidOp;
}

export interface HidHelloRequest extends HidRequestBase {
  op: 'hello';
}

export interface HidPingRequest extends HidRequestBase {
  op: 'ping';
  /**
   * Echoed back as a full 64-bit signed integer, so `Date.now()` round-trips;
   * default 0.
   */
  t?: number;
}

export interface HidGetStateRequest extends HidRequestBase {
  op: 'getState';
}

export interface HidPatchStateRequest extends HidRequestBase {
  op: 'patchState';
  patch: DspStatePatch;
}

export interface HidGetMetricsRequest extends HidRequestBase {
  op: 'getMetrics';
}

export interface HidGetTimingRequest extends HidRequestBase {
  op: 'getTiming';
}

export interface HidResetTimingRequest extends HidRequestBase {
  op: 'resetTiming';
}

export interface HidGetBeatRequest extends HidRequestBase {
  op: 'getBeat';
}

/**
 * Result: `{ events: DspEvent[] }`, newest first. Events are 216..345 bytes
 * each, so the reply holds as many of the requested events as fit one 2048
 * byte message (always the newest ones, at least 5): it may carry fewer than
 * `limit`, never more, and is never `tooLarge`.
 */
export interface HidGetEventsRequest extends HidRequestBase {
  op: 'getEvents';
  /** Clamped to 1..32; default 8. */
  limit?: number;
}

export interface HidGetErrorsRequest extends HidRequestBase {
  op: 'getErrors';
}

export interface HidClearErrorsRequest extends HidRequestBase {
  op: 'clearErrors';
}

export interface HidSubscribeRequest extends HidRequestBase, HidSubscribePatch {
  op: 'subscribe';
}

export interface HidGetTimeRequest extends HidRequestBase {
  op: 'getTime';
}

export interface HidGetTimebaseRequest extends HidRequestBase {
  op: 'getTimebase';
}

export interface HidSetTimebaseRequest extends HidRequestBase {
  op: 'setTimebase';
  /**
   * -1000..1000, + = board clock fast. Optional ONLY when `logDeltaPpm` is
   * present: then the stored calibration is left untouched and only the log
   * threshold changes. At least one of `ppm` / `logDeltaPpm` is required
   * (`badRequest` "missing ppm" otherwise).
   */
  ppm?: number;
  /**
   * Default 'manual'. Used only with `ppm`. Matched case-insensitively;
   * 'pps' is refused (it is measured, not set). A `ppm` of 0 with 'manual'
   * is stored as calSource 'none'.
   */
  source?: 'none' | 'manual' | 'ntp';
  /** 0..4294967295; 0 = GPS UTC when known, else 0. Used only with `ppm`. */
  dateUnix?: number;
  /** 0..4294967295. Used only with `ppm`. */
  durationS?: number;
  /**
   * Timebase-log append threshold (ppm), 0.05..10, persisted to flash
   * (`badParam` out of range; `busy` while the audio stream is open;
   * `flashError`). Skipped without a write when equal to the current value.
   */
  logDeltaPpm?: number;
}

/** Fails with `busy` while audio is streaming, `flashError` on write failure. */
export interface HidSaveTimebaseRequest extends HidRequestBase {
  op: 'saveTimebase';
}

export interface HidClearTimebaseRequest extends HidRequestBase {
  op: 'clearTimebase';
}

/** Result: {@link DspQuartz}. */
export interface HidGetQuartzRequest extends HidRequestBase {
  op: 'getQuartz';
}

/** Result: {@link DspQuartzPulses}. */
export interface HidGetQuartzPulsesRequest extends HidRequestBase {
  op: 'getQuartzPulses';
  /** 1..caps.quartzPulsesMax (8); default the maximum. */
  max?: number;
}

/**
 * Starts a new measurement in the active mode. Result: {@link DspQuartz}.
 * `badRequest` in mechanical mode or without quartz.
 */
export interface HidQuartzRestartRequest extends HidRequestBase {
  op: 'quartzRestart';
}

/** Result: {@link DeviceStatus}. */
export interface HidGetDeviceRequest extends HidRequestBase {
  op: 'getDevice';
}

/**
 * Persists the mode to flash. Result: {@link DeviceStatus}. `badParam` if
 * not usbMic / standalone; `busy` while the USB audio stream is open.
 */
export interface HidSetDeviceRequest extends HidRequestBase {
  op: 'setDevice';
  mode: DeviceMode;
}

/** Sets runRequested. Result: {@link DeviceStatus}. `badRequest` if the backend is unhealthy. */
export interface HidStartRequest extends HidRequestBase {
  op: 'start';
}

/** Clears runRequested. Result: {@link DeviceStatus}. */
export interface HidStopRequest extends HidRequestBase {
  op: 'stop';
}

/**
 * Result: {@link DspBeats}, newest first. The reply holds as many of the
 * requested beats as fit one message (fewer than `max` is possible).
 */
export interface HidGetBeatsRequest extends HidRequestBase {
  op: 'getBeats';
  /** 1..caps.beatsMax (8), default 8; out-of-range values are clamped. */
  max?: number;
}

/** Result: {@link GpsInfo}. `badRequest` "GPS not built in" when caps.gps is false. */
export interface HidGetGpsRequest extends HidRequestBase {
  op: 'getGps';
}

/** Result: {@link TimebaseLog} (one page). */
export interface HidGetTimebaseLogRequest extends HidRequestBase {
  op: 'getTimebaseLog';
  /** First entry of the page, 0 = oldest (default 0; negative is 0). */
  offset?: number;
}

/** Writes flash (`busy` while the audio stream is open). Result: {@link TimebaseLog} (empty, offset 0). */
export interface HidClearTimebaseLogRequest extends HidRequestBase {
  op: 'clearTimebaseLog';
}

/** Result: {@link DspSpectrum} (spectral peaks of the analysis source). */
export interface HidGetSpectrumRequest extends HidRequestBase {
  op: 'getSpectrum';
}

/** Result: {@link DspMatched} (matched-filter score and template state). */
export interface HidGetMatchedRequest extends HidRequestBase {
  op: 'getMatched';
}

export type HidRequest =
  | HidHelloRequest
  | HidPingRequest
  | HidGetStateRequest
  | HidPatchStateRequest
  | HidGetMetricsRequest
  | HidGetTimingRequest
  | HidResetTimingRequest
  | HidGetBeatRequest
  | HidGetSpectrumRequest
  | HidGetMatchedRequest
  | HidGetEventsRequest
  | HidGetErrorsRequest
  | HidClearErrorsRequest
  | HidSubscribeRequest
  | HidGetTimeRequest
  | HidGetTimebaseRequest
  | HidSetTimebaseRequest
  | HidSaveTimebaseRequest
  | HidClearTimebaseRequest
  | HidGetQuartzRequest
  | HidGetQuartzPulsesRequest
  | HidQuartzRestartRequest
  | HidGetDeviceRequest
  | HidSetDeviceRequest
  | HidStartRequest
  | HidStopRequest
  | HidGetBeatsRequest
  | HidGetGpsRequest
  | HidGetTimebaseLogRequest
  | HidClearTimebaseLogRequest;

export interface HidSuccessResponse<T = unknown> extends HidEnvelopeBase {
  id: number;
  ok: true;
  result: T;
}

export interface HidErrorBody {
  code: HidErrorCode;
  message: string;
}

export interface HidErrorResponse extends HidEnvelopeBase {
  id: number;
  ok: false;
  error: HidErrorBody;
}

export type HidResponse<T = unknown> = HidSuccessResponse<T> | HidErrorResponse;

export interface HidNotifyBase extends HidEnvelopeBase {
  notify: true;
}

export interface HidMetricsNotify extends HidNotifyBase {
  op: 'metrics';
  data: DspMetrics;
}

export interface HidTimingNotify extends HidNotifyBase {
  op: 'timing';
  data: DspTiming;
}

export interface HidBeatNotify extends HidNotifyBase {
  op: 'beat';
  data: DspBeatRate;
}

/**
 * New events, oldest first (like `beats` / `quartzPulse`). A notify carries as
 * many unsent events as fit one message; the rest follow in later notifies.
 * Events evicted from the 32-entry ring before they could be sent are lost.
 */
export interface HidEventNotify extends HidNotifyBase {
  op: 'event';
  data: { events: DspEvent[] };
}

export interface HidErrorNotify extends HidNotifyBase {
  op: 'error';
  data: DspError;
}

/** Periodic quartz snapshot (subscribe.quartzHz). */
export interface HidQuartzNotify extends HidNotifyBase {
  op: 'quartz';
  data: DspQuartz;
}

/** One new motor pulse, once its group closed (subscribe.quartzPulses). */
export interface HidQuartzPulseNotify extends HidNotifyBase {
  op: 'quartzPulse';
  data: DspQuartzPulse;
}

/** New beats, oldest first, at most 8 per notify (subscribe.beats). */
export interface HidBeatsNotify extends HidNotifyBase {
  op: 'beats';
  data: BeatsNotifyData;
}

export type HidNotify =
  | HidMetricsNotify
  | HidTimingNotify
  | HidBeatNotify
  | HidEventNotify
  | HidErrorNotify
  | HidQuartzNotify
  | HidQuartzPulseNotify
  | HidBeatsNotify;

/** Any device → host JSON message. */
export type HidInboundMessage = HidResponse | HidNotify;

/** Framing constants (WebHID buffer without report ID). */
export declare const HID_FRAME_HEADER_BYTES = 5;
export declare const HID_FRAME_MAX_CHUNK = 58;
export declare const HID_FRAME_FLAG_MORE = 0x01;
export declare const HID_FRAME_FLAG_NOTIFY_HINT = 0x02;

/** Result of `ping`. */
export interface HidPingResult {
  /** Echo of the request `t`. */
  t?: number;
  uptimeMs: number;
}

/**
 * Typed request body per op: the request type with the envelope fields
 * (`v`, `id`, `op`) removed. Use with {@link HidResultMap} to type a generic
 * `request<Op>(op, body)` helper.
 */
export type HidRequestBodyMap = {
  [R in HidRequest as R['op']]: Omit<R, 'v' | 'id' | 'op'>;
};

/** Typed `result` payload per op. */
export interface HidResultMap {
  hello: HidHelloResult;
  ping: HidPingResult;
  getState: DspState;
  patchState: DspState;
  getMetrics: DspMetrics;
  getTiming: DspTiming;
  resetTiming: { cleared: true };
  getBeat: DspBeatRate;
  getSpectrum: DspSpectrum;
  getMatched: DspMatched;
  getEvents: { events: DspEvent[] };
  getErrors: { errors: DspError[] };
  clearErrors: { cleared: true };
  subscribe: HidSubscription;
  getTime: HidTimeResult;
  getTimebase: Timebase;
  setTimebase: Timebase;
  saveTimebase: Timebase;
  clearTimebase: Timebase;
  getQuartz: DspQuartz;
  getQuartzPulses: DspQuartzPulses;
  quartzRestart: DspQuartz;
  getDevice: DeviceStatus;
  setDevice: DeviceStatus;
  start: DeviceStatus;
  stop: DeviceStatus;
  getBeats: DspBeats;
  getGps: GpsInfo;
  getTimebaseLog: TimebaseLog;
  clearTimebaseLog: TimebaseLog;
}

/** Notify `op` to `data` map. */
export interface HidNotifyDataMap {
  metrics: DspMetrics;
  timing: DspTiming;
  beat: DspBeatRate;
  event: { events: DspEvent[] };
  error: DspError;
  quartz: DspQuartz;
  quartzPulse: DspQuartzPulse;
  beats: BeatsNotifyData;
}
