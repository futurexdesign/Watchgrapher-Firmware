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
 * BLE control service and characteristics (BLE_CONTROL_PLANE.md §4.1). Lowercase,
 * as Web Bluetooth requires. Frames are the HID frames (BLE_CONTROL_PLANE.md §4.3).
 */
export declare const TIMEGRAPHER_BLE_SERVICE_UUID = 'c137e7c0-92b4-4d9e-87e0-9ba757169016';
/** Host → device (Write, WriteWithoutResponse): one frame per ATT write. */
export declare const TIMEGRAPHER_BLE_RX_UUID = 'c137e7c1-92b4-4d9e-87e0-9ba757169016';
/** Device → host (Notify): one frame per notification. */
export declare const TIMEGRAPHER_BLE_TX_UUID = 'c137e7c2-92b4-4d9e-87e0-9ba757169016';
/**
 * Read: UTF-8 JSON of at most 64 bytes (`p`, `mtu`, `fmax`, `seg`, and on firmware
 * with large frames `txMax`: the largest device-to-host frame it uses, 244).
 */
export declare const TIMEGRAPHER_BLE_INFO_UUID = 'c137e7c3-92b4-4d9e-87e0-9ba757169016';

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
 * `linkBusy`: another link holds control (HID_CONTROL_PLANE.md §4.6); the
 * request was not dispatched.
 */
export type HidErrorCode =
  | 'unknownOp'
  | 'badRequest'
  | 'badParam'
  | 'notFound'
  | 'tooLarge'
  | 'busy'
  | 'flashError'
  | 'linkBusy'
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
  /**
   * Landmark timed for the rate fit: 0 = unlock (default), 1 = drop. The drop
   * is timed by a learned beat template (16 beats per side, then 1/32 updates)
   * and gives much lower jitter; the fit runs on unlock times until the
   * templates are learned, then hands over to drop times (see
   * {@link DspBeatRate.timingSource}).
   * Beat error then comes from drop times and includes any difference in lift
   * between the two sides. Changing it restarts the fit and the averages.
   */
  timingLandmark?: 0 | 1;
  /**
   * Seconds, 0..60 (default 0). After a cut the first `settleS` s of the new
   * stretch do not enter the rolling means ({@link DspBeatRate.avg1s},
   * {@link DspBeatRate.avgLong}), for rate, beat error and amplitude. The
   * instant values are not affected. Not persisted.
   */
  settleS?: number;
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

/** Instant or rolling-average view of rate / beat error / amplitude. */
export interface DspBeatTimingWindow {
  rateSDay: number;
  beatErrorMs: number;
  amplitudeDeg: number;
  /**
   * Beat-error angle (degrees) = amplitudeDeg × π × vph / 7200 × beatErrorMs
   * / 1000, at the nominal vph. Rolling: mean of the per-tick angles.
   */
  beatErrorAngleDeg: number;
  /** Instant: 0 or 1. Rolling: ticks contributing in the window. */
  samples: number;
  rateValid: boolean;
  beatErrorValid: boolean;
  amplitudeValid: boolean;
  /** beatErrorAngleDeg is meaningful (BE and amplitude valid, locked). */
  beatErrorAngleValid: boolean;
}

/**
 * State of the beat analysis. `listening`: no tick since the stream reset, or
 * the capture rate is unverified. `locking`: ticks seen, but no lock or the
 * current stretch spans less than 5 s. `locked`: locked and the stretch spans
 * at least 5 s (see `settled`). `noWatch`: the no-watch rule fired and no lock has held a
 * stretch of 2 s since (`locked` may already be true again, without a rate).
 */
export type BeatState = 'listening' | 'locking' | 'locked' | 'noWatch';

/**
 * Whether a timing engine runs. 'off' in `usbMic` device mode: the host does
 * the timing from the audio stream, and the device only detects ticks and the
 * beat rate (`DspBeatRate`) or measures nothing (`DspQuartz`).
 */
export type AnalysisState = 'on' | 'off';

/**
 * Why the latest stretch of beats ended. `reset` is the value after a stream
 * reset; it is not counted in `cuts`.
 */
export type BeatCutReason =
  | 'none'
  | 'reset'
  | 'gap'
  | 'jump'
  | 'rate'
  | 'clock'
  | 'lockLost'
  | 'noWatch';

export interface DspBeatRate {
  vphRaw: number;
  vphNearest: number;
  confidence: number;
  locked: boolean;
  /**
   * See {@link BeatState}. With `analysis` 'off' it follows the lock alone:
   * 'listening', 'locking' (ticks seen, no lock) or 'locked'; never 'noWatch'.
   */
  state: BeatState;
  /**
   * 'off' in `usbMic` mode (HID_CONTROL_PLANE.md §5.4): the timing engine does
   * not run. `locked`, `vphNearest`, `vphRaw`, `confidence`,
   * `intervalSamples` and `state` stay live; every `*Valid` flag is false, the
   * rate, beat-error and amplitude values are 0, `stretchS` is 0 and no beats
   * are recorded. Show "timing runs on the host" instead of the readings.
   */
  analysis: AnalysisState;
  /**
   * Cuts since the stream reset. A cut ends the stretch of beats: the next one
   * starts with an empty fit, empty rolling means and an empty amplitude
   * history. Counted only if the cut ended a stretch that had beats.
   */
  cuts: number;
  /** Reason of the latest cut (see {@link BeatCutReason}). */
  lastCut: BeatCutReason;
  /**
   * Span of the current stretch in seconds (first to latest accepted beat
   * since the last cut, corrected clock); 0 when none is open. `rateValid` and
   * `beatErrorValid` stay false until it reaches 5 (provisional readings; see
   * {@link settled}).
   */
  stretchS: number;
  /**
   * The stretch spans at least 10 s: rate and beat error are final. Between 5
   * and 10 s of stretch they are provisional and may be shown greyed. When it
   * turns true the provisional rate and beat-error samples are dropped from
   * the rolling means (`avg1s`, `avgLong`), which refill from settled
   * readings.
   */
  settled: boolean;
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
  /** RMS scatter of the timed landmark (unlock or drop) about the fit (µs). */
  jitterUs: number;
  /**
   * Landmark the fit runs on: `unlock`, or `drop` after the handover. With
   * `timingLandmark` 1 the fit starts on unlock times while the templates
   * learn, then its stored points are shifted onto the drop time base (not a
   * cut; the fit's rate is continuous).
   */
  timingSource: 'unlock' | 'drop';
  /** Handovers from unlock to drop times since the stream reset. */
  handovers: number;
  /** Beats timed at the drop by the beat template since the last stream reset. */
  dropTimedBeats: number;
  /**
   * Beats that should have been timed at the drop but fell back to the unlock
   * (template learning, correlation below 0.4, or outside the envelope ring)
   * since the last stream reset.
   */
  dropFallbacks: number;
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
  /**
   * Signed beat error (ms), positive when the Tick comes late. Tick is the side
   * that came late over the first seconds of the watch; the label holds until
   * the nominal rate changes, so the value starts positive and can turn
   * negative. Valid with beatErrorValid; beatErrorMs is its magnitude. Equals
   * beatErrorMs with timingLandmark 0.
   */
  beatErrorSignedMs: number;
  /**
   * Where the beat error comes from: `unlock` (the unlock landmark: the
   * unlock-timed fit, or the median over 2 s shape windows of the drop beat
   * error corrected by each side's unlock-to-drop time) or `drop` (the
   * drop-timed fit, while fewer than 3 windows exist or they disagree in sign).
   */
  beatErrorSource: 'unlock' | 'drop';
  /**
   * Amplitude of the Tick side (degrees): median over the 2 s shape windows of
   * the averaging span. null unless timingLandmark is 1 and a window gave this
   * side.
   */
  amplitudeTickDeg: number | null;
  /** Amplitude of the Tock side (degrees); see amplitudeTickDeg. */
  amplitudeTockDeg: number | null;
  /**
   * Shape windows of the averaging span: `used` gave an amplitude on both
   * sides (and make up avgLong.amplitudeDeg), `total` held any beat.
   */
  amplitudeWindows: { used: number; total: number };
  /** Configured lift angle λ (degrees). */
  liftAngleDeg: number;
  /** Latest unlock→drop lift time (ms). */
  liftTimeMs: number;
  /** Instant beat-error angle (degrees); same as instant.beatErrorAngleDeg. */
  beatErrorAngleDeg: number;
  rateValid: boolean;
  beatErrorValid: boolean;
  amplitudeValid: boolean;
  /** beatErrorAngleDeg is meaningful (BE and amplitude valid, locked). */
  beatErrorAngleValid: boolean;
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
   * 'off' in `usbMic` mode: nothing is measured, `state` is 'idle', `reason`
   * is "USB Audio mode" and the mode setting is kept. 'on' otherwise.
   */
  analysis: AnalysisState;
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
  /**
   * Integer part of the sample time of the timed landmark: the unlock, or the
   * drop when beatRate.timingLandmark is 1 (uint64 string).
   */
  sampleIndex: SampleIndexString;
  /** Time of that landmark (s of signal = sample / sourceHz), sub-sample precision. */
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

/**
 * Packed beats (`beatsPacked` notify, packed getBeats result): one base64
 * string (RFC 4648, padded) of a little-endian binary block, format version
 * caps.beatsPacked. Header: u8 fmt (1), u8 n (1..32), u32 seq0, u32 gridId,
 * f32 nominalUs, u64 sample0, i32 k0, then n beats of 17 bytes: u8 dSeq,
 * u8 flags (bit 0 parity, bit 1 amplitudeValid, bits 2-3 status 0 accepted /
 * 1 started / 2 rejected), u32 dSample, u8 frac (1/256 sample; tS =
 * (sampleIndex + frac / 256) / sourceHz), i16 dK, i32 u (0.1 us), u16 amp
 * (0.01 deg, 0 when invalid), u16 lift (us, saturating). Deltas are against
 * the previous beat; the first beat's deltas are 0. A batch never spans a
 * grid or nominal change. Empty string: no beats.
 */
export interface DspBeatsPacked {
  p: string;
}

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
  /** Packed beats format version (1); absent = subscribe.beatsPacked and getBeats.packed do not exist. */
  beatsPacked?: number;
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
  /** Built with BLE (`TG_BLE=1`, Pico 2 W). Absent = false. See BLE_CONTROL_PLANE.md. */
  ble?: boolean;
  /** Serial CLI built (`TG_SERIAL_CLI=1`). Absent = false. */
  cli?: boolean;
}

/** Control link a request arrived on (`hello.link`, `claim`, `controlReleased`). */
export type HidLinkKind = 'usbHid' | 'ble';

/** The link a `hello` arrived on (`hello.link`). */
export interface HidHelloLink {
  kind: HidLinkKind;
  /** Negotiated ATT MTU. Present only for `ble`. */
  mtu?: number;
  /**
   * Device-to-host frame size in bytes that applies to messages after this
   * reply: 63 unless the host sent `hello {frameMax}` on BLE (BLE_CONTROL_PLANE.md
   * §4.3a). Absent on older firmware.
   */
  frameMax?: number;
}

/**
 * Who holds the control plane (`hello.control`, `getLink.control`). Only one
 * link owns control at a time (HID_CONTROL_PLANE.md §4.6).
 */
export interface HidHelloControl {
  /** Owning link, or null when no link owns control. */
  owner: HidLinkKind | null;
  /** True when the owner is the link this request arrived on. */
  self: boolean;
  /**
   * Lease length in ms: an owner that sends no request for this long can be
   * taken over. 10000 in current firmware.
   */
  leaseMs: number;
}

export interface HidHelloResult {
  protocolVersion: HidProtocolVersion;
  fwVersion: string;
  board: string;
  source: string;
  vid: number;
  pid: number;
  caps: HidHelloCaps;
  /** The link this request arrived on. Absent on older firmware. */
  link?: HidHelloLink;
  /** Current owner of the control plane. Absent on older firmware. */
  control?: HidHelloControl;
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
  /**
   * Push new beats as `beatsPacked` notifies (at most 32 per notify, oldest
   * first); replaces `beats` while set. Needs caps.beatsPacked; typed
   * optional for older firmware.
   */
  beatsPacked?: boolean;
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
  | 'clearTimebaseLog'
  | 'claim'
  | 'getLink'
  | 'getDiag';

interface HidEnvelopeBase {
  v: HidProtocolVersion;
}

export interface HidRequestBase extends HidEnvelopeBase {
  id: number;
  op: HidOp;
}

export interface HidHelloRequest extends HidRequestBase {
  op: 'hello';
  /**
   * BLE only: asks for device-to-host frames of up to this many bytes on this
   * connection (BLE_TRANSPORT_PLAN.md §14.3). The device answers this hello
   * with the old size, then uses `min(frameMax, txMax, attMtu - 3)` (INFO
   * `txMax`, 244 on the Pico 2 W); the result's `link.frameMax` reports it.
   * Below 63 or absent keeps the current value. Ignored on USB HID.
   */
  frameMax?: number;
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
 * Result: {@link DspBeats}, newest first, or with `packed: true`
 * {@link DspBeatsPacked}, newest last. The reply holds as many of the
 * requested beats as fit one message (fewer than `max` is possible); a packed
 * reply holds the newest batch only.
 */
export interface HidGetBeatsRequest extends HidRequestBase {
  op: 'getBeats';
  /** 1..caps.beatsMax (8), default 8; out-of-range values are clamped. */
  max?: number;
  /** Return the packed form (needs caps.beatsPacked); `max` then goes up to 32. */
  packed?: boolean;
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

/**
 * Asks to own the control plane (HID_CONTROL_PLANE.md §4.6). Without
 * `takeover` it fails with `linkBusy` while another link holds a live lease.
 * With `takeover: true` it takes control at once; the previous owner gets a
 * `controlReleased` notify. Result: {@link HidClaimResult}.
 */
export interface HidClaimRequest extends HidRequestBase {
  op: 'claim';
  /** Default false. */
  takeover?: boolean;
}

/** Result of `claim`. */
export interface HidClaimResult {
  /** The link that owns control after the request (the requesting link). */
  owner: HidLinkKind;
  /** The owner before the request, or null when none. */
  previous: HidLinkKind | null;
}

/**
 * BLE link status (`getLink.ble`). Present only on builds with `TG_BLE=1`.
 * Counters are since boot unless the firmware resets them.
 */
export interface HidBleLinkStatus {
  /** BLE is turned on (`ble off` turns it off). */
  enabled: boolean;
  /** Advertised local name, `TG-XXXX`. */
  name: string;
  /** Bluetooth device address of the stand, as text. */
  address: string;
  /** Advertising is running (the stand is connectable and no central is connected). */
  advertising: boolean;
  /** A central is connected with notifications on. */
  connected: boolean;
  /** Negotiated ATT MTU (23 until the exchange completes). */
  mtu: number;
  /** Current connection interval (µs). */
  connIntervalUs: number;
  /**
   * Progress of the device's connection-interval request: 0 none yet, 1 asked
   * for 15 ms, 2 asked for 15 to 30 ms after a rejection, 3 gave up.
   */
  connParamStep: 0 | 1 | 2 | 3;
  /** Connection-parameter requests the central rejected. */
  connParamRejects: number;
  /** Frames received on RX. */
  rxFrames: number;
  /** Frames sent on TX. */
  txFrames: number;
  /** Frames dropped because the RX ring was full. */
  rxDropped: number;
  /** RX writes rejected as invalid (longer than 63 bytes, bad length). */
  rxInvalid: number;
  /** TX sends deferred by the stack (backpressure). */
  txRetries: number;
  /** Connections lost since boot. */
  disconnects: number;
  /** HCI disconnect reason of the last loss; 0 = none since boot. */
  lastDisconnectReason: number;
  /** Core running the radio: 0 (`TG_BLE_CORE=0`) or 1 (`TG_BLE_CORE=1`). */
  core: 0 | 1;
  /** Free bytes of core 1's stack (high-water mark). Present only when `core` is 1. */
  core1StackFree?: number;
}

/** Result of `getLink`: every link and who owns control. */
export interface HidLinkStatus {
  /** The USB HID link. */
  usbHid: {
    /** The USB device is enumerated and the HID interface is mounted. */
    mounted: boolean;
  };
  /** The BLE link. Absent without `TG_BLE`. */
  ble?: HidBleLinkStatus;
  /** Same shape as `hello.control`. */
  control: HidHelloControl;
}

/**
 * Asks for the link status. Result: {@link HidLinkStatus}. An owner-only op
 * (HID_CONTROL_PLANE.md §4.6): a non-owner gets `linkBusy` while another link
 * holds a live lease. Otherwise the request auto-claims control for the
 * requesting link, like any other op. To read ownership without claiming,
 * use `hello.control`.
 */
export interface HidGetLinkRequest extends HidRequestBase {
  op: 'getLink';
}

/**
 * Asks for the diagnostic counters. Result: {@link HidDiag}. Same ownership
 * rule as `getLink` (HID_CONTROL_PLANE.md §4.6): `linkBusy` for a non-owner
 * while another link holds a live lease, otherwise auto-claim. Read
 * `hello.control` to see the owner without claiming.
 */
export interface HidGetDiagRequest extends HidRequestBase {
  op: 'getDiag';
}

/**
 * Result of `getDiag`: the firmware diagnostic block (`diag_counters_t`,
 * `include/diagnostics.h`) with camelCase names. Most fields are counters
 * that only increase; the ADC, ring and PCM peak / min / max fields are
 * extrema of the current diagnostic window.
 */
export interface HidDiag {
  /** USB enumerations (bus resets seen by the device). */
  usbEnumerationCount: number;
  /** UAC2 audio streams opened by the host. */
  usbAudioStreamOpenCount: number;
  /** UAC2 audio streams closed. */
  usbAudioStreamCloseCount: number;
  /** USB suspend events. */
  usbSuspendCount: number;
  /** USB resume events. */
  usbResumeCount: number;
  /** Preamp boost rail switched on. */
  boostOnCount: number;
  /** Preamp boost rail switched off. */
  boostOffCount: number;
  /** Preamp settle waits started. */
  preampSettleStartCount: number;
  /** Preamp settle waits completed. */
  preampSettleDoneCount: number;
  /** Preamp starts aborted before settling. */
  preampStartAbortedCount: number;
  /** Total time spent suspended (ms). */
  usbSuspendDurationMs: number;
  /** Samples taken by the ADC or PCM1808 capture path. */
  adcTotalSampleCount: number;
  /** ADC conversion errors. */
  adcConversionErrorCount: number;
  /** Capture DMA buffers completed. */
  dmaCompleteCount: number;
  /** Capture DMA buffers overwritten before they were processed. */
  dmaOverrunCount: number;
  /** PCM ring reads that found it empty (USB audio starved). */
  pcmRingUnderrunCount: number;
  /** PCM ring writes that found it full (samples dropped). */
  pcmRingOverrunCount: number;
  /** USB IN audio packets sent. */
  usbInPacketCount: number;
  /** Audio samples sent in USB IN packets. */
  usbInSampleCount: number;
  /** USB IN writes that could not send a full packet. */
  usbShortWriteCount: number;
  /** Lowest PCM ring fill in the current diagnostic window (samples). */
  ringMinOccupancy: number;
  /** Highest PCM ring fill in the current diagnostic window (samples). */
  ringMaxOccupancy: number;
  /** Lowest ADC code in the current window. */
  adcMinCode: number;
  /** Highest ADC code in the current window. */
  adcMaxCode: number;
  /** Sum of ADC codes in the current window (for the mean). */
  adcMeanCodeAccum: number;
  /** Number of ADC codes summed in the current window. */
  adcMeanCodeCount: number;
  /** Learned or forced midrail (ADC counts) used for DC removal. */
  adcDcCounts: number;
  /** Max |code - dc| in the current window (ADC counts, pre-scale). */
  adcAcPeakCounts: number;
  /** Max |PCM24| after scale, before the optional LPF, in the window. */
  adcPcmPeakAbs: number;
  /** 1 when the PCM1808 clock configuration was accepted, else 0. */
  pcm1808ClockConfigOk: number;
  /** System clock frequency used for the PCM1808 clocks (Hz). */
  pcm1808ClkSysHz: number;
  /** USB clock frequency used for the PCM1808 clocks (Hz). */
  pcm1808ClkUsbHz: number;
  /** Nominal PCM1808 master clock (Hz). */
  pcm1808MclkNominalHz: number;
  /** Waits for the PCM1808 to start or settle before capture. */
  pcm1808StartupWaitCount: number;
  /** PCM1808 DMA blocks completed. */
  pcm1808DmaBlocks: number;
  /** Stereo frames received from the PCM1808 PIO. */
  pcm1808StereoFramesReceived: number;
  /** Mono samples emitted into the analysis path. */
  pcm1808MonoSamplesEmitted: number;
  /** PCM1808 DMA blocks overwritten before they were processed. */
  pcm1808DmaBlockOverruns: number;
  /** PIO FIFO stall or overflow events on the PCM1808 input. */
  pcm1808PioFifoStallOrOverflow: number;
  /** Max |sample| after scale in the window. */
  pcm1808SamplePeakAbs: number;
  /** Left-channel frames at the 24-bit full-scale or minimum code. */
  pcm1808SaturatedCount: number;
  /** Max |left24| of the raw I2S frames in the window (pre-DSP). */
  pcm1808RawLeftPeakAbs: number;
  /** Max |right24| of the raw I2S frames in the window (pre-DSP). */
  pcm1808RawRightPeakAbs: number;
  /** Last measured codec frame rate (milli-hertz; 0 = not measured yet). */
  pcm1808FrameRateMhz: number;
  /** Rate check state: 0 pending, 1 ok, 2 unsupported, 3 unknown. */
  pcm1808RateStatus: number;
  /** Accepted MCLK / fs ratio (256, 384, 512), or 0. */
  pcm1808MclkRatio: number;
  /** Sample repeat currently applied into the 48 kHz ring. */
  pcm1808Upsample: number;
  /** Captured blocks never processed; the source index skips them. */
  pcm1808BlocksLost: number;
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
  | HidClearTimebaseLogRequest
  | HidClaimRequest
  | HidGetLinkRequest
  | HidGetDiagRequest;

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

/** New beats in the packed format, oldest first (subscribe.beatsPacked). */
export interface HidBeatsPackedNotify extends HidNotifyBase {
  op: 'beatsPacked';
  data: DspBeatsPacked;
}

/**
 * Sent to the previous owner when another link takes control (`claim` with
 * `takeover`, or an auto-claim after the lease expired), if its link is still
 * connected (HID_CONTROL_PLANE.md §4.6). The host should stop subscriptions
 * and show that control moved; `to` is the new owner.
 */
export interface HidControlReleasedNotify extends HidNotifyBase {
  op: 'controlReleased';
  data: { to: HidLinkKind };
}

export type HidNotify =
  | HidMetricsNotify
  | HidTimingNotify
  | HidBeatNotify
  | HidEventNotify
  | HidErrorNotify
  | HidQuartzNotify
  | HidQuartzPulseNotify
  | HidBeatsNotify
  | HidBeatsPackedNotify
  | HidControlReleasedNotify;

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
  getBeats: DspBeats | DspBeatsPacked;
  getGps: GpsInfo;
  getTimebaseLog: TimebaseLog;
  clearTimebaseLog: TimebaseLog;
  claim: HidClaimResult;
  getLink: HidLinkStatus;
  getDiag: HidDiag;
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
  beatsPacked: DspBeatsPacked;
  controlReleased: { to: HidLinkKind };
}
