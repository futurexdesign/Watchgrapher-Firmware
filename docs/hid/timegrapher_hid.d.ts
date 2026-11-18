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

/** Max reassembled JSON payload (bytes). */
export declare const TIMEGRAPHER_HID_MAX_JSON_BYTES = 2048;

/** Error codes returned in HidErrorResponse. */
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

/** Full DSP snapshot from getState / patchState result. */
export interface DspState {
  bypass: boolean;
  blocks: DspBlocksState;
}

/** Partial update for patchState (JSON Merge Patch style). */
export type DspStatePatch = {
  bypass?: boolean;
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

export interface HidHelloCaps {
  maxJsonBytes: number;
  metricsHzMax: number;
  beatHzMax: number;
  timingHzMax: number;
  timing: boolean;
  blocks: DspBlockKey[];
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
  events: boolean;
  errors: boolean;
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
  | 'clearTimebase';

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

export interface HidGetEventsRequest extends HidRequestBase {
  op: 'getEvents';
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
  /** -1000..1000, + = board clock fast. */
  ppm: number;
  /** Default 'manual'. */
  source?: 'manual' | 'ntp';
  dateUnix?: number;
  durationS?: number;
}

/** Fails with `busy` while audio is streaming, `flashError` on write failure. */
export interface HidSaveTimebaseRequest extends HidRequestBase {
  op: 'saveTimebase';
}

export interface HidClearTimebaseRequest extends HidRequestBase {
  op: 'clearTimebase';
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
  | HidGetEventsRequest
  | HidGetErrorsRequest
  | HidClearErrorsRequest
  | HidSubscribeRequest
  | HidGetTimeRequest
  | HidGetTimebaseRequest
  | HidSetTimebaseRequest
  | HidSaveTimebaseRequest
  | HidClearTimebaseRequest;

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

export interface HidEventNotify extends HidNotifyBase {
  op: 'event';
  data: { events: DspEvent[] };
}

export interface HidErrorNotify extends HidNotifyBase {
  op: 'error';
  data: DspError;
}

export type HidNotify =
  | HidMetricsNotify
  | HidTimingNotify
  | HidBeatNotify
  | HidEventNotify
  | HidErrorNotify;

/** Any device → host JSON message. */
export type HidInboundMessage = HidResponse | HidNotify;

/** Framing constants (WebHID buffer without report ID). */
export declare const HID_FRAME_HEADER_BYTES = 5;
export declare const HID_FRAME_MAX_CHUNK = 58;
export declare const HID_FRAME_FLAG_MORE = 0x01;
export declare const HID_FRAME_FLAG_NOTIFY_HINT = 0x02;
