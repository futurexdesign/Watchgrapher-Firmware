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
  sourceHz: number;
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
  /** Rolling ~10 s arithmetic mean of instant values. */
  avg10s: DspBeatTimingWindow;
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
  | 'subscribe';

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
  | HidSubscribeRequest;

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
