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

export interface DspBlocksState {
  levelMeter?: BlockState;
  gain?: BlockState<GainParams>;
  limiter?: BlockState;
  filterBank?: BlockState<FilterBankParams>;
  eventDetect?: BlockState<EventDetectParams>;
  beatRate?: BlockState;
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

export interface DspBeatRate {
  vphRaw: number;
  vphNearest: number;
  confidence: number;
  locked: boolean;
  intervalSamples: number;
  eventsSeen: number;
  intervalsHeld: number;
  sourceHz: number;
}

export interface DspEvent {
  startSample: SampleIndexString;
  peakSample: SampleIndexString;
  durationSamples: number;
  riseSamples: number;
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
