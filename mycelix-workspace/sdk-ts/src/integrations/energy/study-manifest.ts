// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

/**
 * Cross-language V1 manifest contract shared with sol-atlas-core.
 *
 * This module binds declared study inputs and configuration identities. It
 * does not authenticate sources, prove that a run occurred, or verify runtime
 * receipts. Keep the field order, integer encoding, enum tags, and golden
 * vector synchronized with Sol Atlas's Rust implementation.
 */

export const ENERGY_STUDY_SCHEMA_VERSION = 1 as const;
export const MAX_ENERGY_STUDY_DATASETS = 256;

const MAX_TEXT_BYTES = 512;
const MAX_U32 = 0xffff_ffff;
const MAX_SAFE_INTEGER = Number.MAX_SAFE_INTEGER;
const DIGEST_DOMAIN = 'luminous-dynamics.energy-study-manifest.v1\0';
const encoder = new TextEncoder();

export type EnergyEvidenceClass = 'observed' | 'curated' | 'scenario' | 'modelled';

export type EnergyPolicyKind =
  | 'baseline'
  | 'rule_based'
  | 'optimization'
  | 'learned'
  | 'external_reference';

export interface EnergyDatasetRefV1 {
  dataset_id: string;
  source_id: string;
  content_sha256: string;
  unit: string;
  evidence_class: EnergyEvidenceClass;
}

export interface EnergyModelRefV1 {
  model_id: string;
  repository: string;
  git_revision: string;
  configuration_sha256: string;
}

export interface EnergyPolicyRefV1 {
  policy_id: string;
  repository: string;
  git_revision: string;
  configuration_sha256: string;
  kind: EnergyPolicyKind;
}

export interface EnergyStudyManifestV1 {
  schema_version: typeof ENERGY_STUDY_SCHEMA_VERSION;
  study_id: string;
  case_id: string;
  region_id: string;
  start_unix_seconds: number;
  interval_seconds: number;
  interval_count: number;
  datasets: EnergyDatasetRefV1[];
  asset_registry_sha256: string;
  constraints_sha256: string;
  model: EnergyModelRefV1;
  policy: EnergyPolicyRefV1;
}

export type EnergyStudyManifestErrorCode =
  | 'INVALID_OBJECT'
  | 'UNKNOWN_FIELD'
  | 'UNSUPPORTED_SCHEMA_VERSION'
  | 'INVALID_TEXT'
  | 'INVALID_SHA256'
  | 'INVALID_GIT_REVISION'
  | 'EMPTY_DATASETS'
  | 'TOO_MANY_DATASETS'
  | 'DUPLICATE_DATASET_ID'
  | 'INVALID_TIME_AXIS'
  | 'TIME_HORIZON_OVERFLOW'
  | 'TIME_OUTSIDE_PORTABLE_RANGE'
  | 'CRYPTO_UNAVAILABLE';

export class EnergyStudyManifestError extends Error {
  constructor(
    public readonly code: EnergyStudyManifestErrorCode,
    message: string
  ) {
    super(message);
    this.name = 'EnergyStudyManifestError';
  }
}

type RecordValue = Record<string, unknown>;

function isRecord(value: unknown): value is RecordValue {
  return typeof value === 'object' && value !== null && !Array.isArray(value);
}

function assertExactKeys(
  value: RecordValue,
  expected: readonly string[],
  field: string
): void {
  const actualKeys = Object.keys(value);
  const expectedSet = new Set(expected);
  if (
    actualKeys.length !== expected.length ||
    actualKeys.some((key) => !expectedSet.has(key))
  ) {
    throw new EnergyStudyManifestError(
      'UNKNOWN_FIELD',
      `${field} must contain exactly: ${expected.join(', ')}`
    );
  }
}

function isWellFormedUnicode(value: string): boolean {
  for (let index = 0; index < value.length; index += 1) {
    const code = value.charCodeAt(index);
    if (code >= 0xd800 && code <= 0xdbff) {
      const next = value.charCodeAt(index + 1);
      if (!(next >= 0xdc00 && next <= 0xdfff)) return false;
      index += 1;
    } else if (code >= 0xdc00 && code <= 0xdfff) {
      return false;
    }
  }
  return true;
}

function requireText(value: unknown, field: string): string {
  if (
    typeof value !== 'string' ||
    value.length === 0 ||
    !isWellFormedUnicode(value) ||
    encoder.encode(value).byteLength > MAX_TEXT_BYTES ||
    value.startsWith(' ') ||
    value.endsWith(' ') ||
    /[\u0000-\u001f\u007f-\u009f]/u.test(value)
  ) {
    throw new EnergyStudyManifestError(
      'INVALID_TEXT',
      `${field} is empty or outside the V1 text domain`
    );
  }
  return value;
}

function requireSha256(value: unknown, field: string): string {
  if (typeof value !== 'string' || !/^[0-9a-f]{64}$/u.test(value)) {
    throw new EnergyStudyManifestError(
      'INVALID_SHA256',
      `${field} must be a lowercase 64-character SHA-256 digest`
    );
  }
  return value;
}

function requireGitRevision(value: unknown, field: string): string {
  if (
    typeof value !== 'string' ||
    !/^(?:[0-9a-f]{40}|[0-9a-f]{64})$/u.test(value)
  ) {
    throw new EnergyStudyManifestError(
      'INVALID_GIT_REVISION',
      `${field} must be an exact lowercase 40- or 64-character Git revision`
    );
  }
  return value;
}

function requireUint32(
  value: unknown,
  field: string,
  allowZero = false
): number {
  if (
    typeof value !== 'number' ||
    !Number.isInteger(value) ||
    value < (allowZero ? 0 : 1) ||
    value > MAX_U32
  ) {
    throw new EnergyStudyManifestError(
      'INVALID_TIME_AXIS',
      `${field} must be an integer in the V1 uint32 range`
    );
  }
  return value;
}

function requireRecord(value: unknown, field: string): RecordValue {
  if (!isRecord(value)) {
    throw new EnergyStudyManifestError('INVALID_OBJECT', `${field} must be an object`);
  }
  return value;
}

const DATASET_KEYS = [
  'dataset_id',
  'source_id',
  'content_sha256',
  'unit',
  'evidence_class',
] as const;
const MODEL_KEYS = [
  'model_id',
  'repository',
  'git_revision',
  'configuration_sha256',
] as const;
const POLICY_KEYS = [
  'policy_id',
  'repository',
  'git_revision',
  'configuration_sha256',
  'kind',
] as const;
const MANIFEST_KEYS = [
  'schema_version',
  'study_id',
  'case_id',
  'region_id',
  'start_unix_seconds',
  'interval_seconds',
  'interval_count',
  'datasets',
  'asset_registry_sha256',
  'constraints_sha256',
  'model',
  'policy',
] as const;

const EVIDENCE_TAG: Record<EnergyEvidenceClass, number> = {
  observed: 1,
  curated: 2,
  scenario: 3,
  modelled: 4,
};

const POLICY_TAG: Record<EnergyPolicyKind, number> = {
  baseline: 1,
  rule_based: 2,
  optimization: 3,
  learned: 4,
  external_reference: 5,
};

/** Validate an untrusted parsed JSON value and return its typed V1 manifest. */
export function validateEnergyStudyManifestV1(
  input: unknown
): EnergyStudyManifestV1 {
  const manifest = requireRecord(input, 'manifest');
  assertExactKeys(manifest, MANIFEST_KEYS, 'manifest');

  if (manifest.schema_version !== ENERGY_STUDY_SCHEMA_VERSION) {
    throw new EnergyStudyManifestError(
      'UNSUPPORTED_SCHEMA_VERSION',
      'only energy-study manifest schema version 1 is supported'
    );
  }

  requireText(manifest.study_id, 'study_id');
  requireText(manifest.case_id, 'case_id');
  requireText(manifest.region_id, 'region_id');

  const startUnixSeconds = manifest.start_unix_seconds;
  if (
    typeof startUnixSeconds !== 'number' ||
    !Number.isSafeInteger(startUnixSeconds)
  ) {
    throw new EnergyStudyManifestError(
      'TIME_OUTSIDE_PORTABLE_RANGE',
      'start_unix_seconds must be an exact JavaScript safe integer'
    );
  }
  const intervalSeconds = requireUint32(manifest.interval_seconds, 'interval_seconds');
  const intervalCount = requireUint32(manifest.interval_count, 'interval_count');
  const durationSeconds = intervalSeconds * intervalCount;
  if (
    !Number.isSafeInteger(durationSeconds) ||
    durationSeconds > MAX_SAFE_INTEGER
  ) {
    throw new EnergyStudyManifestError(
      'TIME_OUTSIDE_PORTABLE_RANGE',
      'study duration is outside the exact cross-language integer range'
    );
  }
  const endSeconds = startUnixSeconds + durationSeconds;
  if (!Number.isSafeInteger(endSeconds)) {
    throw new EnergyStudyManifestError(
      'TIME_OUTSIDE_PORTABLE_RANGE',
      'study end timestamp is outside the exact cross-language integer range'
    );
  }

  if (!Array.isArray(manifest.datasets) || manifest.datasets.length === 0) {
    throw new EnergyStudyManifestError(
      'EMPTY_DATASETS',
      'energy study must declare at least one dataset'
    );
  }
  if (manifest.datasets.length > MAX_ENERGY_STUDY_DATASETS) {
    throw new EnergyStudyManifestError(
      'TOO_MANY_DATASETS',
      `energy study exceeds the ${MAX_ENERGY_STUDY_DATASETS}-dataset limit`
    );
  }

  const datasetIds = new Set<string>();
  for (const [index, rawDataset] of manifest.datasets.entries()) {
    const dataset = requireRecord(rawDataset, `datasets[${index}]`);
    assertExactKeys(dataset, DATASET_KEYS, `datasets[${index}]`);
    const datasetId = requireText(dataset.dataset_id, `datasets[${index}].dataset_id`);
    requireText(dataset.source_id, `datasets[${index}].source_id`);
    requireText(dataset.unit, `datasets[${index}].unit`);
    requireSha256(dataset.content_sha256, `datasets[${index}].content_sha256`);
    if (
      dataset.evidence_class !== 'observed' &&
      dataset.evidence_class !== 'curated' &&
      dataset.evidence_class !== 'scenario' &&
      dataset.evidence_class !== 'modelled'
    ) {
      throw new EnergyStudyManifestError(
        'INVALID_OBJECT',
        `datasets[${index}].evidence_class is not a V1 evidence class`
      );
    }
    if (datasetIds.has(datasetId)) {
      throw new EnergyStudyManifestError(
        'DUPLICATE_DATASET_ID',
        `duplicate dataset_id: ${datasetId}`
      );
    }
    datasetIds.add(datasetId);
  }

  requireSha256(manifest.asset_registry_sha256, 'asset_registry_sha256');
  requireSha256(manifest.constraints_sha256, 'constraints_sha256');

  const model = requireRecord(manifest.model, 'model');
  assertExactKeys(model, MODEL_KEYS, 'model');
  requireText(model.model_id, 'model.model_id');
  requireText(model.repository, 'model.repository');
  requireGitRevision(model.git_revision, 'model.git_revision');
  requireSha256(model.configuration_sha256, 'model.configuration_sha256');

  const policy = requireRecord(manifest.policy, 'policy');
  assertExactKeys(policy, POLICY_KEYS, 'policy');
  requireText(policy.policy_id, 'policy.policy_id');
  requireText(policy.repository, 'policy.repository');
  requireGitRevision(policy.git_revision, 'policy.git_revision');
  requireSha256(policy.configuration_sha256, 'policy.configuration_sha256');
  if (
    policy.kind !== 'baseline' &&
    policy.kind !== 'rule_based' &&
    policy.kind !== 'optimization' &&
    policy.kind !== 'learned' &&
    policy.kind !== 'external_reference'
  ) {
    throw new EnergyStudyManifestError(
      'INVALID_OBJECT',
      'policy.kind is not a V1 policy kind'
    );
  }

  return input as EnergyStudyManifestV1;
}

class CanonicalWriter {
  private readonly chunks: Uint8Array[] = [];
  private byteLength = 0;

  writeBytes(bytes: Uint8Array): void {
    this.chunks.push(bytes);
    this.byteLength += bytes.byteLength;
  }

  writeByte(value: number): void {
    this.writeBytes(Uint8Array.of(value));
  }

  writeUint16(value: number): void {
    const buffer = new ArrayBuffer(2);
    new DataView(buffer).setUint16(0, value, false);
    this.writeBytes(new Uint8Array(buffer));
  }

  writeUint32(value: number): void {
    const buffer = new ArrayBuffer(4);
    new DataView(buffer).setUint32(0, value, false);
    this.writeBytes(new Uint8Array(buffer));
  }

  writeInt64(value: number): void {
    const buffer = new ArrayBuffer(8);
    new DataView(buffer).setBigInt64(0, BigInt(value), false);
    this.writeBytes(new Uint8Array(buffer));
  }

  writeText(value: string): void {
    const encoded = encoder.encode(value);
    const lengthBuffer = new ArrayBuffer(8);
    new DataView(lengthBuffer).setBigUint64(0, BigInt(encoded.byteLength), false);
    this.writeBytes(new Uint8Array(lengthBuffer));
    this.writeBytes(encoded);
  }

  finish(): Uint8Array {
    const result = new Uint8Array(this.byteLength);
    let offset = 0;
    for (const chunk of this.chunks) {
      result.set(chunk, offset);
      offset += chunk.byteLength;
    }
    return result;
  }
}

function compareUtf8(left: string, right: string): number {
  const a = encoder.encode(left);
  const b = encoder.encode(right);
  const sharedLength = Math.min(a.length, b.length);
  for (let index = 0; index < sharedLength; index += 1) {
    if (a[index] !== b[index]) return a[index] - b[index];
  }
  return a.length - b.length;
}

/**
 * Compute the same domain-separated SHA-256 identity as sol-atlas-core's Rust
 * EnergyStudyManifestV1. Dataset order is canonicalized by UTF-8 dataset_id.
 */
export async function digestEnergyStudyManifestV1(input: unknown): Promise<string> {
  const manifest = validateEnergyStudyManifestV1(input);
  if (!globalThis.crypto?.subtle) {
    throw new EnergyStudyManifestError(
      'CRYPTO_UNAVAILABLE',
      'Web Crypto SHA-256 is unavailable in this runtime'
    );
  }

  const writer = new CanonicalWriter();
  writer.writeBytes(encoder.encode(DIGEST_DOMAIN));
  writer.writeUint16(manifest.schema_version);
  writer.writeText(manifest.study_id);
  writer.writeText(manifest.case_id);
  writer.writeText(manifest.region_id);
  writer.writeInt64(manifest.start_unix_seconds);
  writer.writeUint32(manifest.interval_seconds);
  writer.writeUint32(manifest.interval_count);

  const datasets = [...manifest.datasets].sort((left, right) =>
    compareUtf8(left.dataset_id, right.dataset_id)
  );
  writer.writeUint32(datasets.length);
  for (const dataset of datasets) {
    writer.writeText(dataset.dataset_id);
    writer.writeText(dataset.source_id);
    writer.writeText(dataset.content_sha256);
    writer.writeText(dataset.unit);
    writer.writeByte(EVIDENCE_TAG[dataset.evidence_class]);
  }

  writer.writeText(manifest.asset_registry_sha256);
  writer.writeText(manifest.constraints_sha256);
  writer.writeText(manifest.model.model_id);
  writer.writeText(manifest.model.repository);
  writer.writeText(manifest.model.git_revision);
  writer.writeText(manifest.model.configuration_sha256);
  writer.writeText(manifest.policy.policy_id);
  writer.writeText(manifest.policy.repository);
  writer.writeText(manifest.policy.git_revision);
  writer.writeText(manifest.policy.configuration_sha256);
  writer.writeByte(POLICY_TAG[manifest.policy.kind]);

  const canonicalBytes = writer.finish();
  const ownedBuffer = new ArrayBuffer(canonicalBytes.byteLength);
  new Uint8Array(ownedBuffer).set(canonicalBytes);
  const digest = await globalThis.crypto.subtle.digest('SHA-256', ownedBuffer);
  return Array.from(new Uint8Array(digest), (byte) =>
    byte.toString(16).padStart(2, '0')
  ).join('');
}
