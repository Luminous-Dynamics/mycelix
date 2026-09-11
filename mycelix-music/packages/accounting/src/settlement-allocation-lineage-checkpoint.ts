import {
  createPrivateKey,
  createPublicKey,
  sign as signEd25519,
  verify as verifyEd25519,
} from 'node:crypto';
import { buildMerkleCommitment, type Digest } from './merkle.js';
import {
  createSettlementAllocationLineageBoundary,
  requireCanonicalSettlementAllocationHead,
  resolveSettlementAllocationLineage,
  type SettlementAllocationLineageResolution,
  type SettlementAllocationSuccessorLink,
} from './settlement-allocation-lineage.js';
import type { SettlementAllocationAuthority } from './settlement-allocation.js';

const CHECKPOINT_SIGNATURE_DOMAIN = 'mycelix-accounting-settlement-allocation-lineage-checkpoint-signature-v1\0';
export const SETTLEMENT_ALLOCATION_LINEAGE_CHECKPOINT_SIGNING_SCOPE =
  'accounting.settlement-allocation-lineage.checkpoint.sign' as const;
const SHA256_HEX = /^[0-9a-f]{64}$/;
const CANONICAL_UINT = /^(0|[1-9][0-9]*)$/;

export interface SettlementAllocationLineageCheckpoint {
  readonly checkpointId: string;
  readonly sourceRef: string;
  readonly sourceInstanceId: string;
  readonly batchId: string;
  readonly asOf: string;
  readonly observedThrough: string;
  /** Monotonic source ingestion cursor. It is provenance, not economic authority. */
  readonly highWaterMark: string;
  readonly allocationRoots: readonly Digest[];
  readonly linkRoots: readonly Digest[];
  readonly checkpointRoot: Digest;
}

export interface SettlementAllocationLineageCheckpointSigningCapability {
  readonly capabilityId: string;
  readonly scope: typeof SETTLEMENT_ALLOCATION_LINEAGE_CHECKPOINT_SIGNING_SCOPE;
  readonly sourceRef: string;
  readonly sourceInstanceId: string;
  readonly signerKeyId: string;
  readonly validFrom: string;
  readonly validUntil: string;
}

export interface SettlementAllocationLineageCheckpointSigningRequest {
  readonly requestId: string;
  readonly capabilityId: string;
  readonly scope: typeof SETTLEMENT_ALLOCATION_LINEAGE_CHECKPOINT_SIGNING_SCOPE;
  readonly sourceRef: string;
  readonly sourceInstanceId: string;
  readonly signerKeyId: string;
  readonly capabilityValidFrom: string;
  readonly capabilityValidUntil: string;
  readonly checkpointId: string;
  readonly checkpointRoot: Digest;
  readonly batchId: string;
  readonly highWaterMark: string;
  readonly issuedAt: string;
  readonly expiresAt: string;
  readonly nonce: string;
  readonly requestRoot: Digest;
}

export interface CreateSettlementAllocationLineageCheckpointSigningRequestInput {
  readonly requestId: string;
  readonly issuedAt: string;
  readonly expiresAt: string;
  readonly nonce: string;
}

export interface SettlementAllocationLineageCheckpointDetachedSignature {
  readonly requestRoot: Digest;
  readonly signerKeyId: string;
  readonly signedAt: string;
  readonly signatureBase64: string;
}

export interface SignedSettlementAllocationLineageCheckpoint extends SettlementAllocationLineageCheckpoint {
  readonly signerKeyId: string;
  readonly signatureBase64: string;
  readonly signedAt: string;
  readonly signingRequest: SettlementAllocationLineageCheckpointSigningRequest;
}

export interface SettlementAllocationLineageCheckpointTrustPolicy {
  readonly sourceRef: string;
  readonly sourceInstanceId: string;
  readonly signerKeyId: string;
  readonly publicKeyPem: string;
  /** Optional pin for deployments that rotate multiple scoped capabilities on one key. */
  readonly capabilityId?: string;
}

export interface CreateSettlementAllocationLineageCheckpointInput {
  readonly checkpointId: string;
  readonly sourceRef: string;
  readonly sourceInstanceId: string;
  readonly batchId: string;
  readonly asOf: string;
  readonly observedThrough: string;
  readonly highWaterMark: string;
  readonly allocations: readonly SettlementAllocationAuthority[];
  readonly links: readonly SettlementAllocationSuccessorLink[];
}

export interface CheckpointBackedSettlementAllocationLineage {
  readonly resolution: SettlementAllocationLineageResolution;
  readonly checkpoint: SignedSettlementAllocationLineageCheckpoint;
  readonly headAllocation: SettlementAllocationAuthority;
}

const VERIFIED_CHECKPOINTS = new WeakSet<object>();
const CHECKPOINT_BY_RESOLUTION = new WeakMap<object, SignedSettlementAllocationLineageCheckpoint>();

function required(label: string, value: string): string {
  const normalized = value.trim();
  if (!normalized) throw new Error(`${label} must be non-empty`);
  return normalized;
}

function timestamp(label: string, value: string): string {
  const parsed = Date.parse(value);
  if (!Number.isFinite(parsed)) throw new Error(`${label} must be a valid timestamp`);
  return new Date(parsed).toISOString();
}

function assertDigest(label: string, value: string): Digest {
  if (!SHA256_HEX.test(value)) throw new Error(`${label} must be a lowercase SHA-256 digest`);
  return value;
}

function canonicalHighWaterMark(value: string): string {
  const normalized = value.trim();
  if (normalized !== value || !CANONICAL_UINT.test(normalized)) {
    throw new Error('settlement allocation checkpoint highWaterMark must be a canonical unsigned integer');
  }
  return normalized;
}

function exactRoots(left: readonly Digest[], right: readonly Digest[]): boolean {
  return left.length === right.length && left.every((root, index) => root === right[index]);
}

function validateRootSequence(label: string, roots: readonly Digest[]): readonly Digest[] {
  const seen = new Set<string>();
  const result: Digest[] = [];
  for (const root of roots) {
    const digest = assertDigest(`${label} root`, root);
    if (seen.has(digest)) throw new Error(`${label} roots must be unique`);
    seen.add(digest);
    result.push(digest);
  }
  return Object.freeze(result);
}

function checkpointCommitment(input: Omit<SettlementAllocationLineageCheckpoint, 'checkpointRoot'>): Digest {
  return buildMerkleCommitment([{
    recordType: 'settlement_allocation_lineage_checkpoint_v1',
    checkpointId: input.checkpointId,
    sourceRef: input.sourceRef,
    sourceInstanceId: input.sourceInstanceId,
    batchId: input.batchId,
    asOf: input.asOf,
    observedThrough: input.observedThrough,
    highWaterMark: input.highWaterMark,
    allocationRoots: input.allocationRoots,
    linkRoots: input.linkRoots,
  }]).root;
}

function signingRequestCommitment(
  input: Omit<SettlementAllocationLineageCheckpointSigningRequest, 'requestRoot'>,
): Digest {
  return buildMerkleCommitment([{
    recordType: 'settlement_allocation_lineage_checkpoint_signing_request_v1',
    ...input,
  }]).root;
}

function checkpointSigningMessage(requestRoot: Digest, signedAt: string): Buffer {
  return Buffer.from(`${CHECKPOINT_SIGNATURE_DOMAIN}detached-v2\0${requestRoot}\0${signedAt}`, 'utf8');
}

function assertCanonicalSignature(value: string): Buffer {
  if (!value.trim()) throw new Error('settlement allocation checkpoint signature must be non-empty');
  const decoded = Buffer.from(value, 'base64');
  if (decoded.length !== 64 || decoded.toString('base64') !== value) {
    throw new Error('settlement allocation checkpoint signature must be canonical Ed25519 base64');
  }
  return decoded;
}

function canonicalSigningRequest(
  request: SettlementAllocationLineageCheckpointSigningRequest,
  checkpoint?: SettlementAllocationLineageCheckpoint,
): Readonly<SettlementAllocationLineageCheckpointSigningRequest> {
  const committed = Object.freeze({
    requestId: required('checkpoint signing requestId', request.requestId),
    capabilityId: required('checkpoint signing capabilityId', request.capabilityId),
    scope: request.scope,
    sourceRef: required('checkpoint signing sourceRef', request.sourceRef),
    sourceInstanceId: required('checkpoint signing sourceInstanceId', request.sourceInstanceId),
    signerKeyId: required('checkpoint signing signerKeyId', request.signerKeyId),
    capabilityValidFrom: timestamp('checkpoint signing capabilityValidFrom', request.capabilityValidFrom),
    capabilityValidUntil: timestamp('checkpoint signing capabilityValidUntil', request.capabilityValidUntil),
    checkpointId: required('checkpoint signing checkpointId', request.checkpointId),
    checkpointRoot: assertDigest('checkpoint signing checkpointRoot', request.checkpointRoot),
    batchId: required('checkpoint signing batchId', request.batchId),
    highWaterMark: canonicalHighWaterMark(request.highWaterMark),
    issuedAt: timestamp('checkpoint signing issuedAt', request.issuedAt),
    expiresAt: timestamp('checkpoint signing expiresAt', request.expiresAt),
    nonce: required('checkpoint signing nonce', request.nonce),
  });
  if (committed.scope !== SETTLEMENT_ALLOCATION_LINEAGE_CHECKPOINT_SIGNING_SCOPE) {
    throw new Error('checkpoint signing request has unsupported capability scope');
  }
  const validFromMs = Date.parse(committed.capabilityValidFrom);
  const validUntilMs = Date.parse(committed.capabilityValidUntil);
  const issuedAtMs = Date.parse(committed.issuedAt);
  const expiresAtMs = Date.parse(committed.expiresAt);
  if (validUntilMs <= validFromMs) throw new Error('checkpoint signing capability validity window is invalid');
  if (expiresAtMs <= issuedAtMs) throw new Error('checkpoint signing request expiry must be after issuance');
  if (issuedAtMs < validFromMs || expiresAtMs > validUntilMs) {
    throw new Error('checkpoint signing request must remain inside capability validity window');
  }
  const expectedRoot = signingRequestCommitment(committed);
  if (request.requestRoot !== expectedRoot) {
    throw new Error('checkpoint signing request root does not match canonical contents');
  }
  if (checkpoint !== undefined) {
    assertSettlementAllocationLineageCheckpoint(checkpoint);
    if (
      committed.sourceRef !== checkpoint.sourceRef
      || committed.sourceInstanceId !== checkpoint.sourceInstanceId
      || committed.checkpointId !== checkpoint.checkpointId
      || committed.checkpointRoot !== checkpoint.checkpointRoot
      || committed.batchId !== checkpoint.batchId
      || committed.highWaterMark !== checkpoint.highWaterMark
    ) {
      throw new Error('checkpoint signing request does not match checkpoint authority');
    }
  }
  return Object.freeze({ ...committed, requestRoot: expectedRoot });
}

export function createSettlementAllocationLineageCheckpoint(
  input: CreateSettlementAllocationLineageCheckpointInput,
): Readonly<SettlementAllocationLineageCheckpoint> {
  const checkpointId = required('settlement allocation checkpointId', input.checkpointId);
  const sourceRef = required('settlement allocation checkpoint sourceRef', input.sourceRef);
  const sourceInstanceId = required('settlement allocation checkpoint sourceInstanceId', input.sourceInstanceId);
  const batchId = required('settlement allocation checkpoint batchId', input.batchId);
  const asOf = timestamp('settlement allocation checkpoint asOf', input.asOf);
  const observedThrough = timestamp('settlement allocation checkpoint observedThrough', input.observedThrough);
  if (Date.parse(observedThrough) < Date.parse(asOf)) {
    throw new Error('settlement allocation checkpoint must be observed through asOf');
  }
  const highWaterMark = canonicalHighWaterMark(input.highWaterMark);

  const provisionalBoundary = createSettlementAllocationLineageBoundary({
    asOf,
    observedThrough,
    coverage: 'partial',
    sourceRef: `${sourceRef}:checkpoint-construction`,
  });
  const provisional = resolveSettlementAllocationLineage(input.allocations, input.links, provisionalBoundary);
  if (provisional.batchId !== batchId) {
    throw new Error('settlement allocation checkpoint batchId does not match observed lineage');
  }

  const committed = Object.freeze({
    checkpointId,
    sourceRef,
    sourceInstanceId,
    batchId,
    asOf,
    observedThrough,
    highWaterMark,
    allocationRoots: Object.freeze([...provisional.allocationRoots]),
    linkRoots: Object.freeze([...provisional.linkRoots]),
  });
  const checkpointRoot = checkpointCommitment(committed);
  return Object.freeze({ ...committed, checkpointRoot });
}

export function assertSettlementAllocationLineageCheckpoint(
  checkpoint: SettlementAllocationLineageCheckpoint,
): void {
  const committed = Object.freeze({
    checkpointId: required('settlement allocation checkpointId', checkpoint.checkpointId),
    sourceRef: required('settlement allocation checkpoint sourceRef', checkpoint.sourceRef),
    sourceInstanceId: required('settlement allocation checkpoint sourceInstanceId', checkpoint.sourceInstanceId),
    batchId: required('settlement allocation checkpoint batchId', checkpoint.batchId),
    asOf: timestamp('settlement allocation checkpoint asOf', checkpoint.asOf),
    observedThrough: timestamp('settlement allocation checkpoint observedThrough', checkpoint.observedThrough),
    highWaterMark: canonicalHighWaterMark(checkpoint.highWaterMark),
    allocationRoots: validateRootSequence('settlement allocation checkpoint allocation', checkpoint.allocationRoots),
    linkRoots: validateRootSequence('settlement allocation checkpoint link', checkpoint.linkRoots),
  });
  if (Date.parse(committed.observedThrough) < Date.parse(committed.asOf)) {
    throw new Error('settlement allocation checkpoint must be observed through asOf');
  }
  if (committed.allocationRoots.length === 0) {
    throw new Error('settlement allocation checkpoint requires at least one allocation root');
  }
  if (committed.linkRoots.length !== committed.allocationRoots.length - 1) {
    throw new Error('settlement allocation checkpoint must contain exactly one link root per non-initial allocation');
  }
  const expectedRoot = checkpointCommitment(committed);
  if (checkpoint.checkpointRoot !== expectedRoot) {
    throw new Error('settlement allocation checkpoint root does not match canonical contents');
  }
}

export function createSettlementAllocationLineageCheckpointSigningRequest(
  checkpoint: SettlementAllocationLineageCheckpoint,
  capability: SettlementAllocationLineageCheckpointSigningCapability,
  input: CreateSettlementAllocationLineageCheckpointSigningRequestInput,
): Readonly<SettlementAllocationLineageCheckpointSigningRequest> {
  assertSettlementAllocationLineageCheckpoint(checkpoint);
  const capabilityId = required('checkpoint signing capabilityId', capability.capabilityId);
  const sourceRef = required('checkpoint signing capability sourceRef', capability.sourceRef);
  const sourceInstanceId = required('checkpoint signing capability sourceInstanceId', capability.sourceInstanceId);
  const signerKeyId = required('checkpoint signing capability signerKeyId', capability.signerKeyId);
  const capabilityValidFrom = timestamp('checkpoint signing capability validFrom', capability.validFrom);
  const capabilityValidUntil = timestamp('checkpoint signing capability validUntil', capability.validUntil);
  if (capability.scope !== SETTLEMENT_ALLOCATION_LINEAGE_CHECKPOINT_SIGNING_SCOPE) {
    throw new Error('checkpoint signing capability has unsupported scope');
  }
  if (sourceRef !== checkpoint.sourceRef || sourceInstanceId !== checkpoint.sourceInstanceId) {
    throw new Error('checkpoint signing capability does not authorize checkpoint source identity');
  }
  const committed = Object.freeze({
    requestId: required('checkpoint signing requestId', input.requestId),
    capabilityId,
    scope: capability.scope,
    sourceRef,
    sourceInstanceId,
    signerKeyId,
    capabilityValidFrom,
    capabilityValidUntil,
    checkpointId: checkpoint.checkpointId,
    checkpointRoot: checkpoint.checkpointRoot,
    batchId: checkpoint.batchId,
    highWaterMark: checkpoint.highWaterMark,
    issuedAt: timestamp('checkpoint signing issuedAt', input.issuedAt),
    expiresAt: timestamp('checkpoint signing expiresAt', input.expiresAt),
    nonce: required('checkpoint signing nonce', input.nonce),
  });
  const requestRoot = signingRequestCommitment(committed);
  const request = Object.freeze({ ...committed, requestRoot });
  return canonicalSigningRequest(request, checkpoint);
}

export function settlementAllocationLineageCheckpointSigningPayloadBase64(
  request: SettlementAllocationLineageCheckpointSigningRequest,
  signedAt: string,
): string {
  const canonicalRequest = canonicalSigningRequest(request);
  const canonicalSignedAt = timestamp('checkpoint signature signedAt', signedAt);
  const signedAtMs = Date.parse(canonicalSignedAt);
  if (
    signedAtMs < Date.parse(canonicalRequest.issuedAt)
    || signedAtMs > Date.parse(canonicalRequest.expiresAt)
    || signedAtMs < Date.parse(canonicalRequest.capabilityValidFrom)
    || signedAtMs > Date.parse(canonicalRequest.capabilityValidUntil)
  ) {
    throw new Error('checkpoint detached signature time is outside authorized request window');
  }
  return checkpointSigningMessage(canonicalRequest.requestRoot, canonicalSignedAt).toString('base64');
}

export function attachSettlementAllocationLineageCheckpointDetachedSignature(
  checkpoint: SettlementAllocationLineageCheckpoint,
  request: SettlementAllocationLineageCheckpointSigningRequest,
  detached: SettlementAllocationLineageCheckpointDetachedSignature,
  policy: SettlementAllocationLineageCheckpointTrustPolicy,
): Readonly<SignedSettlementAllocationLineageCheckpoint> {
  assertSettlementAllocationLineageCheckpoint(checkpoint);
  const canonicalRequest = canonicalSigningRequest(request, checkpoint);
  const trustedSourceRef = required('trusted checkpoint sourceRef', policy.sourceRef);
  const trustedSourceInstanceId = required('trusted checkpoint sourceInstanceId', policy.sourceInstanceId);
  const trustedSignerKeyId = required('trusted checkpoint signerKeyId', policy.signerKeyId);
  if (checkpoint.sourceRef !== trustedSourceRef || checkpoint.sourceInstanceId !== trustedSourceInstanceId) {
    throw new Error('settlement allocation checkpoint source identity is not trusted');
  }
  if (canonicalRequest.signerKeyId !== trustedSignerKeyId || detached.signerKeyId !== trustedSignerKeyId) {
    throw new Error('settlement allocation checkpoint signer key is not trusted');
  }
  if (policy.capabilityId !== undefined && canonicalRequest.capabilityId !== required('trusted checkpoint capabilityId', policy.capabilityId)) {
    throw new Error('settlement allocation checkpoint signing capability is not trusted');
  }
  if (detached.requestRoot !== canonicalRequest.requestRoot) {
    throw new Error('checkpoint detached signature does not bind the canonical signing request');
  }
  const canonicalSignedAt = timestamp('checkpoint signature signedAt', detached.signedAt);
  const expectedPayload = settlementAllocationLineageCheckpointSigningPayloadBase64(canonicalRequest, canonicalSignedAt);

  const publicKey = createPublicKey(policy.publicKeyPem);
  if (publicKey.asymmetricKeyType !== 'ed25519') {
    throw new Error('settlement allocation checkpoint verification key must be Ed25519');
  }
  const signature = assertCanonicalSignature(detached.signatureBase64);
  const valid = verifyEd25519(
    null,
    Buffer.from(expectedPayload, 'base64'),
    publicKey,
    signature,
  );
  if (!valid) throw new Error('settlement allocation checkpoint signature verification failed');

  const verified = Object.freeze({
    ...checkpoint,
    signerKeyId: trustedSignerKeyId,
    signatureBase64: detached.signatureBase64,
    signedAt: canonicalSignedAt,
    signingRequest: canonicalRequest,
  });
  VERIFIED_CHECKPOINTS.add(verified);
  return verified;
}

/**
 * Deprecated deep-module compatibility helper. It is intentionally omitted from
 * the @mycelix/accounting package index. Operational code must use detached
 * signing requests so database/compiler processes never receive private keys.
 */
export function signSettlementAllocationLineageCheckpoint(
  checkpoint: SettlementAllocationLineageCheckpoint,
  signerKeyId: string,
  privateKeyPem: string,
): Readonly<SignedSettlementAllocationLineageCheckpoint> {
  assertSettlementAllocationLineageCheckpoint(checkpoint);
  const keyId = required('settlement allocation checkpoint signerKeyId', signerKeyId);
  const privateKey = createPrivateKey(privateKeyPem);
  if (privateKey.asymmetricKeyType !== 'ed25519') {
    throw new Error('settlement allocation checkpoint signing key must be Ed25519');
  }
  const issuedAt = checkpoint.observedThrough;
  const expiresAt = new Date(Date.parse(issuedAt) + 60_000).toISOString();
  const request = createSettlementAllocationLineageCheckpointSigningRequest(
    checkpoint,
    {
      capabilityId: `legacy-inline:${keyId}`,
      scope: SETTLEMENT_ALLOCATION_LINEAGE_CHECKPOINT_SIGNING_SCOPE,
      sourceRef: checkpoint.sourceRef,
      sourceInstanceId: checkpoint.sourceInstanceId,
      signerKeyId: keyId,
      validFrom: checkpoint.asOf,
      validUntil: expiresAt,
    },
    {
      requestId: `legacy-inline:${checkpoint.checkpointId}`,
      issuedAt,
      expiresAt,
      nonce: checkpoint.checkpointRoot,
    },
  );
  const payload = Buffer.from(
    settlementAllocationLineageCheckpointSigningPayloadBase64(request, issuedAt),
    'base64',
  );
  const signatureBase64 = signEd25519(null, payload, privateKey).toString('base64');
  return Object.freeze({
    ...checkpoint,
    signerKeyId: keyId,
    signatureBase64,
    signedAt: issuedAt,
    signingRequest: request,
  });
}

export function verifySettlementAllocationLineageCheckpoint(
  checkpoint: SignedSettlementAllocationLineageCheckpoint,
  policy: SettlementAllocationLineageCheckpointTrustPolicy,
): Readonly<SignedSettlementAllocationLineageCheckpoint> {
  return attachSettlementAllocationLineageCheckpointDetachedSignature(
    checkpoint,
    checkpoint.signingRequest,
    {
      requestRoot: checkpoint.signingRequest.requestRoot,
      signerKeyId: checkpoint.signerKeyId,
      signedAt: checkpoint.signedAt,
      signatureBase64: checkpoint.signatureBase64,
    },
    policy,
  );
}

export function resolveCheckpointBackedSettlementAllocationLineage(
  allocations: readonly SettlementAllocationAuthority[],
  links: readonly SettlementAllocationSuccessorLink[],
  checkpoint: SignedSettlementAllocationLineageCheckpoint,
): Readonly<SettlementAllocationLineageResolution> {
  if (!VERIFIED_CHECKPOINTS.has(checkpoint)) {
    throw new Error('settlement allocation checkpoint must be cryptographically verified before lineage resolution');
  }

  const provisionalBoundary = createSettlementAllocationLineageBoundary({
    asOf: checkpoint.asOf,
    observedThrough: checkpoint.observedThrough,
    coverage: 'partial',
    sourceRef: `${checkpoint.sourceRef}:checkpoint-replay`,
  });
  const provisional = resolveSettlementAllocationLineage(allocations, links, provisionalBoundary);
  if (provisional.batchId !== checkpoint.batchId) {
    throw new Error('settlement allocation checkpoint does not match lineage batch');
  }
  if (!exactRoots(provisional.allocationRoots, checkpoint.allocationRoots)) {
    throw new Error('settlement allocation checkpoint allocation roots do not match observed lineage');
  }
  if (!exactRoots(provisional.linkRoots, checkpoint.linkRoots)) {
    throw new Error('settlement allocation checkpoint link roots do not match observed lineage');
  }

  const completeBoundary = createSettlementAllocationLineageBoundary({
    asOf: checkpoint.asOf,
    observedThrough: checkpoint.observedThrough,
    coverage: 'complete',
    sourceRef: `${checkpoint.sourceRef}#${checkpoint.checkpointRoot}`,
  });
  const resolution = resolveSettlementAllocationLineage(allocations, links, completeBoundary);
  const headAllocation = requireCanonicalSettlementAllocationHead(resolution);
  if (headAllocation.allocationRoot !== resolution.headAllocationRoot) {
    throw new Error('checkpoint-backed settlement allocation lineage head mismatch');
  }
  CHECKPOINT_BY_RESOLUTION.set(resolution, checkpoint);
  return resolution;
}

export function requireCheckpointBackedSettlementAllocationLineage(
  resolution: SettlementAllocationLineageResolution,
): Readonly<CheckpointBackedSettlementAllocationLineage> {
  const checkpoint = CHECKPOINT_BY_RESOLUTION.get(resolution);
  if (checkpoint === undefined || !VERIFIED_CHECKPOINTS.has(checkpoint)) {
    throw new Error('settlement allocation lineage requires a verified source checkpoint');
  }
  const headAllocation = requireCanonicalSettlementAllocationHead(resolution);
  if (resolution.batchId !== checkpoint.batchId || resolution.boundary.asOf !== checkpoint.asOf) {
    throw new Error('settlement allocation lineage checkpoint identity mismatch');
  }
  return Object.freeze({ resolution, checkpoint, headAllocation });
}
