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

export interface SignedSettlementAllocationLineageCheckpoint extends SettlementAllocationLineageCheckpoint {
  readonly signerKeyId: string;
  readonly signatureBase64: string;
}

export interface SettlementAllocationLineageCheckpointTrustPolicy {
  readonly sourceRef: string;
  readonly sourceInstanceId: string;
  readonly signerKeyId: string;
  readonly publicKeyPem: string;
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
  if (!CANONICAL_UINT.test(normalized)) {
    throw new Error('settlement allocation checkpoint highWaterMark must be a canonical unsigned integer');
  }
  return normalized;
}

function exactRoots(left: readonly Digest[], right: readonly Digest[]): boolean {
  return left.length === right.length && left.every((root, index) => root === right[index]);
}

function assertCheckpointRootPrefix(
  label: string,
  previousRoots: readonly Digest[],
  candidateRoots: readonly Digest[],
): void {
  if (candidateRoots.length < previousRoots.length) {
    throw new Error(`settlement allocation checkpoint ${label} cannot shrink after acceptance`);
  }
  for (let index = 0; index < previousRoots.length; index += 1) {
    if (candidateRoots[index] !== previousRoots[index]) {
      throw new Error(`settlement allocation checkpoint ${label} cannot rewrite its accepted prefix`);
    }
  }
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

function checkpointSigningMessage(checkpointRoot: Digest): Buffer {
  return Buffer.from(`${CHECKPOINT_SIGNATURE_DOMAIN}${checkpointRoot}`, 'utf8');
}

function assertCanonicalSignature(value: string): Buffer {
  if (!value.trim()) throw new Error('settlement allocation checkpoint signature must be non-empty');
  const decoded = Buffer.from(value, 'base64');
  if (decoded.length !== 64 || decoded.toString('base64') !== value) {
    throw new Error('settlement allocation checkpoint signature must be canonical Ed25519 base64');
  }
  return decoded;
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
  const signatureBase64 = signEd25519(
    null,
    checkpointSigningMessage(checkpoint.checkpointRoot),
    privateKey,
  ).toString('base64');
  return Object.freeze({
    ...checkpoint,
    allocationRoots: validateRootSequence(
      'settlement allocation checkpoint allocation',
      checkpoint.allocationRoots,
    ),
    linkRoots: validateRootSequence(
      'settlement allocation checkpoint link',
      checkpoint.linkRoots,
    ),
    signerKeyId: keyId,
    signatureBase64,
  });
}

export function verifySettlementAllocationLineageCheckpoint(
  checkpoint: SignedSettlementAllocationLineageCheckpoint,
  policy: SettlementAllocationLineageCheckpointTrustPolicy,
): Readonly<SignedSettlementAllocationLineageCheckpoint> {
  assertSettlementAllocationLineageCheckpoint(checkpoint);
  const trustedSourceRef = required('trusted checkpoint sourceRef', policy.sourceRef);
  const trustedSourceInstanceId = required('trusted checkpoint sourceInstanceId', policy.sourceInstanceId);
  const trustedSignerKeyId = required('trusted checkpoint signerKeyId', policy.signerKeyId);
  if (checkpoint.sourceRef !== trustedSourceRef || checkpoint.sourceInstanceId !== trustedSourceInstanceId) {
    throw new Error('settlement allocation checkpoint source identity is not trusted');
  }
  if (checkpoint.signerKeyId !== trustedSignerKeyId) {
    throw new Error('settlement allocation checkpoint signer key is not trusted');
  }

  const publicKey = createPublicKey(policy.publicKeyPem);
  if (publicKey.asymmetricKeyType !== 'ed25519') {
    throw new Error('settlement allocation checkpoint verification key must be Ed25519');
  }
  const signature = assertCanonicalSignature(checkpoint.signatureBase64);
  const valid = verifyEd25519(
    null,
    checkpointSigningMessage(checkpoint.checkpointRoot),
    publicKey,
    signature,
  );
  if (!valid) throw new Error('settlement allocation checkpoint signature verification failed');

  // JSON transports supply mutable arrays. Snapshot and freeze both ordered
  // root sequences so the object authenticated above cannot be changed after
  // the WeakSet provenance marker is minted.
  const verified = Object.freeze({
    ...checkpoint,
    allocationRoots: validateRootSequence(
      'settlement allocation checkpoint allocation',
      checkpoint.allocationRoots,
    ),
    linkRoots: validateRootSequence(
      'settlement allocation checkpoint link',
      checkpoint.linkRoots,
    ),
  });
  VERIFIED_CHECKPOINTS.add(verified);
  return verified;
}

/**
 * Verify a candidate checkpoint advances beyond the last durably accepted checkpoint.
 *
 * IMPORTANT: this comparison is not itself durable replay protection. Callers must
 * load the previous checkpoint and persist the candidate/high-water mark in one
 * serializable transaction or compare-and-swap operation. Otherwise concurrent
 * consumers can still accept stale candidates against the same old cursor.
 */
export function assertSettlementAllocationLineageCheckpointAdvances(
  previousInput: SignedSettlementAllocationLineageCheckpoint,
  candidateInput: SignedSettlementAllocationLineageCheckpoint,
  policy: SettlementAllocationLineageCheckpointTrustPolicy,
): Readonly<SignedSettlementAllocationLineageCheckpoint> {
  const previous = verifySettlementAllocationLineageCheckpoint(previousInput, policy);
  const candidate = verifySettlementAllocationLineageCheckpoint(candidateInput, policy);

  if (
    candidate.sourceRef !== previous.sourceRef
    || candidate.sourceInstanceId !== previous.sourceInstanceId
    || candidate.batchId !== previous.batchId
  ) {
    throw new Error('settlement allocation checkpoint source and batch identity cannot change during cursor advancement');
  }

  if (
    candidate.checkpointId === previous.checkpointId
    && candidate.checkpointRoot !== previous.checkpointRoot
  ) {
    throw new Error('settlement allocation checkpointId cannot be reused for different checkpoint contents');
  }

  // The source checkpoint is an append-only view of this batch's allocation chain.
  // A later cursor may extend the chain or advance observation time, but may not
  // forget or rewrite an allocation/link already present in an accepted checkpoint.
  assertCheckpointRootPrefix('allocation roots', previous.allocationRoots, candidate.allocationRoots);
  assertCheckpointRootPrefix('link roots', previous.linkRoots, candidate.linkRoots);

  const previousHighWaterMark = BigInt(canonicalHighWaterMark(previous.highWaterMark));
  const candidateHighWaterMark = BigInt(canonicalHighWaterMark(candidate.highWaterMark));
  if (candidateHighWaterMark < previousHighWaterMark) {
    throw new Error('settlement allocation checkpoint high-water mark cannot move backwards');
  }

  const previousAsOf = Date.parse(timestamp('previous checkpoint asOf', previous.asOf));
  const candidateAsOf = Date.parse(timestamp('candidate checkpoint asOf', candidate.asOf));
  if (candidateAsOf < previousAsOf) {
    throw new Error('settlement allocation checkpoint asOf cannot move backwards');
  }

  const previousObservedThrough = Date.parse(
    timestamp('previous checkpoint observedThrough', previous.observedThrough),
  );
  const candidateObservedThrough = Date.parse(
    timestamp('candidate checkpoint observedThrough', candidate.observedThrough),
  );
  if (candidateObservedThrough < previousObservedThrough) {
    throw new Error('settlement allocation checkpoint observedThrough cannot move backwards');
  }

  const advances =
    candidateHighWaterMark > previousHighWaterMark
    || candidateAsOf > previousAsOf
    || candidateObservedThrough > previousObservedThrough;
  if (!advances) {
    throw new Error('settlement allocation checkpoint does not advance the accepted source cursor');
  }
  if (candidate.checkpointRoot === previous.checkpointRoot) {
    throw new Error('settlement allocation checkpoint replayed the previous checkpoint root');
  }

  return candidate;
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
