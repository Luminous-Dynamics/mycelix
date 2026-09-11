import { createPublicKey } from 'node:crypto';
import { buildMerkleCommitment, canonicalAccountingValue, type Digest } from './merkle.js';
import {
  verifySettlementAllocationLineageCheckpoint,
  type SignedSettlementAllocationLineageCheckpoint,
} from './settlement-allocation-lineage-checkpoint.js';

export interface SettlementAllocationLineageCheckpointSignerTrustEntry {
  readonly entryId: string;
  readonly sourceRef: string;
  readonly sourceInstanceId: string;
  readonly capabilityId: string;
  readonly signerKeyId: string;
  readonly publicKeyPem: string;
  readonly validFrom: string;
  readonly validUntil: string;
  /** First instant at which this entry must no longer authorize new signatures. */
  readonly revokedAt?: string;
}

export interface SettlementAllocationLineageCheckpointSignerTrustBundle {
  readonly bundleId: string;
  readonly policyAsOf: string;
  readonly entries: readonly SettlementAllocationLineageCheckpointSignerTrustEntry[];
  readonly bundleRoot: Digest;
}

export interface SettlementAllocationLineageCheckpointVerificationReceipt {
  readonly checkpointRoot: Digest;
  readonly signingRequestRoot: Digest;
  readonly signerKeyId: string;
  readonly capabilityId: string;
  readonly signedAt: string;
  readonly trustBundleId: string;
  readonly trustBundleRoot: Digest;
  readonly trustEntryId: string;
  readonly verifiedAt: string;
  readonly receiptRoot: Digest;
}

export interface VerifiedSettlementAllocationLineageCheckpointAuthority {
  readonly checkpoint: SignedSettlementAllocationLineageCheckpoint;
  readonly receipt: SettlementAllocationLineageCheckpointVerificationReceipt;
}

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

function canonicalPublicKeyPem(value: string): string {
  const key = createPublicKey(value);
  if (key.asymmetricKeyType !== 'ed25519') {
    throw new Error('checkpoint signer trust entry public key must be Ed25519');
  }
  return key.export({ format: 'pem', type: 'spki' }).toString();
}

function canonicalTrustEntry(
  entry: SettlementAllocationLineageCheckpointSignerTrustEntry,
): Readonly<SettlementAllocationLineageCheckpointSignerTrustEntry> {
  const validFrom = timestamp('checkpoint signer trust validFrom', entry.validFrom);
  const validUntil = timestamp('checkpoint signer trust validUntil', entry.validUntil);
  if (Date.parse(validUntil) <= Date.parse(validFrom)) {
    throw new Error('checkpoint signer trust validity window must be strictly increasing');
  }
  const revokedAt = entry.revokedAt === undefined
    ? undefined
    : timestamp('checkpoint signer trust revokedAt', entry.revokedAt);
  if (revokedAt !== undefined && (
    Date.parse(revokedAt) < Date.parse(validFrom)
    || Date.parse(revokedAt) > Date.parse(validUntil)
  )) {
    throw new Error('checkpoint signer trust revocation must fall inside its validity window');
  }
  return Object.freeze({
    entryId: required('checkpoint signer trust entryId', entry.entryId),
    sourceRef: required('checkpoint signer trust sourceRef', entry.sourceRef),
    sourceInstanceId: required('checkpoint signer trust sourceInstanceId', entry.sourceInstanceId),
    capabilityId: required('checkpoint signer trust capabilityId', entry.capabilityId),
    signerKeyId: required('checkpoint signer trust signerKeyId', entry.signerKeyId),
    publicKeyPem: canonicalPublicKeyPem(entry.publicKeyPem),
    validFrom,
    validUntil,
    ...(revokedAt === undefined ? {} : { revokedAt }),
  });
}

function windowsOverlap(
  left: SettlementAllocationLineageCheckpointSignerTrustEntry,
  right: SettlementAllocationLineageCheckpointSignerTrustEntry,
): boolean {
  return Date.parse(left.validFrom) < Date.parse(right.validUntil)
    && Date.parse(right.validFrom) < Date.parse(left.validUntil);
}

function sameAuthorityIdentity(
  left: SettlementAllocationLineageCheckpointSignerTrustEntry,
  right: SettlementAllocationLineageCheckpointSignerTrustEntry,
): boolean {
  return left.sourceRef === right.sourceRef
    && left.sourceInstanceId === right.sourceInstanceId
    && left.capabilityId === right.capabilityId
    && left.signerKeyId === right.signerKeyId;
}

function canonicalEntries(
  entries: readonly SettlementAllocationLineageCheckpointSignerTrustEntry[],
): readonly SettlementAllocationLineageCheckpointSignerTrustEntry[] {
  if (entries.length === 0) throw new Error('checkpoint signer trust bundle requires at least one entry');
  const canonical = entries.map(canonicalTrustEntry).sort((a, b) => a.entryId.localeCompare(b.entryId));
  const ids = new Set<string>();
  for (const entry of canonical) {
    if (ids.has(entry.entryId)) throw new Error(`duplicate checkpoint signer trust entry id: ${entry.entryId}`);
    ids.add(entry.entryId);
  }
  for (let i = 0; i < canonical.length; i += 1) {
    for (let j = i + 1; j < canonical.length; j += 1) {
      const left = canonical[i]!;
      const right = canonical[j]!;
      if (sameAuthorityIdentity(left, right) && windowsOverlap(left, right)) {
        throw new Error('checkpoint signer trust bundle contains overlapping authority windows');
      }
    }
  }
  return Object.freeze(canonical);
}

function bundleCommitment(input: Omit<SettlementAllocationLineageCheckpointSignerTrustBundle, 'bundleRoot'>): Digest {
  return buildMerkleCommitment([{
    recordType: 'settlement_allocation_lineage_checkpoint_signer_trust_bundle_v1',
    bundleId: input.bundleId,
    policyAsOf: input.policyAsOf,
    entries: input.entries,
  }]).root;
}

export function createSettlementAllocationLineageCheckpointSignerTrustBundle(input: {
  readonly bundleId: string;
  readonly policyAsOf: string;
  readonly entries: readonly SettlementAllocationLineageCheckpointSignerTrustEntry[];
}): Readonly<SettlementAllocationLineageCheckpointSignerTrustBundle> {
  const committed = Object.freeze({
    bundleId: required('checkpoint signer trust bundleId', input.bundleId),
    policyAsOf: timestamp('checkpoint signer trust policyAsOf', input.policyAsOf),
    entries: canonicalEntries(input.entries),
  });
  return Object.freeze({ ...committed, bundleRoot: bundleCommitment(committed) });
}

export function assertSettlementAllocationLineageCheckpointSignerTrustBundle(
  bundle: SettlementAllocationLineageCheckpointSignerTrustBundle,
): void {
  const expected = createSettlementAllocationLineageCheckpointSignerTrustBundle({
    bundleId: bundle.bundleId,
    policyAsOf: bundle.policyAsOf,
    entries: bundle.entries,
  });
  if (bundle.bundleRoot !== expected.bundleRoot || canonicalAccountingValue(bundle.entries) !== canonicalAccountingValue(expected.entries)) {
    throw new Error('checkpoint signer trust bundle does not match canonical reconstruction');
  }
}

function signingInstantAuthorized(
  entry: SettlementAllocationLineageCheckpointSignerTrustEntry,
  signedAt: string,
): boolean {
  const signedMs = Date.parse(signedAt);
  if (signedMs < Date.parse(entry.validFrom) || signedMs >= Date.parse(entry.validUntil)) return false;
  if (entry.revokedAt !== undefined && signedMs >= Date.parse(entry.revokedAt)) return false;
  return true;
}

export function verifySettlementAllocationLineageCheckpointWithTrustBundle(
  checkpoint: SignedSettlementAllocationLineageCheckpoint,
  bundle: SettlementAllocationLineageCheckpointSignerTrustBundle,
  verifiedAtInput: string,
): Readonly<VerifiedSettlementAllocationLineageCheckpointAuthority> {
  assertSettlementAllocationLineageCheckpointSignerTrustBundle(bundle);
  const verifiedAt = timestamp('checkpoint trust verification verifiedAt', verifiedAtInput);
  const signedAt = timestamp('checkpoint trust verification signedAt', checkpoint.signedAt);
  if (Date.parse(verifiedAt) < Date.parse(signedAt)) {
    throw new Error('checkpoint trust verification cannot predate the checkpoint signature');
  }
  if (Date.parse(bundle.policyAsOf) < Date.parse(signedAt)) {
    throw new Error('checkpoint signer trust policy must be observed through checkpoint signedAt');
  }
  if (Date.parse(bundle.policyAsOf) > Date.parse(verifiedAt)) {
    throw new Error('checkpoint signer trust policy cannot be newer than verification time');
  }

  const capabilityId = checkpoint.signingRequest.capabilityId;
  const candidates = bundle.entries.filter(entry =>
    entry.sourceRef === checkpoint.sourceRef
    && entry.sourceInstanceId === checkpoint.sourceInstanceId
    && entry.capabilityId === capabilityId
    && entry.signerKeyId === checkpoint.signerKeyId
    && signingInstantAuthorized(entry, signedAt));
  if (candidates.length === 0) {
    throw new Error('checkpoint signature has no authorized signer trust entry at signedAt');
  }
  if (candidates.length !== 1) {
    throw new Error('checkpoint signature matches ambiguous signer trust entries');
  }
  const entry = candidates[0]!;

  const verified = verifySettlementAllocationLineageCheckpoint(checkpoint, {
    sourceRef: entry.sourceRef,
    sourceInstanceId: entry.sourceInstanceId,
    signerKeyId: entry.signerKeyId,
    capabilityId: entry.capabilityId,
    publicKeyPem: entry.publicKeyPem,
  });
  const receiptCommitted = Object.freeze({
    checkpointRoot: verified.checkpointRoot,
    signingRequestRoot: verified.signingRequest.requestRoot,
    signerKeyId: verified.signerKeyId,
    capabilityId: verified.signingRequest.capabilityId,
    signedAt: verified.signedAt,
    trustBundleId: bundle.bundleId,
    trustBundleRoot: bundle.bundleRoot,
    trustEntryId: entry.entryId,
    verifiedAt,
  });
  const receiptRoot = buildMerkleCommitment([{
    recordType: 'settlement_allocation_lineage_checkpoint_verification_receipt_v1',
    ...receiptCommitted,
  }]).root;
  return Object.freeze({
    checkpoint: verified,
    receipt: Object.freeze({ ...receiptCommitted, receiptRoot }),
  });
}
