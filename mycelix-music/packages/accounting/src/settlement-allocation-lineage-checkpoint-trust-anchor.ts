import { createPublicKey, verify as verifyEd25519 } from 'node:crypto';
import { buildMerkleCommitment, type Digest } from './merkle.js';
import {
  verifySettlementAllocationLineageCheckpointWithTrustBundle,
  assertSettlementAllocationLineageCheckpointSignerTrustBundle,
  type SettlementAllocationLineageCheckpointSignerTrustBundle,
  type VerifiedSettlementAllocationLineageCheckpointAuthority,
} from './settlement-allocation-lineage-checkpoint-trust.js';
import type { SignedSettlementAllocationLineageCheckpoint } from './settlement-allocation-lineage-checkpoint.js';

export const SETTLEMENT_ALLOCATION_LINEAGE_TRUST_BUNDLE_ATTESTATION_SCOPE =
  'accounting.settlement-allocation-lineage.signer-trust-bundle.attest' as const;
const TRUST_ATTESTATION_SIGNATURE_DOMAIN =
  'mycelix-accounting-settlement-allocation-lineage-trust-bundle-attestation-signature-v1\0';
const SHA256_HEX = /^[0-9a-f]{64}$/;
const CANONICAL_UINT = /^(0|[1-9][0-9]*)$/;

export interface SettlementAllocationLineageTrustAnchorPolicy {
  readonly anchorId: string;
  readonly signerKeyId: string;
  readonly publicKeyPem: string;
  readonly validFrom: string;
  readonly validUntil: string;
  /** Externally pinned rollback floor for accepted trust policy. */
  readonly minimumPolicySequence: string;
}

export interface SettlementAllocationLineageTrustBundleAttestationRequest {
  readonly requestId: string;
  readonly scope: typeof SETTLEMENT_ALLOCATION_LINEAGE_TRUST_BUNDLE_ATTESTATION_SCOPE;
  readonly anchorId: string;
  readonly signerKeyId: string;
  readonly trustBundleId: string;
  readonly trustBundleRoot: Digest;
  readonly policyAsOf: string;
  readonly policySequence: string;
  readonly predecessorBundleRoot?: Digest;
  readonly issuedAt: string;
  readonly expiresAt: string;
  readonly nonce: string;
  readonly requestRoot: Digest;
}

export interface CreateSettlementAllocationLineageTrustBundleAttestationRequestInput {
  readonly requestId: string;
  readonly anchorId: string;
  readonly signerKeyId: string;
  readonly policySequence: string;
  readonly predecessorBundleRoot?: Digest;
  readonly issuedAt: string;
  readonly expiresAt: string;
  readonly nonce: string;
}

export interface SettlementAllocationLineageTrustBundleDetachedSignature {
  readonly signerKeyId: string;
  readonly signedAt: string;
  readonly signatureBase64: string;
}

export interface SettlementAllocationLineageTrustBundleAttestation {
  readonly request: SettlementAllocationLineageTrustBundleAttestationRequest;
  readonly signerKeyId: string;
  readonly signedAt: string;
  readonly signatureBase64: string;
  readonly attestationRoot: Digest;
}

export interface VerifiedSettlementAllocationLineageTrustBundleAttestation {
  readonly bundle: SettlementAllocationLineageCheckpointSignerTrustBundle;
  readonly attestation: SettlementAllocationLineageTrustBundleAttestation;
  readonly anchorId: string;
  readonly policySequence: string;
}

export interface AnchoredSettlementAllocationLineageCheckpointVerificationReceipt {
  readonly checkpointVerificationReceiptRoot: Digest;
  readonly checkpointRoot: Digest;
  readonly trustBundleRoot: Digest;
  readonly trustBundleAttestationRoot: Digest;
  readonly trustAnchorId: string;
  readonly trustPolicySequence: string;
  readonly verifiedAt: string;
  readonly receiptRoot: Digest;
}

export interface AnchoredSettlementAllocationLineageCheckpointAuthority {
  readonly checkpointAuthority: VerifiedSettlementAllocationLineageCheckpointAuthority;
  readonly trustBundleAuthority: VerifiedSettlementAllocationLineageTrustBundleAttestation;
  readonly receipt: AnchoredSettlementAllocationLineageCheckpointVerificationReceipt;
}

const VERIFIED_TRUST_ATTESTATIONS = new WeakSet<object>();
const VERIFIED_ANCHORED_CHECKPOINT_AUTHORITIES = new WeakSet<object>();

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

function digest(label: string, value: string): Digest {
  if (!SHA256_HEX.test(value)) throw new Error(`${label} must be a lowercase SHA-256 digest`);
  return value;
}

function canonicalPositiveUint(label: string, value: string): string {
  if (value !== value.trim() || !CANONICAL_UINT.test(value) || BigInt(value) <= 0n) {
    throw new Error(`${label} must be canonical positive-integer text`);
  }
  return value;
}

function canonicalPublicKeyPem(value: string): string {
  const key = createPublicKey(value);
  if (key.asymmetricKeyType !== 'ed25519') {
    throw new Error('trust anchor public key must be Ed25519');
  }
  return key.export({ format: 'pem', type: 'spki' }).toString();
}

function canonicalSignature(value: string): Buffer {
  if (!value.trim()) throw new Error('trust bundle attestation signature must be non-empty');
  const decoded = Buffer.from(value, 'base64');
  if (decoded.length !== 64 || decoded.toString('base64') !== value) {
    throw new Error('trust bundle attestation signature must be canonical Ed25519 base64');
  }
  return decoded;
}

function requestCommitment(
  input: Omit<SettlementAllocationLineageTrustBundleAttestationRequest, 'requestRoot'>,
): Digest {
  return buildMerkleCommitment([{
    recordType: 'settlement_allocation_lineage_trust_bundle_attestation_request_v1',
    ...input,
  }]).root;
}

function attestationCommitment(
  input: Omit<SettlementAllocationLineageTrustBundleAttestation, 'attestationRoot'>,
): Digest {
  return buildMerkleCommitment([{
    recordType: 'settlement_allocation_lineage_trust_bundle_attestation_v1',
    requestRoot: input.request.requestRoot,
    signerKeyId: input.signerKeyId,
    signedAt: input.signedAt,
    signatureBase64: input.signatureBase64,
  }]).root;
}

function signingMessage(requestRoot: Digest, signedAt: string): Buffer {
  return Buffer.from(`${TRUST_ATTESTATION_SIGNATURE_DOMAIN}${requestRoot}\0${signedAt}`, 'utf8');
}

function canonicalRequest(
  request: SettlementAllocationLineageTrustBundleAttestationRequest,
  bundle: SettlementAllocationLineageCheckpointSignerTrustBundle,
): Readonly<SettlementAllocationLineageTrustBundleAttestationRequest> {
  assertSettlementAllocationLineageCheckpointSignerTrustBundle(bundle);
  const policySequence = canonicalPositiveUint('trust bundle policySequence', request.policySequence);
  const committed = Object.freeze({
    requestId: required('trust bundle attestation requestId', request.requestId),
    scope: request.scope,
    anchorId: required('trust bundle attestation anchorId', request.anchorId),
    signerKeyId: required('trust bundle attestation signerKeyId', request.signerKeyId),
    trustBundleId: required('trust bundle attestation bundleId', request.trustBundleId),
    trustBundleRoot: digest('trust bundle attestation bundleRoot', request.trustBundleRoot),
    policyAsOf: timestamp('trust bundle attestation policyAsOf', request.policyAsOf),
    policySequence,
    ...(request.predecessorBundleRoot === undefined
      ? {}
      : { predecessorBundleRoot: digest('trust bundle attestation predecessorBundleRoot', request.predecessorBundleRoot) }),
    issuedAt: timestamp('trust bundle attestation issuedAt', request.issuedAt),
    expiresAt: timestamp('trust bundle attestation expiresAt', request.expiresAt),
    nonce: required('trust bundle attestation nonce', request.nonce),
  });
  if (committed.scope !== SETTLEMENT_ALLOCATION_LINEAGE_TRUST_BUNDLE_ATTESTATION_SCOPE) {
    throw new Error('trust bundle attestation request has unsupported scope');
  }
  if (committed.trustBundleId !== bundle.bundleId || committed.trustBundleRoot !== bundle.bundleRoot || committed.policyAsOf !== bundle.policyAsOf) {
    throw new Error('trust bundle attestation request does not match canonical trust bundle');
  }
  if (Date.parse(committed.issuedAt) < Date.parse(bundle.policyAsOf)) {
    throw new Error('trust bundle attestation cannot be issued before policyAsOf');
  }
  if (Date.parse(committed.expiresAt) <= Date.parse(committed.issuedAt)) {
    throw new Error('trust bundle attestation request expiry must be after issuance');
  }
  if (policySequence === '1' && committed.predecessorBundleRoot !== undefined) {
    throw new Error('initial trust bundle attestation must not name a predecessor');
  }
  if (policySequence !== '1' && committed.predecessorBundleRoot === undefined) {
    throw new Error('non-initial trust bundle attestation requires predecessor bundle root');
  }
  const expectedRoot = requestCommitment(committed);
  if (request.requestRoot !== expectedRoot) {
    throw new Error('trust bundle attestation request root does not match canonical contents');
  }
  return Object.freeze({ ...committed, requestRoot: expectedRoot });
}

export function createSettlementAllocationLineageTrustBundleAttestationRequest(
  bundle: SettlementAllocationLineageCheckpointSignerTrustBundle,
  input: CreateSettlementAllocationLineageTrustBundleAttestationRequestInput,
): Readonly<SettlementAllocationLineageTrustBundleAttestationRequest> {
  assertSettlementAllocationLineageCheckpointSignerTrustBundle(bundle);
  const committed = Object.freeze({
    requestId: required('trust bundle attestation requestId', input.requestId),
    scope: SETTLEMENT_ALLOCATION_LINEAGE_TRUST_BUNDLE_ATTESTATION_SCOPE,
    anchorId: required('trust bundle attestation anchorId', input.anchorId),
    signerKeyId: required('trust bundle attestation signerKeyId', input.signerKeyId),
    trustBundleId: bundle.bundleId,
    trustBundleRoot: bundle.bundleRoot,
    policyAsOf: bundle.policyAsOf,
    policySequence: canonicalPositiveUint('trust bundle policySequence', input.policySequence),
    ...(input.predecessorBundleRoot === undefined ? {} : { predecessorBundleRoot: digest('trust bundle predecessorBundleRoot', input.predecessorBundleRoot) }),
    issuedAt: timestamp('trust bundle attestation issuedAt', input.issuedAt),
    expiresAt: timestamp('trust bundle attestation expiresAt', input.expiresAt),
    nonce: required('trust bundle attestation nonce', input.nonce),
  });
  const request = Object.freeze({ ...committed, requestRoot: requestCommitment(committed) });
  return canonicalRequest(request, bundle);
}

export function settlementAllocationLineageTrustBundleAttestationPayloadBase64(
  request: SettlementAllocationLineageTrustBundleAttestationRequest,
  bundle: SettlementAllocationLineageCheckpointSignerTrustBundle,
  signedAtInput: string,
): string {
  const canonical = canonicalRequest(request, bundle);
  const signedAt = timestamp('trust bundle attestation signedAt', signedAtInput);
  if (Date.parse(signedAt) < Date.parse(canonical.issuedAt) || Date.parse(signedAt) >= Date.parse(canonical.expiresAt)) {
    throw new Error('trust bundle attestation signature time is outside request window');
  }
  return signingMessage(canonical.requestRoot, signedAt).toString('base64');
}

export function attachSettlementAllocationLineageTrustBundleDetachedSignature(
  bundle: SettlementAllocationLineageCheckpointSignerTrustBundle,
  request: SettlementAllocationLineageTrustBundleAttestationRequest,
  detached: SettlementAllocationLineageTrustBundleDetachedSignature,
): Readonly<SettlementAllocationLineageTrustBundleAttestation> {
  const canonical = canonicalRequest(request, bundle);
  const signerKeyId = required('trust bundle attestation detached signerKeyId', detached.signerKeyId);
  if (signerKeyId !== canonical.signerKeyId) {
    throw new Error('trust bundle attestation detached signature signer does not match request');
  }
  const signedAt = timestamp('trust bundle attestation detached signedAt', detached.signedAt);
  if (Date.parse(signedAt) < Date.parse(canonical.issuedAt) || Date.parse(signedAt) >= Date.parse(canonical.expiresAt)) {
    throw new Error('trust bundle attestation signature time is outside request window');
  }
  canonicalSignature(detached.signatureBase64);
  const committed = Object.freeze({
    request: canonical,
    signerKeyId,
    signedAt,
    signatureBase64: detached.signatureBase64,
  });
  return Object.freeze({ ...committed, attestationRoot: attestationCommitment(committed) });
}

export function verifySettlementAllocationLineageTrustBundleAttestation(
  bundle: SettlementAllocationLineageCheckpointSignerTrustBundle,
  attestation: SettlementAllocationLineageTrustBundleAttestation,
  anchor: SettlementAllocationLineageTrustAnchorPolicy,
): Readonly<VerifiedSettlementAllocationLineageTrustBundleAttestation> {
  const canonical = canonicalRequest(attestation.request, bundle);
  const anchorId = required('trust anchorId', anchor.anchorId);
  const anchorSignerKeyId = required('trust anchor signerKeyId', anchor.signerKeyId);
  const validFrom = timestamp('trust anchor validFrom', anchor.validFrom);
  const validUntil = timestamp('trust anchor validUntil', anchor.validUntil);
  if (Date.parse(validUntil) <= Date.parse(validFrom)) throw new Error('trust anchor validity window must be strictly increasing');
  const minimumSequence = canonicalPositiveUint('trust anchor minimumPolicySequence', anchor.minimumPolicySequence);
  if (BigInt(canonical.policySequence) < BigInt(minimumSequence)) {
    throw new Error('trust bundle policy sequence is below anti-rollback floor');
  }
  if (canonical.anchorId !== anchorId || canonical.signerKeyId !== anchorSignerKeyId || attestation.signerKeyId !== anchorSignerKeyId) {
    throw new Error('trust bundle attestation signer is not authorized by trust anchor');
  }
  const signedAt = timestamp('trust bundle attestation signedAt', attestation.signedAt);
  if (
    Date.parse(canonical.issuedAt) < Date.parse(validFrom)
    || Date.parse(canonical.expiresAt) > Date.parse(validUntil)
    || Date.parse(signedAt) < Date.parse(validFrom)
    || Date.parse(signedAt) >= Date.parse(validUntil)
  ) {
    throw new Error('trust bundle attestation falls outside trust-anchor validity window');
  }
  if (Date.parse(signedAt) < Date.parse(canonical.issuedAt) || Date.parse(signedAt) >= Date.parse(canonical.expiresAt)) {
    throw new Error('trust bundle attestation signature time is outside request window');
  }
  const publicKey = createPublicKey(canonicalPublicKeyPem(anchor.publicKeyPem));
  const signature = canonicalSignature(attestation.signatureBase64);
  if (!verifyEd25519(null, signingMessage(canonical.requestRoot, signedAt), publicKey, signature)) {
    throw new Error('trust bundle attestation signature verification failed');
  }
  const expectedRoot = attestationCommitment({
    request: canonical,
    signerKeyId: anchorSignerKeyId,
    signedAt,
    signatureBase64: attestation.signatureBase64,
  });
  if (attestation.attestationRoot !== expectedRoot) {
    throw new Error('trust bundle attestation root does not match canonical contents');
  }
  const verified = Object.freeze({
    bundle,
    attestation: Object.freeze({
      request: canonical,
      signerKeyId: anchorSignerKeyId,
      signedAt,
      signatureBase64: attestation.signatureBase64,
      attestationRoot: expectedRoot,
    }),
    anchorId,
    policySequence: canonical.policySequence,
  });
  VERIFIED_TRUST_ATTESTATIONS.add(verified);
  return verified;
}

export function assertSettlementAllocationLineageTrustBundleAttestationSuccessor(
  predecessor: VerifiedSettlementAllocationLineageTrustBundleAttestation,
  successor: VerifiedSettlementAllocationLineageTrustBundleAttestation,
): void {
  if (!VERIFIED_TRUST_ATTESTATIONS.has(predecessor) || !VERIFIED_TRUST_ATTESTATIONS.has(successor)) {
    throw new Error('trust bundle attestation lineage requires verified attestation authorities');
  }
  if (predecessor.anchorId !== successor.anchorId) throw new Error('trust bundle attestation successor must preserve trust anchor identity');
  if (BigInt(successor.policySequence) !== BigInt(predecessor.policySequence) + 1n) {
    throw new Error('trust bundle attestation successor must advance policy sequence exactly once');
  }
  if (successor.attestation.request.predecessorBundleRoot !== predecessor.bundle.bundleRoot) {
    throw new Error('trust bundle attestation successor must bind predecessor bundle root');
  }
  if (Date.parse(successor.bundle.policyAsOf) < Date.parse(predecessor.bundle.policyAsOf)) {
    throw new Error('trust bundle attestation successor cannot move policyAsOf backward');
  }
  if (Date.parse(successor.attestation.signedAt) <= Date.parse(predecessor.attestation.signedAt)) {
    throw new Error('trust bundle attestation successor must be signed strictly after predecessor');
  }
}

export function verifySettlementAllocationLineageCheckpointWithAnchoredTrustBundle(
  checkpoint: SignedSettlementAllocationLineageCheckpoint,
  bundle: SettlementAllocationLineageCheckpointSignerTrustBundle,
  attestation: SettlementAllocationLineageTrustBundleAttestation,
  anchor: SettlementAllocationLineageTrustAnchorPolicy,
  verifiedAtInput: string,
): Readonly<AnchoredSettlementAllocationLineageCheckpointAuthority> {
  const trustBundleAuthority = verifySettlementAllocationLineageTrustBundleAttestation(bundle, attestation, anchor);
  const checkpointAuthority = verifySettlementAllocationLineageCheckpointWithTrustBundle(checkpoint, bundle, verifiedAtInput);
  const verifiedAt = timestamp('anchored checkpoint verifiedAt', verifiedAtInput);
  const committed = Object.freeze({
    checkpointVerificationReceiptRoot: checkpointAuthority.receipt.receiptRoot,
    checkpointRoot: checkpointAuthority.checkpoint.checkpointRoot,
    trustBundleRoot: bundle.bundleRoot,
    trustBundleAttestationRoot: trustBundleAuthority.attestation.attestationRoot,
    trustAnchorId: trustBundleAuthority.anchorId,
    trustPolicySequence: trustBundleAuthority.policySequence,
    verifiedAt,
  });
  const receiptRoot = buildMerkleCommitment([{
    recordType: 'settlement_allocation_lineage_checkpoint_anchored_verification_receipt_v1',
    ...committed,
  }]).root;
  const authority = Object.freeze({
    checkpointAuthority,
    trustBundleAuthority,
    receipt: Object.freeze({ ...committed, receiptRoot }),
  });
  VERIFIED_ANCHORED_CHECKPOINT_AUTHORITIES.add(authority);
  return authority;
}

export function requireAnchoredSettlementAllocationLineageCheckpointAuthority(
  authority: AnchoredSettlementAllocationLineageCheckpointAuthority,
): Readonly<AnchoredSettlementAllocationLineageCheckpointAuthority> {
  if (!VERIFIED_ANCHORED_CHECKPOINT_AUTHORITIES.has(authority as object)) {
    throw new Error('settlement allocation lineage checkpoint requires verified anchored trust authority');
  }
  return authority;
}
