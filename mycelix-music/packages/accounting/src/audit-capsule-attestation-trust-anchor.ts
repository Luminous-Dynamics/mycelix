import { createPublicKey, verify as verifyEd25519 } from 'node:crypto';
import {
  type EconomicAuditCapsuleAttestation,
  type EconomicAuditCapsuleAttestationCapability,
} from './audit-capsule-attestation.js';
import {
  assertEconomicAuditCapsuleIssuerTrustBundle,
  verifyEconomicAuditCapsuleAttestationWithTrustBundle,
  type EconomicAuditCapsuleIssuerTrustBundle,
  type VerifiedEconomicAuditCapsuleIssuerAuthority,
} from './audit-capsule-attestation-trust.js';
import type { EconomicAuditCapsuleV2 } from './audit.js';
import { buildMerkleCommitment, type Digest } from './merkle.js';

export const ECONOMIC_AUDIT_ISSUER_TRUST_BUNDLE_ATTESTATION_SCOPE =
  'accounting.economic-audit-capsule.issuer-trust-bundle.attest' as const;
const SIGNATURE_DOMAIN = 'mycelix-accounting-economic-audit-issuer-trust-bundle-attestation-signature-v1\0';
const SHA256_HEX = /^[0-9a-f]{64}$/;
const CANONICAL_UINT = /^(0|[1-9][0-9]*)$/;

export interface EconomicAuditIssuerTrustAnchorPolicy {
  readonly anchorId: string;
  readonly signerKeyId: string;
  readonly publicKeyPem: string;
  readonly validFrom: string;
  readonly validUntil: string;
  readonly minimumPolicySequence: string;
}

export interface EconomicAuditIssuerTrustBundleAttestationRequest {
  readonly requestId: string;
  readonly scope: typeof ECONOMIC_AUDIT_ISSUER_TRUST_BUNDLE_ATTESTATION_SCOPE;
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

export interface CreateEconomicAuditIssuerTrustBundleAttestationRequestInput {
  readonly requestId: string;
  readonly anchorId: string;
  readonly signerKeyId: string;
  readonly policySequence: string;
  readonly predecessorBundleRoot?: Digest;
  readonly issuedAt: string;
  readonly expiresAt: string;
  readonly nonce: string;
}

export interface EconomicAuditIssuerTrustBundleDetachedSignature {
  readonly signerKeyId: string;
  readonly signedAt: string;
  readonly signatureBase64: string;
}

export interface EconomicAuditIssuerTrustBundleAttestation {
  readonly request: EconomicAuditIssuerTrustBundleAttestationRequest;
  readonly signerKeyId: string;
  readonly signedAt: string;
  readonly signatureBase64: string;
  readonly attestationRoot: Digest;
}

export interface VerifiedEconomicAuditIssuerTrustBundleAttestation {
  readonly bundle: EconomicAuditCapsuleIssuerTrustBundle;
  readonly attestation: EconomicAuditIssuerTrustBundleAttestation;
  readonly anchorId: string;
  readonly policySequence: string;
}

export interface AnchoredEconomicAuditCapsuleIssuerVerificationReceipt {
  readonly issuerVerificationReceiptRoot: Digest;
  readonly capsuleDigest: Digest;
  readonly trustBundleRoot: Digest;
  readonly trustBundleAttestationRoot: Digest;
  readonly trustAnchorId: string;
  readonly trustPolicySequence: string;
  readonly verifiedAt: string;
  readonly receiptRoot: Digest;
}

export interface AnchoredEconomicAuditCapsuleIssuerAuthority {
  readonly issuerAuthority: VerifiedEconomicAuditCapsuleIssuerAuthority;
  readonly trustBundleAuthority: VerifiedEconomicAuditIssuerTrustBundleAttestation;
  readonly receipt: AnchoredEconomicAuditCapsuleIssuerVerificationReceipt;
}

const VERIFIED_TRUST_ATTESTATIONS = new WeakSet<object>();
const VERIFIED_ANCHORED_ISSUER_AUTHORITIES = new WeakSet<object>();

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

function positiveUint(label: string, value: string): string {
  if (value !== value.trim() || !CANONICAL_UINT.test(value) || BigInt(value) <= 0n) {
    throw new Error(`${label} must be canonical positive-integer text`);
  }
  return value;
}

function publicKeyPem(value: string): string {
  const key = createPublicKey(value);
  if (key.asymmetricKeyType !== 'ed25519') throw new Error('audit issuer trust anchor public key must be Ed25519');
  return key.export({ format: 'pem', type: 'spki' }).toString();
}

function signature(value: string): Buffer {
  if (!value.trim()) throw new Error('audit issuer trust attestation signature must be non-empty');
  const decoded = Buffer.from(value, 'base64');
  if (decoded.length !== 64 || decoded.toString('base64') !== value) {
    throw new Error('audit issuer trust attestation signature must be canonical Ed25519 base64');
  }
  return decoded;
}

function requestRoot(input: Omit<EconomicAuditIssuerTrustBundleAttestationRequest, 'requestRoot'>): Digest {
  return buildMerkleCommitment([{ recordType: 'economic_audit_issuer_trust_bundle_attestation_request_v1', ...input }]).root;
}

function attestationRoot(input: Omit<EconomicAuditIssuerTrustBundleAttestation, 'attestationRoot'>): Digest {
  return buildMerkleCommitment([{
    recordType: 'economic_audit_issuer_trust_bundle_attestation_v1',
    requestRoot: input.request.requestRoot,
    signerKeyId: input.signerKeyId,
    signedAt: input.signedAt,
    signatureBase64: input.signatureBase64,
  }]).root;
}

function signingMessage(root: Digest, signedAt: string): Buffer {
  return Buffer.from(`${SIGNATURE_DOMAIN}${root}\0${signedAt}`, 'utf8');
}

function canonicalRequest(
  request: EconomicAuditIssuerTrustBundleAttestationRequest,
  bundle: EconomicAuditCapsuleIssuerTrustBundle,
): Readonly<EconomicAuditIssuerTrustBundleAttestationRequest> {
  assertEconomicAuditCapsuleIssuerTrustBundle(bundle);
  const policySequence = positiveUint('audit issuer trust policySequence', request.policySequence);
  const committed = Object.freeze({
    requestId: required('audit issuer trust attestation requestId', request.requestId),
    scope: request.scope,
    anchorId: required('audit issuer trust attestation anchorId', request.anchorId),
    signerKeyId: required('audit issuer trust attestation signerKeyId', request.signerKeyId),
    trustBundleId: required('audit issuer trust attestation bundleId', request.trustBundleId),
    trustBundleRoot: digest('audit issuer trust attestation bundleRoot', request.trustBundleRoot),
    policyAsOf: timestamp('audit issuer trust attestation policyAsOf', request.policyAsOf),
    policySequence,
    ...(request.predecessorBundleRoot === undefined
      ? {}
      : { predecessorBundleRoot: digest('audit issuer trust predecessorBundleRoot', request.predecessorBundleRoot) }),
    issuedAt: timestamp('audit issuer trust attestation issuedAt', request.issuedAt),
    expiresAt: timestamp('audit issuer trust attestation expiresAt', request.expiresAt),
    nonce: required('audit issuer trust attestation nonce', request.nonce),
  });
  if (committed.scope !== ECONOMIC_AUDIT_ISSUER_TRUST_BUNDLE_ATTESTATION_SCOPE) {
    throw new Error('audit issuer trust attestation request has unsupported scope');
  }
  if (
    committed.trustBundleId !== bundle.bundleId
    || committed.trustBundleRoot !== bundle.bundleRoot
    || committed.policyAsOf !== bundle.policyAsOf
  ) throw new Error('audit issuer trust attestation request does not match canonical trust bundle');
  if (Date.parse(committed.issuedAt) < Date.parse(bundle.policyAsOf)) {
    throw new Error('audit issuer trust attestation cannot be issued before policyAsOf');
  }
  if (Date.parse(committed.expiresAt) <= Date.parse(committed.issuedAt)) {
    throw new Error('audit issuer trust attestation request expiry must be after issuance');
  }
  if (policySequence === '1' && committed.predecessorBundleRoot !== undefined) {
    throw new Error('initial audit issuer trust attestation must not name a predecessor');
  }
  if (policySequence !== '1' && committed.predecessorBundleRoot === undefined) {
    throw new Error('non-initial audit issuer trust attestation requires predecessor bundle root');
  }
  const expected = requestRoot(committed);
  if (request.requestRoot !== expected) throw new Error('audit issuer trust attestation request root does not match canonical contents');
  return Object.freeze({ ...committed, requestRoot: expected });
}

export function createEconomicAuditIssuerTrustBundleAttestationRequest(
  bundle: EconomicAuditCapsuleIssuerTrustBundle,
  input: CreateEconomicAuditIssuerTrustBundleAttestationRequestInput,
): Readonly<EconomicAuditIssuerTrustBundleAttestationRequest> {
  const committed = Object.freeze({
    requestId: required('audit issuer trust attestation requestId', input.requestId),
    scope: ECONOMIC_AUDIT_ISSUER_TRUST_BUNDLE_ATTESTATION_SCOPE,
    anchorId: required('audit issuer trust attestation anchorId', input.anchorId),
    signerKeyId: required('audit issuer trust attestation signerKeyId', input.signerKeyId),
    trustBundleId: bundle.bundleId,
    trustBundleRoot: bundle.bundleRoot,
    policyAsOf: bundle.policyAsOf,
    policySequence: positiveUint('audit issuer trust policySequence', input.policySequence),
    ...(input.predecessorBundleRoot === undefined ? {} : { predecessorBundleRoot: input.predecessorBundleRoot }),
    issuedAt: timestamp('audit issuer trust attestation issuedAt', input.issuedAt),
    expiresAt: timestamp('audit issuer trust attestation expiresAt', input.expiresAt),
    nonce: required('audit issuer trust attestation nonce', input.nonce),
  });
  const request = Object.freeze({ ...committed, requestRoot: requestRoot(committed) });
  return canonicalRequest(request, bundle);
}

export function economicAuditIssuerTrustBundleAttestationPayloadBase64(
  request: EconomicAuditIssuerTrustBundleAttestationRequest,
  bundle: EconomicAuditCapsuleIssuerTrustBundle,
  signedAtInput: string,
): string {
  const canonical = canonicalRequest(request, bundle);
  const signedAt = timestamp('audit issuer trust attestation signedAt', signedAtInput);
  if (Date.parse(signedAt) < Date.parse(canonical.issuedAt) || Date.parse(signedAt) >= Date.parse(canonical.expiresAt)) {
    throw new Error('audit issuer trust attestation signature time is outside request window');
  }
  return signingMessage(canonical.requestRoot, signedAt).toString('base64');
}

export function attachEconomicAuditIssuerTrustBundleDetachedSignature(
  bundle: EconomicAuditCapsuleIssuerTrustBundle,
  request: EconomicAuditIssuerTrustBundleAttestationRequest,
  detached: EconomicAuditIssuerTrustBundleDetachedSignature,
): Readonly<EconomicAuditIssuerTrustBundleAttestation> {
  const canonical = canonicalRequest(request, bundle);
  const signerKeyId = required('audit issuer trust detached signerKeyId', detached.signerKeyId);
  if (signerKeyId !== canonical.signerKeyId) throw new Error('audit issuer trust detached signature signer does not match request');
  const signedAt = timestamp('audit issuer trust detached signedAt', detached.signedAt);
  if (Date.parse(signedAt) < Date.parse(canonical.issuedAt) || Date.parse(signedAt) >= Date.parse(canonical.expiresAt)) {
    throw new Error('audit issuer trust attestation signature time is outside request window');
  }
  signature(detached.signatureBase64);
  const committed = Object.freeze({ request: canonical, signerKeyId, signedAt, signatureBase64: detached.signatureBase64 });
  return Object.freeze({ ...committed, attestationRoot: attestationRoot(committed) });
}

export function verifyEconomicAuditIssuerTrustBundleAttestation(
  bundle: EconomicAuditCapsuleIssuerTrustBundle,
  attestation: EconomicAuditIssuerTrustBundleAttestation,
  anchor: EconomicAuditIssuerTrustAnchorPolicy,
): Readonly<VerifiedEconomicAuditIssuerTrustBundleAttestation> {
  const canonical = canonicalRequest(attestation.request, bundle);
  const anchorId = required('audit issuer trust anchorId', anchor.anchorId);
  const signerKeyId = required('audit issuer trust anchor signerKeyId', anchor.signerKeyId);
  const validFrom = timestamp('audit issuer trust anchor validFrom', anchor.validFrom);
  const validUntil = timestamp('audit issuer trust anchor validUntil', anchor.validUntil);
  if (Date.parse(validUntil) <= Date.parse(validFrom)) throw new Error('audit issuer trust anchor validity window must be strictly increasing');
  if (BigInt(canonical.policySequence) < BigInt(positiveUint('audit issuer trust anchor minimumPolicySequence', anchor.minimumPolicySequence))) {
    throw new Error('audit issuer trust bundle policy sequence is below anti-rollback floor');
  }
  if (canonical.anchorId !== anchorId || canonical.signerKeyId !== signerKeyId || attestation.signerKeyId !== signerKeyId) {
    throw new Error('audit issuer trust attestation signer is not authorized by trust anchor');
  }
  const signedAt = timestamp('audit issuer trust attestation signedAt', attestation.signedAt);
  if (
    Date.parse(canonical.issuedAt) < Date.parse(validFrom)
    || Date.parse(canonical.expiresAt) > Date.parse(validUntil)
    || Date.parse(signedAt) < Date.parse(validFrom)
    || Date.parse(signedAt) >= Date.parse(validUntil)
  ) throw new Error('audit issuer trust attestation falls outside trust-anchor validity window');
  if (Date.parse(signedAt) < Date.parse(canonical.issuedAt) || Date.parse(signedAt) >= Date.parse(canonical.expiresAt)) {
    throw new Error('audit issuer trust attestation signature time is outside request window');
  }
  const key = createPublicKey(publicKeyPem(anchor.publicKeyPem));
  if (!verifyEd25519(null, signingMessage(canonical.requestRoot, signedAt), key, signature(attestation.signatureBase64))) {
    throw new Error('audit issuer trust attestation signature verification failed');
  }
  const expected = attestationRoot({ request: canonical, signerKeyId, signedAt, signatureBase64: attestation.signatureBase64 });
  if (attestation.attestationRoot !== expected) throw new Error('audit issuer trust attestation root does not match canonical contents');
  const verified = Object.freeze({
    bundle,
    attestation: Object.freeze({ request: canonical, signerKeyId, signedAt, signatureBase64: attestation.signatureBase64, attestationRoot: expected }),
    anchorId,
    policySequence: canonical.policySequence,
  });
  VERIFIED_TRUST_ATTESTATIONS.add(verified);
  return verified;
}

export function assertEconomicAuditIssuerTrustBundleAttestationSuccessor(
  predecessor: VerifiedEconomicAuditIssuerTrustBundleAttestation,
  successor: VerifiedEconomicAuditIssuerTrustBundleAttestation,
): void {
  if (!VERIFIED_TRUST_ATTESTATIONS.has(predecessor) || !VERIFIED_TRUST_ATTESTATIONS.has(successor)) {
    throw new Error('audit issuer trust attestation lineage requires verified authorities');
  }
  if (predecessor.anchorId !== successor.anchorId) throw new Error('audit issuer trust attestation successor must preserve trust anchor identity');
  if (BigInt(successor.policySequence) !== BigInt(predecessor.policySequence) + 1n) {
    throw new Error('audit issuer trust attestation successor must advance policy sequence exactly once');
  }
  if (successor.attestation.request.predecessorBundleRoot !== predecessor.bundle.bundleRoot) {
    throw new Error('audit issuer trust attestation successor must bind predecessor bundle root');
  }
  if (Date.parse(successor.bundle.policyAsOf) < Date.parse(predecessor.bundle.policyAsOf)) {
    throw new Error('audit issuer trust attestation successor cannot move policyAsOf backward');
  }
  if (Date.parse(successor.attestation.signedAt) <= Date.parse(predecessor.attestation.signedAt)) {
    throw new Error('audit issuer trust attestation successor must be signed strictly after predecessor');
  }
}

export function verifyEconomicAuditCapsuleAttestationWithAnchoredIssuerTrust(
  capsule: EconomicAuditCapsuleV2,
  capability: EconomicAuditCapsuleAttestationCapability,
  attestation: EconomicAuditCapsuleAttestation,
  bundle: EconomicAuditCapsuleIssuerTrustBundle,
  bundleAttestation: EconomicAuditIssuerTrustBundleAttestation,
  anchor: EconomicAuditIssuerTrustAnchorPolicy,
  verifiedAtInput: string,
): Readonly<AnchoredEconomicAuditCapsuleIssuerAuthority> {
  const trustBundleAuthority = verifyEconomicAuditIssuerTrustBundleAttestation(bundle, bundleAttestation, anchor);
  const verifiedAt = timestamp('anchored audit issuer verifiedAt', verifiedAtInput);
  if (Date.parse(trustBundleAuthority.attestation.signedAt) > Date.parse(verifiedAt)) {
    throw new Error('audit issuer trust attestation cannot be newer than anchored verification time');
  }
  const issuerAuthority = verifyEconomicAuditCapsuleAttestationWithTrustBundle(
    capsule, capability, attestation, bundle, verifiedAt,
  );
  const committed = Object.freeze({
    issuerVerificationReceiptRoot: issuerAuthority.receipt.receiptRoot,
    capsuleDigest: issuerAuthority.receipt.capsuleDigest,
    trustBundleRoot: bundle.bundleRoot,
    trustBundleAttestationRoot: trustBundleAuthority.attestation.attestationRoot,
    trustAnchorId: trustBundleAuthority.anchorId,
    trustPolicySequence: trustBundleAuthority.policySequence,
    verifiedAt,
  });
  const receiptRoot = buildMerkleCommitment([{
    recordType: 'economic_audit_capsule_anchored_issuer_verification_receipt_v1',
    ...committed,
  }]).root;
  const authority = Object.freeze({
    issuerAuthority,
    trustBundleAuthority,
    receipt: Object.freeze({ ...committed, receiptRoot }),
  });
  VERIFIED_ANCHORED_ISSUER_AUTHORITIES.add(authority);
  return authority;
}

export function requireAnchoredEconomicAuditCapsuleIssuerAuthority(
  authority: AnchoredEconomicAuditCapsuleIssuerAuthority,
): Readonly<AnchoredEconomicAuditCapsuleIssuerAuthority> {
  if (!VERIFIED_ANCHORED_ISSUER_AUTHORITIES.has(authority)) {
    throw new Error('anchored audit capsule issuer authority must be produced by root trust verification');
  }
  return authority;
}
