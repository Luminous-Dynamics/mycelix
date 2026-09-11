import { createPublicKey, verify as verifyEd25519 } from 'node:crypto';
import { buildMerkleCommitment, type Digest } from './merkle.js';
import {
  type ReportingEvidencePackage,
  type ReportingEvidencePackageAttestation,
  type ReportingEvidencePackageAttestationCapability,
} from './interop-line-proof-attestation.js';
import {
  assertReportingEvidencePackageIssuerTrustBundle,
  requireVerifiedReportingEvidencePackageIssuerAuthority,
  verifyReportingEvidencePackageAttestationWithTrustBundle,
  type ReportingEvidencePackageIssuerTrustBundle,
  type VerifiedReportingEvidencePackageIssuerAuthority,
} from './interop-line-proof-attestation-trust.js';

export const REPORTING_EVIDENCE_ISSUER_TRUST_BUNDLE_ATTESTATION_SCOPE =
  'accounting.reporting-evidence-package.issuer-trust-bundle.attest' as const;
const SIGNATURE_DOMAIN = 'mycelix-accounting-reporting-evidence-issuer-trust-bundle-attestation-signature-v1\0';
const SHA256_HEX = /^[0-9a-f]{64}$/;
const CANONICAL_UINT = /^(0|[1-9][0-9]*)$/;

export interface ReportingEvidenceIssuerTrustAnchorPolicy {
  readonly anchorId: string;
  readonly signerKeyId: string;
  readonly publicKeyPem: string;
  readonly validFrom: string;
  readonly validUntil: string;
  readonly minimumPolicySequence: string;
}

export interface ReportingEvidenceIssuerTrustBundleAttestationRequest {
  readonly requestId: string;
  readonly scope: typeof REPORTING_EVIDENCE_ISSUER_TRUST_BUNDLE_ATTESTATION_SCOPE;
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

export interface CreateReportingEvidenceIssuerTrustBundleAttestationRequestInput {
  readonly requestId: string;
  readonly anchorId: string;
  readonly signerKeyId: string;
  readonly policySequence: string;
  readonly predecessorBundleRoot?: Digest;
  readonly issuedAt: string;
  readonly expiresAt: string;
  readonly nonce: string;
}

export interface ReportingEvidenceIssuerTrustBundleDetachedSignature {
  readonly signerKeyId: string;
  readonly signedAt: string;
  readonly signatureBase64: string;
}

export interface ReportingEvidenceIssuerTrustBundleAttestation {
  readonly request: ReportingEvidenceIssuerTrustBundleAttestationRequest;
  readonly signerKeyId: string;
  readonly signedAt: string;
  readonly signatureBase64: string;
  readonly attestationRoot: Digest;
}

export interface VerifiedReportingEvidenceIssuerTrustBundleAttestation {
  readonly bundle: ReportingEvidencePackageIssuerTrustBundle;
  readonly attestation: ReportingEvidenceIssuerTrustBundleAttestation;
  readonly anchorId: string;
  readonly policySequence: string;
}

export interface AnchoredReportingEvidencePackageIssuerVerificationReceipt {
  readonly issuerVerificationReceiptRoot: Digest;
  readonly packageRoot: Digest;
  readonly trustBundleRoot: Digest;
  readonly trustBundleAttestationRoot: Digest;
  readonly trustAnchorId: string;
  readonly trustPolicySequence: string;
  readonly verifiedAt: string;
  readonly receiptRoot: Digest;
}

export interface AnchoredReportingEvidencePackageIssuerAuthority {
  readonly issuerAuthority: VerifiedReportingEvidencePackageIssuerAuthority;
  readonly trustBundleAuthority: VerifiedReportingEvidenceIssuerTrustBundleAttestation;
  readonly receipt: AnchoredReportingEvidencePackageIssuerVerificationReceipt;
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
  if (key.asymmetricKeyType !== 'ed25519') {
    throw new Error('reporting evidence issuer trust anchor public key must be Ed25519');
  }
  return key.export({ format: 'pem', type: 'spki' }).toString();
}

function signature(value: string): Buffer {
  if (!value.trim()) throw new Error('reporting evidence issuer trust attestation signature must be non-empty');
  const decoded = Buffer.from(value, 'base64');
  if (decoded.length !== 64 || decoded.toString('base64') !== value) {
    throw new Error('reporting evidence issuer trust attestation signature must be canonical Ed25519 base64');
  }
  return decoded;
}

function requestRoot(input: Omit<ReportingEvidenceIssuerTrustBundleAttestationRequest, 'requestRoot'>): Digest {
  return buildMerkleCommitment([{
    recordType: 'reporting_evidence_issuer_trust_bundle_attestation_request_v1',
    ...input,
  }]).root;
}

function attestationRoot(input: Omit<ReportingEvidenceIssuerTrustBundleAttestation, 'attestationRoot'>): Digest {
  return buildMerkleCommitment([{
    recordType: 'reporting_evidence_issuer_trust_bundle_attestation_v1',
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
  request: ReportingEvidenceIssuerTrustBundleAttestationRequest,
  bundle: ReportingEvidencePackageIssuerTrustBundle,
): Readonly<ReportingEvidenceIssuerTrustBundleAttestationRequest> {
  assertReportingEvidencePackageIssuerTrustBundle(bundle);
  const policySequence = positiveUint('reporting evidence issuer trust policySequence', request.policySequence);
  const committed = Object.freeze({
    requestId: required('reporting evidence issuer trust attestation requestId', request.requestId),
    scope: request.scope,
    anchorId: required('reporting evidence issuer trust attestation anchorId', request.anchorId),
    signerKeyId: required('reporting evidence issuer trust attestation signerKeyId', request.signerKeyId),
    trustBundleId: required('reporting evidence issuer trust attestation bundleId', request.trustBundleId),
    trustBundleRoot: digest('reporting evidence issuer trust attestation bundleRoot', request.trustBundleRoot),
    policyAsOf: timestamp('reporting evidence issuer trust attestation policyAsOf', request.policyAsOf),
    policySequence,
    ...(request.predecessorBundleRoot === undefined
      ? {}
      : { predecessorBundleRoot: digest('reporting evidence issuer trust predecessorBundleRoot', request.predecessorBundleRoot) }),
    issuedAt: timestamp('reporting evidence issuer trust attestation issuedAt', request.issuedAt),
    expiresAt: timestamp('reporting evidence issuer trust attestation expiresAt', request.expiresAt),
    nonce: required('reporting evidence issuer trust attestation nonce', request.nonce),
  });
  if (committed.scope !== REPORTING_EVIDENCE_ISSUER_TRUST_BUNDLE_ATTESTATION_SCOPE) {
    throw new Error('reporting evidence issuer trust attestation request has unsupported scope');
  }
  if (
    committed.trustBundleId !== bundle.bundleId
    || committed.trustBundleRoot !== bundle.bundleRoot
    || committed.policyAsOf !== bundle.policyAsOf
  ) throw new Error('reporting evidence issuer trust attestation request does not match canonical trust bundle');
  if (Date.parse(committed.issuedAt) < Date.parse(bundle.policyAsOf)) {
    throw new Error('reporting evidence issuer trust attestation cannot be issued before policyAsOf');
  }
  if (Date.parse(committed.expiresAt) <= Date.parse(committed.issuedAt)) {
    throw new Error('reporting evidence issuer trust attestation request expiry must be after issuance');
  }
  if (policySequence === '1' && committed.predecessorBundleRoot !== undefined) {
    throw new Error('initial reporting evidence issuer trust attestation must not name a predecessor');
  }
  if (policySequence !== '1' && committed.predecessorBundleRoot === undefined) {
    throw new Error('non-initial reporting evidence issuer trust attestation requires predecessor bundle root');
  }
  const expected = requestRoot(committed);
  if (request.requestRoot !== expected) {
    throw new Error('reporting evidence issuer trust attestation request root does not match canonical contents');
  }
  return Object.freeze({ ...committed, requestRoot: expected });
}

export function createReportingEvidenceIssuerTrustBundleAttestationRequest(
  bundle: ReportingEvidencePackageIssuerTrustBundle,
  input: CreateReportingEvidenceIssuerTrustBundleAttestationRequestInput,
): Readonly<ReportingEvidenceIssuerTrustBundleAttestationRequest> {
  const committed = Object.freeze({
    requestId: required('reporting evidence issuer trust attestation requestId', input.requestId),
    scope: REPORTING_EVIDENCE_ISSUER_TRUST_BUNDLE_ATTESTATION_SCOPE,
    anchorId: required('reporting evidence issuer trust attestation anchorId', input.anchorId),
    signerKeyId: required('reporting evidence issuer trust attestation signerKeyId', input.signerKeyId),
    trustBundleId: bundle.bundleId,
    trustBundleRoot: bundle.bundleRoot,
    policyAsOf: bundle.policyAsOf,
    policySequence: positiveUint('reporting evidence issuer trust policySequence', input.policySequence),
    ...(input.predecessorBundleRoot === undefined ? {} : { predecessorBundleRoot: input.predecessorBundleRoot }),
    issuedAt: timestamp('reporting evidence issuer trust attestation issuedAt', input.issuedAt),
    expiresAt: timestamp('reporting evidence issuer trust attestation expiresAt', input.expiresAt),
    nonce: required('reporting evidence issuer trust attestation nonce', input.nonce),
  });
  const request = Object.freeze({ ...committed, requestRoot: requestRoot(committed) });
  return canonicalRequest(request, bundle);
}

export function reportingEvidenceIssuerTrustBundleAttestationPayloadBase64(
  request: ReportingEvidenceIssuerTrustBundleAttestationRequest,
  bundle: ReportingEvidencePackageIssuerTrustBundle,
  signedAtInput: string,
): string {
  const canonical = canonicalRequest(request, bundle);
  const signedAt = timestamp('reporting evidence issuer trust attestation signedAt', signedAtInput);
  if (Date.parse(signedAt) < Date.parse(canonical.issuedAt) || Date.parse(signedAt) >= Date.parse(canonical.expiresAt)) {
    throw new Error('reporting evidence issuer trust attestation signature time is outside request window');
  }
  return signingMessage(canonical.requestRoot, signedAt).toString('base64');
}

export function attachReportingEvidenceIssuerTrustBundleDetachedSignature(
  bundle: ReportingEvidencePackageIssuerTrustBundle,
  request: ReportingEvidenceIssuerTrustBundleAttestationRequest,
  detached: ReportingEvidenceIssuerTrustBundleDetachedSignature,
): Readonly<ReportingEvidenceIssuerTrustBundleAttestation> {
  const canonical = canonicalRequest(request, bundle);
  const signerKeyId = required('reporting evidence issuer trust detached signerKeyId', detached.signerKeyId);
  if (signerKeyId !== canonical.signerKeyId) {
    throw new Error('reporting evidence issuer trust detached signature signer does not match request');
  }
  const signedAt = timestamp('reporting evidence issuer trust detached signedAt', detached.signedAt);
  if (Date.parse(signedAt) < Date.parse(canonical.issuedAt) || Date.parse(signedAt) >= Date.parse(canonical.expiresAt)) {
    throw new Error('reporting evidence issuer trust attestation signature time is outside request window');
  }
  signature(detached.signatureBase64);
  const committed = Object.freeze({
    request: canonical,
    signerKeyId,
    signedAt,
    signatureBase64: detached.signatureBase64,
  });
  return Object.freeze({ ...committed, attestationRoot: attestationRoot(committed) });
}

export function verifyReportingEvidenceIssuerTrustBundleAttestation(
  bundle: ReportingEvidencePackageIssuerTrustBundle,
  attestation: ReportingEvidenceIssuerTrustBundleAttestation,
  anchor: ReportingEvidenceIssuerTrustAnchorPolicy,
): Readonly<VerifiedReportingEvidenceIssuerTrustBundleAttestation> {
  const canonical = canonicalRequest(attestation.request, bundle);
  const anchorId = required('reporting evidence issuer trust anchorId', anchor.anchorId);
  const signerKeyId = required('reporting evidence issuer trust anchor signerKeyId', anchor.signerKeyId);
  const validFrom = timestamp('reporting evidence issuer trust anchor validFrom', anchor.validFrom);
  const validUntil = timestamp('reporting evidence issuer trust anchor validUntil', anchor.validUntil);
  if (Date.parse(validUntil) <= Date.parse(validFrom)) {
    throw new Error('reporting evidence issuer trust anchor validity window must be strictly increasing');
  }
  const minimum = positiveUint('reporting evidence issuer trust anchor minimumPolicySequence', anchor.minimumPolicySequence);
  if (BigInt(canonical.policySequence) < BigInt(minimum)) {
    throw new Error('reporting evidence issuer trust bundle policy sequence is below anti-rollback floor');
  }
  if (canonical.anchorId !== anchorId || canonical.signerKeyId !== signerKeyId || attestation.signerKeyId !== signerKeyId) {
    throw new Error('reporting evidence issuer trust attestation signer is not authorized by trust anchor');
  }
  const signedAt = timestamp('reporting evidence issuer trust attestation signedAt', attestation.signedAt);
  if (
    Date.parse(canonical.issuedAt) < Date.parse(validFrom)
    || Date.parse(canonical.expiresAt) > Date.parse(validUntil)
    || Date.parse(signedAt) < Date.parse(validFrom)
    || Date.parse(signedAt) >= Date.parse(validUntil)
  ) throw new Error('reporting evidence issuer trust attestation falls outside trust-anchor validity window');
  if (Date.parse(signedAt) < Date.parse(canonical.issuedAt) || Date.parse(signedAt) >= Date.parse(canonical.expiresAt)) {
    throw new Error('reporting evidence issuer trust attestation signature time is outside request window');
  }
  const key = createPublicKey(publicKeyPem(anchor.publicKeyPem));
  if (!verifyEd25519(null, signingMessage(canonical.requestRoot, signedAt), key, signature(attestation.signatureBase64))) {
    throw new Error('reporting evidence issuer trust attestation signature verification failed');
  }
  const expected = attestationRoot({
    request: canonical,
    signerKeyId,
    signedAt,
    signatureBase64: attestation.signatureBase64,
  });
  if (attestation.attestationRoot !== expected) {
    throw new Error('reporting evidence issuer trust attestation root does not match canonical contents');
  }
  const verified = Object.freeze({
    bundle,
    attestation: Object.freeze({
      request: canonical,
      signerKeyId,
      signedAt,
      signatureBase64: attestation.signatureBase64,
      attestationRoot: expected,
    }),
    anchorId,
    policySequence: canonical.policySequence,
  });
  VERIFIED_TRUST_ATTESTATIONS.add(verified);
  return verified;
}

export function requireVerifiedReportingEvidenceIssuerTrustBundleAttestation(
  authority: VerifiedReportingEvidenceIssuerTrustBundleAttestation,
): Readonly<VerifiedReportingEvidenceIssuerTrustBundleAttestation> {
  if (!VERIFIED_TRUST_ATTESTATIONS.has(authority)) {
    throw new Error('reporting evidence issuer trust bundle attestation must be produced by root verification');
  }
  return authority;
}

export function assertReportingEvidenceIssuerTrustBundleAttestationSuccessor(
  predecessor: VerifiedReportingEvidenceIssuerTrustBundleAttestation,
  successor: VerifiedReportingEvidenceIssuerTrustBundleAttestation,
): void {
  requireVerifiedReportingEvidenceIssuerTrustBundleAttestation(predecessor);
  requireVerifiedReportingEvidenceIssuerTrustBundleAttestation(successor);
  if (predecessor.anchorId !== successor.anchorId) {
    throw new Error('reporting evidence issuer trust successor must preserve trust anchor identity');
  }
  if (BigInt(successor.policySequence) !== BigInt(predecessor.policySequence) + 1n) {
    throw new Error('reporting evidence issuer trust successor must advance policy sequence exactly once');
  }
  if (successor.attestation.request.predecessorBundleRoot !== predecessor.bundle.bundleRoot) {
    throw new Error('reporting evidence issuer trust successor must bind predecessor bundle root');
  }
  if (Date.parse(successor.bundle.policyAsOf) < Date.parse(predecessor.bundle.policyAsOf)) {
    throw new Error('reporting evidence issuer trust successor cannot move policyAsOf backward');
  }
  if (Date.parse(successor.attestation.signedAt) <= Date.parse(predecessor.attestation.signedAt)) {
    throw new Error('reporting evidence issuer trust successor must be signed strictly after predecessor');
  }
}

export function verifyReportingEvidencePackageAttestationWithAnchoredIssuerTrust(
  pkg: ReportingEvidencePackage,
  capability: ReportingEvidencePackageAttestationCapability,
  attestation: ReportingEvidencePackageAttestation,
  bundle: ReportingEvidencePackageIssuerTrustBundle,
  trustAttestation: ReportingEvidenceIssuerTrustBundleAttestation,
  anchor: ReportingEvidenceIssuerTrustAnchorPolicy,
  verifiedAtInput: string,
): Readonly<AnchoredReportingEvidencePackageIssuerAuthority> {
  const trustBundleAuthority = verifyReportingEvidenceIssuerTrustBundleAttestation(bundle, trustAttestation, anchor);
  requireVerifiedReportingEvidenceIssuerTrustBundleAttestation(trustBundleAuthority);
  const verifiedAt = timestamp('anchored reporting evidence issuer verifiedAt', verifiedAtInput);
  if (Date.parse(trustBundleAuthority.attestation.signedAt) > Date.parse(verifiedAt)) {
    throw new Error('reporting evidence issuer root attestation cannot authorize an earlier verification view');
  }
  const issuerAuthority = verifyReportingEvidencePackageAttestationWithTrustBundle(
    pkg,
    capability,
    attestation,
    bundle,
    verifiedAt,
  );
  requireVerifiedReportingEvidencePackageIssuerAuthority(issuerAuthority);

  const receiptFields = Object.freeze({
    issuerVerificationReceiptRoot: issuerAuthority.receipt.receiptRoot,
    packageRoot: issuerAuthority.receipt.packageRoot,
    trustBundleRoot: bundle.bundleRoot,
    trustBundleAttestationRoot: trustBundleAuthority.attestation.attestationRoot,
    trustAnchorId: trustBundleAuthority.anchorId,
    trustPolicySequence: trustBundleAuthority.policySequence,
    verifiedAt,
  });
  const receiptRoot = buildMerkleCommitment([{
    recordType: 'anchored_reporting_evidence_package_issuer_verification_receipt_v1',
    ...receiptFields,
  }]).root;
  const authority = Object.freeze({
    issuerAuthority,
    trustBundleAuthority,
    receipt: Object.freeze({ ...receiptFields, receiptRoot }),
  });
  VERIFIED_ANCHORED_ISSUER_AUTHORITIES.add(authority);
  return authority;
}

export function requireAnchoredReportingEvidencePackageIssuerAuthority(
  authority: AnchoredReportingEvidencePackageIssuerAuthority,
): Readonly<AnchoredReportingEvidencePackageIssuerAuthority> {
  if (!VERIFIED_ANCHORED_ISSUER_AUTHORITIES.has(authority)) {
    throw new Error('reporting evidence issuer authority must be produced by root-anchored verification');
  }
  return authority;
}
