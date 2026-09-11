import { createPublicKey } from 'node:crypto';
import { buildMerkleCommitment, canonicalAccountingValue, type Digest } from './merkle.js';
import {
  requireVerifiedReportingEvidencePackageAuthority,
  verifyReportingEvidencePackageAttestation,
  type ReportingEvidencePackage,
  type ReportingEvidencePackageAttestation,
  type ReportingEvidencePackageAttestationCapability,
  type VerifiedReportingEvidencePackageAuthority,
} from './interop-line-proof-attestation.js';

export interface ReportingEvidencePackageIssuerTrustEntry {
  readonly entryId: string;
  readonly issuerId: string;
  readonly signerKeyId: string;
  readonly capabilityRoot: Digest;
  readonly publicKeyPem: string;
  readonly validFrom: string;
  readonly validUntil: string;
  /** First instant at which this entry cannot authorize a new signature. */
  readonly revokedAt?: string;
}

export interface ReportingEvidencePackageIssuerTrustBundle {
  readonly bundleId: string;
  readonly policyAsOf: string;
  readonly entries: readonly ReportingEvidencePackageIssuerTrustEntry[];
  readonly bundleRoot: Digest;
}

export interface ReportingEvidencePackageIssuerVerificationReceipt {
  readonly packageRoot: Digest;
  readonly attestationVerificationReceiptRoot: Digest;
  readonly attestationRoot: Digest;
  readonly capabilityRoot: Digest;
  readonly issuerId: string;
  readonly signerKeyId: string;
  readonly signedAt: string;
  readonly trustBundleId: string;
  readonly trustBundleRoot: Digest;
  readonly trustEntryId: string;
  readonly policyAsOf: string;
  readonly verifiedAt: string;
  readonly receiptRoot: Digest;
}

export interface VerifiedReportingEvidencePackageIssuerAuthority {
  readonly attestationAuthority: VerifiedReportingEvidencePackageAuthority;
  readonly trustBundle: ReportingEvidencePackageIssuerTrustBundle;
  readonly trustEntry: ReportingEvidencePackageIssuerTrustEntry;
  readonly receipt: ReportingEvidencePackageIssuerVerificationReceipt;
}

const VERIFIED_REPORTING_EVIDENCE_ISSUER_AUTHORITIES = new WeakSet<object>();
const SHA256_HEX = /^[0-9a-f]{64}$/;

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

function canonicalPublicKeyPem(value: string): string {
  const key = createPublicKey(value);
  if (key.asymmetricKeyType !== 'ed25519') {
    throw new Error('reporting evidence issuer trust public key must be Ed25519');
  }
  return key.export({ format: 'pem', type: 'spki' }).toString();
}

function canonicalEntry(
  entry: ReportingEvidencePackageIssuerTrustEntry,
): Readonly<ReportingEvidencePackageIssuerTrustEntry> {
  const validFrom = timestamp('reporting evidence issuer trust validFrom', entry.validFrom);
  const validUntil = timestamp('reporting evidence issuer trust validUntil', entry.validUntil);
  if (Date.parse(validUntil) <= Date.parse(validFrom)) {
    throw new Error('reporting evidence issuer trust validity window must be strictly increasing');
  }
  const revokedAt = entry.revokedAt === undefined
    ? undefined
    : timestamp('reporting evidence issuer trust revokedAt', entry.revokedAt);
  if (revokedAt !== undefined && (
    Date.parse(revokedAt) < Date.parse(validFrom)
    || Date.parse(revokedAt) > Date.parse(validUntil)
  )) {
    throw new Error('reporting evidence issuer trust revocation must fall inside its validity window');
  }
  return Object.freeze({
    entryId: required('reporting evidence issuer trust entryId', entry.entryId),
    issuerId: required('reporting evidence issuer trust issuerId', entry.issuerId),
    signerKeyId: required('reporting evidence issuer trust signerKeyId', entry.signerKeyId),
    capabilityRoot: digest('reporting evidence issuer trust capabilityRoot', entry.capabilityRoot),
    publicKeyPem: canonicalPublicKeyPem(entry.publicKeyPem),
    validFrom,
    validUntil,
    ...(revokedAt === undefined ? {} : { revokedAt }),
  });
}

function windowsOverlap(
  left: ReportingEvidencePackageIssuerTrustEntry,
  right: ReportingEvidencePackageIssuerTrustEntry,
): boolean {
  return Date.parse(left.validFrom) < Date.parse(right.validUntil)
    && Date.parse(right.validFrom) < Date.parse(left.validUntil);
}

function sameAuthorityIdentity(
  left: ReportingEvidencePackageIssuerTrustEntry,
  right: ReportingEvidencePackageIssuerTrustEntry,
): boolean {
  return left.issuerId === right.issuerId
    && left.signerKeyId === right.signerKeyId
    && left.capabilityRoot === right.capabilityRoot;
}

function canonicalEntries(
  entries: readonly ReportingEvidencePackageIssuerTrustEntry[],
): readonly ReportingEvidencePackageIssuerTrustEntry[] {
  if (entries.length === 0) throw new Error('reporting evidence issuer trust bundle requires at least one entry');
  const canonical = entries.map(canonicalEntry).sort((a, b) => a.entryId.localeCompare(b.entryId));
  const ids = new Set<string>();
  for (const entry of canonical) {
    if (ids.has(entry.entryId)) throw new Error(`duplicate reporting evidence issuer trust entry id: ${entry.entryId}`);
    ids.add(entry.entryId);
  }
  for (let i = 0; i < canonical.length; i += 1) {
    for (let j = i + 1; j < canonical.length; j += 1) {
      const left = canonical[i]!;
      const right = canonical[j]!;
      if (sameAuthorityIdentity(left, right) && windowsOverlap(left, right)) {
        throw new Error('reporting evidence issuer trust bundle contains overlapping authority windows');
      }
    }
  }
  return Object.freeze(canonical);
}

function bundleCommitment(
  input: Omit<ReportingEvidencePackageIssuerTrustBundle, 'bundleRoot'>,
): Digest {
  return buildMerkleCommitment([{
    recordType: 'reporting_evidence_package_issuer_trust_bundle_v1',
    bundleId: input.bundleId,
    policyAsOf: input.policyAsOf,
    entries: input.entries,
  }]).root;
}

export function createReportingEvidencePackageIssuerTrustBundle(input: {
  readonly bundleId: string;
  readonly policyAsOf: string;
  readonly entries: readonly ReportingEvidencePackageIssuerTrustEntry[];
}): Readonly<ReportingEvidencePackageIssuerTrustBundle> {
  const committed = Object.freeze({
    bundleId: required('reporting evidence issuer trust bundleId', input.bundleId),
    policyAsOf: timestamp('reporting evidence issuer trust policyAsOf', input.policyAsOf),
    entries: canonicalEntries(input.entries),
  });
  return Object.freeze({ ...committed, bundleRoot: bundleCommitment(committed) });
}

export function assertReportingEvidencePackageIssuerTrustBundle(
  bundle: ReportingEvidencePackageIssuerTrustBundle,
): void {
  const expected = createReportingEvidencePackageIssuerTrustBundle({
    bundleId: bundle.bundleId,
    policyAsOf: bundle.policyAsOf,
    entries: bundle.entries,
  });
  if (
    bundle.bundleRoot !== expected.bundleRoot
    || canonicalAccountingValue(bundle.entries) !== canonicalAccountingValue(expected.entries)
  ) {
    throw new Error('reporting evidence issuer trust bundle does not match canonical reconstruction');
  }
}

function signingInstantAuthorized(
  entry: ReportingEvidencePackageIssuerTrustEntry,
  signedAt: string,
): boolean {
  const signedMs = Date.parse(signedAt);
  if (signedMs < Date.parse(entry.validFrom) || signedMs >= Date.parse(entry.validUntil)) return false;
  if (entry.revokedAt !== undefined && signedMs >= Date.parse(entry.revokedAt)) return false;
  return true;
}

export function verifyReportingEvidencePackageAttestationWithTrustBundle(
  pkg: ReportingEvidencePackage,
  capability: ReportingEvidencePackageAttestationCapability,
  attestation: ReportingEvidencePackageAttestation,
  bundle: ReportingEvidencePackageIssuerTrustBundle,
  verifiedAtInput: string,
): Readonly<VerifiedReportingEvidencePackageIssuerAuthority> {
  assertReportingEvidencePackageIssuerTrustBundle(bundle);
  const signedAt = timestamp('reporting evidence issuer trust verification signedAt', attestation.signedAt);
  const verifiedAt = timestamp('reporting evidence issuer trust verification verifiedAt', verifiedAtInput);
  if (Date.parse(verifiedAt) < Date.parse(signedAt)) {
    throw new Error('reporting evidence issuer trust verification cannot predate attestation signature');
  }
  if (Date.parse(bundle.policyAsOf) < Date.parse(signedAt)) {
    throw new Error('reporting evidence issuer trust policy must be observed through attestation signedAt');
  }
  if (Date.parse(bundle.policyAsOf) > Date.parse(verifiedAt)) {
    throw new Error('reporting evidence issuer trust policy cannot be newer than verification time');
  }

  const candidates = bundle.entries.filter(entry =>
    entry.issuerId === attestation.request.issuerId
    && entry.signerKeyId === attestation.signerKeyId
    && entry.capabilityRoot === capability.capabilityRoot
    && signingInstantAuthorized(entry, signedAt));
  if (candidates.length === 0) {
    throw new Error('reporting evidence attestation has no authorized issuer trust entry at signedAt');
  }
  if (candidates.length !== 1) {
    throw new Error('reporting evidence attestation matches ambiguous issuer trust entries');
  }
  const entry = candidates[0]!;

  const attestationAuthority = verifyReportingEvidencePackageAttestation(
    pkg,
    capability,
    attestation,
    {
      issuerId: entry.issuerId,
      signerKeyId: entry.signerKeyId,
      capabilityRoot: entry.capabilityRoot,
      publicKeyPem: entry.publicKeyPem,
    },
    verifiedAt,
  );
  requireVerifiedReportingEvidencePackageAuthority(attestationAuthority);

  const receiptFields = Object.freeze({
    packageRoot: attestationAuthority.receipt.packageRoot,
    attestationVerificationReceiptRoot: attestationAuthority.receipt.receiptRoot,
    attestationRoot: attestationAuthority.attestation.attestationRoot,
    capabilityRoot: entry.capabilityRoot,
    issuerId: entry.issuerId,
    signerKeyId: entry.signerKeyId,
    signedAt: attestationAuthority.attestation.signedAt,
    trustBundleId: bundle.bundleId,
    trustBundleRoot: bundle.bundleRoot,
    trustEntryId: entry.entryId,
    policyAsOf: bundle.policyAsOf,
    verifiedAt,
  });
  const receiptRoot = buildMerkleCommitment([{
    recordType: 'reporting_evidence_package_issuer_verification_receipt_v1',
    ...receiptFields,
  }]).root;
  const authority = Object.freeze({
    attestationAuthority,
    trustBundle: bundle,
    trustEntry: entry,
    receipt: Object.freeze({ ...receiptFields, receiptRoot }),
  });
  VERIFIED_REPORTING_EVIDENCE_ISSUER_AUTHORITIES.add(authority);
  return authority;
}

export function requireVerifiedReportingEvidencePackageIssuerAuthority(
  authority: VerifiedReportingEvidencePackageIssuerAuthority,
): Readonly<VerifiedReportingEvidencePackageIssuerAuthority> {
  if (!VERIFIED_REPORTING_EVIDENCE_ISSUER_AUTHORITIES.has(authority)) {
    throw new Error('reporting evidence issuer authority must be produced by trust-bundle verification');
  }
  return authority;
}
