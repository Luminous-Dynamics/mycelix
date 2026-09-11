import { createPublicKey } from 'node:crypto';
import {
  requireVerifiedEconomicAuditCapsuleAttestation,
  verifyEconomicAuditCapsuleAttestation,
  type EconomicAuditCapsuleAttestation,
  type EconomicAuditCapsuleAttestationCapability,
  type VerifiedEconomicAuditCapsuleAttestation,
} from './audit-capsule-attestation.js';
import type { EconomicAuditCapsuleV2 } from './audit.js';
import { buildMerkleCommitment, canonicalAccountingValue, type Digest } from './merkle.js';

export interface EconomicAuditCapsuleIssuerTrustEntry {
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

export interface EconomicAuditCapsuleIssuerTrustBundle {
  readonly bundleId: string;
  readonly policyAsOf: string;
  readonly entries: readonly EconomicAuditCapsuleIssuerTrustEntry[];
  readonly bundleRoot: Digest;
}

export interface EconomicAuditCapsuleIssuerVerificationReceipt {
  readonly capsuleDigest: Digest;
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

export interface VerifiedEconomicAuditCapsuleIssuerAuthority {
  readonly attestationAuthority: VerifiedEconomicAuditCapsuleAttestation;
  readonly trustBundle: EconomicAuditCapsuleIssuerTrustBundle;
  readonly trustEntry: EconomicAuditCapsuleIssuerTrustEntry;
  readonly receipt: EconomicAuditCapsuleIssuerVerificationReceipt;
}

const VERIFIED_AUDIT_ISSUER_AUTHORITIES = new WeakSet<object>();
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
    throw new Error('audit issuer trust entry public key must be Ed25519');
  }
  return key.export({ format: 'pem', type: 'spki' }).toString();
}

function canonicalEntry(
  entry: EconomicAuditCapsuleIssuerTrustEntry,
): Readonly<EconomicAuditCapsuleIssuerTrustEntry> {
  const validFrom = timestamp('audit issuer trust validFrom', entry.validFrom);
  const validUntil = timestamp('audit issuer trust validUntil', entry.validUntil);
  if (Date.parse(validUntil) <= Date.parse(validFrom)) {
    throw new Error('audit issuer trust validity window must be strictly increasing');
  }
  const revokedAt = entry.revokedAt === undefined
    ? undefined
    : timestamp('audit issuer trust revokedAt', entry.revokedAt);
  if (revokedAt !== undefined && (
    Date.parse(revokedAt) < Date.parse(validFrom)
    || Date.parse(revokedAt) > Date.parse(validUntil)
  )) {
    throw new Error('audit issuer trust revocation must fall inside its validity window');
  }
  return Object.freeze({
    entryId: required('audit issuer trust entryId', entry.entryId),
    issuerId: required('audit issuer trust issuerId', entry.issuerId),
    signerKeyId: required('audit issuer trust signerKeyId', entry.signerKeyId),
    capabilityRoot: digest('audit issuer trust capabilityRoot', entry.capabilityRoot),
    publicKeyPem: canonicalPublicKeyPem(entry.publicKeyPem),
    validFrom,
    validUntil,
    ...(revokedAt === undefined ? {} : { revokedAt }),
  });
}

function windowsOverlap(
  left: EconomicAuditCapsuleIssuerTrustEntry,
  right: EconomicAuditCapsuleIssuerTrustEntry,
): boolean {
  return Date.parse(left.validFrom) < Date.parse(right.validUntil)
    && Date.parse(right.validFrom) < Date.parse(left.validUntil);
}

function sameAuthorityIdentity(
  left: EconomicAuditCapsuleIssuerTrustEntry,
  right: EconomicAuditCapsuleIssuerTrustEntry,
): boolean {
  return left.issuerId === right.issuerId
    && left.signerKeyId === right.signerKeyId
    && left.capabilityRoot === right.capabilityRoot;
}

function canonicalEntries(
  entries: readonly EconomicAuditCapsuleIssuerTrustEntry[],
): readonly EconomicAuditCapsuleIssuerTrustEntry[] {
  if (entries.length === 0) throw new Error('audit issuer trust bundle requires at least one entry');
  const canonical = entries.map(canonicalEntry).sort((a, b) => a.entryId.localeCompare(b.entryId));
  const ids = new Set<string>();
  for (const entry of canonical) {
    if (ids.has(entry.entryId)) throw new Error(`duplicate audit issuer trust entry id: ${entry.entryId}`);
    ids.add(entry.entryId);
  }
  for (let i = 0; i < canonical.length; i += 1) {
    for (let j = i + 1; j < canonical.length; j += 1) {
      const left = canonical[i]!;
      const right = canonical[j]!;
      if (sameAuthorityIdentity(left, right) && windowsOverlap(left, right)) {
        throw new Error('audit issuer trust bundle contains overlapping authority windows');
      }
    }
  }
  return Object.freeze(canonical);
}

function bundleCommitment(
  input: Omit<EconomicAuditCapsuleIssuerTrustBundle, 'bundleRoot'>,
): Digest {
  return buildMerkleCommitment([{
    recordType: 'economic_audit_capsule_issuer_trust_bundle_v1',
    bundleId: input.bundleId,
    policyAsOf: input.policyAsOf,
    entries: input.entries,
  }]).root;
}

export function createEconomicAuditCapsuleIssuerTrustBundle(input: {
  readonly bundleId: string;
  readonly policyAsOf: string;
  readonly entries: readonly EconomicAuditCapsuleIssuerTrustEntry[];
}): Readonly<EconomicAuditCapsuleIssuerTrustBundle> {
  const committed = Object.freeze({
    bundleId: required('audit issuer trust bundleId', input.bundleId),
    policyAsOf: timestamp('audit issuer trust policyAsOf', input.policyAsOf),
    entries: canonicalEntries(input.entries),
  });
  return Object.freeze({ ...committed, bundleRoot: bundleCommitment(committed) });
}

export function assertEconomicAuditCapsuleIssuerTrustBundle(
  bundle: EconomicAuditCapsuleIssuerTrustBundle,
): void {
  const expected = createEconomicAuditCapsuleIssuerTrustBundle({
    bundleId: bundle.bundleId,
    policyAsOf: bundle.policyAsOf,
    entries: bundle.entries,
  });
  if (
    bundle.bundleRoot !== expected.bundleRoot
    || canonicalAccountingValue(bundle.entries) !== canonicalAccountingValue(expected.entries)
  ) {
    throw new Error('audit issuer trust bundle does not match canonical reconstruction');
  }
}

function signingInstantAuthorized(
  entry: EconomicAuditCapsuleIssuerTrustEntry,
  signedAt: string,
): boolean {
  const signedMs = Date.parse(signedAt);
  if (signedMs < Date.parse(entry.validFrom) || signedMs >= Date.parse(entry.validUntil)) return false;
  if (entry.revokedAt !== undefined && signedMs >= Date.parse(entry.revokedAt)) return false;
  return true;
}

export function verifyEconomicAuditCapsuleAttestationWithTrustBundle(
  capsule: EconomicAuditCapsuleV2,
  capability: EconomicAuditCapsuleAttestationCapability,
  attestation: EconomicAuditCapsuleAttestation,
  bundle: EconomicAuditCapsuleIssuerTrustBundle,
  verifiedAtInput: string,
): Readonly<VerifiedEconomicAuditCapsuleIssuerAuthority> {
  assertEconomicAuditCapsuleIssuerTrustBundle(bundle);
  const signedAt = timestamp('audit issuer trust verification signedAt', attestation.signedAt);
  const verifiedAt = timestamp('audit issuer trust verification verifiedAt', verifiedAtInput);
  if (Date.parse(verifiedAt) < Date.parse(signedAt)) {
    throw new Error('audit issuer trust verification cannot predate the attestation signature');
  }
  if (Date.parse(bundle.policyAsOf) < Date.parse(signedAt)) {
    throw new Error('audit issuer trust policy must be observed through attestation signedAt');
  }
  if (Date.parse(bundle.policyAsOf) > Date.parse(verifiedAt)) {
    throw new Error('audit issuer trust policy cannot be newer than verification time');
  }

  const candidates = bundle.entries.filter(entry =>
    entry.issuerId === attestation.request.issuerId
    && entry.signerKeyId === attestation.signerKeyId
    && entry.capabilityRoot === capability.capabilityRoot
    && signingInstantAuthorized(entry, signedAt));
  if (candidates.length === 0) {
    throw new Error('audit capsule attestation has no authorized issuer trust entry at signedAt');
  }
  if (candidates.length !== 1) {
    throw new Error('audit capsule attestation matches ambiguous issuer trust entries');
  }
  const entry = candidates[0]!;

  const attestationAuthority = verifyEconomicAuditCapsuleAttestation(
    capsule,
    capability,
    attestation,
    {
      issuerId: entry.issuerId,
      signerKeyId: entry.signerKeyId,
      publicKeyPem: entry.publicKeyPem,
      capabilityRoot: entry.capabilityRoot,
      validFrom: entry.validFrom,
      validUntil: entry.validUntil,
    },
    verifiedAt,
  );
  requireVerifiedEconomicAuditCapsuleAttestation(attestationAuthority);

  const receiptFields = Object.freeze({
    capsuleDigest: attestationAuthority.receipt.capsuleDigest,
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
    recordType: 'economic_audit_capsule_issuer_verification_receipt_v1',
    ...receiptFields,
  }]).root;
  const authority = Object.freeze({
    attestationAuthority,
    trustBundle: bundle,
    trustEntry: entry,
    receipt: Object.freeze({ ...receiptFields, receiptRoot }),
  });
  VERIFIED_AUDIT_ISSUER_AUTHORITIES.add(authority);
  return authority;
}

export function requireVerifiedEconomicAuditCapsuleIssuerAuthority(
  authority: VerifiedEconomicAuditCapsuleIssuerAuthority,
): Readonly<VerifiedEconomicAuditCapsuleIssuerAuthority> {
  if (!VERIFIED_AUDIT_ISSUER_AUTHORITIES.has(authority)) {
    throw new Error('audit capsule issuer authority must be produced by trust-bundle verification');
  }
  return authority;
}
