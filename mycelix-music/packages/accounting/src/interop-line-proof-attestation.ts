import { createPublicKey, verify as verifyEd25519 } from 'node:crypto';
import { buildMerkleCommitment, type Digest } from './merkle.js';
import type {
  AttestedCisacCrdReportingProjection,
  AttestedDdexRoyaltyReportingProjection,
} from './interop.js';
import {
  assertReportingSafeDisclosureCommitment,
  type AttestedReportingEvidencePackage,
  verifyReportingSafeObligationEvidence,
} from './interop-line-proofs.js';

export const REPORTING_EVIDENCE_PACKAGE_ATTESTATION_SCOPE =
  'accounting.reporting-evidence-package.attest' as const;
const SIGNATURE_DOMAIN = 'mycelix-accounting-reporting-evidence-package-attestation-signature-v1\0';
const SHA256_HEX = /^[0-9a-f]{64}$/;

export type ReportingEvidenceProjection =
  | AttestedDdexRoyaltyReportingProjection
  | AttestedCisacCrdReportingProjection;

export type ReportingEvidencePackage = AttestedReportingEvidencePackage<ReportingEvidenceProjection>;

export interface ReportingEvidenceProfileGrant {
  readonly format: 'DDEX' | 'CISAC_CRD';
  readonly profileVersion: string;
}

export interface ReportingEvidencePackageAttestationCapability {
  readonly capabilityId: string;
  readonly scope: typeof REPORTING_EVIDENCE_PACKAGE_ATTESTATION_SCOPE;
  readonly issuerId: string;
  readonly signerKeyId: string;
  readonly compilerId: string;
  readonly compilerBuildDigest: Digest;
  readonly allowedProfiles: readonly ReportingEvidenceProfileGrant[];
  readonly validFrom: string;
  readonly validUntil: string;
  readonly capabilityRoot: Digest;
}

export interface CreateReportingEvidencePackageAttestationCapabilityInput {
  readonly capabilityId: string;
  readonly issuerId: string;
  readonly signerKeyId: string;
  readonly compilerId: string;
  readonly compilerBuildDigest: Digest;
  readonly allowedProfiles: readonly ReportingEvidenceProfileGrant[];
  readonly validFrom: string;
  readonly validUntil: string;
}

export interface ReportingEvidencePackageAttestationRequest {
  readonly requestId: string;
  readonly scope: typeof REPORTING_EVIDENCE_PACKAGE_ATTESTATION_SCOPE;
  readonly capabilityRoot: Digest;
  readonly issuerId: string;
  readonly signerKeyId: string;
  readonly packageRoot: Digest;
  readonly projectionRoot: Digest;
  readonly sidecarRoot: Digest;
  readonly capsuleDigest: Digest;
  readonly format: 'DDEX' | 'CISAC_CRD';
  readonly profileVersion: string;
  readonly compilerId: string;
  readonly compilerBuildDigest: Digest;
  readonly issuedAt: string;
  readonly expiresAt: string;
  readonly nonce: string;
  readonly requestRoot: Digest;
}

export interface ReportingEvidencePackageDetachedSignature {
  readonly requestRoot: Digest;
  readonly signerKeyId: string;
  readonly signedAt: string;
  readonly signatureBase64: string;
}

export interface ReportingEvidencePackageAttestation {
  readonly request: ReportingEvidencePackageAttestationRequest;
  readonly signerKeyId: string;
  readonly signedAt: string;
  readonly signatureBase64: string;
  readonly attestationRoot: Digest;
}

export interface ReportingEvidencePackageAttestationTrustPolicy {
  readonly issuerId: string;
  readonly signerKeyId: string;
  readonly capabilityRoot: Digest;
  readonly publicKeyPem: string;
}

export interface ReportingEvidencePackageAttestationReceipt {
  readonly packageRoot: Digest;
  readonly projectionRoot: Digest;
  readonly sidecarRoot: Digest;
  readonly capsuleDigest: Digest;
  readonly capabilityRoot: Digest;
  readonly requestRoot: Digest;
  readonly issuerId: string;
  readonly signerKeyId: string;
  readonly signedAt: string;
  readonly verifiedAt: string;
  readonly receiptRoot: Digest;
}

export interface VerifiedReportingEvidencePackageAuthority {
  readonly package: ReportingEvidencePackage;
  readonly capability: ReportingEvidencePackageAttestationCapability;
  readonly attestation: ReportingEvidencePackageAttestation;
  readonly receipt: ReportingEvidencePackageAttestationReceipt;
}

const VERIFIED_REPORTING_EVIDENCE_AUTHORITIES = new WeakSet<object>();

function required(label: string, value: string): string {
  const normalized = value.trim();
  if (!normalized) throw new Error(`${label} must be non-empty`);
  return normalized;
}

function digest(label: string, value: string): Digest {
  if (!SHA256_HEX.test(value)) throw new Error(`${label} must be a lowercase SHA-256 digest`);
  return value;
}

function timestamp(label: string, value: string): string {
  const parsed = Date.parse(value);
  if (!Number.isFinite(parsed)) throw new Error(`${label} must be a valid timestamp`);
  return new Date(parsed).toISOString();
}

function canonicalPublicKeyPem(value: string): string {
  const key = createPublicKey(value);
  if (key.asymmetricKeyType !== 'ed25519') throw new Error('reporting evidence attestation public key must be Ed25519');
  return key.export({ format: 'pem', type: 'spki' }).toString();
}

function canonicalSignature(value: string): Buffer {
  if (!value.trim()) throw new Error('reporting evidence attestation signature must be non-empty');
  const decoded = Buffer.from(value, 'base64');
  if (decoded.length !== 64 || decoded.toString('base64') !== value) {
    throw new Error('reporting evidence attestation signature must be canonical Ed25519 base64');
  }
  return decoded;
}

function canonicalProfileGrant(grant: ReportingEvidenceProfileGrant): Readonly<ReportingEvidenceProfileGrant> {
  if (grant.format !== 'DDEX' && grant.format !== 'CISAC_CRD') {
    throw new Error('reporting evidence capability contains unsupported format');
  }
  return Object.freeze({ format: grant.format, profileVersion: required('reporting evidence profileVersion', grant.profileVersion) });
}

function canonicalProfiles(grants: readonly ReportingEvidenceProfileGrant[]): readonly ReportingEvidenceProfileGrant[] {
  if (grants.length === 0) throw new Error('reporting evidence capability requires at least one profile grant');
  const ordered = grants.map(canonicalProfileGrant).sort((left, right) => {
    const format = left.format.localeCompare(right.format);
    return format !== 0 ? format : left.profileVersion.localeCompare(right.profileVersion);
  });
  const seen = new Set<string>();
  for (const grant of ordered) {
    const key = `${grant.format}\0${grant.profileVersion}`;
    if (seen.has(key)) throw new Error('reporting evidence capability profile grants must be unique');
    seen.add(key);
  }
  return Object.freeze(ordered);
}

function capabilityRoot(input: Omit<ReportingEvidencePackageAttestationCapability, 'capabilityRoot'>): Digest {
  return buildMerkleCommitment([{
    recordType: 'reporting_evidence_package_attestation_capability_v1',
    ...input,
  }]).root;
}

export function createReportingEvidencePackageAttestationCapability(
  input: CreateReportingEvidencePackageAttestationCapabilityInput,
): Readonly<ReportingEvidencePackageAttestationCapability> {
  const validFrom = timestamp('reporting evidence capability validFrom', input.validFrom);
  const validUntil = timestamp('reporting evidence capability validUntil', input.validUntil);
  if (Date.parse(validFrom) >= Date.parse(validUntil)) {
    throw new Error('reporting evidence capability validFrom must precede validUntil');
  }
  const committed = Object.freeze({
    capabilityId: required('reporting evidence capabilityId', input.capabilityId),
    scope: REPORTING_EVIDENCE_PACKAGE_ATTESTATION_SCOPE,
    issuerId: required('reporting evidence issuerId', input.issuerId),
    signerKeyId: required('reporting evidence signerKeyId', input.signerKeyId),
    compilerId: required('reporting evidence compilerId', input.compilerId),
    compilerBuildDigest: digest('reporting evidence compilerBuildDigest', input.compilerBuildDigest),
    allowedProfiles: canonicalProfiles(input.allowedProfiles),
    validFrom,
    validUntil,
  });
  return Object.freeze({ ...committed, capabilityRoot: capabilityRoot(committed) });
}

function projectionIdentity(pkg: ReportingEvidencePackage): Readonly<{
  format: 'DDEX' | 'CISAC_CRD';
  profileVersion: string;
  projectionRoot: Digest;
  capsuleDigest: Digest;
}> {
  if (pkg.authority !== 'reporting_projection_only') throw new Error('reporting evidence package must remain reporting_projection_only');
  if (pkg.evidenceStatus !== 'compiler_derived_line_evidence_not_issuer_attested') {
    throw new Error('reporting evidence package has unsupported evidence status');
  }
  assertReportingSafeDisclosureCommitment(pkg.disclosure);
  const projection = pkg.projection;
  if (projection.format !== 'DDEX' && projection.format !== 'CISAC_CRD') {
    throw new Error('reporting evidence package contains unsupported projection format');
  }
  const profileVersion = required('reporting evidence projection profileVersion', projection.profileVersion);
  const projectionRoot = digest('reporting evidence projectionRoot', projection.projectionRoot);
  const capsuleDigest = digest('reporting evidence capsuleDigest', projection.audit.capsuleDigest);
  const expectedProjectionRoot = buildMerkleCommitment([{
    recordType: projection.format === 'DDEX'
      ? 'ddex_royalty_reporting_projection_attested_audit_v1'
      : 'cisac_crd_reporting_projection_attested_audit_v1',
    authority: projection.authority,
    profileVersion,
    source: projection.source,
    audit: projection.audit,
    lines: projection.lines,
  }]).root;
  if (projectionRoot !== expectedProjectionRoot) {
    throw new Error('reporting evidence projection root does not match canonical projection contents');
  }
  if (pkg.disclosure.capsuleDigest !== capsuleDigest) {
    throw new Error('reporting evidence sidecar and projection do not reference the same audit capsule');
  }
  if (pkg.disclosure.statementSnapshotRoot !== projection.audit.statementSnapshotRoot) {
    throw new Error('reporting evidence sidecar and projection do not reference the same statement snapshot');
  }
  if (pkg.disclosure.internalObligationRoot !== projection.audit.obligationRoot) {
    throw new Error('reporting evidence sidecar and projection do not reference the same internal obligation root');
  }
  if (pkg.lineEvidence.length !== projection.lines.length) {
    throw new Error('reporting evidence package requires one line proof per projection line');
  }

  const proofByObligationId = new Map<string, (typeof pkg.lineEvidence)[number]>();
  for (const evidence of pkg.lineEvidence) {
    if (!verifyReportingSafeObligationEvidence(evidence)) throw new Error('reporting evidence package contains invalid Merkle proof');
    if (
      evidence.sidecarRoot !== pkg.disclosure.sidecarRoot
      || evidence.disclosureRoot !== pkg.disclosure.disclosureRoot
      || evidence.disclosureCount !== pkg.disclosure.disclosureCount
    ) throw new Error('reporting line proof is not bound to package disclosure sidecar');
    if (proofByObligationId.has(evidence.leaf.obligationId)) {
      throw new Error('reporting evidence package contains duplicate obligation proof');
    }
    proofByObligationId.set(evidence.leaf.obligationId, evidence);
  }

  if (projection.format === 'DDEX') {
    for (const line of projection.lines) {
      const evidence = proofByObligationId.get(line.obligationReference);
      if (!evidence) throw new Error('DDEX projection line is missing reporting-safe obligation proof');
      if (
        line.payeeReference !== evidence.leaf.beneficiaryId
        || line.royaltyAmount.amountMinor !== evidence.leaf.amountMinor
        || line.royaltyAmount.currency !== evidence.leaf.currency
      ) throw new Error('DDEX projection line does not match reporting-safe obligation proof');
    }
  } else {
    for (const line of projection.lines) {
      const evidence = proofByObligationId.get(line.distributionReference);
      if (!evidence) throw new Error('CRD projection line is missing reporting-safe obligation proof');
      if (
        line.interestedPartyReference !== evidence.leaf.beneficiaryId
        || line.royaltyAmount.amountMinor !== evidence.leaf.amountMinor
        || line.royaltyAmount.currency !== evidence.leaf.currency
      ) throw new Error('CRD projection line does not match reporting-safe obligation proof');
    }
  }

  const expectedPackageRoot = buildMerkleCommitment([{
    recordType: 'attested_reporting_evidence_package_v1',
    authority: pkg.authority,
    evidenceStatus: pkg.evidenceStatus,
    projectionRoot,
    sidecarRoot: pkg.disclosure.sidecarRoot,
    lineEvidence: pkg.lineEvidence,
  }]).root;
  if (pkg.packageRoot !== expectedPackageRoot) throw new Error('reporting evidence package root does not match canonical contents');
  return Object.freeze({ format: projection.format, profileVersion, projectionRoot, capsuleDigest });
}

function assertCapability(capability: ReportingEvidencePackageAttestationCapability): void {
  const canonical = createReportingEvidencePackageAttestationCapability(capability);
  if (canonical.capabilityRoot !== capability.capabilityRoot) {
    throw new Error('reporting evidence capability root does not match canonical contents');
  }
}

function assertPackageAllowed(
  pkg: ReportingEvidencePackage,
  capability: ReportingEvidencePackageAttestationCapability,
): ReturnType<typeof projectionIdentity> {
  assertCapability(capability);
  const identity = projectionIdentity(pkg);
  if (
    pkg.disclosure.compilerId !== capability.compilerId
    || pkg.disclosure.compilerBuildDigest !== capability.compilerBuildDigest
  ) throw new Error('reporting evidence package compiler identity is outside capability');
  if (!capability.allowedProfiles.some(grant => grant.format === identity.format && grant.profileVersion === identity.profileVersion)) {
    throw new Error('reporting evidence projection profile is outside capability');
  }
  return identity;
}

function requestRoot(input: Omit<ReportingEvidencePackageAttestationRequest, 'requestRoot'>): Digest {
  return buildMerkleCommitment([{
    recordType: 'reporting_evidence_package_attestation_request_v1',
    ...input,
  }]).root;
}

export function createReportingEvidencePackageAttestationRequest(
  pkg: ReportingEvidencePackage,
  capability: ReportingEvidencePackageAttestationCapability,
  input: {
    readonly requestId: string;
    readonly issuedAt: string;
    readonly expiresAt: string;
    readonly nonce: string;
  },
): Readonly<ReportingEvidencePackageAttestationRequest> {
  const identity = assertPackageAllowed(pkg, capability);
  const issuedAt = timestamp('reporting evidence request issuedAt', input.issuedAt);
  const expiresAt = timestamp('reporting evidence request expiresAt', input.expiresAt);
  if (Date.parse(issuedAt) < Date.parse(capability.validFrom) || Date.parse(expiresAt) > Date.parse(capability.validUntil)) {
    throw new Error('reporting evidence request falls outside capability validity window');
  }
  if (Date.parse(issuedAt) >= Date.parse(expiresAt)) throw new Error('reporting evidence request expiry must follow issuance');
  const committed = Object.freeze({
    requestId: required('reporting evidence requestId', input.requestId),
    scope: REPORTING_EVIDENCE_PACKAGE_ATTESTATION_SCOPE,
    capabilityRoot: capability.capabilityRoot,
    issuerId: capability.issuerId,
    signerKeyId: capability.signerKeyId,
    packageRoot: digest('reporting evidence packageRoot', pkg.packageRoot),
    projectionRoot: identity.projectionRoot,
    sidecarRoot: pkg.disclosure.sidecarRoot,
    capsuleDigest: identity.capsuleDigest,
    format: identity.format,
    profileVersion: identity.profileVersion,
    compilerId: capability.compilerId,
    compilerBuildDigest: capability.compilerBuildDigest,
    issuedAt,
    expiresAt,
    nonce: required('reporting evidence request nonce', input.nonce),
  });
  return Object.freeze({ ...committed, requestRoot: requestRoot(committed) });
}

function canonicalRequest(
  pkg: ReportingEvidencePackage,
  capability: ReportingEvidencePackageAttestationCapability,
  request: ReportingEvidencePackageAttestationRequest,
): Readonly<ReportingEvidencePackageAttestationRequest> {
  const rebuilt = createReportingEvidencePackageAttestationRequest(pkg, capability, {
    requestId: request.requestId,
    issuedAt: request.issuedAt,
    expiresAt: request.expiresAt,
    nonce: request.nonce,
  });
  if (
    request.scope !== rebuilt.scope
    || request.capabilityRoot !== rebuilt.capabilityRoot
    || request.issuerId !== rebuilt.issuerId
    || request.signerKeyId !== rebuilt.signerKeyId
    || request.packageRoot !== rebuilt.packageRoot
    || request.projectionRoot !== rebuilt.projectionRoot
    || request.sidecarRoot !== rebuilt.sidecarRoot
    || request.capsuleDigest !== rebuilt.capsuleDigest
    || request.format !== rebuilt.format
    || request.profileVersion !== rebuilt.profileVersion
    || request.compilerId !== rebuilt.compilerId
    || request.compilerBuildDigest !== rebuilt.compilerBuildDigest
    || request.requestRoot !== rebuilt.requestRoot
  ) throw new Error('reporting evidence attestation request does not match canonical package/capability contents');
  return rebuilt;
}

function signingMessage(root: Digest, signedAt: string): Buffer {
  return Buffer.from(`${SIGNATURE_DOMAIN}${root}\0${signedAt}`, 'utf8');
}

export function reportingEvidencePackageAttestationPayloadBase64(
  pkg: ReportingEvidencePackage,
  capability: ReportingEvidencePackageAttestationCapability,
  request: ReportingEvidencePackageAttestationRequest,
  signedAtInput: string,
): string {
  const canonical = canonicalRequest(pkg, capability, request);
  const signedAt = timestamp('reporting evidence signedAt', signedAtInput);
  if (Date.parse(signedAt) < Date.parse(canonical.issuedAt) || Date.parse(signedAt) >= Date.parse(canonical.expiresAt)) {
    throw new Error('reporting evidence signature time is outside request window');
  }
  return signingMessage(canonical.requestRoot, signedAt).toString('base64');
}

function attestationRoot(input: Omit<ReportingEvidencePackageAttestation, 'attestationRoot'>): Digest {
  return buildMerkleCommitment([{
    recordType: 'reporting_evidence_package_attestation_v1',
    requestRoot: input.request.requestRoot,
    signerKeyId: input.signerKeyId,
    signedAt: input.signedAt,
    signatureBase64: input.signatureBase64,
  }]).root;
}

export function attachReportingEvidencePackageDetachedSignature(
  pkg: ReportingEvidencePackage,
  capability: ReportingEvidencePackageAttestationCapability,
  request: ReportingEvidencePackageAttestationRequest,
  detached: ReportingEvidencePackageDetachedSignature,
): Readonly<ReportingEvidencePackageAttestation> {
  const canonical = canonicalRequest(pkg, capability, request);
  if (detached.requestRoot !== canonical.requestRoot) throw new Error('reporting evidence detached signature requestRoot mismatch');
  if (detached.signerKeyId !== canonical.signerKeyId) throw new Error('reporting evidence detached signer does not match request');
  const signedAt = timestamp('reporting evidence detached signedAt', detached.signedAt);
  if (Date.parse(signedAt) < Date.parse(canonical.issuedAt) || Date.parse(signedAt) >= Date.parse(canonical.expiresAt)) {
    throw new Error('reporting evidence signature time is outside request window');
  }
  canonicalSignature(detached.signatureBase64);
  const committed = Object.freeze({
    request: canonical,
    signerKeyId: canonical.signerKeyId,
    signedAt,
    signatureBase64: detached.signatureBase64,
  });
  return Object.freeze({ ...committed, attestationRoot: attestationRoot(committed) });
}

export function verifyReportingEvidencePackageAttestation(
  pkg: ReportingEvidencePackage,
  capability: ReportingEvidencePackageAttestationCapability,
  attestation: ReportingEvidencePackageAttestation,
  policy: ReportingEvidencePackageAttestationTrustPolicy,
  verifiedAtInput: string,
): Readonly<VerifiedReportingEvidencePackageAuthority> {
  const canonical = canonicalRequest(pkg, capability, attestation.request);
  if (policy.issuerId !== capability.issuerId || policy.signerKeyId !== capability.signerKeyId) {
    throw new Error('reporting evidence trust policy does not authorize issuer/signer');
  }
  if (policy.capabilityRoot !== capability.capabilityRoot) {
    throw new Error('reporting evidence trust policy does not pin exact capability root');
  }
  if (attestation.signerKeyId !== canonical.signerKeyId) throw new Error('reporting evidence attestation signer mismatch');
  const signedAt = timestamp('reporting evidence attestation signedAt', attestation.signedAt);
  if (Date.parse(signedAt) < Date.parse(canonical.issuedAt) || Date.parse(signedAt) >= Date.parse(canonical.expiresAt)) {
    throw new Error('reporting evidence signature time is outside request window');
  }
  const verifiedAt = timestamp('reporting evidence verifiedAt', verifiedAtInput);
  if (Date.parse(verifiedAt) < Date.parse(signedAt)) throw new Error('reporting evidence verification cannot predate signature');
  const key = createPublicKey(canonicalPublicKeyPem(policy.publicKeyPem));
  if (!verifyEd25519(null, signingMessage(canonical.requestRoot, signedAt), key, canonicalSignature(attestation.signatureBase64))) {
    throw new Error('reporting evidence attestation signature verification failed');
  }
  const expectedAttestationRoot = attestationRoot({
    request: canonical,
    signerKeyId: canonical.signerKeyId,
    signedAt,
    signatureBase64: attestation.signatureBase64,
  });
  if (attestation.attestationRoot !== expectedAttestationRoot) {
    throw new Error('reporting evidence attestation root does not match canonical contents');
  }
  const committedReceipt = Object.freeze({
    packageRoot: canonical.packageRoot,
    projectionRoot: canonical.projectionRoot,
    sidecarRoot: canonical.sidecarRoot,
    capsuleDigest: canonical.capsuleDigest,
    capabilityRoot: canonical.capabilityRoot,
    requestRoot: canonical.requestRoot,
    issuerId: canonical.issuerId,
    signerKeyId: canonical.signerKeyId,
    signedAt,
    verifiedAt,
  });
  const receiptRoot = buildMerkleCommitment([{
    recordType: 'reporting_evidence_package_attestation_receipt_v1',
    ...committedReceipt,
  }]).root;
  const authority = Object.freeze({
    package: pkg,
    capability,
    attestation: Object.freeze({
      request: canonical,
      signerKeyId: canonical.signerKeyId,
      signedAt,
      signatureBase64: attestation.signatureBase64,
      attestationRoot: expectedAttestationRoot,
    }),
    receipt: Object.freeze({ ...committedReceipt, receiptRoot }),
  });
  VERIFIED_REPORTING_EVIDENCE_AUTHORITIES.add(authority);
  return authority;
}

export function requireVerifiedReportingEvidencePackageAuthority(
  authority: VerifiedReportingEvidencePackageAuthority,
): Readonly<VerifiedReportingEvidencePackageAuthority> {
  if (!VERIFIED_REPORTING_EVIDENCE_AUTHORITIES.has(authority)) {
    throw new Error('reporting evidence package authority must be produced by detached signature verification');
  }
  return authority;
}
