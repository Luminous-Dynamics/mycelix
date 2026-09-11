import { createPublicKey } from 'node:crypto';
import {
  accountingWireDigestFromText,
  deserializeAccountingWire,
  serializeAccountingWire,
} from './accounting-wire.js';
import {
  economicAuditCapsuleDigest,
  type EconomicAuditCapsuleV2,
} from './audit.js';
import type {
  EconomicAuditCapsuleAttestation,
  EconomicAuditCapsuleAttestationCapability,
} from './audit-capsule-attestation.js';
import type { EconomicAuditCapsuleIssuerTrustBundle } from './audit-capsule-attestation-trust.js';
import {
  requireAnchoredEconomicAuditCapsuleIssuerAuthority,
  verifyEconomicAuditCapsuleAttestationWithAnchoredIssuerTrust,
  type AnchoredEconomicAuditCapsuleIssuerAuthority,
  type EconomicAuditIssuerTrustAnchorPolicy,
  type EconomicAuditIssuerTrustBundleAttestation,
} from './audit-capsule-attestation-trust-anchor.js';
import type { AttestedAuditReportingReference } from './interop.js';
import type {
  ReportingEvidencePackage,
  ReportingEvidencePackageAttestation,
  ReportingEvidencePackageAttestationCapability,
} from './interop-line-proof-attestation.js';
import type { ReportingEvidencePackageIssuerTrustBundle } from './interop-line-proof-attestation-trust.js';
import {
  requireAnchoredReportingEvidencePackageIssuerAuthority,
  verifyReportingEvidencePackageAttestationWithAnchoredIssuerTrust,
  type AnchoredReportingEvidencePackageIssuerAuthority,
  type ReportingEvidenceIssuerTrustAnchorPolicy,
  type ReportingEvidenceIssuerTrustBundleAttestation,
} from './interop-line-proof-attestation-trust-anchor.js';
import { buildMerkleCommitment, type Digest } from './merkle.js';

export const PORTABLE_REPORTING_EVIDENCE_PROTOCOL = 'mycelix-reporting-evidence-portable-v1' as const;
const SHA256_HEX = /^[0-9a-f]{64}$/;
const CANONICAL_POSITIVE_UINT = /^[1-9][0-9]*$/;

export interface PortableReportingEvidenceBundleV1 {
  readonly artifactKind: typeof PORTABLE_REPORTING_EVIDENCE_PROTOCOL;
  readonly protocolVersion: 1;
  readonly capsule: EconomicAuditCapsuleV2;
  readonly auditCapability: EconomicAuditCapsuleAttestationCapability;
  readonly auditAttestation: EconomicAuditCapsuleAttestation;
  readonly auditIssuerTrustBundle: EconomicAuditCapsuleIssuerTrustBundle;
  readonly auditIssuerTrustAttestation: EconomicAuditIssuerTrustBundleAttestation;
  readonly package: ReportingEvidencePackage;
  readonly reportingCapability: ReportingEvidencePackageAttestationCapability;
  readonly reportingAttestation: ReportingEvidencePackageAttestation;
  readonly reportingIssuerTrustBundle: ReportingEvidencePackageIssuerTrustBundle;
  readonly reportingIssuerTrustAttestation: ReportingEvidenceIssuerTrustBundleAttestation;
  readonly bundleRoot: Digest;
}

export interface PortableReportingEvidenceVerificationReceipt {
  readonly wireDigest: Digest;
  readonly bundleRoot: Digest;
  readonly capsuleDigest: Digest;
  readonly auditAnchoredVerificationReceiptRoot: Digest;
  readonly auditRootPolicyRoot: Digest;
  readonly packageRoot: Digest;
  readonly reportingAnchoredVerificationReceiptRoot: Digest;
  readonly reportingRootPolicyRoot: Digest;
  readonly auditTrustAnchorId: string;
  readonly auditTrustPolicySequence: string;
  readonly reportingTrustAnchorId: string;
  readonly reportingTrustPolicySequence: string;
  readonly lineEvidenceScope: 'issuer_attested_compiler_derived_merkle_not_zero_knowledge';
  readonly verifiedAt: string;
  readonly receiptRoot: Digest;
}

export interface VerifiedPortableReportingEvidence {
  readonly bundle: PortableReportingEvidenceBundleV1;
  readonly auditAuthority: AnchoredEconomicAuditCapsuleIssuerAuthority;
  readonly reportingAuthority: AnchoredReportingEvidencePackageIssuerAuthority;
  readonly receipt: PortableReportingEvidenceVerificationReceipt;
}

const VERIFIED_PORTABLE_REPORTING_EVIDENCE = new WeakSet<object>();

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

function rootPolicyRoot(
  kind: 'audit' | 'reporting',
  policy: EconomicAuditIssuerTrustAnchorPolicy | ReportingEvidenceIssuerTrustAnchorPolicy,
): Digest {
  const anchorId = required(`portable ${kind} root anchorId`, policy.anchorId);
  const signerKeyId = required(`portable ${kind} root signerKeyId`, policy.signerKeyId);
  const key = createPublicKey(policy.publicKeyPem);
  if (key.asymmetricKeyType !== 'ed25519') {
    throw new Error(`portable ${kind} root public key must be Ed25519`);
  }
  const publicKeyPem = key.export({ format: 'pem', type: 'spki' }).toString();
  const validFrom = timestamp(`portable ${kind} root validFrom`, policy.validFrom);
  const validUntil = timestamp(`portable ${kind} root validUntil`, policy.validUntil);
  if (Date.parse(validUntil) <= Date.parse(validFrom)) {
    throw new Error(`portable ${kind} root validity window must be strictly increasing`);
  }
  const minimumPolicySequence = policy.minimumPolicySequence;
  if (
    minimumPolicySequence !== minimumPolicySequence.trim()
    || !CANONICAL_POSITIVE_UINT.test(minimumPolicySequence)
  ) {
    throw new Error(`portable ${kind} root minimumPolicySequence must be canonical positive-integer text`);
  }
  return buildMerkleCommitment([{
    recordType: 'portable_external_root_policy_v1',
    kind,
    anchorId,
    signerKeyId,
    publicKeyPem,
    validFrom,
    validUntil,
    minimumPolicySequence,
  }]).root;
}

function exactTopLevelBundle(value: unknown): PortableReportingEvidenceBundleV1 {
  if (value === null || typeof value !== 'object' || Array.isArray(value)) {
    throw new Error('portable reporting evidence must decode to an object');
  }
  const record = value as Record<string, unknown>;
  const expected = [
    'artifactKind',
    'auditAttestation',
    'auditCapability',
    'auditIssuerTrustAttestation',
    'auditIssuerTrustBundle',
    'bundleRoot',
    'capsule',
    'package',
    'protocolVersion',
    'reportingAttestation',
    'reportingCapability',
    'reportingIssuerTrustAttestation',
    'reportingIssuerTrustBundle',
  ] as const;
  const keys = Object.keys(record);
  if (keys.length !== expected.length || keys.some((key, index) => key !== expected[index])) {
    throw new Error('portable reporting evidence has unexpected top-level fields');
  }
  if (record.artifactKind !== PORTABLE_REPORTING_EVIDENCE_PROTOCOL || record.protocolVersion !== 1) {
    throw new Error('unsupported portable reporting evidence protocol/version');
  }
  return value as PortableReportingEvidenceBundleV1;
}

function bundleRoot(input: Omit<PortableReportingEvidenceBundleV1, 'bundleRoot'>): Digest {
  return buildMerkleCommitment([{
    recordType: 'portable_reporting_evidence_bundle_v1',
    artifactKind: input.artifactKind,
    protocolVersion: input.protocolVersion,
    capsuleDigest: economicAuditCapsuleDigest(input.capsule),
    auditCapabilityRoot: digest('portable audit capabilityRoot', input.auditCapability.capabilityRoot),
    auditAttestationRoot: digest('portable audit attestationRoot', input.auditAttestation.attestationRoot),
    auditIssuerTrustBundleRoot: digest('portable audit trust bundleRoot', input.auditIssuerTrustBundle.bundleRoot),
    auditIssuerTrustAttestationRoot: digest(
      'portable audit trust attestationRoot',
      input.auditIssuerTrustAttestation.attestationRoot,
    ),
    packageRoot: digest('portable packageRoot', input.package.packageRoot),
    reportingCapabilityRoot: digest('portable reporting capabilityRoot', input.reportingCapability.capabilityRoot),
    reportingAttestationRoot: digest('portable reporting attestationRoot', input.reportingAttestation.attestationRoot),
    reportingIssuerTrustBundleRoot: digest(
      'portable reporting trust bundleRoot',
      input.reportingIssuerTrustBundle.bundleRoot,
    ),
    reportingIssuerTrustAttestationRoot: digest(
      'portable reporting trust attestationRoot',
      input.reportingIssuerTrustAttestation.attestationRoot,
    ),
  }]).root;
}

export function createPortableReportingEvidenceBundle(input: {
  readonly capsule: EconomicAuditCapsuleV2;
  readonly auditCapability: EconomicAuditCapsuleAttestationCapability;
  readonly auditAttestation: EconomicAuditCapsuleAttestation;
  readonly auditIssuerTrustBundle: EconomicAuditCapsuleIssuerTrustBundle;
  readonly auditIssuerTrustAttestation: EconomicAuditIssuerTrustBundleAttestation;
  readonly package: ReportingEvidencePackage;
  readonly reportingCapability: ReportingEvidencePackageAttestationCapability;
  readonly reportingAttestation: ReportingEvidencePackageAttestation;
  readonly reportingIssuerTrustBundle: ReportingEvidencePackageIssuerTrustBundle;
  readonly reportingIssuerTrustAttestation: ReportingEvidenceIssuerTrustBundleAttestation;
}): Readonly<PortableReportingEvidenceBundleV1> {
  const committed = Object.freeze({
    artifactKind: PORTABLE_REPORTING_EVIDENCE_PROTOCOL,
    protocolVersion: 1 as const,
    capsule: input.capsule,
    auditCapability: input.auditCapability,
    auditAttestation: input.auditAttestation,
    auditIssuerTrustBundle: input.auditIssuerTrustBundle,
    auditIssuerTrustAttestation: input.auditIssuerTrustAttestation,
    package: input.package,
    reportingCapability: input.reportingCapability,
    reportingAttestation: input.reportingAttestation,
    reportingIssuerTrustBundle: input.reportingIssuerTrustBundle,
    reportingIssuerTrustAttestation: input.reportingIssuerTrustAttestation,
  });
  return Object.freeze({ ...committed, bundleRoot: bundleRoot(committed) });
}

export function serializePortableReportingEvidenceBundle(bundle: PortableReportingEvidenceBundleV1): string {
  const expected = createPortableReportingEvidenceBundle(bundle);
  if (bundle.bundleRoot !== expected.bundleRoot) {
    throw new Error('portable reporting evidence bundleRoot does not match canonical contents');
  }
  return serializeAccountingWire(expected);
}

function assertAuditReference(
  bundle: PortableReportingEvidenceBundleV1,
  auditAuthority: AnchoredEconomicAuditCapsuleIssuerAuthority,
): void {
  const capsule = bundle.capsule;
  const projection = bundle.package.projection;
  const reference: AttestedAuditReportingReference = projection.audit;
  const capsuleDigest = economicAuditCapsuleDigest(capsule);
  const expected = Object.freeze({
    provenanceKind: 'root_attested_economic_audit_capsule_v1' as const,
    capsuleProtocolVersion: 2 as const,
    capsuleDigest,
    statementSnapshotRoot: capsule.statementSnapshotRoot,
    obligationRoot: capsule.statement.obligationRoot,
    adjustmentRoot: capsule.statement.adjustmentRoot,
    settlementRoot: capsule.statement.settlementRoot,
    compilerId: capsule.generatedBy.id,
    compilerBuildDigest: capsule.generatedBy.buildDigest,
    issuerVerificationReceiptRoot: auditAuthority.issuerAuthority.receipt.receiptRoot,
    anchoredIssuerVerificationReceiptRoot: auditAuthority.receipt.receiptRoot,
    issuerTrustBundleRoot: auditAuthority.receipt.trustBundleRoot,
    issuerTrustBundleAttestationRoot: auditAuthority.receipt.trustBundleAttestationRoot,
    issuerTrustAnchorId: auditAuthority.receipt.trustAnchorId,
    issuerTrustPolicySequence: auditAuthority.receipt.trustPolicySequence,
    issuerVerifiedAt: auditAuthority.receipt.verifiedAt,
    lineEvidenceScope: 'statement_level_reference_only' as const,
  });
  const provenanceRoot = buildMerkleCommitment([{
    recordType: 'interop_root_attested_audit_reference_v1',
    ...expected,
  }]).root;

  for (const [key, value] of Object.entries(expected)) {
    if ((reference as unknown as Record<string, unknown>)[key] !== value) {
      throw new Error(`portable reporting audit reference mismatch: ${key}`);
    }
  }
  if (reference.provenanceRoot !== provenanceRoot) {
    throw new Error('portable reporting audit reference provenanceRoot does not match verified audit authority');
  }
  if (
    projection.source.statementId !== capsule.statement.statementId
    || projection.source.statementCommitmentRoot !== capsule.statementSnapshotRoot
  ) {
    throw new Error('portable reporting source does not match verified audit statement');
  }
  const generatedAt = timestamp('portable reporting generatedAt', projection.source.generatedAt);
  if (Date.parse(generatedAt) < Date.parse(capsule.statement.asOf)) {
    throw new Error('portable reporting projection predates audited statement asOf');
  }
  if (Date.parse(generatedAt) < Date.parse(auditAuthority.receipt.verifiedAt)) {
    throw new Error('portable reporting projection predates verified audit authority');
  }

  const disclosure = bundle.package.disclosure;
  if (
    disclosure.capsuleDigest !== capsuleDigest
    || disclosure.statementId !== capsule.statement.statementId
    || disclosure.statementSnapshotRoot !== capsule.statementSnapshotRoot
    || disclosure.internalObligationRoot !== capsule.statement.obligationRoot
    || disclosure.beneficiaryId !== capsule.statement.beneficiaryId
    || disclosure.currency !== capsule.statement.gross.currency
    || disclosure.compilerId !== capsule.generatedBy.id
    || disclosure.compilerBuildDigest !== capsule.generatedBy.buildDigest
  ) {
    throw new Error('portable reporting disclosure sidecar does not match verified audit capsule');
  }
}

export function verifyPortableReportingEvidenceWire(
  wireText: string,
  policies: {
    readonly auditIssuerRoot: EconomicAuditIssuerTrustAnchorPolicy;
    readonly reportingIssuerRoot: ReportingEvidenceIssuerTrustAnchorPolicy;
    readonly verifiedAt: string;
  },
): Readonly<VerifiedPortableReportingEvidence> {
  const wireDigest = accountingWireDigestFromText(wireText);
  const bundle = exactTopLevelBundle(deserializeAccountingWire(wireText));
  const canonicalBundle = createPortableReportingEvidenceBundle(bundle);
  if (bundle.bundleRoot !== canonicalBundle.bundleRoot) {
    throw new Error('portable reporting evidence bundleRoot does not match decoded contents');
  }
  const auditRootPolicyRoot = rootPolicyRoot('audit', policies.auditIssuerRoot);
  const reportingRootPolicyRoot = rootPolicyRoot('reporting', policies.reportingIssuerRoot);

  const auditVerifiedAt = timestamp(
    'portable audit issuer verifiedAt',
    bundle.package.projection.audit.issuerVerifiedAt,
  );
  const auditAuthority = verifyEconomicAuditCapsuleAttestationWithAnchoredIssuerTrust(
    bundle.capsule,
    bundle.auditCapability,
    bundle.auditAttestation,
    bundle.auditIssuerTrustBundle,
    bundle.auditIssuerTrustAttestation,
    policies.auditIssuerRoot,
    auditVerifiedAt,
  );
  requireAnchoredEconomicAuditCapsuleIssuerAuthority(auditAuthority);

  const verifiedAt = timestamp('portable reporting verifiedAt', policies.verifiedAt);
  const reportingAuthority = verifyReportingEvidencePackageAttestationWithAnchoredIssuerTrust(
    bundle.package,
    bundle.reportingCapability,
    bundle.reportingAttestation,
    bundle.reportingIssuerTrustBundle,
    bundle.reportingIssuerTrustAttestation,
    policies.reportingIssuerRoot,
    verifiedAt,
  );
  requireAnchoredReportingEvidencePackageIssuerAuthority(reportingAuthority);
  assertAuditReference(bundle, auditAuthority);

  const receiptFields = Object.freeze({
    wireDigest,
    bundleRoot: canonicalBundle.bundleRoot,
    capsuleDigest: economicAuditCapsuleDigest(bundle.capsule),
    auditAnchoredVerificationReceiptRoot: auditAuthority.receipt.receiptRoot,
    auditRootPolicyRoot,
    packageRoot: bundle.package.packageRoot,
    reportingAnchoredVerificationReceiptRoot: reportingAuthority.receipt.receiptRoot,
    reportingRootPolicyRoot,
    auditTrustAnchorId: auditAuthority.receipt.trustAnchorId,
    auditTrustPolicySequence: auditAuthority.receipt.trustPolicySequence,
    reportingTrustAnchorId: reportingAuthority.receipt.trustAnchorId,
    reportingTrustPolicySequence: reportingAuthority.receipt.trustPolicySequence,
    lineEvidenceScope: 'issuer_attested_compiler_derived_merkle_not_zero_knowledge' as const,
    verifiedAt,
  });
  const receiptRoot = buildMerkleCommitment([{
    recordType: 'portable_reporting_evidence_verification_receipt_v1',
    ...receiptFields,
  }]).root;
  const authority = Object.freeze({
    bundle: canonicalBundle,
    auditAuthority,
    reportingAuthority,
    receipt: Object.freeze({ ...receiptFields, receiptRoot }),
  });
  VERIFIED_PORTABLE_REPORTING_EVIDENCE.add(authority);
  return authority;
}

export function requireVerifiedPortableReportingEvidence(
  authority: VerifiedPortableReportingEvidence,
): Readonly<VerifiedPortableReportingEvidence> {
  if (!VERIFIED_PORTABLE_REPORTING_EVIDENCE.has(authority)) {
    throw new Error('portable reporting evidence authority must be produced by canonical wire verification');
  }
  return authority;
}
