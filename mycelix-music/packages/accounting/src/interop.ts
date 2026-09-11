import {
  assertEconomicAuditCapsule,
  economicAuditCapsuleDigest,
  type EconomicAuditCapsuleV2,
} from './audit.js';
import {
  requireAnchoredEconomicAuditCapsuleIssuerAuthority,
  type AnchoredEconomicAuditCapsuleIssuerAuthority,
} from './audit-capsule-attestation-trust-anchor.js';
import { buildMerkleCommitment, type Digest } from './merkle.js';
import { type Money } from './money.js';

export interface ReportingProjectionSource {
  readonly statementId: string;
  readonly statementCommitmentRoot: string;
  readonly generatedAt: string;
}

export interface RoyaltyReportingLine {
  readonly obligationId: string;
  readonly workReference: string;
  readonly beneficiaryReference: string;
  readonly amount: Money;
  readonly usageReference?: string;
  readonly territory?: string;
}

export interface ReportingMoney {
  readonly amountMinor: string;
  readonly currency: string;
}

export interface DdexRoyaltyReportingLine {
  readonly obligationReference: string;
  readonly workReference: string;
  readonly payeeReference: string;
  readonly royaltyAmount: ReportingMoney;
  readonly usageReference?: string;
  readonly territory?: string;
}

export interface DdexRoyaltyReportingProjection {
  readonly format: 'DDEX';
  readonly authority: 'reporting_projection_only';
  readonly profileVersion: string;
  readonly source: ReportingProjectionSource;
  readonly lines: readonly DdexRoyaltyReportingLine[];
}

export interface CisacCrdReportingLine {
  readonly distributionReference: string;
  readonly workReference: string;
  readonly interestedPartyReference: string;
  readonly royaltyAmount: ReportingMoney;
}

export interface CisacCrdReportingProjection {
  readonly format: 'CISAC_CRD';
  readonly authority: 'reporting_projection_only';
  readonly profileVersion: string;
  readonly source: ReportingProjectionSource;
  readonly lines: readonly CisacCrdReportingLine[];
}

export interface AttestedAuditReportingReference {
  readonly provenanceKind: 'root_attested_economic_audit_capsule_v1';
  readonly capsuleProtocolVersion: 2;
  readonly capsuleDigest: Digest;
  readonly statementSnapshotRoot: Digest;
  readonly obligationRoot: Digest;
  readonly adjustmentRoot: Digest;
  readonly settlementRoot: Digest;
  readonly compilerId: string;
  readonly compilerBuildDigest: Digest;
  readonly issuerVerificationReceiptRoot: Digest;
  readonly anchoredIssuerVerificationReceiptRoot: Digest;
  readonly issuerTrustBundleRoot: Digest;
  readonly issuerTrustBundleAttestationRoot: Digest;
  readonly issuerTrustAnchorId: string;
  readonly issuerTrustPolicySequence: string;
  readonly issuerVerifiedAt: string;
  /**
   * This reference authenticates the statement-level audit lineage only. It is
   * not a Merkle inclusion proof for each exported royalty line.
   */
  readonly lineEvidenceScope: 'statement_level_reference_only';
  readonly provenanceRoot: Digest;
}

export interface AttestedDdexRoyaltyReportingProjection extends DdexRoyaltyReportingProjection {
  readonly audit: AttestedAuditReportingReference;
  readonly projectionRoot: Digest;
}

export interface AttestedCisacCrdReportingProjection extends CisacCrdReportingProjection {
  readonly audit: AttestedAuditReportingReference;
  readonly projectionRoot: Digest;
}

export interface AttestedReportingProjectionInput {
  readonly profileVersion: string;
  readonly capsule: EconomicAuditCapsuleV2;
  readonly anchoredIssuer: AnchoredEconomicAuditCapsuleIssuerAuthority;
  readonly generatedAt: string;
  readonly lines: readonly RoyaltyReportingLine[];
}

function timestamp(label: string, value: string): string {
  const parsed = Date.parse(value);
  if (!Number.isFinite(parsed)) throw new Error(`${label} must be a valid timestamp`);
  return new Date(parsed).toISOString();
}

function validateSource(source: ReportingProjectionSource): ReportingProjectionSource {
  if (!source.statementId.trim() || !source.statementCommitmentRoot.trim()) {
    throw new Error('reporting source must bind a statement id and commitment root');
  }
  const generatedAt = timestamp('reporting source generatedAt', source.generatedAt);
  return Object.freeze({ ...source, generatedAt });
}

function validateProfileVersion(profileVersion: string): string {
  const normalized = profileVersion.trim();
  if (!normalized) throw new Error('external reporting profileVersion must be explicit');
  return normalized;
}

function normalizedLines(lines: readonly RoyaltyReportingLine[]): readonly RoyaltyReportingLine[] {
  const seen = new Set<string>();
  const normalized = [...lines].sort((a, b) => a.obligationId.localeCompare(b.obligationId));
  for (const line of normalized) {
    if (!line.obligationId.trim() || !line.workReference.trim() || !line.beneficiaryReference.trim()) {
      throw new Error('reporting lines require obligation, work, and beneficiary references');
    }
    if (seen.has(line.obligationId)) throw new Error('reporting projection contains duplicate obligation id');
    if (line.amount.amountMinor < 0n) throw new Error('reporting royalty amount must be non-negative');
    seen.add(line.obligationId);
  }
  return normalized;
}

function assertAttestedLineContext(
  capsule: EconomicAuditCapsuleV2,
  lines: readonly RoyaltyReportingLine[],
): void {
  for (const line of lines) {
    if (line.beneficiaryReference !== capsule.statement.beneficiaryId) {
      throw new Error('attested reporting line beneficiary does not match audited statement beneficiary');
    }
    if (line.amount.currency !== capsule.statement.gross.currency) {
      throw new Error('attested reporting line currency does not match audited statement currency');
    }
  }
}

function reportingMoney(amount: Money): ReportingMoney {
  return Object.freeze({ amountMinor: amount.amountMinor.toString(10), currency: amount.currency });
}

function attestedAuditReference(input: AttestedReportingProjectionInput): Readonly<{
  source: ReportingProjectionSource;
  audit: AttestedAuditReportingReference;
}> {
  assertEconomicAuditCapsule(input.capsule);
  if (input.capsule.protocolVersion !== 2) throw new Error('attested reporting requires EconomicAuditCapsule v2');
  const authority = requireAnchoredEconomicAuditCapsuleIssuerAuthority(input.anchoredIssuer);
  const capsuleDigest = economicAuditCapsuleDigest(input.capsule);
  if (authority.receipt.capsuleDigest !== capsuleDigest) {
    throw new Error('attested reporting issuer authority is not bound to the supplied audit capsule');
  }
  const generatedAt = timestamp('attested reporting generatedAt', input.generatedAt);
  if (Date.parse(generatedAt) < Date.parse(input.capsule.statement.asOf)) {
    throw new Error('attested reporting cannot be generated before the audited statement asOf');
  }
  if (Date.parse(generatedAt) < Date.parse(authority.receipt.verifiedAt)) {
    throw new Error('attested reporting cannot predate anchored issuer verification');
  }

  const source = Object.freeze({
    statementId: input.capsule.statement.statementId,
    statementCommitmentRoot: input.capsule.statementSnapshotRoot,
    generatedAt,
  });
  const committed = Object.freeze({
    provenanceKind: 'root_attested_economic_audit_capsule_v1' as const,
    capsuleProtocolVersion: 2 as const,
    capsuleDigest,
    statementSnapshotRoot: input.capsule.statementSnapshotRoot,
    obligationRoot: input.capsule.statement.obligationRoot,
    adjustmentRoot: input.capsule.statement.adjustmentRoot,
    settlementRoot: input.capsule.statement.settlementRoot,
    compilerId: input.capsule.generatedBy.id,
    compilerBuildDigest: input.capsule.generatedBy.buildDigest,
    issuerVerificationReceiptRoot: authority.issuerAuthority.receipt.receiptRoot,
    anchoredIssuerVerificationReceiptRoot: authority.receipt.receiptRoot,
    issuerTrustBundleRoot: authority.receipt.trustBundleRoot,
    issuerTrustBundleAttestationRoot: authority.receipt.trustBundleAttestationRoot,
    issuerTrustAnchorId: authority.receipt.trustAnchorId,
    issuerTrustPolicySequence: authority.receipt.trustPolicySequence,
    issuerVerifiedAt: authority.receipt.verifiedAt,
    lineEvidenceScope: 'statement_level_reference_only' as const,
  });
  const provenanceRoot = buildMerkleCommitment([{
    recordType: 'interop_root_attested_audit_reference_v1',
    ...committed,
  }]).root;
  return Object.freeze({ source, audit: Object.freeze({ ...committed, provenanceRoot }) });
}

function ddexLines(lines: readonly RoyaltyReportingLine[]): readonly DdexRoyaltyReportingLine[] {
  return Object.freeze(normalizedLines(lines).map(line => Object.freeze({
    obligationReference: line.obligationId,
    workReference: line.workReference,
    payeeReference: line.beneficiaryReference,
    royaltyAmount: reportingMoney(line.amount),
    ...(line.usageReference ? { usageReference: line.usageReference } : {}),
    ...(line.territory ? { territory: line.territory } : {}),
  })));
}

function crdLines(lines: readonly RoyaltyReportingLine[]): readonly CisacCrdReportingLine[] {
  return Object.freeze(normalizedLines(lines).map(line => Object.freeze({
    distributionReference: line.obligationId,
    workReference: line.workReference,
    interestedPartyReference: line.beneficiaryReference,
    royaltyAmount: reportingMoney(line.amount),
  })));
}

/**
 * Create a DDEX reporting projection from authoritative Mycelix accounting
 * records. `profileVersion` is supplied explicitly because external profiles
 * evolve; this adapter does not make the external document the internal ledger
 * and does not claim profile conformance beyond the fields represented here.
 */
export function exportDdexRoyaltyReportingProjection(input: {
  profileVersion: string;
  source: ReportingProjectionSource;
  lines: readonly RoyaltyReportingLine[];
}): Readonly<DdexRoyaltyReportingProjection> {
  return Object.freeze({
    format: 'DDEX',
    authority: 'reporting_projection_only',
    profileVersion: validateProfileVersion(input.profileVersion),
    source: validateSource(input.source),
    lines: ddexLines(input.lines),
  });
}

/**
 * Create a CISAC CRD reporting projection. As with the DDEX projection, the
 * report carries exact Mycelix statement provenance and cannot create, modify,
 * or extinguish a royalty obligation.
 */
export function exportCisacCrdReportingProjection(input: {
  profileVersion: string;
  source: ReportingProjectionSource;
  lines: readonly RoyaltyReportingLine[];
}): Readonly<CisacCrdReportingProjection> {
  return Object.freeze({
    format: 'CISAC_CRD',
    authority: 'reporting_projection_only',
    profileVersion: validateProfileVersion(input.profileVersion),
    source: validateSource(input.source),
    lines: crdLines(input.lines),
  });
}

/**
 * Create a DDEX projection whose statement-level source provenance is derived
 * from a root-attested EconomicAuditCapsule v2. The report remains a reporting
 * projection only; `projectionRoot` is a deterministic content commitment, not
 * an issuer signature or internal ledger authority.
 */
export function exportAttestedDdexRoyaltyReportingProjection(
  input: AttestedReportingProjectionInput,
): Readonly<AttestedDdexRoyaltyReportingProjection> {
  const profileVersion = validateProfileVersion(input.profileVersion);
  const normalized = normalizedLines(input.lines);
  assertAttestedLineContext(input.capsule, normalized);
  const { source, audit } = attestedAuditReference(input);
  const lines = ddexLines(normalized);
  const projectionRoot = buildMerkleCommitment([{
    recordType: 'ddex_royalty_reporting_projection_attested_audit_v1',
    authority: 'reporting_projection_only',
    profileVersion,
    source,
    audit,
    lines,
  }]).root;
  return Object.freeze({
    format: 'DDEX',
    authority: 'reporting_projection_only',
    profileVersion,
    source,
    lines,
    audit,
    projectionRoot,
  });
}

/**
 * Create a CISAC CRD projection with the same root-attested statement-level
 * audit reference. This does not imply CRD profile conformance beyond this
 * adapter and does not provide per-line obligation inclusion proofs.
 */
export function exportAttestedCisacCrdReportingProjection(
  input: AttestedReportingProjectionInput,
): Readonly<AttestedCisacCrdReportingProjection> {
  const profileVersion = validateProfileVersion(input.profileVersion);
  const normalized = normalizedLines(input.lines);
  assertAttestedLineContext(input.capsule, normalized);
  const { source, audit } = attestedAuditReference(input);
  const lines = crdLines(normalized);
  const projectionRoot = buildMerkleCommitment([{
    recordType: 'cisac_crd_reporting_projection_attested_audit_v1',
    authority: 'reporting_projection_only',
    profileVersion,
    source,
    audit,
    lines,
  }]).root;
  return Object.freeze({
    format: 'CISAC_CRD',
    authority: 'reporting_projection_only',
    profileVersion,
    source,
    lines,
    audit,
    projectionRoot,
  });
}
