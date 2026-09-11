import {
  assertEconomicAuditCapsule,
  economicAuditCapsuleDigest,
  type EconomicAuditCapsuleV2,
} from './audit.js';
import {
  type AttestedCisacCrdReportingProjection,
  type AttestedDdexRoyaltyReportingProjection,
  type AttestedReportingProjectionInput,
  type RoyaltyReportingLine,
  exportAttestedCisacCrdReportingProjection,
  exportAttestedDdexRoyaltyReportingProjection,
} from './interop.js';
import {
  buildMerkleCommitment,
  buildMerkleInclusionProof,
  verifyMerkleInclusionProof,
  type Digest,
  type MerkleInclusionProof,
} from './merkle.js';
import {
  assertRoyaltyObligationAuthority,
  committedRoyaltyObligationPrincipal,
  type RoyaltyObligationAuthority,
} from './obligation-authority.js';

const SHA256_HEX = /^[0-9a-f]{64}$/;
const CANONICAL_MINOR_UNITS = /^(0|[1-9][0-9]*)$/;

export interface ReportingSafeObligationLeaf {
  readonly recordType: 'reporting_safe_obligation_v1';
  readonly obligationId: string;
  readonly authorityRoot: Digest;
  readonly beneficiaryId: string;
  readonly amountMinor: string;
  readonly currency: string;
}

export interface ReportingSafeDisclosureCommitment {
  readonly protocolVersion: 1;
  readonly authority: 'compiler_derived_sidecar_not_issuer_attested';
  readonly capsuleDigest: Digest;
  readonly statementId: string;
  readonly statementSnapshotRoot: Digest;
  readonly internalObligationRoot: Digest;
  readonly beneficiaryId: string;
  readonly currency: string;
  readonly disclosureRoot: Digest;
  readonly disclosureCount: number;
  readonly compilerId: string;
  readonly compilerBuildDigest: Digest;
  readonly sidecarRoot: Digest;
}

export interface ReportingSafeObligationEvidence {
  readonly protocolVersion: 1;
  readonly proofKind: 'reporting_safe_merkle_inclusion_not_zero_knowledge';
  readonly fieldEvidenceScope: 'obligation_id_authority_root_beneficiary_amount_currency_only';
  readonly sidecarRoot: Digest;
  readonly disclosureRoot: Digest;
  readonly disclosureCount: number;
  readonly leaf: ReportingSafeObligationLeaf;
  readonly proof: MerkleInclusionProof;
}

export interface AttestedReportingEvidencePackage<TProjection> {
  readonly authority: 'reporting_projection_only';
  readonly evidenceStatus: 'compiler_derived_line_evidence_not_issuer_attested';
  readonly projection: TProjection;
  readonly disclosure: ReportingSafeDisclosureCommitment;
  readonly lineEvidence: readonly ReportingSafeObligationEvidence[];
  readonly packageRoot: Digest;
}

export interface AttestedReportingEvidenceInput extends AttestedReportingProjectionInput {
  readonly obligations: readonly RoyaltyObligationAuthority[];
}

function digest(label: string, value: string): Digest {
  if (!SHA256_HEX.test(value)) throw new Error(`${label} must be a lowercase SHA-256 digest`);
  return value;
}

function internalStatementObligationLeaf(obligation: RoyaltyObligationAuthority): Readonly<Record<string, unknown>> {
  assertRoyaltyObligationAuthority(obligation);
  const principal = committedRoyaltyObligationPrincipal(obligation);
  return Object.freeze({
    id: principal.id,
    authorityRoot: obligation.authorityRoot,
    beneficiaryId: principal.beneficiaryId,
    amountMinor: principal.amountMinor,
    currency: principal.currency,
    observedAt: principal.observedAt,
    usageEvidenceRef: principal.usageEvidenceRef,
    rightsResolutionRef: principal.rightsResolutionRef,
    economicTermsRef: principal.economicTermsRef,
  });
}

function reportingSafeLeaf(obligation: RoyaltyObligationAuthority): Readonly<ReportingSafeObligationLeaf> {
  assertRoyaltyObligationAuthority(obligation);
  const principal = committedRoyaltyObligationPrincipal(obligation);
  return Object.freeze({
    recordType: 'reporting_safe_obligation_v1',
    obligationId: principal.id as string,
    authorityRoot: digest('reporting-safe obligation authorityRoot', obligation.authorityRoot),
    beneficiaryId: principal.beneficiaryId as string,
    amountMinor: (principal.amountMinor as bigint).toString(10),
    currency: principal.currency as string,
  });
}

function orderedObligations(
  capsule: EconomicAuditCapsuleV2,
  obligations: readonly RoyaltyObligationAuthority[],
): readonly RoyaltyObligationAuthority[] {
  assertEconomicAuditCapsule(capsule);
  if (capsule.protocolVersion !== 2) throw new Error('reporting-safe disclosure requires EconomicAuditCapsule v2');
  if (obligations.length === 0) throw new Error('reporting-safe disclosure requires obligations');

  const ordered = [...obligations].sort((left, right) => left.id.localeCompare(right.id));
  const seen = new Set<string>();
  for (const obligation of ordered) {
    assertRoyaltyObligationAuthority(obligation);
    if (seen.has(obligation.id)) throw new Error(`reporting-safe disclosure contains duplicate obligation id: ${obligation.id}`);
    seen.add(obligation.id);
    if (obligation.beneficiaryId !== capsule.statement.beneficiaryId) {
      throw new Error('reporting-safe disclosure obligation beneficiary does not match audited statement');
    }
    if (obligation.amount.currency !== capsule.statement.gross.currency) {
      throw new Error('reporting-safe disclosure obligation currency does not match audited statement');
    }
  }

  const internal = buildMerkleCommitment(ordered.map(internalStatementObligationLeaf));
  if (internal.root !== capsule.statement.obligationRoot) {
    throw new Error('reporting-safe disclosure obligations do not reconstruct audited internal obligation root');
  }
  return Object.freeze(ordered);
}

export function createReportingSafeDisclosureCommitment(input: {
  readonly capsule: EconomicAuditCapsuleV2;
  readonly obligations: readonly RoyaltyObligationAuthority[];
}): Readonly<ReportingSafeDisclosureCommitment> {
  const ordered = orderedObligations(input.capsule, input.obligations);
  const leaves = ordered.map(reportingSafeLeaf);
  const disclosure = buildMerkleCommitment(leaves);
  const committed = Object.freeze({
    protocolVersion: 1 as const,
    authority: 'compiler_derived_sidecar_not_issuer_attested' as const,
    capsuleDigest: economicAuditCapsuleDigest(input.capsule),
    statementId: input.capsule.statement.statementId,
    statementSnapshotRoot: input.capsule.statementSnapshotRoot,
    internalObligationRoot: input.capsule.statement.obligationRoot,
    beneficiaryId: input.capsule.statement.beneficiaryId,
    currency: input.capsule.statement.gross.currency,
    disclosureRoot: disclosure.root,
    disclosureCount: disclosure.count,
    compilerId: input.capsule.generatedBy.id,
    compilerBuildDigest: input.capsule.generatedBy.buildDigest,
  });
  const sidecarRoot = buildMerkleCommitment([{
    recordType: 'reporting_safe_disclosure_sidecar_v1',
    ...committed,
  }]).root;
  return Object.freeze({ ...committed, sidecarRoot });
}

function expectedSidecarRoot(commitment: ReportingSafeDisclosureCommitment): Digest {
  const committed = {
    protocolVersion: commitment.protocolVersion,
    authority: commitment.authority,
    capsuleDigest: commitment.capsuleDigest,
    statementId: commitment.statementId,
    statementSnapshotRoot: commitment.statementSnapshotRoot,
    internalObligationRoot: commitment.internalObligationRoot,
    beneficiaryId: commitment.beneficiaryId,
    currency: commitment.currency,
    disclosureRoot: commitment.disclosureRoot,
    disclosureCount: commitment.disclosureCount,
    compilerId: commitment.compilerId,
    compilerBuildDigest: commitment.compilerBuildDigest,
  } as const;
  return buildMerkleCommitment([{ recordType: 'reporting_safe_disclosure_sidecar_v1', ...committed }]).root;
}

export function assertReportingSafeDisclosureCommitment(
  commitment: ReportingSafeDisclosureCommitment,
): void {
  if (commitment.protocolVersion !== 1 || commitment.authority !== 'compiler_derived_sidecar_not_issuer_attested') {
    throw new Error('unsupported reporting-safe disclosure commitment');
  }
  digest('reporting-safe capsuleDigest', commitment.capsuleDigest);
  digest('reporting-safe statementSnapshotRoot', commitment.statementSnapshotRoot);
  digest('reporting-safe internalObligationRoot', commitment.internalObligationRoot);
  digest('reporting-safe disclosureRoot', commitment.disclosureRoot);
  digest('reporting-safe compilerBuildDigest', commitment.compilerBuildDigest);
  digest('reporting-safe sidecarRoot', commitment.sidecarRoot);
  if (!commitment.statementId.trim() || !commitment.beneficiaryId.trim() || !commitment.currency.trim() || !commitment.compilerId.trim()) {
    throw new Error('reporting-safe disclosure commitment identity fields must be non-empty');
  }
  if (!Number.isSafeInteger(commitment.disclosureCount) || commitment.disclosureCount <= 0) {
    throw new Error('reporting-safe disclosure count must be a positive safe integer');
  }
  if (commitment.sidecarRoot !== expectedSidecarRoot(commitment)) {
    throw new Error('reporting-safe sidecar root does not match canonical contents');
  }
}

export function createReportingSafeObligationEvidence(input: {
  readonly capsule: EconomicAuditCapsuleV2;
  readonly obligations: readonly RoyaltyObligationAuthority[];
  readonly obligationId: string;
}): Readonly<ReportingSafeObligationEvidence> {
  const ordered = orderedObligations(input.capsule, input.obligations);
  const leaves = ordered.map(reportingSafeLeaf);
  const index = ordered.findIndex(obligation => obligation.id === input.obligationId);
  if (index < 0) throw new Error(`reporting-safe disclosure obligation not found: ${input.obligationId}`);
  const commitment = createReportingSafeDisclosureCommitment({ capsule: input.capsule, obligations: ordered });
  const evidence = Object.freeze({
    protocolVersion: 1 as const,
    proofKind: 'reporting_safe_merkle_inclusion_not_zero_knowledge' as const,
    fieldEvidenceScope: 'obligation_id_authority_root_beneficiary_amount_currency_only' as const,
    sidecarRoot: commitment.sidecarRoot,
    disclosureRoot: commitment.disclosureRoot,
    disclosureCount: commitment.disclosureCount,
    leaf: leaves[index]!,
    proof: buildMerkleInclusionProof(leaves, index),
  });
  return evidence;
}

export function verifyReportingSafeObligationEvidence(
  evidence: ReportingSafeObligationEvidence,
): boolean {
  if (
    evidence.protocolVersion !== 1
    || evidence.proofKind !== 'reporting_safe_merkle_inclusion_not_zero_knowledge'
    || evidence.fieldEvidenceScope !== 'obligation_id_authority_root_beneficiary_amount_currency_only'
    || !SHA256_HEX.test(evidence.sidecarRoot)
    || !SHA256_HEX.test(evidence.disclosureRoot)
    || !SHA256_HEX.test(evidence.leaf.authorityRoot)
    || evidence.leaf.recordType !== 'reporting_safe_obligation_v1'
    || !evidence.leaf.obligationId.trim()
    || !evidence.leaf.beneficiaryId.trim()
    || !CANONICAL_MINOR_UNITS.test(evidence.leaf.amountMinor)
    || !evidence.leaf.currency.trim()
    || evidence.proof.count !== evidence.disclosureCount
  ) return false;
  return verifyMerkleInclusionProof(evidence.leaf, evidence.proof, evidence.disclosureRoot);
}

function assertLineMatchesEvidence(line: RoyaltyReportingLine, evidence: ReportingSafeObligationEvidence): void {
  if (!verifyReportingSafeObligationEvidence(evidence)) throw new Error('reporting line evidence is invalid');
  if (line.obligationId !== evidence.leaf.obligationId) throw new Error('reporting line obligation id does not match line evidence');
  if (line.beneficiaryReference !== evidence.leaf.beneficiaryId) throw new Error('reporting line beneficiary does not match line evidence');
  if (line.amount.amountMinor.toString(10) !== evidence.leaf.amountMinor || line.amount.currency !== evidence.leaf.currency) {
    throw new Error('reporting line amount does not match line evidence');
  }
}

function buildEvidencePackage<TProjection>(input: {
  readonly projection: TProjection;
  readonly projectionRoot: Digest;
  readonly capsule: EconomicAuditCapsuleV2;
  readonly obligations: readonly RoyaltyObligationAuthority[];
  readonly lines: readonly RoyaltyReportingLine[];
}): Readonly<AttestedReportingEvidencePackage<TProjection>> {
  const disclosure = createReportingSafeDisclosureCommitment({ capsule: input.capsule, obligations: input.obligations });
  const evidence = input.lines.map(line => createReportingSafeObligationEvidence({
    capsule: input.capsule,
    obligations: input.obligations,
    obligationId: line.obligationId,
  }));
  input.lines.forEach((line, index) => assertLineMatchesEvidence(line, evidence[index]!));
  const packageRoot = buildMerkleCommitment([{
    recordType: 'attested_reporting_evidence_package_v1',
    authority: 'reporting_projection_only',
    evidenceStatus: 'compiler_derived_line_evidence_not_issuer_attested',
    projectionRoot: input.projectionRoot,
    sidecarRoot: disclosure.sidecarRoot,
    lineEvidence: evidence,
  }]).root;
  return Object.freeze({
    authority: 'reporting_projection_only',
    evidenceStatus: 'compiler_derived_line_evidence_not_issuer_attested',
    projection: input.projection,
    disclosure,
    lineEvidence: Object.freeze(evidence),
    packageRoot,
  });
}

export function exportDdexReportingEvidencePackage(
  input: AttestedReportingEvidenceInput,
): Readonly<AttestedReportingEvidencePackage<AttestedDdexRoyaltyReportingProjection>> {
  const projection = exportAttestedDdexRoyaltyReportingProjection(input);
  return buildEvidencePackage({
    projection,
    projectionRoot: projection.projectionRoot,
    capsule: input.capsule,
    obligations: input.obligations,
    lines: input.lines,
  });
}

export function exportCisacCrdReportingEvidencePackage(
  input: AttestedReportingEvidenceInput,
): Readonly<AttestedReportingEvidencePackage<AttestedCisacCrdReportingProjection>> {
  const projection = exportAttestedCisacCrdReportingProjection(input);
  return buildEvidencePackage({
    projection,
    projectionRoot: projection.projectionRoot,
    capsule: input.capsule,
    obligations: input.obligations,
    lines: input.lines,
  });
}
