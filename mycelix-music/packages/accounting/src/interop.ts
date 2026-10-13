import { buildMerkleCommitment, type Digest } from './merkle.js';
import { type Money } from './money.js';

export interface ReportingProjectionSource {
  readonly statementId: string;
  /** SHA-256 root of the authoritative Mycelix statement commitment. */
  readonly statementCommitmentRoot: Digest;
  readonly generatedAt: string;
}

export interface RoyaltyReportingLine {
  readonly obligationId: string;
  readonly workReference: string;
  readonly beneficiaryReference: string;
  readonly amount: Money;
  readonly usageReference?: string;
  /** Two-letter territory code, normalized to uppercase. */
  readonly territory?: string;
}

export interface ReportingMoney {
  /** Exact integer minor units; never converted through JavaScript number. */
  readonly amountMinor: string;
  readonly currency: string;
}

export const ROYALTY_REPORTING_MAPPING_VERSION = 'mycelix-royalty-reporting-mapping-v1' as const;
export const REPORTING_STANDARD_CONFORMANCE = 'mapping_only_not_schema_conformant' as const;

interface ReportingProjectionBase {
  readonly authority: 'reporting_projection_only';
  readonly mappingVersion: typeof ROYALTY_REPORTING_MAPPING_VERSION;
  /**
   * This is a Mycelix field mapping, not an official DDEX/CISAC message. A
   * format-specific adapter must still map to the licensed/current schema,
   * apply profile rules, and validate against its official schema.
   */
  readonly standardConformance: typeof REPORTING_STANDARD_CONFORMANCE;
  /** Native amount unit is preserved; no FX conversion or ISO 4217 mapping occurs here. */
  readonly currencySemantics: 'source_accounting_unit_preserved_no_fx_conversion';
  readonly profileVersion: string;
  readonly source: ReportingProjectionSource;
}

export interface DdexRoyaltyReportingLine {
  readonly obligationReference: string;
  readonly workReference: string;
  readonly payeeReference: string;
  readonly royaltyAmount: ReportingMoney;
  readonly usageReference?: string;
  readonly territory?: string;
}

export interface DdexRoyaltyReportingProjection extends ReportingProjectionBase {
  readonly format: 'DDEX';
  readonly source: ReportingProjectionSource;
  readonly lines: readonly DdexRoyaltyReportingLine[];
  readonly projectionRoot: Digest;
}

export interface CisacCrdReportingLine {
  readonly distributionReference: string;
  readonly workReference: string;
  readonly interestedPartyReference: string;
  readonly royaltyAmount: ReportingMoney;
}

export interface CisacCrdReportingProjection extends ReportingProjectionBase {
  readonly format: 'CISAC_CRD';
  readonly source: ReportingProjectionSource;
  readonly lines: readonly CisacCrdReportingLine[];
  readonly projectionRoot: Digest;
}

const SHA256_HEX = /^[0-9a-f]{64}$/;
const REPORTING_ROOT_DOMAIN = 'mycelix-royalty-reporting-projection-v1';

function required(label: string, value: string): string {
  const normalized = value.trim();
  if (!normalized) throw new Error(`${label} must be non-empty`);
  return normalized;
}

function validateSource(source: ReportingProjectionSource): ReportingProjectionSource {
  const statementId = required('reporting source statementId', source.statementId);
  const statementCommitmentRoot = source.statementCommitmentRoot;
  if (!SHA256_HEX.test(statementCommitmentRoot)) {
    throw new Error('reporting source statementCommitmentRoot must be a lowercase SHA-256 digest');
  }
  const parsed = Date.parse(source.generatedAt);
  if (!Number.isFinite(parsed)) {
    throw new Error('reporting source generatedAt must be a valid timestamp');
  }
  return Object.freeze({
    statementId,
    statementCommitmentRoot,
    generatedAt: new Date(parsed).toISOString(),
  });
}

function validateProfileVersion(profileVersion: string): string {
  return required('external reporting profileVersion', profileVersion);
}

function normalizedLines(lines: readonly RoyaltyReportingLine[]): readonly RoyaltyReportingLine[] {
  const normalized = lines.map(line => {
    const obligationId = required('reporting obligationId', line.obligationId);
    const workReference = required('reporting workReference', line.workReference);
    const beneficiaryReference = required('reporting beneficiaryReference', line.beneficiaryReference);
    if (typeof line.amount?.amountMinor !== 'bigint') {
      throw new Error('reporting royalty amount must use exact bigint minor units');
    }
    const currency = required('reporting currency', line.amount.currency).toUpperCase();
    const usageReference = line.usageReference === undefined
      ? undefined
      : required('reporting usageReference', line.usageReference);
    const territory = line.territory === undefined
      ? undefined
      : required('reporting territory', line.territory).toUpperCase();
    if (territory !== undefined && !/^[A-Z]{2}$/.test(territory)) {
      throw new Error('reporting territory must be a two-letter uppercase code');
    }
    if (line.amount.amountMinor < 0n) throw new Error('reporting royalty amount must be non-negative');
    return Object.freeze({
      obligationId,
      workReference,
      beneficiaryReference,
      amount: Object.freeze({ amountMinor: line.amount.amountMinor, currency }),
      ...(usageReference === undefined ? {} : { usageReference }),
      ...(territory === undefined ? {} : { territory }),
    });
  }).sort((a, b) => a.obligationId < b.obligationId ? -1 : a.obligationId > b.obligationId ? 1 : 0);

  const seen = new Set<string>();
  for (const line of normalized) {
    if (seen.has(line.obligationId)) throw new Error('reporting projection contains duplicate obligation id');
    seen.add(line.obligationId);
  }
  return Object.freeze(normalized);
}

function reportingMoney(amount: Money): ReportingMoney {
  return Object.freeze({ amountMinor: amount.amountMinor.toString(10), currency: amount.currency });
}

function computeProjectionRoot(
  value: Omit<DdexRoyaltyReportingProjection, 'projectionRoot'> | Omit<CisacCrdReportingProjection, 'projectionRoot'>,
): Digest {
  return buildMerkleCommitment([{
    recordType: REPORTING_ROOT_DOMAIN,
    ...value,
  }]).root;
}

/**
 * Recompute a mapping projection's integrity commitment after storage or
 * transport. This detects edits relative to the carried root; it is not a
 * signature and does not authenticate the producer.
 */
export function assertDdexRoyaltyReportingProjection(
  projection: DdexRoyaltyReportingProjection,
): void {
  const { projectionRoot, ...payload } = projection;
  if (!SHA256_HEX.test(projectionRoot) || computeProjectionRoot(payload) !== projectionRoot) {
    throw new Error('DDEX mapping projection root does not match its canonical contents');
  }
}

export function assertCisacCrdReportingProjection(
  projection: CisacCrdReportingProjection,
): void {
  const { projectionRoot, ...payload } = projection;
  if (!SHA256_HEX.test(projectionRoot) || computeProjectionRoot(payload) !== projectionRoot) {
    throw new Error('CISAC CRD mapping projection root does not match its canonical contents');
  }
}

/**
 * Create a deterministic Mycelix mapping projection for a DDEX-related export.
 * This function does not produce an official DDEX XML/flat-file message or
 * assert conformance to RDR-R, ERN, or another DDEX profile.
 */
export function exportDdexRoyaltyReportingProjection(input: {
  profileVersion: string;
  source: ReportingProjectionSource;
  lines: readonly RoyaltyReportingLine[];
}): Readonly<DdexRoyaltyReportingProjection> {
  const profileVersion = validateProfileVersion(input.profileVersion);
  const source = validateSource(input.source);
  const lines = Object.freeze(normalizedLines(input.lines).map(line => Object.freeze({
    obligationReference: line.obligationId,
    workReference: line.workReference,
    payeeReference: line.beneficiaryReference,
    royaltyAmount: reportingMoney(line.amount),
    ...(line.usageReference === undefined ? {} : { usageReference: line.usageReference }),
    ...(line.territory === undefined ? {} : { territory: line.territory }),
  })));
  const payload = Object.freeze({
    format: 'DDEX' as const,
    authority: 'reporting_projection_only' as const,
    mappingVersion: ROYALTY_REPORTING_MAPPING_VERSION,
    standardConformance: REPORTING_STANDARD_CONFORMANCE,
    currencySemantics: 'source_accounting_unit_preserved_no_fx_conversion' as const,
    profileVersion,
    source,
    lines,
  });
  return Object.freeze({ ...payload, projectionRoot: computeProjectionRoot(payload) });
}

/**
 * Create a deterministic Mycelix mapping projection for a CISAC CRD export.
 * This function does not produce a CRD distribution file or assert conformance
 * to the official CISAC CRD document pack or territory-specific requirements.
 */
export function exportCisacCrdReportingProjection(input: {
  profileVersion: string;
  source: ReportingProjectionSource;
  lines: readonly RoyaltyReportingLine[];
}): Readonly<CisacCrdReportingProjection> {
  const profileVersion = validateProfileVersion(input.profileVersion);
  const source = validateSource(input.source);
  const lines = Object.freeze(normalizedLines(input.lines).map(line => Object.freeze({
    distributionReference: line.obligationId,
    workReference: line.workReference,
    interestedPartyReference: line.beneficiaryReference,
    royaltyAmount: reportingMoney(line.amount),
  })));
  const payload = Object.freeze({
    format: 'CISAC_CRD' as const,
    authority: 'reporting_projection_only' as const,
    mappingVersion: ROYALTY_REPORTING_MAPPING_VERSION,
    standardConformance: REPORTING_STANDARD_CONFORMANCE,
    currencySemantics: 'source_accounting_unit_preserved_no_fx_conversion' as const,
    profileVersion,
    source,
    lines,
  });
  return Object.freeze({ ...payload, projectionRoot: computeProjectionRoot(payload) });
}
