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

function validateSource(source: ReportingProjectionSource): ReportingProjectionSource {
  if (!source.statementId.trim() || !source.statementCommitmentRoot.trim()) {
    throw new Error('reporting source must bind a statement id and commitment root');
  }
  if (!Number.isFinite(Date.parse(source.generatedAt))) {
    throw new Error('reporting source generatedAt must be a valid timestamp');
  }
  return Object.freeze({ ...source });
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

function reportingMoney(amount: Money): ReportingMoney {
  return Object.freeze({ amountMinor: amount.amountMinor.toString(10), currency: amount.currency });
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
  const lines = normalizedLines(input.lines).map(line => Object.freeze({
    obligationReference: line.obligationId,
    workReference: line.workReference,
    payeeReference: line.beneficiaryReference,
    royaltyAmount: reportingMoney(line.amount),
    ...(line.usageReference ? { usageReference: line.usageReference } : {}),
    ...(line.territory ? { territory: line.territory } : {}),
  }));
  return Object.freeze({
    format: 'DDEX',
    authority: 'reporting_projection_only',
    profileVersion: validateProfileVersion(input.profileVersion),
    source: validateSource(input.source),
    lines: Object.freeze(lines),
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
  const lines = normalizedLines(input.lines).map(line => Object.freeze({
    distributionReference: line.obligationId,
    workReference: line.workReference,
    interestedPartyReference: line.beneficiaryReference,
    royaltyAmount: reportingMoney(line.amount),
  }));
  return Object.freeze({
    format: 'CISAC_CRD',
    authority: 'reporting_projection_only',
    profileVersion: validateProfileVersion(input.profileVersion),
    source: validateSource(input.source),
    lines: Object.freeze(lines),
  });
}
