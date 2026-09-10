// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

/**
 * Append-only creator-accounting persistence boundary.
 *
 * This store exposes no update/delete operations. Exact-content replay is
 * idempotent; an authority ID reused with different immutable content fails
 * closed. PostgreSQL triggers and CHECK constraints provide a second line of defense.
 */

const DIGEST_RE = /^[0-9a-f]{64}$/;
const MINOR_RE = /^(0|[1-9][0-9]*)$/;

export type EligibilityCode =
  | 'eligible'
  | 'awaiting_payee_route'
  | 'rights_conflict'
  | 'legal_hold'
  | 'tax_documentation_required'
  | 'awaiting_fx_quote'
  | 'dormant_beneficiary';

const ELIGIBILITY_CODES: ReadonlySet<string> = new Set<EligibilityCode>([
  'eligible',
  'awaiting_payee_route',
  'rights_conflict',
  'legal_hold',
  'tax_documentation_required',
  'awaiting_fx_quote',
  'dormant_beneficiary',
]);

export type SettlementObservationState =
  | 'authorized'
  | 'submitted'
  | 'accepted'
  | 'confirmed'
  | 'finalized'
  | 'failed'
  | 'rejected'
  | 'reversed'
  | 'disputed';

const SETTLEMENT_STATES: ReadonlySet<string> = new Set<SettlementObservationState>([
  'authorized', 'submitted', 'accepted', 'confirmed', 'finalized',
  'failed', 'rejected', 'reversed', 'disputed',
]);
const STATEMENT_KINDS = new Set(['periodic', 'supplemental', 'adjustment', 'reconciliation'] as const);
const COMPLETENESS_KINDS = new Set(['complete', 'partial', 'indeterminate'] as const);

export type StatementSnapshotKind = 'periodic' | 'supplemental' | 'adjustment' | 'reconciliation';
export type StatementCompletenessKind = 'complete' | 'partial' | 'indeterminate';

export interface RoyaltyObligationRecordInput {
  readonly id: string;
  readonly beneficiaryId: string;
  readonly amountMinor: string;
  readonly currency: string;
  readonly observedAt: Date | string;
  readonly usageEvidenceRef: string;
  readonly rightsResolutionRef: string;
  readonly economicTermsRef: string;
  readonly obligationRoot: string;
}

export interface RoyaltyEligibilityObservationInput {
  readonly id: string;
  readonly obligationId: string;
  readonly code: EligibilityCode;
  readonly reason?: string;
  readonly sourceRef: string;
  readonly observedAt: Date | string;
  readonly observationRoot: string;
}

export interface RoyaltyDeductionRecordInput {
  readonly id: string;
  readonly beneficiaryId: string;
  readonly amountMinor: string;
  readonly currency: string;
  readonly basis: string;
  readonly authorityRef: string;
  readonly observedAt: Date | string;
  readonly deductionRoot: string;
}

export interface SettlementAttemptObservationRecordInput {
  readonly id: string;
  readonly attemptId: string;
  readonly batchId: string;
  readonly obligationSetRoot: string;
  readonly eligibilityAsOf: Date | string;
  readonly eligibilityEvidenceRoot: string;
  readonly state: SettlementObservationState;
  readonly observedAt: Date | string;
  readonly railReceiptRef?: string;
  readonly supersedesAttemptId?: string;
  readonly observationRoot: string;
}

export interface RoyaltyStatementSnapshotRecordInput {
  readonly statementId: string;
  readonly kind: StatementSnapshotKind;
  readonly predecessorStatementId?: string;
  readonly beneficiaryId: string;
  readonly periodStart: Date | string;
  readonly periodEnd: Date | string;
  readonly asOf: Date | string;
  readonly obligationRoot: string;
  readonly adjustmentRoot: string;
  readonly settlementRoot: string;
  readonly grossMinor: string;
  readonly heldMinor: string;
  readonly deductionMinor: string;
  readonly netPayableMinor: string;
  readonly paidMinor: string;
  readonly currency: string;
  readonly completenessKind: StatementCompletenessKind;
  readonly completenessData: unknown;
  readonly snapshotRoot: string;
}

interface ImmutableDelegate {
  findUnique(args: { where: { id: string } }): Promise<Record<string, unknown> | null>;
  create(args: { data: Record<string, unknown> }): Promise<Record<string, unknown>>;
}
interface StatementDelegate {
  findUnique(args: { where: { statementId: string } }): Promise<Record<string, unknown> | null>;
  create(args: { data: Record<string, unknown> }): Promise<Record<string, unknown>>;
}

/** Minimal structural surface implemented by PrismaClient and test doubles. */
export interface AccountingAuthorityPrismaClient {
  readonly royaltyObligationRecord: unknown;
  readonly royaltyEligibilityObservation: unknown;
  readonly royaltyDeductionRecord: unknown;
  readonly settlementAttemptObservationRecord: unknown;
  readonly royaltyStatementSnapshotRecord: unknown;
}

function required(label: string, value: string): string {
  const normalized = value.trim();
  if (!normalized) throw new Error(`${label} must be non-empty`);
  return normalized;
}
function currency(value: string): string { return required('currency', value).toUpperCase(); }
function minor(label: string, value: string): string {
  const normalized = value.trim();
  if (!MINOR_RE.test(normalized)) throw new Error(`${label} must be a canonical non-negative integer string`);
  return normalized;
}
function digest(label: string, value: string): string {
  const normalized = value.trim();
  if (!DIGEST_RE.test(normalized)) throw new Error(`${label} must be a lowercase SHA-256 digest`);
  return normalized;
}
function timestamp(label: string, value: Date | string): Date {
  const date = value instanceof Date ? new Date(value.getTime()) : new Date(value);
  if (!Number.isFinite(date.getTime())) throw new Error(`${label} must be a valid timestamp`);
  return date;
}
function optionalRef(label: string, value: string | undefined): string | undefined {
  return value === undefined ? undefined : required(label, value);
}

function canonical(value: unknown): string {
  if (value === null) return 'null';
  if (value instanceof Date) return `date:${value.toISOString()}`;
  if (typeof value === 'string') return `string:${JSON.stringify(value)}`;
  if (typeof value === 'boolean') return value ? 'true' : 'false';
  if (typeof value === 'number') {
    if (!Number.isFinite(value)) throw new Error('immutable record values require finite numbers');
    return `number:${value}`;
  }
  if (typeof value === 'bigint') return `bigint:${value.toString(10)}`;
  if (Array.isArray(value)) return `[${value.map(canonical).join(',')}]`;
  if (typeof value === 'object') {
    const record = value as Record<string, unknown>;
    return `{${Object.keys(record).sort().map(key => {
      if (record[key] === undefined) throw new Error('immutable record values forbid undefined');
      return `${JSON.stringify(key)}:${canonical(record[key])}`;
    }).join(',')}}`;
  }
  throw new Error(`unsupported immutable record value: ${typeof value}`);
}
function assertJson(value: unknown): void { canonical(value); }

function immutableDelegate(value: unknown, label: string): ImmutableDelegate {
  const delegate = value as Partial<ImmutableDelegate> | null;
  if (!delegate || typeof delegate.findUnique !== 'function' || typeof delegate.create !== 'function') {
    throw new Error(`${label} Prisma delegate is unavailable; run prisma generate for the authority schema`);
  }
  return delegate as ImmutableDelegate;
}
function statementDelegate(value: unknown): StatementDelegate {
  const delegate = value as Partial<StatementDelegate> | null;
  if (!delegate || typeof delegate.findUnique !== 'function' || typeof delegate.create !== 'function') {
    throw new Error('royaltyStatementSnapshotRecord Prisma delegate is unavailable; run prisma generate for the authority schema');
  }
  return delegate as StatementDelegate;
}

function assertImmutableReplay(
  label: string,
  identity: string,
  existing: Record<string, unknown>,
  expected: Record<string, unknown>,
): void {
  for (const [key, expectedValue] of Object.entries(expected)) {
    if (canonical(existing[key]) !== canonical(expectedValue)) {
      throw new Error(`${label} ${identity} already exists with different immutable content at ${key}`);
    }
  }
}

async function appendById(
  delegate: ImmutableDelegate,
  label: string,
  data: Record<string, unknown> & { id: string },
): Promise<Record<string, unknown>> {
  const existing = await delegate.findUnique({ where: { id: data.id } });
  if (existing) {
    assertImmutableReplay(label, data.id, existing, data);
    return existing;
  }
  try {
    return await delegate.create({ data });
  } catch (error) {
    const raced = await delegate.findUnique({ where: { id: data.id } });
    if (raced) {
      assertImmutableReplay(label, data.id, raced, data);
      return raced;
    }
    throw error;
  }
}

async function appendStatement(
  delegate: StatementDelegate,
  data: Record<string, unknown> & { statementId: string },
): Promise<Record<string, unknown>> {
  const existing = await delegate.findUnique({ where: { statementId: data.statementId } });
  if (existing) {
    assertImmutableReplay('royalty statement snapshot', data.statementId, existing, data);
    return existing;
  }
  try {
    return await delegate.create({ data });
  } catch (error) {
    const raced = await delegate.findUnique({ where: { statementId: data.statementId } });
    if (raced) {
      assertImmutableReplay('royalty statement snapshot', data.statementId, raced, data);
      return raced;
    }
    throw error;
  }
}

export class AccountingAuthorityStore {
  constructor(private readonly client: AccountingAuthorityPrismaClient) {}

  async appendObligation(input: RoyaltyObligationRecordInput): Promise<Record<string, unknown>> {
    const data = {
      id: required('obligation id', input.id),
      beneficiaryId: required('obligation beneficiaryId', input.beneficiaryId),
      amountMinor: minor('obligation amountMinor', input.amountMinor),
      currency: currency(input.currency),
      observedAt: timestamp('obligation observedAt', input.observedAt),
      usageEvidenceRef: required('usageEvidenceRef', input.usageEvidenceRef),
      rightsResolutionRef: required('rightsResolutionRef', input.rightsResolutionRef),
      economicTermsRef: required('economicTermsRef', input.economicTermsRef),
      obligationRoot: digest('obligationRoot', input.obligationRoot),
    };
    return appendById(immutableDelegate(this.client.royaltyObligationRecord, 'royaltyObligationRecord'), 'royalty obligation', data);
  }

  async appendEligibilityObservation(input: RoyaltyEligibilityObservationInput): Promise<Record<string, unknown>> {
    if (!ELIGIBILITY_CODES.has(input.code)) {
      throw new Error(`unsupported source eligibility code: ${String(input.code)}`);
    }
    const reason = optionalRef('eligibility reason', input.reason);
    const data = {
      id: required('eligibility observation id', input.id),
      obligationId: required('eligibility obligationId', input.obligationId),
      code: input.code,
      ...(reason === undefined ? {} : { reason }),
      sourceRef: required('eligibility sourceRef', input.sourceRef),
      observedAt: timestamp('eligibility observedAt', input.observedAt),
      observationRoot: digest('eligibility observationRoot', input.observationRoot),
    };
    return appendById(immutableDelegate(this.client.royaltyEligibilityObservation, 'royaltyEligibilityObservation'), 'royalty eligibility observation', data);
  }

  async appendDeduction(input: RoyaltyDeductionRecordInput): Promise<Record<string, unknown>> {
    const data = {
      id: required('deduction id', input.id),
      beneficiaryId: required('deduction beneficiaryId', input.beneficiaryId),
      amountMinor: minor('deduction amountMinor', input.amountMinor),
      currency: currency(input.currency),
      basis: required('deduction basis', input.basis),
      authorityRef: required('deduction authorityRef', input.authorityRef),
      observedAt: timestamp('deduction observedAt', input.observedAt),
      deductionRoot: digest('deductionRoot', input.deductionRoot),
    };
    return appendById(immutableDelegate(this.client.royaltyDeductionRecord, 'royaltyDeductionRecord'), 'royalty deduction', data);
  }

  async appendSettlementObservation(input: SettlementAttemptObservationRecordInput): Promise<Record<string, unknown>> {
    if (!SETTLEMENT_STATES.has(input.state)) throw new Error(`unsupported settlement state: ${String(input.state)}`);
    const railReceiptRef = optionalRef('railReceiptRef', input.railReceiptRef);
    if (input.state === 'finalized' && railReceiptRef === undefined) {
      throw new Error('finalized settlement observation requires railReceiptRef');
    }
    const eligibilityAsOf = timestamp('settlement eligibilityAsOf', input.eligibilityAsOf);
    const observedAt = timestamp('settlement observedAt', input.observedAt);
    if (observedAt.getTime() < eligibilityAsOf.getTime()) {
      throw new Error('settlement observation cannot predate its eligibility snapshot');
    }
    const supersedesAttemptId = optionalRef('supersedesAttemptId', input.supersedesAttemptId);
    const data = {
      id: required('settlement observation id', input.id),
      attemptId: required('settlement attemptId', input.attemptId),
      batchId: required('settlement batchId', input.batchId),
      obligationSetRoot: digest('settlement obligationSetRoot', input.obligationSetRoot),
      eligibilityAsOf,
      eligibilityEvidenceRoot: digest('settlement eligibilityEvidenceRoot', input.eligibilityEvidenceRoot),
      state: input.state,
      observedAt,
      ...(railReceiptRef === undefined ? {} : { railReceiptRef }),
      ...(supersedesAttemptId === undefined ? {} : { supersedesAttemptId }),
      observationRoot: digest('settlement observationRoot', input.observationRoot),
    };
    return appendById(immutableDelegate(this.client.settlementAttemptObservationRecord, 'settlementAttemptObservationRecord'), 'settlement attempt observation', data);
  }

  async appendStatementSnapshot(input: RoyaltyStatementSnapshotRecordInput): Promise<Record<string, unknown>> {
    if (!STATEMENT_KINDS.has(input.kind)) throw new Error(`unsupported statement kind: ${input.kind}`);
    if (!COMPLETENESS_KINDS.has(input.completenessKind)) {
      throw new Error(`unsupported statement completeness kind: ${input.completenessKind}`);
    }
    const predecessorStatementId = optionalRef('predecessorStatementId', input.predecessorStatementId);
    if (input.kind === 'periodic' && predecessorStatementId !== undefined) throw new Error('periodic statement snapshot cannot name a predecessor');
    if (input.kind !== 'periodic' && predecessorStatementId === undefined) throw new Error('non-periodic statement snapshot requires a predecessor');

    const periodStart = timestamp('statement periodStart', input.periodStart);
    const periodEnd = timestamp('statement periodEnd', input.periodEnd);
    const asOf = timestamp('statement asOf', input.asOf);
    if (periodStart.getTime() >= periodEnd.getTime()) throw new Error('statement periodStart must precede periodEnd');

    const grossMinor = minor('statement grossMinor', input.grossMinor);
    const heldMinor = minor('statement heldMinor', input.heldMinor);
    const deductionMinor = minor('statement deductionMinor', input.deductionMinor);
    const netPayableMinor = minor('statement netPayableMinor', input.netPayableMinor);
    const paidMinor = minor('statement paidMinor', input.paidMinor);
    if (BigInt(grossMinor) !== BigInt(heldMinor) + BigInt(deductionMinor) + BigInt(netPayableMinor)) {
      throw new Error('statement snapshot does not conserve gross value');
    }
    if (BigInt(paidMinor) > BigInt(netPayableMinor)) throw new Error('statement snapshot paidMinor cannot exceed netPayableMinor');
    assertJson(input.completenessData);

    const data = {
      statementId: required('statementId', input.statementId),
      kind: input.kind,
      ...(predecessorStatementId === undefined ? {} : { predecessorStatementId }),
      beneficiaryId: required('statement beneficiaryId', input.beneficiaryId),
      periodStart,
      periodEnd,
      asOf,
      obligationRoot: digest('statement obligationRoot', input.obligationRoot),
      adjustmentRoot: digest('statement adjustmentRoot', input.adjustmentRoot),
      settlementRoot: digest('statement settlementRoot', input.settlementRoot),
      grossMinor,
      heldMinor,
      deductionMinor,
      netPayableMinor,
      paidMinor,
      currency: currency(input.currency),
      completenessKind: input.completenessKind,
      completenessData: input.completenessData,
      snapshotRoot: digest('statement snapshotRoot', input.snapshotRoot),
    };
    return appendStatement(statementDelegate(this.client.royaltyStatementSnapshotRecord), data);
  }
}
