import { canonicalAccountingValue } from './merkle.js';
import { money } from './money.js';
import { createRoyaltyObligationAuthority, type RoyaltyObligationAuthority } from './obligation-authority.js';
import {
  projectDeductionRecord,
  projectEligibilityObservationRecord,
  projectRoyaltyObligationRecord,
  projectSettlementObservationRecord,
  projectStatementSnapshotRecord,
  type PersistedDeductionRecord,
  type PersistedEligibilityObservationRecord,
  type PersistedRoyaltyObligationRecord,
  type PersistedSettlementObservationRecord,
  type PersistedStatementSnapshotRecord,
} from './persistence-projection.js';
import type { StatementDeduction } from './projection.js';
import {
  SettlementAttemptState,
  type SettlementAttemptObservation,
} from './recovery.js';
import type { SettlementEligibilityObservation } from './settlement.js';
import { createStatementSnapshot, type RoyaltyStatementSnapshot } from './statements.js';

const MINOR_RE = /^(0|[1-9][0-9]*)$/;
const SETTLEMENT_STATES: ReadonlySet<string> = new Set(Object.values(SettlementAttemptState));

type DateLike = string | Date;

export type VerifiablePersistedRoyaltyObligationRecord =
  Omit<PersistedRoyaltyObligationRecord, 'observedAt'> & { readonly observedAt: DateLike };
export type VerifiablePersistedEligibilityObservationRecord =
  Omit<PersistedEligibilityObservationRecord, 'observedAt'> & { readonly observedAt: DateLike };
export type VerifiablePersistedDeductionRecord =
  Omit<PersistedDeductionRecord, 'observedAt'> & { readonly observedAt: DateLike };
export type VerifiablePersistedSettlementObservationRecord =
  Omit<PersistedSettlementObservationRecord, 'eligibilityAsOf' | 'observedAt'> & {
    readonly eligibilityAsOf: DateLike;
    readonly observedAt: DateLike;
  };
export type VerifiablePersistedStatementSnapshotRecord =
  Omit<PersistedStatementSnapshotRecord, 'periodStart' | 'periodEnd' | 'asOf'> & {
    readonly periodStart: DateLike;
    readonly periodEnd: DateLike;
    readonly asOf: DateLike;
  };

function canonicalTimestamp(label: string, value: DateLike): string {
  const date = value instanceof Date ? new Date(value.getTime()) : new Date(value);
  if (!Number.isFinite(date.getTime())) throw new Error(`${label} must be a valid timestamp`);
  return date.toISOString();
}

function canonicalMinor(label: string, value: string): bigint {
  if (!MINOR_RE.test(value)) throw new Error(`${label} must be a canonical non-negative integer string`);
  return BigInt(value);
}

function sameCanonicalValue(left: unknown, right: unknown): boolean {
  if (left === undefined || right === undefined) return left === right;
  return canonicalAccountingValue(left) === canonicalAccountingValue(right);
}

function assertProjectedFields(
  label: string,
  actual: Readonly<Record<string, unknown>>,
  expected: Readonly<Record<string, unknown>>,
): void {
  for (const [field, expectedValue] of Object.entries(expected)) {
    if (!sameCanonicalValue(actual[field], expectedValue)) {
      throw new Error(`${label} does not match canonical projection at ${field}`);
    }
  }
}

export function verifyPersistedRoyaltyObligation(
  record: VerifiablePersistedRoyaltyObligationRecord,
): Readonly<RoyaltyObligationAuthority> {
  const observedAt = canonicalTimestamp('persisted obligation observedAt', record.observedAt);
  const authority = createRoyaltyObligationAuthority({
    id: record.id,
    beneficiaryId: record.beneficiaryId,
    amount: money(canonicalMinor('persisted obligation amountMinor', record.amountMinor), record.currency),
    observedAt,
    provenance: {
      usageEvidenceRef: record.usageEvidenceRef,
      rightsResolutionRef: record.rightsResolutionRef,
      economicTermsRef: record.economicTermsRef,
    },
  });
  const expected = projectRoyaltyObligationRecord(authority);
  const normalized = { ...record, observedAt } as Readonly<Record<string, unknown>>;
  assertProjectedFields('persisted royalty obligation', normalized, expected as unknown as Readonly<Record<string, unknown>>);
  return authority;
}

export function verifyPersistedEligibilityObservation(
  record: VerifiablePersistedEligibilityObservationRecord,
): Readonly<SettlementEligibilityObservation> {
  const observedAt = canonicalTimestamp('persisted eligibility observedAt', record.observedAt);
  const observation: SettlementEligibilityObservation = Object.freeze({
    id: record.id,
    obligationId: record.obligationId,
    code: record.code,
    ...(record.reason === undefined ? {} : { reason: record.reason }),
    sourceRef: record.sourceRef,
    observedAt,
  });
  const expected = projectEligibilityObservationRecord(observation);
  const normalized = { ...record, observedAt } as Readonly<Record<string, unknown>>;
  assertProjectedFields('persisted eligibility observation', normalized, expected as unknown as Readonly<Record<string, unknown>>);
  return Object.freeze({
    id: expected.id,
    obligationId: expected.obligationId,
    code: expected.code,
    ...(expected.reason === undefined ? {} : { reason: expected.reason }),
    sourceRef: expected.sourceRef,
    observedAt: expected.observedAt,
  });
}

export function verifyPersistedDeduction(
  record: VerifiablePersistedDeductionRecord,
): Readonly<StatementDeduction> {
  const observedAt = canonicalTimestamp('persisted deduction observedAt', record.observedAt);
  const deduction: StatementDeduction = Object.freeze({
    id: record.id,
    beneficiaryId: record.beneficiaryId,
    amount: money(canonicalMinor('persisted deduction amountMinor', record.amountMinor), record.currency),
    basis: record.basis,
    observedAt,
  });
  const expected = projectDeductionRecord(deduction, record.authorityRef);
  const normalized = { ...record, observedAt } as Readonly<Record<string, unknown>>;
  assertProjectedFields('persisted deduction', normalized, expected as unknown as Readonly<Record<string, unknown>>);
  return deduction;
}

export function verifyPersistedSettlementObservation(
  record: VerifiablePersistedSettlementObservationRecord,
): Readonly<SettlementAttemptObservation> {
  if (!SETTLEMENT_STATES.has(record.state)) {
    throw new Error(`persisted settlement state is unsupported: ${String(record.state)}`);
  }
  const eligibilityAsOf = canonicalTimestamp('persisted settlement eligibilityAsOf', record.eligibilityAsOf);
  const observedAt = canonicalTimestamp('persisted settlement observedAt', record.observedAt);
  const observation: SettlementAttemptObservation = Object.freeze({
    attemptId: record.attemptId,
    batchId: record.batchId,
    obligationSetRoot: record.obligationSetRoot,
    eligibilityAsOf,
    eligibilityEvidenceRoot: record.eligibilityEvidenceRoot,
    state: record.state,
    observedAt,
    ...(record.railReceiptRef === undefined ? {} : { railReceiptRef: record.railReceiptRef }),
    ...(record.supersedesAttemptId === undefined ? {} : { supersedesAttemptId: record.supersedesAttemptId }),
  });
  const expected = projectSettlementObservationRecord(record.id, observation);
  const normalized = { ...record, eligibilityAsOf, observedAt } as Readonly<Record<string, unknown>>;
  assertProjectedFields('persisted settlement observation', normalized, expected as unknown as Readonly<Record<string, unknown>>);
  return Object.freeze({
    attemptId: expected.attemptId,
    batchId: expected.batchId,
    obligationSetRoot: expected.obligationSetRoot,
    eligibilityAsOf: expected.eligibilityAsOf,
    eligibilityEvidenceRoot: expected.eligibilityEvidenceRoot,
    state: expected.state,
    observedAt: expected.observedAt,
    ...(expected.railReceiptRef === undefined ? {} : { railReceiptRef: expected.railReceiptRef }),
    ...(expected.supersedesAttemptId === undefined ? {} : { supersedesAttemptId: expected.supersedesAttemptId }),
  });
}

export function verifyPersistedStatementSnapshot(
  record: VerifiablePersistedStatementSnapshotRecord,
): Readonly<RoyaltyStatementSnapshot> {
  const periodStart = canonicalTimestamp('persisted statement periodStart', record.periodStart);
  const periodEnd = canonicalTimestamp('persisted statement periodEnd', record.periodEnd);
  const asOf = canonicalTimestamp('persisted statement asOf', record.asOf);
  const statement = createStatementSnapshot({
    statementId: record.statementId,
    kind: record.kind,
    ...(record.predecessorStatementId === undefined ? {} : { predecessorStatementId: record.predecessorStatementId }),
    beneficiaryId: record.beneficiaryId,
    period: { startInclusive: periodStart, endExclusive: periodEnd },
    asOf,
    obligationRoot: record.obligationRoot,
    adjustmentRoot: record.adjustmentRoot,
    settlementRoot: record.settlementRoot,
    gross: money(canonicalMinor('persisted statement grossMinor', record.grossMinor), record.currency),
    held: money(canonicalMinor('persisted statement heldMinor', record.heldMinor), record.currency),
    deductions: money(canonicalMinor('persisted statement deductionMinor', record.deductionMinor), record.currency),
    netPayable: money(canonicalMinor('persisted statement netPayableMinor', record.netPayableMinor), record.currency),
    paid: money(canonicalMinor('persisted statement paidMinor', record.paidMinor), record.currency),
    completeness: record.completenessData,
  });
  const expected = projectStatementSnapshotRecord(statement);
  const normalized = { ...record, periodStart, periodEnd, asOf } as Readonly<Record<string, unknown>>;
  assertProjectedFields('persisted statement snapshot', normalized, expected as unknown as Readonly<Record<string, unknown>>);
  return statement;
}
