import { createRoyaltyDeductionAuthority, type RoyaltyDeductionAuthority } from './deduction-authority.js';
import { canonicalAccountingValue } from './merkle.js';
import { money } from './money.js';
import type { DeterministicNettingBatch } from './netting.js';
import { createRoyaltyObligationAuthority, type RoyaltyObligationAuthority } from './obligation-authority.js';
import {
  projectDeductionRecord,
  projectEligibilityObservationRecord,
  projectRoyaltyObligationRecord,
  projectSettlementAllocationRecord,
  projectSettlementObservationRecord,
  projectStatementSnapshotRecord,
  type PersistedDeductionRecord,
  type PersistedEligibilityObservationRecord,
  type PersistedRoyaltyObligationRecord,
  type PersistedSettlementAllocationRecord,
  type PersistedSettlementObservationRecord,
  type PersistedStatementSnapshotRecord,
} from './persistence-projection.js';
import type { StatementDeduction } from './projection.js';
import {
  SettlementAttemptState,
  reconstructSettlementRecovery,
  type SettlementAttemptObservation,
  type SettlementRecoveryState,
} from './recovery.js';
import { createSettlementAllocationAuthority, type SettlementAllocationAuthority } from './settlement-allocation.js';
import type { SettlementEligibilityObservation } from './settlement.js';
import { createStatementSnapshot, type RoyaltyStatementSnapshot } from './statements.js';

const MINOR_RE = /^(0|[1-9][0-9]*)$/;
const DIGEST_RE = /^[0-9a-f]{64}$/;
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
export type VerifiablePersistedSettlementAllocationRecord =
  Omit<PersistedSettlementAllocationRecord, 'eligibilityAsOf' | 'allocatedAt'> & {
    readonly eligibilityAsOf: DateLike;
    readonly allocatedAt: DateLike;
    readonly deductionRoots: unknown;
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

function canonicalDigestArray(label: string, value: unknown): readonly string[] {
  if (!Array.isArray(value)) throw new Error(`${label} must be an array`);
  const roots = value.map((item, index) => {
    if (typeof item !== 'string' || !DIGEST_RE.test(item)) throw new Error(`${label}[${index}] must be a lowercase SHA-256 digest`);
    return item;
  });
  const sorted = [...roots].sort();
  if (roots.some((root, index) => root !== sorted[index])) throw new Error(`${label} must be canonically sorted`);
  if (new Set(roots).size !== roots.length) throw new Error(`${label} must not contain duplicates`);
  return Object.freeze(roots);
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
  const deduction = createRoyaltyDeductionAuthority({
    id: record.id,
    beneficiaryId: record.beneficiaryId,
    amount: money(canonicalMinor('persisted deduction amountMinor', record.amountMinor), record.currency),
    basis: record.basis,
    authorityRef: record.authorityRef,
    observedAt,
  });
  const expected = projectDeductionRecord(deduction);
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
  const hasAmountMinor = record.settledAmountMinor !== undefined;
  const hasCurrency = record.settledCurrency !== undefined;
  if (hasAmountMinor !== hasCurrency) {
    throw new Error('persisted settled amount requires both settledAmountMinor and settledCurrency');
  }
  const settledAmount = hasAmountMinor
    ? money(canonicalMinor('persisted settledAmountMinor', record.settledAmountMinor!), record.settledCurrency!)
    : undefined;
  const observation: SettlementAttemptObservation = Object.freeze({
    attemptId: record.attemptId,
    batchId: record.batchId,
    obligationSetRoot: record.obligationSetRoot,
    eligibilityAsOf,
    eligibilityEvidenceRoot: record.eligibilityEvidenceRoot,
    state: record.state,
    observedAt,
    ...(record.railReceiptRef === undefined ? {} : { railReceiptRef: record.railReceiptRef }),
    ...(settledAmount === undefined ? {} : { settledAmount }),
    ...(record.supersedesAttemptId === undefined ? {} : { supersedesAttemptId: record.supersedesAttemptId }),
  });
  const expected = projectSettlementObservationRecord(record.id, observation);
  const normalized = { ...record, eligibilityAsOf, observedAt } as Readonly<Record<string, unknown>>;
  assertProjectedFields('persisted settlement observation', normalized, expected as unknown as Readonly<Record<string, unknown>>);
  return observation;
}

export function verifyPersistedSettlementRecovery(
  batch: DeterministicNettingBatch,
  records: readonly VerifiablePersistedSettlementObservationRecord[],
): Readonly<SettlementRecoveryState> {
  const observations = records.map(verifyPersistedSettlementObservation);
  return reconstructSettlementRecovery(batch, observations);
}

export function verifyPersistedSettlementAllocation(
  record: VerifiablePersistedSettlementAllocationRecord,
  batch: DeterministicNettingBatch,
  recovery: SettlementRecoveryState,
  deductions: readonly RoyaltyDeductionAuthority[],
): Readonly<SettlementAllocationAuthority> {
  const eligibilityAsOf = canonicalTimestamp('persisted allocation eligibilityAsOf', record.eligibilityAsOf);
  const allocatedAt = canonicalTimestamp('persisted allocation allocatedAt', record.allocatedAt);
  const deductionRoots = canonicalDigestArray('persisted allocation deductionRoots', record.deductionRoots);
  const expectedRoots = [...deductions].map(item => item.deductionRoot).sort();
  if (deductionRoots.length !== expectedRoots.length || deductionRoots.some((root, index) => root !== expectedRoots[index])) {
    throw new Error('persisted allocation deduction roots do not match authoritative deductions');
  }
  const allocation = createSettlementAllocationAuthority({
    allocationId: record.allocationId,
    batch,
    recovery,
    deductions,
    residualHeld: money(canonicalMinor('persisted allocation residualHeldMinor', record.residualHeldMinor), record.currency),
    ...(record.residualAuthorityRef === undefined ? {} : { residualAuthorityRef: record.residualAuthorityRef }),
    allocatedAt,
  });
  const expected = projectSettlementAllocationRecord(allocation, batch, recovery, deductions);
  const normalized = { ...record, eligibilityAsOf, allocatedAt, deductionRoots } as Readonly<Record<string, unknown>>;
  assertProjectedFields('persisted settlement allocation', normalized, expected as unknown as Readonly<Record<string, unknown>>);
  canonicalMinor('persisted allocation creatorPaidMinor', record.creatorPaidMinor);
  canonicalMinor('persisted allocation deductionTotalMinor', record.deductionTotalMinor);
  return allocation;
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
