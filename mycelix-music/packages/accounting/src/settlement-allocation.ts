import { buildMerkleCommitment, type Digest } from './merkle.js';
import { addMoney, assertSameCurrency, money, type Money } from './money.js';
import type { DeterministicNettingBatch } from './netting.js';
import {
  assertRoyaltyDeductionAuthority,
  type RoyaltyDeductionAuthority,
} from './deduction-authority.js';
import type { SettlementRecoveryState } from './recovery.js';

export interface SettlementAllocationAuthority {
  readonly allocationId: string;
  readonly batchId: string;
  readonly obligationSetRoot: Digest;
  readonly eligibilityAsOf: string;
  readonly eligibilityEvidenceRoot: Digest;
  readonly beneficiaryId: string;
  readonly currency: string;
  readonly railReceiptRef: string;
  readonly creatorPaid: Money;
  readonly deductionRoots: readonly Digest[];
  readonly deductionTotal: Money;
  readonly residualHeld: Money;
  readonly residualAuthorityRef?: string;
  readonly allocatedAt: string;
  /** Economic discharge requires exact conservation and no unresolved residual. */
  readonly obligationSetDischarged: boolean;
  readonly allocationRoot: Digest;
}

export interface CreateSettlementAllocationInput {
  readonly allocationId: string;
  readonly batch: DeterministicNettingBatch;
  readonly recovery: SettlementRecoveryState;
  readonly deductions: readonly RoyaltyDeductionAuthority[];
  readonly residualHeld: Money;
  readonly residualAuthorityRef?: string;
  readonly allocatedAt: string;
}

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

function parsedTime(label: string, value: string): number {
  const parsed = Date.parse(value);
  if (!Number.isFinite(parsed)) throw new Error(`${label} must be a valid timestamp`);
  return parsed;
}

function receiptBacked(recovery: SettlementRecoveryState): boolean {
  return recovery.status === 'finalized' || recovery.status === 'partial_finality';
}

function assertRecoveryMatchesBatch(batch: DeterministicNettingBatch, recovery: SettlementRecoveryState): void {
  if (recovery.batchId !== batch.batchId || recovery.obligationSetRoot !== batch.obligationSetRoot) {
    throw new Error('settlement allocation recovery is not bound to the supplied batch');
  }
  if (recovery.eligibilityAsOf !== batch.eligibilityAsOf || recovery.eligibilityEvidenceRoot !== batch.eligibilityEvidenceRoot) {
    throw new Error('settlement allocation recovery is not bound to the supplied eligibility snapshot');
  }
  if (!receiptBacked(recovery) || !recovery.finalReceiptRef?.trim() || !recovery.finalSettledAmount) {
    throw new Error('settlement allocation requires rail-finalized receipt and amount evidence');
  }
}

function sameMoney(left: Money, right: Money): boolean {
  return left.amountMinor === right.amountMinor && left.currency === right.currency;
}

function sameRoots(left: readonly Digest[], right: readonly Digest[]): boolean {
  return left.length === right.length && left.every((root, index) => root === right[index]);
}

export function createSettlementAllocationAuthority(
  input: CreateSettlementAllocationInput,
): Readonly<SettlementAllocationAuthority> {
  const allocationId = required('settlement allocation id', input.allocationId);
  const allocatedAt = timestamp('settlement allocation allocatedAt', input.allocatedAt);
  const allocatedAtMs = Date.parse(allocatedAt);
  const { batch, recovery } = input;
  assertRecoveryMatchesBatch(batch, recovery);

  const creatorPaid = money(recovery.finalSettledAmount!.amountMinor, recovery.finalSettledAmount!.currency);
  assertSameCurrency(creatorPaid, batch.grossAmount);
  if (creatorPaid.amountMinor <= 0n) throw new Error('settlement allocation creator paid amount must be positive');
  if (creatorPaid.amountMinor > batch.grossAmount.amountMinor) {
    throw new Error('settlement allocation creator paid amount cannot exceed batch gross');
  }
  if (recovery.lastObservedAt === undefined) throw new Error('settlement allocation requires rail-finality observation time');
  const recoveryObservedAt = parsedTime('settlement recovery lastObservedAt', recovery.lastObservedAt);
  if (recoveryObservedAt > allocatedAtMs) {
    throw new Error('settlement allocation cannot predate its rail-finality evidence');
  }

  const orderedDeductions = [...input.deductions].sort((a, b) => a.deductionRoot.localeCompare(b.deductionRoot));
  const seenRoots = new Set<string>();
  let deductionTotal = money(0n, batch.currency);
  for (const deduction of orderedDeductions) {
    assertRoyaltyDeductionAuthority(deduction);
    if (seenRoots.has(deduction.deductionRoot)) throw new Error(`duplicate settlement deduction root: ${deduction.deductionRoot}`);
    seenRoots.add(deduction.deductionRoot);
    if (deduction.beneficiaryId !== batch.beneficiaryId) throw new Error('settlement deduction beneficiary mismatch');
    assertSameCurrency(deduction.amount, batch.grossAmount);
    const deductionObservedAt = parsedTime('settlement deduction observedAt', deduction.observedAt);
    if (deductionObservedAt > allocatedAtMs) throw new Error('settlement allocation cannot predate deduction evidence');
    deductionTotal = addMoney(deductionTotal, deduction.amount);
  }

  const residualHeld = money(input.residualHeld.amountMinor, input.residualHeld.currency);
  assertSameCurrency(residualHeld, batch.grossAmount);
  if (residualHeld.amountMinor < 0n) throw new Error('settlement residual held amount must be non-negative');
  const residualAuthorityRef = input.residualAuthorityRef === undefined
    ? undefined
    : required('settlement residualAuthorityRef', input.residualAuthorityRef);
  if (residualHeld.amountMinor > 0n && residualAuthorityRef === undefined) {
    throw new Error('nonzero settlement residual requires residualAuthorityRef');
  }
  if (residualHeld.amountMinor === 0n && residualAuthorityRef !== undefined) {
    throw new Error('zero settlement residual must not claim residualAuthorityRef');
  }

  const accounted = addMoney(addMoney(creatorPaid, deductionTotal), residualHeld);
  if (accounted.amountMinor !== batch.grossAmount.amountMinor) {
    throw new Error('settlement allocation must conserve batch gross = creator paid + deductions + residual held');
  }

  const obligationSetDischarged = residualHeld.amountMinor === 0n;
  const deductionRoots = Object.freeze(orderedDeductions.map(item => item.deductionRoot));
  const committed = Object.freeze({
    allocationId,
    batchId: batch.batchId,
    obligationSetRoot: batch.obligationSetRoot,
    eligibilityAsOf: batch.eligibilityAsOf,
    eligibilityEvidenceRoot: batch.eligibilityEvidenceRoot,
    beneficiaryId: batch.beneficiaryId,
    currency: batch.currency,
    railReceiptRef: recovery.finalReceiptRef!,
    creatorPaidMinor: creatorPaid.amountMinor,
    deductionRoots,
    deductionTotalMinor: deductionTotal.amountMinor,
    residualHeldMinor: residualHeld.amountMinor,
    residualAuthorityRef: residualAuthorityRef ?? null,
    allocatedAt,
    obligationSetDischarged,
  });
  const allocationRoot = buildMerkleCommitment([{ recordType: 'settlement_allocation_v1', ...committed }]).root;

  return Object.freeze({
    allocationId,
    batchId: batch.batchId,
    obligationSetRoot: batch.obligationSetRoot,
    eligibilityAsOf: batch.eligibilityAsOf,
    eligibilityEvidenceRoot: batch.eligibilityEvidenceRoot,
    beneficiaryId: batch.beneficiaryId,
    currency: batch.currency,
    railReceiptRef: recovery.finalReceiptRef!,
    creatorPaid,
    deductionRoots,
    deductionTotal,
    residualHeld,
    ...(residualAuthorityRef === undefined ? {} : { residualAuthorityRef }),
    allocatedAt,
    obligationSetDischarged,
    allocationRoot,
  });
}

export function assertSettlementAllocationAuthority(
  allocation: SettlementAllocationAuthority,
  input: Omit<CreateSettlementAllocationInput, 'allocationId' | 'allocatedAt' | 'residualHeld' | 'residualAuthorityRef'>,
): void {
  const expected = createSettlementAllocationAuthority({
    allocationId: allocation.allocationId,
    batch: input.batch,
    recovery: input.recovery,
    deductions: input.deductions,
    residualHeld: allocation.residualHeld,
    ...(allocation.residualAuthorityRef === undefined ? {} : { residualAuthorityRef: allocation.residualAuthorityRef }),
    allocatedAt: allocation.allocatedAt,
  });
  const exact = allocation.allocationId === expected.allocationId
    && allocation.batchId === expected.batchId
    && allocation.obligationSetRoot === expected.obligationSetRoot
    && allocation.eligibilityAsOf === expected.eligibilityAsOf
    && allocation.eligibilityEvidenceRoot === expected.eligibilityEvidenceRoot
    && allocation.beneficiaryId === expected.beneficiaryId
    && allocation.currency === expected.currency
    && allocation.railReceiptRef === expected.railReceiptRef
    && sameMoney(allocation.creatorPaid, expected.creatorPaid)
    && sameRoots(allocation.deductionRoots, expected.deductionRoots)
    && sameMoney(allocation.deductionTotal, expected.deductionTotal)
    && sameMoney(allocation.residualHeld, expected.residualHeld)
    && allocation.residualAuthorityRef === expected.residualAuthorityRef
    && allocation.allocatedAt === expected.allocatedAt
    && allocation.obligationSetDischarged === expected.obligationSetDischarged
    && allocation.allocationRoot === expected.allocationRoot;
  if (!exact) throw new Error('settlement allocation does not match authoritative reconstruction');
}
