import { buildMerkleCommitment } from './merkle.js';
import { addMoney, assertSameCurrency, money, subtractMoney, type Money } from './money.js';
import type { DeterministicNettingBatch } from './netting.js';
import type { SettlementRecoveryState } from './recovery.js';
import {
  SettlementEligibilityCode,
  assessCarryForward,
  type RoyaltyObligation,
  type SettlementEpoch,
} from './settlement.js';
import {
  createStatementSnapshot,
  type AccountingPeriod,
  type CompletenessState,
  type RoyaltyStatementSnapshot,
  type StatementKind,
} from './statements.js';

export interface StatementDeduction {
  readonly id: string;
  readonly beneficiaryId: string;
  readonly amount: Money;
  readonly basis: string;
}

export interface StatementSettlementEvidence {
  readonly batch: DeterministicNettingBatch;
  readonly recovery: SettlementRecoveryState;
  /** Exact value represented by the rail receipt, after rail-specific deductions. */
  readonly settledAmount: Money;
}

export interface CompileRoyaltyStatementInput {
  readonly statementId: string;
  readonly kind: StatementKind;
  readonly predecessorStatementId?: string;
  readonly beneficiaryId: string;
  readonly period: AccountingPeriod;
  readonly asOf: string;
  readonly completeness: CompletenessState;
  readonly settlementEpoch: SettlementEpoch;
  readonly obligations: readonly RoyaltyObligation[];
  readonly deductions?: readonly StatementDeduction[];
  readonly settlements?: readonly StatementSettlementEvidence[];
}

function withinPeriod(observedAt: string, period: AccountingPeriod): boolean {
  const observed = Date.parse(observedAt);
  const start = Date.parse(period.startInclusive);
  const end = Date.parse(period.endExclusive);
  return Number.isFinite(observed) && Number.isFinite(start) && Number.isFinite(end)
    && observed >= start && observed < end;
}

function committedObligation(obligation: RoyaltyObligation): Readonly<Record<string, unknown>> {
  return Object.freeze({
    id: obligation.id,
    beneficiaryId: obligation.beneficiaryId,
    amountMinor: obligation.amount.amountMinor,
    currency: obligation.amount.currency,
    observedAt: obligation.observedAt,
    routeAvailable: obligation.routeAvailable ?? true,
    rightsConflict: obligation.rightsConflict ?? false,
    legalHold: obligation.legalHold ?? false,
    taxDocumentationRequired: obligation.taxDocumentationRequired ?? false,
    taxDocumentationPresent: obligation.taxDocumentationPresent ?? false,
    fxQuoteRequired: obligation.fxQuoteRequired ?? false,
    fxQuotePresent: obligation.fxQuotePresent ?? false,
    dormantBeneficiary: obligation.dormantBeneficiary ?? false,
  });
}

function sumMoney(values: readonly Money[], currency: string): Money {
  let total = money(0n, currency);
  for (const value of values) total = addMoney(total, value);
  return total;
}

function validateDeduction(deduction: StatementDeduction, beneficiaryId: string, currency: string): void {
  if (!deduction.id.trim() || !deduction.basis.trim()) throw new Error('statement deduction requires id and basis');
  if (deduction.beneficiaryId !== beneficiaryId) throw new Error('statement deduction beneficiary mismatch');
  if (deduction.amount.amountMinor < 0n) throw new Error('statement deduction must be non-negative');
  assertSameCurrency(deduction.amount, money(0n, currency));
}

function validateSettlementEvidence(
  evidence: StatementSettlementEvidence,
  beneficiaryId: string,
  currency: string,
): void {
  const { batch, recovery, settledAmount } = evidence;
  if (batch.beneficiaryId !== beneficiaryId) throw new Error('settlement batch beneficiary mismatch');
  if (batch.currency !== currency) throw new Error('settlement batch currency mismatch');
  if (recovery.batchId !== batch.batchId || recovery.obligationSetRoot !== batch.obligationSetRoot) {
    throw new Error('settlement recovery is not bound to the supplied batch');
  }
  if (settledAmount.amountMinor < 0n) throw new Error('settled amount must be non-negative');
  assertSameCurrency(settledAmount, batch.grossAmount);
  if (settledAmount.amountMinor > batch.grossAmount.amountMinor) {
    throw new Error('settled amount cannot exceed batch gross amount');
  }
}

export function compileRoyaltyStatement(
  input: CompileRoyaltyStatementInput,
): Readonly<RoyaltyStatementSnapshot> {
  if (input.obligations.length === 0) throw new Error('statement compilation requires obligations');
  const orderedObligations = [...input.obligations].sort((a, b) => a.id.localeCompare(b.id));
  const seenObligations = new Set<string>();
  const currency = orderedObligations[0]!.amount.currency;

  for (const obligation of orderedObligations) {
    if (!obligation.id.trim()) throw new Error('obligation id must be non-empty');
    if (seenObligations.has(obligation.id)) throw new Error(`duplicate obligation id: ${obligation.id}`);
    seenObligations.add(obligation.id);
    if (obligation.beneficiaryId !== input.beneficiaryId) throw new Error('statement obligation beneficiary mismatch');
    if (!withinPeriod(obligation.observedAt, input.period)) {
      throw new Error(`obligation ${obligation.id} falls outside the statement period`);
    }
    assertSameCurrency(obligation.amount, money(0n, currency));
  }

  const gross = sumMoney(orderedObligations.map(item => item.amount), currency);
  const assessment = assessCarryForward(orderedObligations, input.settlementEpoch);
  const explicitlyHeld = sumMoney(
    assessment.held.map(item => item.obligation.amount),
    currency,
  );
  const thresholdHold = assessment.eligibility.code === SettlementEligibilityCode.BelowThreshold
    ? assessment.carriedForward
    : money(0n, currency);
  const held = addMoney(explicitlyHeld, thresholdHold);

  const orderedDeductions = [...(input.deductions ?? [])].sort((a, b) => a.id.localeCompare(b.id));
  const seenDeductions = new Set<string>();
  for (const deduction of orderedDeductions) {
    validateDeduction(deduction, input.beneficiaryId, currency);
    if (seenDeductions.has(deduction.id)) throw new Error(`duplicate deduction id: ${deduction.id}`);
    seenDeductions.add(deduction.id);
  }
  const deductions = sumMoney(orderedDeductions.map(item => item.amount), currency);
  const distributableBeforeDeductions = subtractMoney(gross, held);
  if (deductions.amountMinor > distributableBeforeDeductions.amountMinor) {
    throw new Error('deductions cannot exceed presently distributable value');
  }
  const netPayable = subtractMoney(distributableBeforeDeductions, deductions);

  const orderedSettlements = [...(input.settlements ?? [])].sort((a, b) => a.batch.batchId.localeCompare(b.batch.batchId));
  const seenBatches = new Set<string>();
  const finalizedObligationIds = new Set<string>();
  let paid = money(0n, currency);
  for (const evidence of orderedSettlements) {
    validateSettlementEvidence(evidence, input.beneficiaryId, currency);
    if (seenBatches.has(evidence.batch.batchId)) throw new Error(`duplicate settlement batch: ${evidence.batch.batchId}`);
    seenBatches.add(evidence.batch.batchId);
    for (const obligationId of evidence.batch.obligationIds) {
      if (!seenObligations.has(obligationId)) {
        throw new Error(`settlement batch references obligation outside statement: ${obligationId}`);
      }
      if (evidence.recovery.status === 'finalized') {
        if (finalizedObligationIds.has(obligationId)) {
          throw new Error(`obligation appears in more than one finalized settlement: ${obligationId}`);
        }
        finalizedObligationIds.add(obligationId);
      }
    }
    if (evidence.recovery.status === 'finalized') paid = addMoney(paid, evidence.settledAmount);
  }
  if (paid.amountMinor > netPayable.amountMinor) {
    throw new Error('finalized settlement receipts exceed statement net payable value');
  }

  const obligationRoot = buildMerkleCommitment(orderedObligations.map(committedObligation)).root;
  const adjustmentRoot = buildMerkleCommitment(orderedDeductions.map(deduction => ({
    id: deduction.id,
    beneficiaryId: deduction.beneficiaryId,
    amountMinor: deduction.amount.amountMinor,
    currency: deduction.amount.currency,
    basis: deduction.basis,
  }))).root;
  const settlementRoot = buildMerkleCommitment(orderedSettlements.map(evidence => ({
    batchId: evidence.batch.batchId,
    obligationSetRoot: evidence.batch.obligationSetRoot,
    obligationIds: [...evidence.batch.obligationIds].sort(),
    recoveryStatus: evidence.recovery.status,
    finalReceiptRef: evidence.recovery.finalReceiptRef ?? null,
    settledAmountMinor: evidence.settledAmount.amountMinor,
    currency: evidence.settledAmount.currency,
  }))).root;

  return createStatementSnapshot({
    statementId: input.statementId,
    kind: input.kind,
    predecessorStatementId: input.predecessorStatementId,
    beneficiaryId: input.beneficiaryId,
    period: input.period,
    asOf: input.asOf,
    obligationRoot,
    adjustmentRoot,
    settlementRoot,
    gross,
    held,
    deductions,
    netPayable,
    paid,
    completeness: input.completeness,
  });
}
