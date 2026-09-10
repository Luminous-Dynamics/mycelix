import { assertRoyaltyDeductionAuthority, type RoyaltyDeductionAuthority } from './deduction-authority.js';
import { buildMerkleCommitment } from './merkle.js';
import { addMoney, assertSameCurrency, money, subtractMoney, type Money } from './money.js';
import {
  assertDeterministicNettingBatch,
  type DeterministicNettingBatch,
} from './netting.js';
import type { SettlementRecoveryState } from './recovery.js';
import {
  SettlementEligibilityCode,
  assertEligibilityObservationReferences,
  assessCarryForward,
  buildSettlementEligibilityEvidenceCommitment,
  partitionObligationsAtCutoff,
  type RoyaltyObligation,
  type SettlementEligibilityObservation,
  type SettlementEpoch,
} from './settlement.js';
import {
  createStatementSnapshot,
  type AccountingPeriod,
  type CompletenessState,
  type RoyaltyStatementSnapshot,
  type StatementKind,
} from './statements.js';

/** Compatibility name for the canonical, root-bearing deduction authority. */
export type StatementDeduction = RoyaltyDeductionAuthority;

export interface StatementSettlementEvidence {
  readonly batch: DeterministicNettingBatch;
  readonly recovery: SettlementRecoveryState;
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
  readonly eligibilityObservations: readonly SettlementEligibilityObservation[];
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
    authorityRoot: obligation.authorityRoot,
    beneficiaryId: obligation.beneficiaryId,
    amountMinor: obligation.amount.amountMinor,
    currency: obligation.amount.currency,
    observedAt: obligation.observedAt,
    usageEvidenceRef: obligation.provenance.usageEvidenceRef,
    rightsResolutionRef: obligation.provenance.rightsResolutionRef,
    economicTermsRef: obligation.provenance.economicTermsRef,
  });
}

function sumMoney(values: readonly Money[], currency: string): Money {
  let total = money(0n, currency);
  for (const value of values) total = addMoney(total, value);
  return total;
}

function validateDeduction(
  deduction: StatementDeduction,
  beneficiaryId: string,
  currency: string,
  statementAsOf: number,
): void {
  assertRoyaltyDeductionAuthority(deduction);
  if (deduction.beneficiaryId !== beneficiaryId) throw new Error('statement deduction beneficiary mismatch');
  assertSameCurrency(deduction.amount, money(0n, currency));
  const observedAt = Date.parse(deduction.observedAt);
  if (observedAt > statementAsOf) throw new Error('statement deduction was observed after the statement asOf');
}

function hasReceiptBackedFinality(recovery: SettlementRecoveryState): boolean {
  return recovery.status === 'finalized' || recovery.status === 'partial_finality';
}

function validateSettlementEvidence(
  evidence: StatementSettlementEvidence,
  beneficiaryId: string,
  currency: string,
  settlementEpochId: string,
  statementAsOf: number,
): void {
  const { batch, recovery } = evidence;
  if (batch.beneficiaryId !== beneficiaryId) throw new Error('settlement batch beneficiary mismatch');
  if (batch.currency !== currency) throw new Error('settlement batch currency mismatch');
  if (batch.epochId !== settlementEpochId) throw new Error('settlement batch epoch mismatch');
  if (recovery.batchId !== batch.batchId || recovery.obligationSetRoot !== batch.obligationSetRoot) {
    throw new Error('settlement recovery is not bound to the supplied batch');
  }
  if (
    recovery.eligibilityAsOf !== batch.eligibilityAsOf
    || recovery.eligibilityEvidenceRoot !== batch.eligibilityEvidenceRoot
  ) {
    throw new Error('settlement recovery is not bound to the supplied eligibility snapshot');
  }

  if (hasReceiptBackedFinality(recovery)) {
    if (!recovery.finalReceiptRef?.trim() || !recovery.finalSettledAmount) {
      throw new Error('receipt-backed settlement recovery requires a final receipt and exact settled amount');
    }
    if (recovery.finalSettledAmount.amountMinor <= 0n) throw new Error('receipt-backed settled amount must be positive');
    assertSameCurrency(recovery.finalSettledAmount, batch.grossAmount);
    if (recovery.finalSettledAmount.amountMinor > batch.grossAmount.amountMinor) {
      throw new Error('receipt-backed settled amount cannot exceed batch gross amount');
    }
    if (recovery.status === 'finalized') {
      if (!recovery.obligationSetSettled) throw new Error('whole-batch finalized recovery must mark the obligation set settled');
      if (recovery.finalSettledAmount.amountMinor !== batch.grossAmount.amountMinor) {
        throw new Error('whole-batch finalized recovery must equal batch gross amount');
      }
    } else {
      if (recovery.obligationSetSettled) throw new Error('partial finality cannot mark the obligation set settled');
      if (recovery.finalSettledAmount.amountMinor >= batch.grossAmount.amountMinor) {
        throw new Error('partial finality must be less than batch gross amount');
      }
    }
  } else if (recovery.obligationSetSettled) {
    throw new Error('non-final settlement recovery cannot mark the obligation set settled');
  }

  if (recovery.status !== 'never_attempted' && !recovery.lastObservedAt) {
    throw new Error('observed settlement recovery requires lastObservedAt');
  }
  if (recovery.lastObservedAt) {
    const lastObservedAt = Date.parse(recovery.lastObservedAt);
    if (!Number.isFinite(lastObservedAt)) throw new Error('settlement recovery lastObservedAt must be valid');
    if (lastObservedAt > statementAsOf) throw new Error('settlement evidence cannot be later than statement asOf');
  }
}

export function compileRoyaltyStatement(
  input: CompileRoyaltyStatementInput,
): Readonly<RoyaltyStatementSnapshot> {
  if (input.obligations.length === 0) throw new Error('statement compilation requires obligations');
  const statementAsOf = Date.parse(input.asOf);
  if (!Number.isFinite(statementAsOf)) throw new Error('statement asOf must be a valid timestamp');
  const eligibilityAsOf = Date.parse(input.settlementEpoch.eligibilityAsOf);
  if (!Number.isFinite(eligibilityAsOf)) throw new Error('settlement eligibilityAsOf must be a valid timestamp');
  if (eligibilityAsOf > statementAsOf) throw new Error('settlement eligibilityAsOf cannot be later than statement asOf');

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
    if (Date.parse(obligation.observedAt) > statementAsOf) {
      throw new Error(`obligation ${obligation.id} was observed after the statement asOf`);
    }
    assertSameCurrency(obligation.amount, money(0n, currency));
  }
  assertEligibilityObservationReferences(orderedObligations, input.eligibilityObservations);

  const gross = sumMoney(orderedObligations.map(item => item.amount), currency);
  const cutoffPartition = partitionObligationsAtCutoff(orderedObligations, input.settlementEpoch);
  const postCutoffHeld = sumMoney(cutoffPartition.nextEpoch.map(item => item.amount), currency);
  const eligibilityEvidence = buildSettlementEligibilityEvidenceCommitment(
    cutoffPartition.inEpoch,
    input.eligibilityObservations,
    input.settlementEpoch.eligibilityAsOf,
  );

  let eligibilityHeld = money(0n, currency);
  let thresholdHold = money(0n, currency);
  if (cutoffPartition.inEpoch.length > 0) {
    const assessment = assessCarryForward(cutoffPartition.inEpoch, input.settlementEpoch, input.eligibilityObservations);
    eligibilityHeld = sumMoney(assessment.held.map(item => item.obligation.amount), currency);
    thresholdHold = assessment.eligibility.code === SettlementEligibilityCode.BelowThreshold
      ? assessment.carriedForward
      : money(0n, currency);
  }
  const held = addMoney(postCutoffHeld, addMoney(eligibilityHeld, thresholdHold));

  const orderedDeductions = [...(input.deductions ?? [])].sort((a, b) => a.id.localeCompare(b.id));
  const seenDeductions = new Set<string>();
  for (const deduction of orderedDeductions) {
    validateDeduction(deduction, input.beneficiaryId, currency, statementAsOf);
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
  const receiptBackedObligationIds = new Set<string>();
  let paid = money(0n, currency);
  for (const evidence of orderedSettlements) {
    assertDeterministicNettingBatch(evidence.batch, orderedObligations, input.settlementEpoch, input.eligibilityObservations);
    validateSettlementEvidence(evidence, input.beneficiaryId, currency, input.settlementEpoch.id.trim(), statementAsOf);
    if (seenBatches.has(evidence.batch.batchId)) throw new Error(`duplicate settlement batch: ${evidence.batch.batchId}`);
    seenBatches.add(evidence.batch.batchId);
    for (const obligationId of evidence.batch.obligationIds) {
      if (!seenObligations.has(obligationId)) throw new Error(`settlement batch references obligation outside statement: ${obligationId}`);
      if (cutoffPartition.nextEpoch.some(obligation => obligation.id === obligationId)) {
        throw new Error(`settlement batch references post-cutoff obligation: ${obligationId}`);
      }
      if (hasReceiptBackedFinality(evidence.recovery)) {
        if (receiptBackedObligationIds.has(obligationId)) {
          throw new Error(`obligation appears in more than one receipt-backed settlement: ${obligationId}`);
        }
        receiptBackedObligationIds.add(obligationId);
      }
    }
    if (hasReceiptBackedFinality(evidence.recovery)) {
      paid = addMoney(paid, evidence.recovery.finalSettledAmount!);
    }
  }
  if (paid.amountMinor > netPayable.amountMinor) {
    throw new Error('receipt-backed settlement amounts exceed statement net payable value');
  }

  const obligationRoot = buildMerkleCommitment(orderedObligations.map(committedObligation)).root;
  const adjustmentRoot = buildMerkleCommitment(orderedDeductions.map(deduction => ({
    evidenceKind: 'royalty_deduction_authority_v1',
    id: deduction.id,
    deductionRoot: deduction.deductionRoot,
  }))).root;
  const settlementRoot = buildMerkleCommitment([
    {
      evidenceKind: 'settlement_eligibility_v1',
      eligibilityAsOf: eligibilityEvidence.asOf,
      eligibilityEvidenceRoot: eligibilityEvidence.root,
    },
    ...orderedSettlements.map(evidence => ({
      evidenceKind: 'settlement_execution_v3',
      batchId: evidence.batch.batchId,
      obligationSetRoot: evidence.batch.obligationSetRoot,
      eligibilityAsOf: evidence.batch.eligibilityAsOf,
      eligibilityEvidenceRoot: evidence.batch.eligibilityEvidenceRoot,
      obligationIds: [...evidence.batch.obligationIds].sort(),
      recoveryStatus: evidence.recovery.status,
      obligationSetSettled: evidence.recovery.obligationSetSettled,
      finalReceiptRef: evidence.recovery.finalReceiptRef ?? null,
      finalSettledAmountMinor: evidence.recovery.finalSettledAmount?.amountMinor ?? null,
      currency: evidence.recovery.finalSettledAmount?.currency ?? evidence.batch.currency,
    })),
  ]).root;

  return createStatementSnapshot({
    statementId: input.statementId,
    kind: input.kind,
    ...(input.predecessorStatementId === undefined ? {} : { predecessorStatementId: input.predecessorStatementId }),
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
