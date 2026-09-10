import { buildMerkleCommitment, type Digest } from './merkle.js';
import { addMoney, money, type Money } from './money.js';
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

export interface DeterministicNettingBatch {
  readonly batchId: string;
  readonly epochId: string;
  readonly beneficiaryId: string;
  readonly currency: string;
  readonly obligationIds: readonly string[];
  readonly obligationSetRoot: Digest;
  readonly eligibilityAsOf: string;
  readonly eligibilityEvidenceRoot: Digest;
  readonly grossAmount: Money;
}

function groupKey(obligation: RoyaltyObligation): string {
  return `${obligation.beneficiaryId}\u0000${obligation.amount.currency}`;
}

function assertUniqueObligationIds(obligations: readonly RoyaltyObligation[]): void {
  const seen = new Set<string>();
  for (const obligation of obligations) {
    if (seen.has(obligation.id)) throw new Error(`duplicate netting obligation id: ${obligation.id}`);
    seen.add(obligation.id);
  }
}

function sameObligationIds(left: readonly string[], right: readonly string[]): boolean {
  return left.length === right.length && left.every((id, index) => id === right[index]);
}

export function buildDeterministicNettingBatches(
  obligations: readonly RoyaltyObligation[],
  epoch: SettlementEpoch,
  eligibilityObservations: readonly SettlementEligibilityObservation[],
): readonly DeterministicNettingBatch[] {
  if (obligations.length === 0) return Object.freeze([]);
  assertUniqueObligationIds(obligations);
  assertEligibilityObservationReferences(obligations, eligibilityObservations);
  const { inEpoch } = partitionObligationsAtCutoff(obligations, epoch);
  const groups = new Map<string, RoyaltyObligation[]>();
  for (const obligation of inEpoch) {
    const key = groupKey(obligation);
    const group = groups.get(key) ?? [];
    group.push(obligation);
    groups.set(key, group);
  }

  const batches: DeterministicNettingBatch[] = [];
  for (const key of [...groups.keys()].sort()) {
    const group = groups.get(key)!.slice().sort((a, b) => a.id.localeCompare(b.id));
    const assessment = assessCarryForward(group, epoch, eligibilityObservations);
    const eligibilityEvidence = buildSettlementEligibilityEvidenceCommitment(
      group,
      eligibilityObservations,
      epoch.eligibilityAsOf,
    );
    if (assessment.eligibility.code !== SettlementEligibilityCode.Eligible) continue;

    const payable = [...assessment.payableNow].sort((a, b) => a.id.localeCompare(b.id));
    if (payable.length === 0) continue;
    let grossAmount = money(0n, payable[0]!.amount.currency);
    for (const obligation of payable) grossAmount = addMoney(grossAmount, obligation.amount);

    const committedLines = payable.map(obligation => ({
      id: obligation.id,
      authorityRoot: obligation.authorityRoot,
      beneficiaryId: obligation.beneficiaryId,
      amountMinor: obligation.amount.amountMinor,
      currency: obligation.amount.currency,
      observedAt: obligation.observedAt,
    }));
    const obligationSetRoot = buildMerkleCommitment(committedLines).root;
    const beneficiaryId = payable[0]!.beneficiaryId;
    const currency = payable[0]!.amount.currency;
    const batchIdentityRoot = buildMerkleCommitment([{
      epochId: epoch.id.trim(),
      cutoff: new Date(Date.parse(epoch.cutoff)).toISOString(),
      eligibilityAsOf: eligibilityEvidence.asOf,
      beneficiaryId,
      currency,
      obligationSetRoot,
      eligibilityEvidenceRoot: eligibilityEvidence.root,
      grossAmountMinor: grossAmount.amountMinor,
    }]).root;

    batches.push(Object.freeze({
      batchId: `netting:${epoch.id.trim()}:${batchIdentityRoot}`,
      epochId: epoch.id.trim(),
      beneficiaryId,
      currency,
      obligationIds: Object.freeze(payable.map(obligation => obligation.id)),
      obligationSetRoot,
      eligibilityAsOf: eligibilityEvidence.asOf,
      eligibilityEvidenceRoot: eligibilityEvidence.root,
      grossAmount,
    }));
  }

  return Object.freeze(batches);
}

/**
 * Reconstruct a supplied batch from authoritative obligations, append-only
 * eligibility evidence and the epoch. A serialized/queued batch is transport
 * data, never accounting authority.
 */
export function assertDeterministicNettingBatch(
  batch: DeterministicNettingBatch,
  authoritativeObligations: readonly RoyaltyObligation[],
  epoch: SettlementEpoch,
  eligibilityObservations: readonly SettlementEligibilityObservation[],
): void {
  const expected = buildDeterministicNettingBatches(
    authoritativeObligations,
    epoch,
    eligibilityObservations,
  ).find(candidate => candidate.beneficiaryId === batch.beneficiaryId && candidate.currency === batch.currency);
  if (!expected) throw new Error('netting batch has no authoritative payable obligation set');

  const exact = batch.batchId === expected.batchId
    && batch.epochId === expected.epochId
    && batch.beneficiaryId === expected.beneficiaryId
    && batch.currency === expected.currency
    && batch.obligationSetRoot === expected.obligationSetRoot
    && batch.eligibilityAsOf === expected.eligibilityAsOf
    && batch.eligibilityEvidenceRoot === expected.eligibilityEvidenceRoot
    && batch.grossAmount.currency === expected.grossAmount.currency
    && batch.grossAmount.amountMinor === expected.grossAmount.amountMinor
    && sameObligationIds(batch.obligationIds, expected.obligationIds);

  if (!exact) {
    throw new Error('netting batch does not match deterministic authoritative reconstruction');
  }
}
