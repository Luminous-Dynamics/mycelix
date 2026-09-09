import { buildMerkleCommitment, type Digest } from './merkle.js';
import { addMoney, money, type Money } from './money.js';
import {
  SettlementEligibilityCode,
  assessCarryForward,
  partitionObligationsAtCutoff,
  type RoyaltyObligation,
  type SettlementEpoch,
} from './settlement.js';

export interface DeterministicNettingBatch {
  readonly batchId: string;
  readonly epochId: string;
  readonly beneficiaryId: string;
  readonly currency: string;
  readonly obligationIds: readonly string[];
  readonly obligationSetRoot: Digest;
  readonly grossAmount: Money;
}

function groupKey(obligation: RoyaltyObligation): string {
  return `${obligation.beneficiaryId}\u0000${obligation.amount.currency}`;
}

export function buildDeterministicNettingBatches(
  obligations: readonly RoyaltyObligation[],
  epoch: SettlementEpoch,
): readonly DeterministicNettingBatch[] {
  if (obligations.length === 0) return Object.freeze([]);
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
    const assessment = assessCarryForward(group, epoch);
    if (assessment.eligibility.code !== SettlementEligibilityCode.Eligible) continue;

    const payable = [...assessment.payableNow].sort((a, b) => a.id.localeCompare(b.id));
    if (payable.length === 0) continue;
    let grossAmount = money(0n, payable[0]!.amount.currency);
    for (const obligation of payable) grossAmount = addMoney(grossAmount, obligation.amount);

    const committedLines = payable.map(obligation => ({
      id: obligation.id,
      beneficiaryId: obligation.beneficiaryId,
      amountMinor: obligation.amount.amountMinor,
      currency: obligation.amount.currency,
      observedAt: obligation.observedAt,
    }));
    const obligationSetRoot = buildMerkleCommitment(committedLines).root;
    const beneficiaryId = payable[0]!.beneficiaryId;
    const currency = payable[0]!.amount.currency;
    batches.push(Object.freeze({
      batchId: `netting:${epoch.id}:${obligationSetRoot}`,
      epochId: epoch.id,
      beneficiaryId,
      currency,
      obligationIds: Object.freeze(payable.map(obligation => obligation.id)),
      obligationSetRoot,
      grossAmount,
    }));
  }

  return Object.freeze(batches);
}
