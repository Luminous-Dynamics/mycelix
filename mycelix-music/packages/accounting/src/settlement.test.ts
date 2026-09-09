import { describe, expect, it } from 'vitest';
import {
  SettlementEligibilityCode,
  assessCarryForward,
  classifySettlementEligibility,
  partitionObligationsAtCutoff,
  type RoyaltyObligation,
} from './settlement.js';
import { money } from './money.js';

const epoch = {
  id: 'epoch:2026-09',
  cutoff: '2026-09-30T23:59:59Z',
  minimumPayout: money(500n),
} as const;

function obligation(id: string, amountMinor: bigint, observedAt = '2026-09-20T00:00:00Z'): RoyaltyObligation {
  return { id, beneficiaryId: 'creator:alice', amount: money(amountMinor), observedAt, routeAvailable: true };
}

describe('settlement eligibility and carry-forward', () => {
  it('preserves micro-royalties until the threshold is crossed', () => {
    const assessment = assessCarryForward([
      obligation('o1', 2n),
      obligation('o2', 270n),
      obligation('o3', 311n),
    ], epoch);
    expect(assessment.carriedForward.amountMinor).toBe(583n);
    expect(assessment.eligibility.code).toBe(SettlementEligibilityCode.Eligible);
    expect(assessment.payableNow.map(item => item.id)).toEqual(['o1', 'o2', 'o3']);
  });

  it('reports below-threshold without erasing the obligation', () => {
    const assessment = assessCarryForward([obligation('o1', 2n)], epoch);
    expect(assessment.eligibility.code).toBe(SettlementEligibilityCode.BelowThreshold);
    expect(assessment.payableNow[0]?.id).toBe('o1');
    expect(assessment.carriedForward.amountMinor).toBe(2n);
  });

  it('distinguishes not payable now from not owed', () => {
    const held = { ...obligation('o1', 327n), routeAvailable: false };
    expect(classifySettlementEligibility(held).code).toBe(SettlementEligibilityCode.AwaitingPayeeRoute);
    const assessment = assessCarryForward([held], epoch);
    expect(assessment.held[0]?.obligation.amount.amountMinor).toBe(327n);
  });

  it('moves late observations to the next epoch deterministically', () => {
    const partition = partitionObligationsAtCutoff([
      obligation('early', 1n),
      obligation('late', 1n, '2026-10-01T00:00:00Z'),
    ], epoch);
    expect(partition.inEpoch.map(item => item.id)).toEqual(['early']);
    expect(partition.nextEpoch.map(item => item.id)).toEqual(['late']);
  });
});
