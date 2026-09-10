import { describe, expect, it } from 'vitest';
import {
  assertDeterministicNettingBatch,
  buildDeterministicNettingBatches,
} from './netting.js';
import { money } from './money.js';
import { createRoyaltyObligationAuthority } from './obligation-authority.js';
import {
  SettlementEligibilityCode,
  type ObservableSettlementEligibilityCode,
  type RoyaltyObligation,
  type SettlementEligibilityObservation,
} from './settlement.js';

const epoch = {
  id: 'epoch:2026-09',
  cutoff: '2026-09-30T23:59:59Z',
  eligibilityAsOf: '2026-10-01T00:00:00Z',
  minimumPayout: money(500n),
} as const;

function obligation(id: string, amountMinor: bigint, usageRef = `usage:${id}`): RoyaltyObligation {
  return createRoyaltyObligationAuthority({
    id,
    beneficiaryId: 'creator:alice',
    amount: money(amountMinor),
    observedAt: '2026-09-20T00:00:00Z',
    provenance: {
      usageEvidenceRef: usageRef,
      rightsResolutionRef: `rights:${id}`,
      economicTermsRef: 'terms:v1',
    },
  });
}

function eligibility(
  obligationId: string,
  code: ObservableSettlementEligibilityCode = SettlementEligibilityCode.Eligible,
  sourceRef = 'eligibility-authority:v1',
): SettlementEligibilityObservation {
  return {
    id: `eligibility:${obligationId}:${code}:${sourceRef}`,
    obligationId,
    code,
    sourceRef,
    observedAt: '2026-09-30T12:00:00Z',
  };
}

function allEligible(obligations: readonly RoyaltyObligation[]): readonly SettlementEligibilityObservation[] {
  return obligations.map(item => eligibility(item.id));
}

describe('deterministic netting', () => {
  it('produces identical batch identity regardless of input order', () => {
    const obligations = [obligation('o3', 311n), obligation('o1', 2n), obligation('o2', 270n)];
    const observations = allEligible(obligations);
    const first = buildDeterministicNettingBatches(obligations, epoch, observations);
    const second = buildDeterministicNettingBatches([...obligations].reverse(), epoch, [...observations].reverse());
    expect(second).toEqual(first);
    expect(first[0]?.obligationIds).toEqual(['o1', 'o2', 'o3']);
    expect(first[0]?.grossAmount.amountMinor).toBe(583n);
  });

  it('does not net obligations held by explicit eligibility evidence', () => {
    const payable = obligation('payable', 600n);
    const held = obligation('held', 400n);
    const batches = buildDeterministicNettingBatches([payable, held], epoch, [
      eligibility(payable.id),
      eligibility(held.id, SettlementEligibilityCode.AwaitingPayeeRoute),
    ]);
    expect(batches[0]?.obligationIds).toEqual(['payable']);
  });

  it('does not treat missing eligibility evidence as permission to pay', () => {
    expect(buildDeterministicNettingBatches([obligation('unknown', 600n)], epoch, [])).toEqual([]);
  });

  it('does not erase a below-threshold obligation by creating a zero batch', () => {
    const debt = obligation('dust', 2n);
    expect(buildDeterministicNettingBatches([debt], epoch, [eligibility(debt.id)])).toEqual([]);
  });

  it('rejects duplicate obligation ids before value can be double-netted', () => {
    const first = obligation('duplicate', 300n);
    expect(() => buildDeterministicNettingBatches([
      first,
      obligation('duplicate', 300n),
    ], epoch, [eligibility(first.id)])).toThrow(/duplicate netting obligation id/);
  });

  it('rejects eligibility evidence for an obligation outside the authoritative input', () => {
    const debt = obligation('o1', 600n);
    expect(() => buildDeterministicNettingBatches([debt], epoch, [
      eligibility('o2'),
    ])).toThrow(/unknown obligation/);
  });

  it('rejects a serialized batch whose economic fields differ from reconstruction', () => {
    const obligations = [obligation('o1', 600n)];
    const observations = allEligible(obligations);
    const batch = buildDeterministicNettingBatches(obligations, epoch, observations)[0]!;
    expect(() => assertDeterministicNettingBatch({
      ...batch,
      grossAmount: money(601n),
    }, obligations, epoch, observations)).toThrow(/deterministic authoritative reconstruction/);
  });

  it('changes batch identity when immutable obligation provenance changes', () => {
    const firstDebt = obligation('o1', 600n, 'usage:epoch:a');
    const secondDebt = obligation('o1', 600n, 'usage:epoch:b');
    const observation = eligibility('o1');
    const first = buildDeterministicNettingBatches([firstDebt], epoch, [observation])[0]!;
    const second = buildDeterministicNettingBatches([secondDebt], epoch, [observation])[0]!;
    expect(second.obligationSetRoot).not.toBe(first.obligationSetRoot);
    expect(second.batchId).not.toBe(first.batchId);
  });

  it('changes batch identity when eligibility evidence provenance changes', () => {
    const debt = obligation('o1', 600n);
    const first = buildDeterministicNettingBatches([
      debt,
    ], epoch, [eligibility(debt.id, SettlementEligibilityCode.Eligible, 'eligibility-authority:v1')])[0]!;
    const second = buildDeterministicNettingBatches([
      debt,
    ], epoch, [eligibility(debt.id, SettlementEligibilityCode.Eligible, 'eligibility-authority:v2')])[0]!;
    expect(second.obligationSetRoot).toBe(first.obligationSetRoot);
    expect(second.eligibilityEvidenceRoot).not.toBe(first.eligibilityEvidenceRoot);
    expect(second.batchId).not.toBe(first.batchId);
  });
});
