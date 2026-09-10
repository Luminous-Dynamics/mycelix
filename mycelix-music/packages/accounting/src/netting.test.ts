import { describe, expect, it } from 'vitest';
import { assertDeterministicNettingBatch, buildDeterministicNettingBatches } from './netting.js';
import { createRoyaltyObligationAuthority } from './obligation-authority.js';
import { money } from './money.js';
import { SettlementEligibilityCode, type RoyaltyObligation, type SettlementEligibilityObservation, type SettlementEpoch } from './settlement.js';

const epoch: SettlementEpoch = { id: 'epoch:2026-09', cutoff: '2026-09-30T23:59:59Z', eligibilityAsOf: '2026-09-30T23:59:59Z', minimumPayout: money(500n) };

function obligation(id: string, amountMinor: bigint, usageRef = `usage:${id}`): RoyaltyObligation {
  return createRoyaltyObligationAuthority({ id, beneficiaryId: 'creator:alice', amount: money(amountMinor), observedAt: '2026-09-20T00:00:00Z', provenance: { usageEvidenceRef: usageRef, rightsResolutionRef: `rights:${id}`, economicTermsRef: 'terms:v1' } });
}
function eligibility(obligationId: string, code: SettlementEligibilityObservation['code'] = SettlementEligibilityCode.Eligible, sourceRef = 'eligibility-authority:v1'): SettlementEligibilityObservation {
  return { id: `elig:${obligationId}:${sourceRef}`, obligationId, code, sourceRef, observedAt: '2026-09-25T00:00:00Z' };
}

describe('deterministic evidence-bound netting', () => {
  it('produces identical batch identity regardless of obligation or evidence input order', () => {
    const obligations = [obligation('o3', 311n), obligation('o1', 2n), obligation('o2', 270n)];
    const observations = obligations.map(item => eligibility(item.id));
    const first = buildDeterministicNettingBatches(obligations, epoch, observations);
    const second = buildDeterministicNettingBatches([...obligations].reverse(), epoch, [...observations].reverse());
    expect(second).toEqual(first);
    expect(first[0]?.obligationIds).toEqual(['o1', 'o2', 'o3']);
    expect(first[0]?.grossAmount.amountMinor).toBe(583n);
  });
  it('does not net obligations whose latest eligibility evidence holds them', () => {
    const payable = obligation('payable', 600n);
    const held = obligation('held', 400n);
    const batches = buildDeterministicNettingBatches([payable, held], epoch, [eligibility('payable'), eligibility('held', SettlementEligibilityCode.AwaitingPayeeRoute)]);
    expect(batches[0]?.obligationIds).toEqual(['payable']);
  });
  it('fails closed when eligibility evidence is missing', () => {
    expect(buildDeterministicNettingBatches([obligation('unqualified', 600n)], epoch, [])).toEqual([]);
  });
  it('does not erase a below-threshold obligation by creating a zero batch', () => {
    const dust = obligation('dust', 2n);
    expect(buildDeterministicNettingBatches([dust], epoch, [eligibility('dust')])).toEqual([]);
  });
  it('rejects duplicate obligation ids before value can be double-netted', () => {
    const duplicated = [obligation('duplicate', 300n), obligation('duplicate', 300n)];
    expect(() => buildDeterministicNettingBatches(duplicated, epoch, [eligibility('duplicate')])).toThrow(/duplicate netting obligation id/);
  });
  it('rejects a serialized batch whose economic fields differ from reconstruction', () => {
    const obligations = [obligation('o1', 600n)];
    const observations = [eligibility('o1')];
    const batch = buildDeterministicNettingBatches(obligations, epoch, observations)[0]!;
    expect(() => assertDeterministicNettingBatch({ ...batch, grossAmount: money(601n) }, obligations, epoch, observations)).toThrow(/deterministic authoritative reconstruction/);
  });
  it('changes obligation-set and batch identity when immutable debt provenance changes', () => {
    const first = buildDeterministicNettingBatches([obligation('o1', 600n, 'usage:epoch:a')], epoch, [eligibility('o1')])[0]!;
    const second = buildDeterministicNettingBatches([obligation('o1', 600n, 'usage:epoch:b')], epoch, [eligibility('o1')])[0]!;
    expect(second.obligationSetRoot).not.toBe(first.obligationSetRoot);
    expect(second.batchId).not.toBe(first.batchId);
  });
  it('changes settlement-plan identity when eligibility provenance changes without rewriting debt', () => {
    const debt = obligation('o1', 600n);
    const first = buildDeterministicNettingBatches([debt], epoch, [eligibility('o1', SettlementEligibilityCode.Eligible, 'route-registry:v1')])[0]!;
    const second = buildDeterministicNettingBatches([debt], epoch, [eligibility('o1', SettlementEligibilityCode.Eligible, 'route-registry:v2')])[0]!;
    expect(second.obligationSetRoot).toBe(first.obligationSetRoot);
    expect(second.eligibilityEvidenceRoot).not.toBe(first.eligibilityEvidenceRoot);
    expect(second.batchId).not.toBe(first.batchId);
  });
  it('rejects eligibility observations outside the authoritative input set', () => {
    expect(() => buildDeterministicNettingBatches([obligation('o1', 600n)], epoch, [eligibility('unknown')])).toThrow(/references unknown obligation/);
  });
});
