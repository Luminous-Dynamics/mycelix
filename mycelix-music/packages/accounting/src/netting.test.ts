import { describe, expect, it } from 'vitest';
import {
  assertDeterministicNettingBatch,
  buildDeterministicNettingBatches,
} from './netting.js';
import { money } from './money.js';
import { createRoyaltyObligationAuthority } from './obligation-authority.js';
import { type RoyaltyObligation } from './settlement.js';

const epoch = {
  id: 'epoch:2026-09',
  cutoff: '2026-09-30T23:59:59Z',
  minimumPayout: money(500n),
} as const;

function obligation(id: string, amountMinor: bigint, routeAvailable = true, usageRef = `usage:${id}`): RoyaltyObligation {
  return {
    ...createRoyaltyObligationAuthority({
      id,
      beneficiaryId: 'creator:alice',
      amount: money(amountMinor),
      observedAt: '2026-09-20T00:00:00Z',
      provenance: {
        usageEvidenceRef: usageRef,
        rightsResolutionRef: `rights:${id}`,
        economicTermsRef: 'terms:v1',
      },
    }),
    routeAvailable,
  };
}

describe('deterministic netting', () => {
  it('produces identical batch identity regardless of input order', () => {
    const obligations = [obligation('o3', 311n), obligation('o1', 2n), obligation('o2', 270n)];
    const first = buildDeterministicNettingBatches(obligations, epoch);
    const second = buildDeterministicNettingBatches([...obligations].reverse(), epoch);
    expect(second).toEqual(first);
    expect(first[0]?.obligationIds).toEqual(['o1', 'o2', 'o3']);
    expect(first[0]?.grossAmount.amountMinor).toBe(583n);
  });

  it('does not net obligations that are not presently payable', () => {
    const batches = buildDeterministicNettingBatches([
      obligation('payable', 600n),
      obligation('held', 400n, false),
    ], epoch);
    expect(batches[0]?.obligationIds).toEqual(['payable']);
  });

  it('does not erase a below-threshold obligation by creating a zero batch', () => {
    expect(buildDeterministicNettingBatches([obligation('dust', 2n)], epoch)).toEqual([]);
  });

  it('rejects duplicate obligation ids before value can be double-netted', () => {
    expect(() => buildDeterministicNettingBatches([
      obligation('duplicate', 300n),
      obligation('duplicate', 300n),
    ], epoch)).toThrow(/duplicate netting obligation id/);
  });

  it('rejects a serialized batch whose economic fields differ from reconstruction', () => {
    const obligations = [obligation('o1', 600n)];
    const batch = buildDeterministicNettingBatches(obligations, epoch)[0]!;
    expect(() => assertDeterministicNettingBatch({
      ...batch,
      grossAmount: money(601n),
    }, obligations, epoch)).toThrow(/deterministic authoritative reconstruction/);
  });

  it('changes batch identity when immutable provenance changes', () => {
    const first = buildDeterministicNettingBatches([obligation('o1', 600n, true, 'usage:epoch:a')], epoch)[0]!;
    const second = buildDeterministicNettingBatches([obligation('o1', 600n, true, 'usage:epoch:b')], epoch)[0]!;
    expect(second.obligationSetRoot).not.toBe(first.obligationSetRoot);
    expect(second.batchId).not.toBe(first.batchId);
  });
});
