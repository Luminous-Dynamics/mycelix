import { describe, expect, it } from 'vitest';
import { createRoyaltyDeductionAuthority, assertRoyaltyDeductionAuthority } from './deduction-authority.js';
import { money } from './money.js';

function deduction(authorityRef = 'tax-authority:notice:1') {
  return createRoyaltyDeductionAuthority({
    id: 'deduction:1',
    beneficiaryId: 'creator:alice',
    amount: money(50n, 'USD'),
    basis: 'tax:withholding:v1',
    authorityRef,
    observedAt: '2026-09-10T00:00:00Z',
  });
}

describe('royalty deduction authority', () => {
  it('derives a stable root from canonical authority evidence', () => {
    expect(deduction().deductionRoot).toBe(deduction().deductionRoot);
  });

  it('changes root when authority provenance changes', () => {
    expect(deduction('tax-authority:notice:1').deductionRoot)
      .not.toBe(deduction('tax-authority:notice:2').deductionRoot);
  });

  it('normalizes equivalent textual and temporal representations before hashing', () => {
    const normalized = createRoyaltyDeductionAuthority({
      id: ' deduction:1 ',
      beneficiaryId: ' creator:alice ',
      amount: money(50n, 'usd'),
      basis: ' tax:withholding:v1 ',
      authorityRef: ' tax-authority:notice:1 ',
      observedAt: '2026-09-10T02:00:00+02:00',
    });
    expect(normalized).toEqual(deduction());
  });

  it('fails closed on missing authority and tampered roots', () => {
    expect(() => createRoyaltyDeductionAuthority({
      id: 'deduction:1', beneficiaryId: 'creator:alice', amount: money(50n, 'USD'),
      basis: 'tax:withholding:v1', authorityRef: ' ', observedAt: '2026-09-10T00:00:00Z',
    })).toThrow(/authorityRef must be non-empty/);
    expect(() => assertRoyaltyDeductionAuthority({ ...deduction(), deductionRoot: 'f'.repeat(64) }))
      .toThrow(/deductionRoot does not match/);
  });
});
