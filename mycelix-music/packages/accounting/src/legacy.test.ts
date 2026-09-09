import { describe, expect, it } from 'vitest';
import { observeLegacyRoyaltyPayment } from './legacy.js';

const row = {
  id: 'legacy:payment:1',
  songId: 'song:7',
  recipientAddress: '0x1234',
  amount: '500',
  currency: 'usd',
  txHash: '0xreceipt',
  period: '2026-09',
  playCount: 42,
  paidAt: '2026-10-02T00:00:00Z',
};

describe('legacy royalty payment quarantine', () => {
  it('imports historical rows only as receipt projection evidence', () => {
    const observation = observeLegacyRoyaltyPayment(row);
    expect(observation.authority).toBe('receipt_projection_only');
    expect(observation.migrationWarning).toBe('does_not_create_or_extinguish_obligation');
    expect(observation.amount.amountMinor).toBe(500n);
    expect(observation.amount.currency).toBe('USD');
  });

  it('produces a stable observation commitment', () => {
    expect(observeLegacyRoyaltyPayment(row).observationRoot)
      .toBe(observeLegacyRoyaltyPayment({ ...row }).observationRoot);
  });

  it('refuses ambiguous decimal or signed legacy values', () => {
    expect(() => observeLegacyRoyaltyPayment({ ...row, amount: '1.25' })).toThrow(/integer string/);
    expect(() => observeLegacyRoyaltyPayment({ ...row, amount: '-1' })).toThrow(/integer string/);
  });
});
