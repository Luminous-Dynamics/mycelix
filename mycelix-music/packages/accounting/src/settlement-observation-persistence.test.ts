import { describe, expect, it } from 'vitest';
import { money } from './money.js';
import { projectSettlementObservationRecord } from './persistence-projection.js';
import { SettlementAttemptState, type SettlementAttemptObservation } from './recovery.js';

const finalized = (amountMinor: bigint): SettlementAttemptObservation => ({
  attemptId: 'attempt:positive-finality',
  batchId: 'batch:positive-finality',
  obligationSetRoot: '1'.repeat(64),
  eligibilityAsOf: '2026-09-10T00:00:00Z',
  eligibilityEvidenceRoot: '2'.repeat(64),
  state: SettlementAttemptState.Finalized,
  observedAt: '2026-09-10T00:01:00Z',
  railReceiptRef: 'rail:receipt:positive-finality',
  settledAmount: money(amountMinor, 'USD'),
});

describe('settlement observation persistence boundary', () => {
  it('rejects zero-value finalized evidence before it can be serialized', () => {
    expect(() => projectSettlementObservationRecord('observation:zero', finalized(0n)))
      .toThrow(/settled amount must be positive/);
  });

  it('continues to serialize positive finalized evidence canonically', () => {
    const persisted = projectSettlementObservationRecord('observation:positive', finalized(1n));
    expect(persisted.settledAmountMinor).toBe('1');
    expect(persisted.settledCurrency).toBe('USD');
  });
});
