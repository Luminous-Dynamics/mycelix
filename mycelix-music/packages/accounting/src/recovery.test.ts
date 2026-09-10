import { describe, expect, it } from 'vitest';
import { buildDeterministicNettingBatches } from './netting.js';
import { money } from './money.js';
import {
  SettlementAttemptState,
  mayStartSettlementAttempt,
  reconstructSettlementRecovery,
  type SettlementAttemptObservation,
} from './recovery.js';
import type { RoyaltyObligation, SettlementEpoch } from './settlement.js';

const obligation: RoyaltyObligation = {
  id: 'obl:1',
  beneficiaryId: 'did:example:artist',
  amount: money(500n, 'USD'),
  observedAt: '2026-09-01T00:00:00Z',
};
const epoch: SettlementEpoch = {
  id: 'epoch:2026-09',
  cutoff: '2026-09-30T23:59:59Z',
  minimumPayout: money(100n, 'USD'),
};
const batch = buildDeterministicNettingBatches([obligation], epoch)[0]!;

function obs(
  state: SettlementAttemptState,
  observedAt: string,
  extra: Partial<SettlementAttemptObservation> = {},
): SettlementAttemptObservation {
  return {
    attemptId: 'attempt:1',
    batchId: batch.batchId,
    obligationSetRoot: batch.obligationSetRoot,
    state,
    observedAt,
    ...extra,
  };
}

describe('settlement recovery', () => {
  it('reconstructs a never-attempted batch without queue state', () => {
    const state = reconstructSettlementRecovery(batch, []);
    expect(state.status).toBe('never_attempted');
    expect(mayStartSettlementAttempt(state)).toBe(true);
    expect(state.obligationSetSettled).toBe(false);
  });

  it('is deterministic under replayed observations', () => {
    const observations = [
      obs(SettlementAttemptState.Authorized, '2026-09-10T00:00:00Z'),
      obs(SettlementAttemptState.Submitted, '2026-09-10T00:01:00Z'),
      obs(SettlementAttemptState.Submitted, '2026-09-10T00:01:00Z'),
      obs(SettlementAttemptState.Finalized, '2026-09-10T00:03:00Z', { railReceiptRef: 'rail:receipt:7' }),
    ];
    const recovered = reconstructSettlementRecovery(batch, observations);
    expect(recovered.status).toBe('finalized');
    expect(recovered.obligationSetSettled).toBe(true);
    expect(recovered.finalReceiptRef).toBe('rail:receipt:7');
    expect(mayStartSettlementAttempt(recovered)).toBe(false);
  });

  it('allows a retry only after an explicitly superseded failure', () => {
    const observations: SettlementAttemptObservation[] = [
      obs(SettlementAttemptState.Authorized, '2026-09-10T00:00:00Z'),
      obs(SettlementAttemptState.Failed, '2026-09-10T00:01:00Z'),
      obs(SettlementAttemptState.Authorized, '2026-09-10T01:00:00Z', {
        attemptId: 'attempt:2',
        supersedesAttemptId: 'attempt:1',
      }),
    ];
    expect(reconstructSettlementRecovery(batch, observations).status).toBe('in_flight');
  });

  it('rejects an initial attempt that claims to supersede nonexistent history', () => {
    expect(() => reconstructSettlementRecovery(batch, [
      obs(SettlementAttemptState.Authorized, '2026-09-10T00:00:00Z', {
        supersedesAttemptId: 'attempt:missing',
      }),
    ])).toThrow(/initial settlement attempt cannot supersede/);
  });

  it('rejects an overlapping retry even when it names the failed attempt', () => {
    expect(() => reconstructSettlementRecovery(batch, [
      obs(SettlementAttemptState.Authorized, '2026-09-10T00:00:00Z'),
      obs(SettlementAttemptState.Failed, '2026-09-10T00:10:00Z'),
      obs(SettlementAttemptState.Authorized, '2026-09-10T00:05:00Z', {
        attemptId: 'attempt:2',
        supersedesAttemptId: 'attempt:1',
      }),
    ])).toThrow(/after the prior terminal observation/);
  });

  it('blocks retry after a reversal while retaining the finalized receipt evidence', () => {
    const observations = [
      obs(SettlementAttemptState.Authorized, '2026-09-10T00:00:00Z'),
      obs(SettlementAttemptState.Submitted, '2026-09-10T00:01:00Z'),
      obs(SettlementAttemptState.Confirmed, '2026-09-10T00:02:00Z'),
      obs(SettlementAttemptState.Finalized, '2026-09-10T00:03:00Z', { railReceiptRef: 'rail:receipt:7' }),
      obs(SettlementAttemptState.Reversed, '2026-09-10T00:04:00Z'),
    ];
    const recovered = reconstructSettlementRecovery(batch, observations);
    expect(recovered.status).toBe('blocked_ambiguous');
    expect(recovered.obligationSetSettled).toBe(false);
    expect(recovered.finalReceiptRef).toBe('rail:receipt:7');
    expect(mayStartSettlementAttempt(recovered)).toBe(false);
  });

  it('rejects finalized evidence without a durable receipt even if later reversed', () => {
    expect(() => reconstructSettlementRecovery(batch, [
      obs(SettlementAttemptState.Confirmed, '2026-09-10T00:02:00Z'),
      obs(SettlementAttemptState.Finalized, '2026-09-10T00:03:00Z'),
      obs(SettlementAttemptState.Reversed, '2026-09-10T00:04:00Z'),
    ])).toThrow(/every finalized settlement observation requires/);
  });

  it('rejects a finalized receipt identity that changes on replay', () => {
    expect(() => reconstructSettlementRecovery(batch, [
      obs(SettlementAttemptState.Finalized, '2026-09-10T00:03:00Z', { railReceiptRef: 'rail:receipt:7' }),
      obs(SettlementAttemptState.Finalized, '2026-09-10T00:04:00Z', { railReceiptRef: 'rail:receipt:8' }),
    ])).toThrow(/receipt reference changed/);
  });

  it('rejects evidence bound to a different obligation set', () => {
    expect(() => reconstructSettlementRecovery(batch, [
      { ...obs(SettlementAttemptState.Authorized, '2026-09-10T00:00:00Z'), obligationSetRoot: 'wrong-root' },
    ])).toThrow(/obligationSetRoot mismatch/);
  });

  it('rejects a second attempt while the first is still in flight', () => {
    expect(() => reconstructSettlementRecovery(batch, [
      obs(SettlementAttemptState.Authorized, '2026-09-10T00:00:00Z'),
      obs(SettlementAttemptState.Authorized, '2026-09-10T00:02:00Z', {
        attemptId: 'attempt:2',
        supersedesAttemptId: 'attempt:1',
      }),
    ])).toThrow(/prior attempt is retryable/);
  });
});
