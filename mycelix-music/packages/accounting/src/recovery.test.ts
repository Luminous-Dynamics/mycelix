import { describe, expect, it } from 'vitest';
import { buildDeterministicNettingBatches } from './netting.js';
import { createRoyaltyObligationAuthority } from './obligation-authority.js';
import { money } from './money.js';
import { SettlementAttemptState, mayStartSettlementAttempt, reconstructSettlementRecovery, type SettlementAttemptObservation } from './recovery.js';
import { SettlementEligibilityCode, type RoyaltyObligation, type SettlementEligibilityObservation, type SettlementEpoch } from './settlement.js';

const obligation: RoyaltyObligation = createRoyaltyObligationAuthority({ id: 'obl:1', beneficiaryId: 'did:example:artist', amount: money(500n, 'USD'), observedAt: '2026-09-01T00:00:00Z', provenance: { usageEvidenceRef: 'usage:obl:1', rightsResolutionRef: 'rights:obl:1', economicTermsRef: 'terms:v1' } });
const epoch: SettlementEpoch = { id: 'epoch:2026-09', cutoff: '2026-09-30T23:59:59Z', eligibilityAsOf: '2026-09-30T23:59:59Z', minimumPayout: money(100n, 'USD') };
const eligibility: SettlementEligibilityObservation[] = [{ id: 'elig:obl:1', obligationId: 'obl:1', code: SettlementEligibilityCode.Eligible, sourceRef: 'eligibility-authority:v1', observedAt: '2026-09-29T00:00:00Z' }];
const batch = buildDeterministicNettingBatches([obligation], epoch, eligibility)[0]!;

function obs(state: SettlementAttemptState, observedAt: string, extra: Partial<SettlementAttemptObservation> = {}): SettlementAttemptObservation {
  return { attemptId: 'attempt:1', batchId: batch.batchId, obligationSetRoot: batch.obligationSetRoot, eligibilityAsOf: batch.eligibilityAsOf, eligibilityEvidenceRoot: batch.eligibilityEvidenceRoot, state, observedAt, ...extra };
}

function finalized(observedAt: string, extra: Partial<SettlementAttemptObservation> = {}): SettlementAttemptObservation {
  return obs(SettlementAttemptState.Finalized, observedAt, {
    railReceiptRef: 'rail:receipt:7',
    settledAmount: money(500n, 'USD'),
    ...extra,
  });
}

describe('settlement recovery', () => {
  it('reconstructs a never-attempted batch without queue state while retaining eligibility identity', () => {
    const state = reconstructSettlementRecovery(batch, []);
    expect(state.status).toBe('never_attempted');
    expect(mayStartSettlementAttempt(state)).toBe(true);
    expect(state.obligationSetSettled).toBe(false);
    expect(state.eligibilityAsOf).toBe(batch.eligibilityAsOf);
    expect(state.eligibilityEvidenceRoot).toBe(batch.eligibilityEvidenceRoot);
  });

  it('rejects execution evidence observed before the eligibility snapshot exists', () => {
    expect(() => reconstructSettlementRecovery(batch, [
      obs(SettlementAttemptState.Authorized, '2026-09-30T23:59:58Z'),
    ])).toThrow(/cannot precede the eligibility snapshot/);
  });

  it('is deterministic under replayed observations and derives whole-batch finality from receipt amount evidence', () => {
    const recovered = reconstructSettlementRecovery(batch, [
      obs(SettlementAttemptState.Authorized, '2026-10-01T00:00:00Z'),
      obs(SettlementAttemptState.Submitted, '2026-10-01T00:01:00Z'),
      obs(SettlementAttemptState.Submitted, '2026-10-01T00:01:00Z'),
      finalized('2026-10-01T00:03:00Z'),
    ]);
    expect(recovered.status).toBe('finalized');
    expect(recovered.obligationSetSettled).toBe(true);
    expect(recovered.finalReceiptRef).toBe('rail:receipt:7');
    expect(recovered.finalSettledAmount).toEqual(money(500n, 'USD'));
  });

  it('classifies partial rail finality without erasing residual debt or enabling retry', () => {
    const recovered = reconstructSettlementRecovery(batch, [
      finalized('2026-10-01T00:03:00Z', { settledAmount: money(475n, 'USD') }),
    ]);
    expect(recovered.status).toBe('partial_finality');
    expect(recovered.obligationSetSettled).toBe(false);
    expect(recovered.finalReceiptRef).toBe('rail:receipt:7');
    expect(recovered.finalSettledAmount).toEqual(money(475n, 'USD'));
    expect(recovered.reason).toMatch(/residual allocation or reconciliation/);
    expect(mayStartSettlementAttempt(recovered)).toBe(false);
  });

  it('allows retry only after an explicitly superseded failure', () => {
    const observations: SettlementAttemptObservation[] = [
      obs(SettlementAttemptState.Authorized, '2026-10-01T00:00:00Z'),
      obs(SettlementAttemptState.Failed, '2026-10-01T00:01:00Z'),
      obs(SettlementAttemptState.Authorized, '2026-10-01T01:00:00Z', { attemptId: 'attempt:2', supersedesAttemptId: 'attempt:1' }),
    ];
    expect(reconstructSettlementRecovery(batch, observations).status).toBe('in_flight');
  });

  it('rejects an initial attempt that claims a predecessor', () => {
    expect(() => reconstructSettlementRecovery(batch, [
      obs(SettlementAttemptState.Authorized, '2026-10-01T00:00:00Z', { supersedesAttemptId: 'attempt:missing' }),
    ])).toThrow(/initial settlement attempt cannot supersede/);
  });

  it('rejects an overlapping retry', () => {
    expect(() => reconstructSettlementRecovery(batch, [
      obs(SettlementAttemptState.Authorized, '2026-10-01T00:00:00Z'),
      obs(SettlementAttemptState.Failed, '2026-10-01T00:10:00Z'),
      obs(SettlementAttemptState.Authorized, '2026-10-01T00:05:00Z', { attemptId: 'attempt:2', supersedesAttemptId: 'attempt:1' }),
    ])).toThrow(/after the prior terminal observation/);
  });

  it('blocks retry after reversal while retaining finalized receipt and amount evidence', () => {
    const recovered = reconstructSettlementRecovery(batch, [
      obs(SettlementAttemptState.Authorized, '2026-10-01T00:00:00Z'),
      obs(SettlementAttemptState.Confirmed, '2026-10-01T00:02:00Z'),
      finalized('2026-10-01T00:03:00Z'),
      obs(SettlementAttemptState.Reversed, '2026-10-01T00:04:00Z'),
    ]);
    expect(recovered.status).toBe('blocked_ambiguous');
    expect(recovered.obligationSetSettled).toBe(false);
    expect(recovered.finalReceiptRef).toBe('rail:receipt:7');
    expect(recovered.finalSettledAmount).toEqual(money(500n, 'USD'));
    expect(mayStartSettlementAttempt(recovered)).toBe(false);
  });

  it('rejects finalized evidence without a durable receipt', () => {
    expect(() => reconstructSettlementRecovery(batch, [
      obs(SettlementAttemptState.Finalized, '2026-10-01T00:03:00Z', { settledAmount: money(500n, 'USD') }),
    ])).toThrow(/requires a rail receipt/);
  });

  it('rejects finalized evidence without an exact settled amount', () => {
    expect(() => reconstructSettlementRecovery(batch, [
      obs(SettlementAttemptState.Finalized, '2026-10-01T00:03:00Z', { railReceiptRef: 'rail:receipt:7' }),
    ])).toThrow(/requires an exact settled amount/);
  });

  it('rejects zero-value finalized evidence and excessive rail finality', () => {
    expect(() => reconstructSettlementRecovery(batch, [
      finalized('2026-10-01T00:03:00Z', { settledAmount: money(0n, 'USD') }),
    ])).toThrow(/must be positive/);
    expect(() => reconstructSettlementRecovery(batch, [
      finalized('2026-10-01T00:03:00Z', { settledAmount: money(501n, 'USD') }),
    ])).toThrow(/cannot exceed batch gross amount/);
  });

  it('rejects settled amount on non-final evidence', () => {
    expect(() => reconstructSettlementRecovery(batch, [
      obs(SettlementAttemptState.Confirmed, '2026-10-01T00:02:00Z', { settledAmount: money(500n, 'USD') }),
    ])).toThrow(/only on finalized/);
  });

  it('rejects a finalized receipt identity that changes on replay', () => {
    expect(() => reconstructSettlementRecovery(batch, [
      finalized('2026-10-01T00:03:00Z'),
      finalized('2026-10-01T00:04:00Z', { railReceiptRef: 'rail:receipt:8' }),
    ])).toThrow(/receipt reference changed/);
  });

  it('rejects a finalized amount that changes on replay', () => {
    expect(() => reconstructSettlementRecovery(batch, [
      finalized('2026-10-01T00:03:00Z'),
      finalized('2026-10-01T00:04:00Z', { settledAmount: money(499n, 'USD') }),
    ])).toThrow(/settled amount changed/);
  });

  it('rejects evidence bound to a different obligation set', () => {
    expect(() => reconstructSettlementRecovery(batch, [
      { ...obs(SettlementAttemptState.Authorized, '2026-10-01T00:00:00Z'), obligationSetRoot: 'wrong-root' },
    ])).toThrow(/obligationSetRoot mismatch/);
  });

  it('rejects evidence bound to a different eligibility snapshot', () => {
    expect(() => reconstructSettlementRecovery(batch, [
      { ...obs(SettlementAttemptState.Authorized, '2026-10-01T00:00:00Z'), eligibilityEvidenceRoot: 'f'.repeat(64) },
    ])).toThrow(/eligibilityEvidenceRoot mismatch/);
  });

  it('rejects a second attempt while the first is still in flight', () => {
    expect(() => reconstructSettlementRecovery(batch, [
      obs(SettlementAttemptState.Authorized, '2026-10-01T00:00:00Z'),
      obs(SettlementAttemptState.Authorized, '2026-10-01T00:02:00Z', { attemptId: 'attempt:2', supersedesAttemptId: 'attempt:1' }),
    ])).toThrow(/prior attempt is retryable/);
  });
});
