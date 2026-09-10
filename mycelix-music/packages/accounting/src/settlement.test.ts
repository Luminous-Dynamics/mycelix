import { describe, expect, it } from 'vitest';
import { createRoyaltyObligationAuthority } from './obligation-authority.js';
import { money } from './money.js';
import {
  SettlementEligibilityCode,
  assessCarryForward,
  buildSettlementEligibilityEvidenceCommitment,
  partitionObligationsAtCutoff,
  reconstructSettlementEligibility,
  type RoyaltyObligation,
  type SettlementEligibilityObservation,
  type SettlementEpoch,
} from './settlement.js';

const epoch: SettlementEpoch = {
  id: 'epoch:2026-09',
  cutoff: '2026-09-30T23:59:59Z',
  eligibilityAsOf: '2026-09-30T23:59:59Z',
  minimumPayout: money(500n),
};

function obligation(id: string, amountMinor: bigint, observedAt = '2026-09-20T00:00:00Z'): RoyaltyObligation {
  return createRoyaltyObligationAuthority({ id, beneficiaryId: 'creator:alice', amount: money(amountMinor), observedAt, provenance: {
    usageEvidenceRef: `usage:${id}`, rightsResolutionRef: `rights:${id}`, economicTermsRef: 'terms:v1',
  } });
}

function observation(id: string, obligationId: string, code: SettlementEligibilityObservation['code'], observedAt = '2026-09-25T00:00:00Z', sourceRef = 'eligibility-authority:v1'): SettlementEligibilityObservation {
  return { id, obligationId, code, sourceRef, observedAt };
}

describe('settlement eligibility evidence and carry-forward', () => {
  it('preserves micro-royalties until the threshold is crossed', () => {
    const obligations = [obligation('o1', 2n), obligation('o2', 270n), obligation('o3', 311n)];
    const observations = obligations.map(item => observation(`elig:${item.id}`, item.id, SettlementEligibilityCode.Eligible));
    const assessment = assessCarryForward(obligations, epoch, observations);
    expect(assessment.carriedForward.amountMinor).toBe(583n);
    expect(assessment.eligibility.code).toBe(SettlementEligibilityCode.Eligible);
    expect(assessment.payableNow.map(item => item.id)).toEqual(['o1', 'o2', 'o3']);
  });

  it('reports below-threshold as a compiler result without erasing the obligation', () => {
    const debt = obligation('o1', 2n);
    const assessment = assessCarryForward([debt], epoch, [observation('elig:o1', 'o1', SettlementEligibilityCode.Eligible)]);
    expect(assessment.eligibility.code).toBe(SettlementEligibilityCode.BelowThreshold);
    expect(assessment.payableNow[0]?.id).toBe('o1');
    expect(assessment.carriedForward.amountMinor).toBe(2n);
  });

  it('treats missing eligibility evidence as held debt, never as implicit permission', () => {
    const debt = obligation('o1', 600n);
    const assessment = assessCarryForward([debt], epoch, []);
    expect(assessment.eligibility.code).toBe(SettlementEligibilityCode.AwaitingEligibilityEvidence);
    expect(assessment.held[0]?.obligation.id).toBe('o1');
    expect(assessment.payableNow).toEqual([]);
  });

  it('distinguishes an unavailable payout route from immutable debt principal', () => {
    const debt = obligation('o1', 600n);
    const held = reconstructSettlementEligibility(debt, [observation('elig:o1', 'o1', SettlementEligibilityCode.AwaitingPayeeRoute)], epoch.eligibilityAsOf);
    expect(held.eligibility.code).toBe(SettlementEligibilityCode.AwaitingPayeeRoute);
    expect(debt.authorityRoot).toBe(obligation('o1', 600n).authorityRoot);
  });

  it('uses latest observable eligibility evidence at or before eligibilityAsOf', () => {
    const debt = obligation('o1', 600n);
    const reconstructed = reconstructSettlementEligibility(debt, [
      observation('elig:blocked', 'o1', SettlementEligibilityCode.AwaitingPayeeRoute, '2026-09-20T00:00:00Z'),
      observation('elig:ready', 'o1', SettlementEligibilityCode.Eligible, '2026-09-29T00:00:00Z'),
    ], epoch.eligibilityAsOf);
    expect(reconstructed.eligibility.code).toBe(SettlementEligibilityCode.Eligible);
    expect(reconstructed.observation?.id).toBe('elig:ready');
  });

  it('does not allow future eligibility evidence to leak backward', () => {
    const debt = obligation('o1', 600n);
    const reconstructed = reconstructSettlementEligibility(debt, [
      observation('elig:blocked', 'o1', SettlementEligibilityCode.AwaitingPayeeRoute, '2026-09-20T00:00:00Z'),
      observation('elig:future', 'o1', SettlementEligibilityCode.Eligible, '2026-10-01T00:00:00Z'),
    ], epoch.eligibilityAsOf);
    expect(reconstructed.eligibility.code).toBe(SettlementEligibilityCode.AwaitingPayeeRoute);
  });

  it('rejects ambiguous competing observations at the latest timestamp', () => {
    const debt = obligation('o1', 600n);
    expect(() => reconstructSettlementEligibility(debt, [
      observation('elig:a', 'o1', SettlementEligibilityCode.Eligible, '2026-09-29T00:00:00Z', 'source:a'),
      observation('elig:b', 'o1', SettlementEligibilityCode.LegalHold, '2026-09-29T00:00:00Z', 'source:b'),
    ], epoch.eligibilityAsOf)).toThrow(/ambiguous eligibility evidence/);
  });

  it('rejects compiler-only eligibility codes as external observations', () => {
    const debt = obligation('o1', 600n);
    const forged = { id: 'elig:forged', obligationId: 'o1', code: SettlementEligibilityCode.BelowThreshold, sourceRef: 'source:forged', observedAt: '2026-09-29T00:00:00Z' } as unknown as SettlementEligibilityObservation;
    expect(() => reconstructSettlementEligibility(debt, [forged], epoch.eligibilityAsOf)).toThrow(/compiler-only code/);
  });

  it('commits selected eligibility source and as-of boundary', () => {
    const debt = obligation('o1', 600n);
    const first = buildSettlementEligibilityEvidenceCommitment([debt], [observation('elig:o1', 'o1', SettlementEligibilityCode.Eligible, '2026-09-29T00:00:00Z', 'source:v1')], epoch.eligibilityAsOf);
    const second = buildSettlementEligibilityEvidenceCommitment([debt], [observation('elig:o1', 'o1', SettlementEligibilityCode.Eligible, '2026-09-29T00:00:00Z', 'source:v2')], epoch.eligibilityAsOf);
    expect(second.root).not.toBe(first.root);
  });

  it('moves late obligations to the next epoch deterministically', () => {
    const partition = partitionObligationsAtCutoff([obligation('early', 1n), obligation('late', 1n, '2026-10-01T00:00:00Z')], epoch);
    expect(partition.inEpoch.map(item => item.id)).toEqual(['early']);
    expect(partition.nextEpoch.map(item => item.id)).toEqual(['late']);
  });

  it('rejects principal tampering without a matching authority root', () => {
    const valid = obligation('o1', 500n);
    expect(() => reconstructSettlementEligibility({ ...valid, amount: money(501n) }, [], epoch.eligibilityAsOf)).toThrow(/authorityRoot does not match immutable provenance/);
  });
});
