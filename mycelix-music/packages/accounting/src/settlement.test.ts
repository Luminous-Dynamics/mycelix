import { describe, expect, it } from 'vitest';
import { createRoyaltyObligationAuthority } from './obligation-authority.js';
import {
  SettlementEligibilityCode,
  assessCarryForward,
  buildSettlementEligibilityEvidenceCommitment,
  partitionObligationsAtCutoff,
  reconstructSettlementEligibility,
  type ObservableSettlementEligibilityCode,
  type RoyaltyObligation,
  type SettlementEligibilityObservation,
} from './settlement.js';
import { money } from './money.js';

const epoch = {
  id: 'epoch:2026-09',
  cutoff: '2026-09-30T23:59:59Z',
  eligibilityAsOf: '2026-10-01T00:00:00Z',
  minimumPayout: money(500n),
} as const;

function obligation(id: string, amountMinor: bigint, observedAt = '2026-09-20T00:00:00Z'): RoyaltyObligation {
  return createRoyaltyObligationAuthority({
    id,
    beneficiaryId: 'creator:alice',
    amount: money(amountMinor),
    observedAt,
    provenance: {
      usageEvidenceRef: `usage:${id}`,
      rightsResolutionRef: `rights:${id}`,
      economicTermsRef: 'terms:v1',
    },
  });
}

function eligibility(
  obligationId: string,
  code: ObservableSettlementEligibilityCode = SettlementEligibilityCode.Eligible,
  observedAt = '2026-09-30T12:00:00Z',
  id = `eligibility:${obligationId}:${code}`,
): SettlementEligibilityObservation {
  return {
    id,
    obligationId,
    code,
    sourceRef: 'eligibility-authority:v1',
    observedAt,
  };
}

describe('settlement eligibility and carry-forward', () => {
  it('preserves micro-royalties until the threshold is crossed', () => {
    const obligations = [obligation('o1', 2n), obligation('o2', 270n), obligation('o3', 311n)];
    const assessment = assessCarryForward(
      obligations,
      epoch,
      obligations.map(item => eligibility(item.id)),
    );
    expect(assessment.carriedForward.amountMinor).toBe(583n);
    expect(assessment.eligibility.code).toBe(SettlementEligibilityCode.Eligible);
    expect(assessment.payableNow.map(item => item.id)).toEqual(['o1', 'o2', 'o3']);
  });

  it('reports below-threshold without erasing the obligation', () => {
    const debt = obligation('o1', 2n);
    const assessment = assessCarryForward([debt], epoch, [eligibility(debt.id)]);
    expect(assessment.eligibility.code).toBe(SettlementEligibilityCode.BelowThreshold);
    expect(assessment.payableNow[0]?.id).toBe('o1');
    expect(assessment.carriedForward.amountMinor).toBe(2n);
  });

  it('distinguishes not payable now from not owed using evidence, not mutable debt flags', () => {
    const debt = obligation('o1', 327n);
    const assessment = assessCarryForward([
      debt,
    ], epoch, [eligibility(debt.id, SettlementEligibilityCode.AwaitingPayeeRoute)]);
    expect(assessment.held[0]?.obligation.amount.amountMinor).toBe(327n);
    expect(assessment.held[0]?.eligibility.code).toBe(SettlementEligibilityCode.AwaitingPayeeRoute);
    expect(debt.authorityRoot).toBe(obligation('o1', 327n).authorityRoot);
  });

  it('fails closed when no eligibility evidence is observable', () => {
    const debt = obligation('o1', 600n);
    const reconstructed = reconstructSettlementEligibility(debt, [], epoch.eligibilityAsOf);
    expect(reconstructed.eligibility.code).toBe(SettlementEligibilityCode.AwaitingEligibilityEvidence);
    const assessment = assessCarryForward([debt], epoch, []);
    expect(assessment.payableNow).toEqual([]);
    expect(assessment.held[0]?.obligation.id).toBe('o1');
  });

  it('ignores future eligibility evidence rather than leaking it backward', () => {
    const debt = obligation('o1', 600n);
    const reconstructed = reconstructSettlementEligibility(debt, [
      eligibility(debt.id, SettlementEligibilityCode.Eligible, '2026-10-02T00:00:00Z'),
    ], epoch.eligibilityAsOf);
    expect(reconstructed.eligibility.code).toBe(SettlementEligibilityCode.AwaitingEligibilityEvidence);
  });

  it('uses the latest observable eligibility event', () => {
    const debt = obligation('o1', 600n);
    const reconstructed = reconstructSettlementEligibility(debt, [
      eligibility(debt.id, SettlementEligibilityCode.AwaitingPayeeRoute, '2026-09-29T00:00:00Z', 'eligibility:1'),
      eligibility(debt.id, SettlementEligibilityCode.Eligible, '2026-09-30T00:00:00Z', 'eligibility:2'),
    ], epoch.eligibilityAsOf);
    expect(reconstructed.eligibility.code).toBe(SettlementEligibilityCode.Eligible);
    expect(reconstructed.observation?.id).toBe('eligibility:2');
  });

  it('rejects competing latest observations at the same timestamp', () => {
    const debt = obligation('o1', 600n);
    expect(() => reconstructSettlementEligibility(debt, [
      eligibility(debt.id, SettlementEligibilityCode.Eligible, '2026-09-30T00:00:00Z', 'eligibility:1'),
      eligibility(debt.id, SettlementEligibilityCode.LegalHold, '2026-09-30T00:00:00Z', 'eligibility:2'),
    ], epoch.eligibilityAsOf)).toThrow(/ambiguous eligibility evidence/);
  });

  it('rejects an observation id replayed with different content', () => {
    const debt = obligation('o1', 600n);
    expect(() => reconstructSettlementEligibility(debt, [
      eligibility(debt.id, SettlementEligibilityCode.Eligible, '2026-09-30T00:00:00Z', 'eligibility:same'),
      eligibility(debt.id, SettlementEligibilityCode.LegalHold, '2026-09-30T01:00:00Z', 'eligibility:same'),
    ], epoch.eligibilityAsOf)).toThrow(/id reused with different evidence/);
  });

  it('binds source identity and eligibility as-of into the evidence root', () => {
    const debt = obligation('o1', 600n);
    const base = eligibility(debt.id);
    const first = buildSettlementEligibilityEvidenceCommitment([debt], [base], epoch.eligibilityAsOf);
    const second = buildSettlementEligibilityEvidenceCommitment(
      [debt],
      [{ ...base, sourceRef: 'eligibility-authority:v2' }],
      epoch.eligibilityAsOf,
    );
    expect(first.root).not.toBe(second.root);
  });

  it('moves late obligations to the next epoch deterministically', () => {
    const partition = partitionObligationsAtCutoff([
      obligation('early', 1n),
      obligation('late', 1n, '2026-10-01T00:00:00Z'),
    ], epoch);
    expect(partition.inEpoch.map(item => item.id)).toEqual(['early']);
    expect(partition.nextEpoch.map(item => item.id)).toEqual(['late']);
  });

  it('rejects principal tampering without a matching authority root', () => {
    const valid = obligation('o1', 500n);
    expect(() => reconstructSettlementEligibility({
      ...valid,
      amount: money(501n),
    }, [eligibility(valid.id)], epoch.eligibilityAsOf)).toThrow(/authorityRoot does not match immutable provenance/);
  });
});
