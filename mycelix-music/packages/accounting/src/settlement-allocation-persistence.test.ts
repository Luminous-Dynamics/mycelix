import { describe, expect, it } from 'vitest';
import { createRoyaltyDeductionAuthority } from './deduction-authority.js';
import { buildDeterministicNettingBatches } from './netting.js';
import { money } from './money.js';
import { createRoyaltyObligationAuthority } from './obligation-authority.js';
import { projectSettlementAllocationRecord } from './persistence-projection.js';
import { verifyPersistedSettlementAllocation } from './persistence-verification.js';
import { reconstructSettlementRecovery, SettlementAttemptState } from './recovery.js';
import { createSettlementAllocationAuthority } from './settlement-allocation.js';
import { SettlementEligibilityCode, type SettlementEpoch } from './settlement.js';

const obligation = createRoyaltyObligationAuthority({
  id: 'obl:allocation', beneficiaryId: 'artist:1', amount: money(500n, 'USD'), observedAt: '2026-09-10T00:00:00Z',
  provenance: { usageEvidenceRef: 'usage:allocation', rightsResolutionRef: 'rights:allocation', economicTermsRef: 'terms:v1' },
});
const epoch: SettlementEpoch = {
  id: 'epoch:allocation', cutoff: '2026-09-30T23:59:59Z', eligibilityAsOf: '2026-10-01T00:00:00Z', minimumPayout: money(100n, 'USD'),
};
const eligibility = [{
  id: 'elig:allocation', obligationId: obligation.id, code: SettlementEligibilityCode.Eligible,
  sourceRef: 'eligibility:v1', observedAt: '2026-09-30T00:00:00Z',
}] as const;
const batch = buildDeterministicNettingBatches([obligation], epoch, eligibility)[0]!;
const recovery = reconstructSettlementRecovery(batch, [{
  attemptId: 'attempt:allocation', batchId: batch.batchId, obligationSetRoot: batch.obligationSetRoot,
  eligibilityAsOf: batch.eligibilityAsOf, eligibilityEvidenceRoot: batch.eligibilityEvidenceRoot,
  state: SettlementAttemptState.Finalized, observedAt: '2026-10-01T00:01:00Z',
  railReceiptRef: 'rail:receipt:allocation', settledAmount: money(450n, 'USD'),
}]);
const deduction = createRoyaltyDeductionAuthority({
  id: 'deduction:allocation', beneficiaryId: 'artist:1', amount: money(50n, 'USD'),
  basis: 'tax:withholding:v1', authorityRef: 'tax:notice:allocation', observedAt: '2026-10-01T00:01:30Z',
});
const allocation = createSettlementAllocationAuthority({
  allocationId: 'allocation:1', batch, recovery, deductions: [deduction], residualHeld: money(0n, 'USD'), allocatedAt: '2026-10-01T00:02:00Z',
});

describe('settlement allocation persistence replay', () => {
  it('projects and verifies exact allocation authority from Prisma-style timestamps', () => {
    const persisted = projectSettlementAllocationRecord(allocation, batch, recovery, [deduction]);
    const verified = verifyPersistedSettlementAllocation({
      ...persisted,
      eligibilityAsOf: new Date(persisted.eligibilityAsOf),
      allocatedAt: new Date(persisted.allocatedAt),
    }, batch, recovery, [deduction]);
    expect(verified.allocationRoot).toBe(allocation.allocationRoot);
    expect(verified.obligationSetDischarged).toBe(true);
  });

  it('detects derived creator amount changed behind an unchanged allocation root', () => {
    const persisted = projectSettlementAllocationRecord(allocation, batch, recovery, [deduction]);
    expect(() => verifyPersistedSettlementAllocation({ ...persisted, creatorPaidMinor: '449' }, batch, recovery, [deduction]))
      .toThrow(/canonical projection at creatorPaidMinor/);
  });

  it('detects residual/discharge tampering behind an unchanged allocation root', () => {
    const persisted = projectSettlementAllocationRecord(allocation, batch, recovery, [deduction]);
    expect(() => verifyPersistedSettlementAllocation({ ...persisted, residualHeldMinor: '1' }, batch, recovery, [deduction]))
      .toThrow(/must conserve batch gross/);
    expect(() => verifyPersistedSettlementAllocation({ ...persisted, obligationSetDischarged: false }, batch, recovery, [deduction]))
      .toThrow(/canonical projection at obligationSetDischarged/);
  });

  it('requires the exact authoritative deduction-root set', () => {
    const persisted = projectSettlementAllocationRecord(allocation, batch, recovery, [deduction]);
    const other = createRoyaltyDeductionAuthority({
      id: 'deduction:other',
      beneficiaryId: 'artist:1',
      amount: money(50n, 'USD'),
      basis: 'tax:withholding:v1',
      authorityRef: 'tax:notice:other',
      observedAt: '2026-10-01T00:01:30Z',
    });
    expect(() => verifyPersistedSettlementAllocation(persisted, batch, recovery, [other]))
      .toThrow(/deduction roots do not match authoritative deductions/);
  });

  it('rejects malformed, unsorted, and duplicate deduction-root storage', () => {
    const second = createRoyaltyDeductionAuthority({
      id: 'deduction:second', beneficiaryId: 'artist:1', amount: money(0n, 'USD'), basis: 'reporting:zero:v1', authorityRef: 'authority:zero', observedAt: '2026-10-01T00:01:30Z',
    });
    const withTwo = createSettlementAllocationAuthority({
      allocationId: 'allocation:two', batch, recovery, deductions: [deduction, second], residualHeld: money(0n, 'USD'), allocatedAt: '2026-10-01T00:02:00Z',
    });
    const persisted = projectSettlementAllocationRecord(withTwo, batch, recovery, [deduction, second]);
    expect(() => verifyPersistedSettlementAllocation({ ...persisted, deductionRoots: [...persisted.deductionRoots].reverse() }, batch, recovery, [deduction, second]))
      .toThrow(/canonically sorted/);
    expect(() => verifyPersistedSettlementAllocation({ ...persisted, deductionRoots: [persisted.deductionRoots[0], persisted.deductionRoots[0]] }, batch, recovery, [deduction, second]))
      .toThrow(/must not contain duplicates/);
    expect(() => verifyPersistedSettlementAllocation({ ...persisted, deductionRoots: ['not-a-digest'] }, batch, recovery, [deduction, second]))
      .toThrow(/lowercase SHA-256 digest/);
  });

  it('rejects an allocation root changed independently of its canonical content', () => {
    const persisted = projectSettlementAllocationRecord(allocation, batch, recovery, [deduction]);
    expect(() => verifyPersistedSettlementAllocation({ ...persisted, allocationRoot: 'f'.repeat(64) }, batch, recovery, [deduction]))
      .toThrow(/canonical projection at allocationRoot/);
  });
});
