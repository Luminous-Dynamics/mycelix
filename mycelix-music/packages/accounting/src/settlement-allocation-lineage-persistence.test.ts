import { describe, expect, it } from 'vitest';
import { createRoyaltyDeductionAuthority } from './deduction-authority.js';
import { money } from './money.js';
import { buildDeterministicNettingBatches } from './netting.js';
import { createRoyaltyObligationAuthority } from './obligation-authority.js';
import { reconstructSettlementRecovery, SettlementAttemptState } from './recovery.js';
import {
  createSettlementAllocationSuccessorLink,
} from './settlement-allocation-lineage.js';
import {
  projectSettlementAllocationSuccessorLinkRecord,
  verifyPersistedSettlementAllocationSuccessorLink,
} from './settlement-allocation-lineage-persistence.js';
import { createSettlementAllocationAuthority } from './settlement-allocation.js';
import { SettlementEligibilityCode, type SettlementEpoch } from './settlement.js';

const obligation = createRoyaltyObligationAuthority({
  id: 'obl:link-persist', beneficiaryId: 'creator:link-persist', amount: money(500n, 'USD'),
  observedAt: '2026-09-10T00:00:00Z',
  provenance: { usageEvidenceRef: 'usage:1', rightsResolutionRef: 'rights:1', economicTermsRef: 'terms:1' },
});
const epoch: SettlementEpoch = {
  id: 'epoch:link-persist', cutoff: '2026-09-30T23:59:59Z', eligibilityAsOf: '2026-10-01T00:00:00Z', minimumPayout: money(100n, 'USD'),
};
const eligibility = [{ id: 'elig:1', obligationId: obligation.id, code: SettlementEligibilityCode.Eligible, sourceRef: 'eligibility:1', observedAt: '2026-09-30T00:00:00Z' }] as const;
const batch = buildDeterministicNettingBatches([obligation], epoch, eligibility)[0]!;
const recovery = reconstructSettlementRecovery(batch, [{
  attemptId: 'attempt:1', batchId: batch.batchId, obligationSetRoot: batch.obligationSetRoot,
  eligibilityAsOf: batch.eligibilityAsOf, eligibilityEvidenceRoot: batch.eligibilityEvidenceRoot,
  state: SettlementAttemptState.Finalized, observedAt: '2026-10-01T00:01:00Z',
  railReceiptRef: 'rail:receipt:1', settledAmount: money(425n, 'USD'),
}]);
const deduction50 = createRoyaltyDeductionAuthority({
  id: 'deduction:50', beneficiaryId: batch.beneficiaryId, amount: money(50n, 'USD'), basis: 'tax:v1', authorityRef: 'tax:50', observedAt: '2026-10-01T00:01:30Z',
});
const deduction25 = createRoyaltyDeductionAuthority({
  id: 'deduction:25', beneficiaryId: batch.beneficiaryId, amount: money(25n, 'USD'), basis: 'tax:v1', authorityRef: 'tax:25', observedAt: '2026-10-01T00:02:30Z',
});
const initial = createSettlementAllocationAuthority({
  allocationId: 'allocation:initial', batch, recovery, deductions: [deduction50], residualHeld: money(25n, 'USD'), residualAuthorityRef: 'residual:25', allocatedAt: '2026-10-01T00:02:00Z',
});
const successor = createSettlementAllocationAuthority({
  allocationId: 'allocation:successor', batch, recovery, deductions: [deduction50, deduction25], residualHeld: money(0n, 'USD'), allocatedAt: '2026-10-01T00:03:00Z',
});
const link = createSettlementAllocationSuccessorLink({
  linkId: 'link:1', predecessor: initial, successor, supersessionEvidenceRef: 'reconciliation:1', linkedAt: '2026-10-01T00:04:00Z',
});

describe('persisted settlement allocation successor links', () => {
  it('round-trips a canonical link with Prisma-style Date timestamp', () => {
    const persisted = projectSettlementAllocationSuccessorLinkRecord(link, initial, successor);
    const verified = verifyPersistedSettlementAllocationSuccessorLink({ ...persisted, linkedAt: new Date(persisted.linkedAt) }, initial, successor);
    expect(verified.linkRoot).toBe(link.linkRoot);
  });

  it('rejects endpoint, supersession evidence, timestamp and root tampering', () => {
    const persisted = projectSettlementAllocationSuccessorLinkRecord(link, initial, successor);
    expect(() => verifyPersistedSettlementAllocationSuccessorLink({ ...persisted, predecessorAllocationRoot: 'f'.repeat(64) }, initial, successor)).toThrow(/canonical projection at predecessorAllocationRoot/);
    expect(() => verifyPersistedSettlementAllocationSuccessorLink({ ...persisted, successorAllocationRoot: 'e'.repeat(64) }, initial, successor)).toThrow(/canonical projection at successorAllocationRoot/);
    expect(() => verifyPersistedSettlementAllocationSuccessorLink({ ...persisted, supersessionEvidenceRef: 'reconciliation:other' }, initial, successor)).toThrow(/canonical projection at supersessionEvidenceRef/);
    expect(() => verifyPersistedSettlementAllocationSuccessorLink({ ...persisted, linkedAt: '2026-10-01T00:04:01Z' }, initial, successor)).toThrow(/canonical projection at linkRoot/);
    expect(() => verifyPersistedSettlementAllocationSuccessorLink({ ...persisted, linkRoot: 'd'.repeat(64) }, initial, successor)).toThrow(/canonical projection at linkRoot/);
  });
});
