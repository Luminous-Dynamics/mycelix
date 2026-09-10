import { describe, expect, it } from 'vitest';
import { createRoyaltyDeductionAuthority } from './deduction-authority.js';
import { buildDeterministicNettingBatches } from './netting.js';
import { money } from './money.js';
import { createRoyaltyObligationAuthority } from './obligation-authority.js';
import { reconstructSettlementRecovery, SettlementAttemptState } from './recovery.js';
import { assertSettlementAllocationAuthority, createSettlementAllocationAuthority } from './settlement-allocation.js';
import { SettlementEligibilityCode, type SettlementEpoch } from './settlement.js';

const obligation = createRoyaltyObligationAuthority({
  id: 'obl:1', beneficiaryId: 'artist:1', amount: money(500n, 'USD'), observedAt: '2026-09-10T00:00:00Z',
  provenance: { usageEvidenceRef: 'usage:1', rightsResolutionRef: 'rights:1', economicTermsRef: 'terms:v1' },
});
const epoch: SettlementEpoch = { id: 'epoch:1', cutoff: '2026-09-30T23:59:59Z', eligibilityAsOf: '2026-10-01T00:00:00Z', minimumPayout: money(100n, 'USD') };
const eligibility = [{ id: 'elig:1', obligationId: obligation.id, code: SettlementEligibilityCode.Eligible, sourceRef: 'eligibility:v1', observedAt: '2026-09-30T00:00:00Z' }] as const;
const batch = buildDeterministicNettingBatches([obligation], epoch, eligibility)[0]!;

function recovery(amountMinor: bigint) {
  return reconstructSettlementRecovery(batch, [{ attemptId: 'attempt:1', batchId: batch.batchId, obligationSetRoot: batch.obligationSetRoot, eligibilityAsOf: batch.eligibilityAsOf, eligibilityEvidenceRoot: batch.eligibilityEvidenceRoot, state: SettlementAttemptState.Finalized, observedAt: '2026-10-01T00:01:00Z', railReceiptRef: 'rail:receipt:1', settledAmount: money(amountMinor, 'USD') }]);
}
function deduction(amountMinor: bigint, authorityRef = 'tax:notice:1') {
  return createRoyaltyDeductionAuthority({ id: 'deduction:1', beneficiaryId: 'artist:1', amount: money(amountMinor, 'USD'), basis: 'tax:withholding:v1', authorityRef, observedAt: '2026-10-01T00:01:30Z' });
}

describe('settlement allocation authority', () => {
  it('discharges the obligation set only when receipt plus deductions exactly consume batch gross', () => {
    const allocation = createSettlementAllocationAuthority({ allocationId: 'allocation:1', batch, recovery: recovery(450n), deductions: [deduction(50n)], residualHeld: money(0n, 'USD'), allocatedAt: '2026-10-01T00:02:00Z' });
    expect(allocation.creatorPaid.amountMinor).toBe(450n);
    expect(allocation.deductionTotal.amountMinor).toBe(50n);
    expect(allocation.residualHeld.amountMinor).toBe(0n);
    expect(allocation.obligationSetDischarged).toBe(true);
  });

  it('preserves an unresolved residual as held value and refuses discharge', () => {
    const allocation = createSettlementAllocationAuthority({ allocationId: 'allocation:held', batch, recovery: recovery(425n), deductions: [deduction(50n)], residualHeld: money(25n, 'USD'), residualAuthorityRef: 'reconciliation:case:7', allocatedAt: '2026-10-01T00:02:00Z' });
    expect(allocation.obligationSetDischarged).toBe(false);
    expect(allocation.residualHeld.amountMinor).toBe(25n);
  });

  it('rejects conservation failure instead of allowing value to disappear', () => {
    expect(() => createSettlementAllocationAuthority({ allocationId: 'allocation:bad', batch, recovery: recovery(450n), deductions: [deduction(25n)], residualHeld: money(0n, 'USD'), allocatedAt: '2026-10-01T00:02:00Z' })).toThrow(/must conserve batch gross/);
  });

  it('requires authority for every nonzero residual hold', () => {
    expect(() => createSettlementAllocationAuthority({ allocationId: 'allocation:unauthorized-residual', batch, recovery: recovery(450n), deductions: [], residualHeld: money(50n, 'USD'), allocatedAt: '2026-10-01T00:02:00Z' })).toThrow(/requires residualAuthorityRef/);
  });

  it('binds deduction provenance into the allocation root', () => {
    const first = createSettlementAllocationAuthority({ allocationId: 'allocation:1', batch, recovery: recovery(450n), deductions: [deduction(50n, 'tax:notice:1')], residualHeld: money(0n, 'USD'), allocatedAt: '2026-10-01T00:02:00Z' });
    const second = createSettlementAllocationAuthority({ allocationId: 'allocation:1', batch, recovery: recovery(450n), deductions: [deduction(50n, 'tax:notice:2')], residualHeld: money(0n, 'USD'), allocatedAt: '2026-10-01T00:02:00Z' });
    expect(first.allocationRoot).not.toBe(second.allocationRoot);
  });

  it('rejects allocations compiled before rail or deduction evidence exists', () => {
    expect(() => createSettlementAllocationAuthority({ allocationId: 'allocation:early', batch, recovery: recovery(450n), deductions: [deduction(50n)], residualHeld: money(0n, 'USD'), allocatedAt: '2026-10-01T00:01:15Z' })).toThrow(/predate deduction evidence/);
  });

  it('rejects redundant field tampering behind an unchanged allocation root', () => {
    const recovered = recovery(450n);
    const withholding = deduction(50n);
    const allocation = createSettlementAllocationAuthority({ allocationId: 'allocation:1', batch, recovery: recovered, deductions: [withholding], residualHeld: money(0n, 'USD'), allocatedAt: '2026-10-01T00:02:00Z' });
    expect(() => assertSettlementAllocationAuthority({ ...allocation, creatorPaid: money(449n, 'USD') }, { batch, recovery: recovered, deductions: [withholding] })).toThrow(/authoritative reconstruction/);
  });
});
