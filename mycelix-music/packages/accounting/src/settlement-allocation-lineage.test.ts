import { describe, expect, it } from 'vitest';
import { createRoyaltyDeductionAuthority } from './deduction-authority.js';
import { money } from './money.js';
import { buildDeterministicNettingBatches } from './netting.js';
import { createRoyaltyObligationAuthority } from './obligation-authority.js';
import { reconstructSettlementRecovery, SettlementAttemptState } from './recovery.js';
import {
  assertSettlementAllocationLineageBoundary,
  assertSettlementAllocationLineageStep,
  assertSettlementAllocationSuccessorLink,
  createSettlementAllocationLineageBoundary,
  createSettlementAllocationSuccessorLink,
  requireCanonicalSettlementAllocationHead,
  resolveSettlementAllocationLineage,
} from './settlement-allocation-lineage.js';
import { createSettlementAllocationAuthority } from './settlement-allocation.js';
import { SettlementEligibilityCode, type SettlementEpoch } from './settlement.js';

const obligation = createRoyaltyObligationAuthority({
  id: 'obl:lineage',
  beneficiaryId: 'artist:lineage',
  amount: money(500n, 'USD'),
  observedAt: '2026-09-10T00:00:00Z',
  provenance: {
    usageEvidenceRef: 'usage:lineage',
    rightsResolutionRef: 'rights:lineage',
    economicTermsRef: 'terms:lineage',
  },
});
const epoch: SettlementEpoch = {
  id: 'epoch:lineage',
  cutoff: '2026-09-30T23:59:59Z',
  eligibilityAsOf: '2026-10-01T00:00:00Z',
  minimumPayout: money(100n, 'USD'),
};
const eligibility = [{
  id: 'elig:lineage',
  obligationId: obligation.id,
  code: SettlementEligibilityCode.Eligible,
  sourceRef: 'eligibility:lineage',
  observedAt: '2026-09-30T00:00:00Z',
}] as const;
const batch = buildDeterministicNettingBatches([obligation], epoch, eligibility)[0]!;

function recovery(amountMinor: bigint, receipt: string) {
  return reconstructSettlementRecovery(batch, [{
    attemptId: `attempt:${receipt}`,
    batchId: batch.batchId,
    obligationSetRoot: batch.obligationSetRoot,
    eligibilityAsOf: batch.eligibilityAsOf,
    eligibilityEvidenceRoot: batch.eligibilityEvidenceRoot,
    state: SettlementAttemptState.Finalized,
    observedAt: '2026-10-01T00:01:00Z',
    railReceiptRef: receipt,
    settledAmount: money(amountMinor, 'USD'),
  }]);
}

function deduction(id: string, amountMinor: bigint) {
  return createRoyaltyDeductionAuthority({
    id,
    beneficiaryId: batch.beneficiaryId,
    amount: money(amountMinor, 'USD'),
    basis: `authority-basis:${id}`,
    authorityRef: `authority-ref:${id}`,
    observedAt: '2026-10-01T00:01:30Z',
  });
}

const deduction50 = deduction('deduction:50', 50n);
const deduction25 = deduction('deduction:25', 25n);
const deduction75 = deduction('deduction:75', 75n);
const extra50 = deduction('deduction:extra-50', 50n);

function allocation(input: {
  allocationId: string;
  creatorPaidMinor: bigint;
  receipt: string;
  deductions: readonly ReturnType<typeof deduction>[];
  residualHeldMinor: bigint;
  allocatedAt: string;
}) {
  return createSettlementAllocationAuthority({
    allocationId: input.allocationId,
    batch,
    recovery: recovery(input.creatorPaidMinor, input.receipt),
    deductions: input.deductions,
    residualHeld: money(input.residualHeldMinor, 'USD'),
    ...(input.residualHeldMinor === 0n ? {} : { residualAuthorityRef: `residual:${input.allocationId}` }),
    allocatedAt: input.allocatedAt,
  });
}

const initial = allocation({
  allocationId: 'allocation:initial',
  creatorPaidMinor: 425n,
  receipt: 'rail:receipt:425',
  deductions: [deduction50],
  residualHeldMinor: 25n,
  allocatedAt: '2026-10-01T00:02:00Z',
});
const deductionResolved = allocation({
  allocationId: 'allocation:deduction-resolved',
  creatorPaidMinor: 425n,
  receipt: 'rail:receipt:425',
  deductions: [deduction50, deduction25],
  residualHeldMinor: 0n,
  allocatedAt: '2026-10-01T00:03:00Z',
});
const paymentResolved = allocation({
  allocationId: 'allocation:payment-resolved',
  creatorPaidMinor: 450n,
  receipt: 'rail:receipt:450',
  deductions: [deduction50],
  residualHeldMinor: 0n,
  allocatedAt: '2026-10-01T00:03:30Z',
});

function linkTo(successor: typeof initial, suffix: string) {
  return createSettlementAllocationSuccessorLink({
    linkId: `allocation-link:${suffix}`,
    predecessor: initial,
    successor,
    supersessionEvidenceRef: `reconciliation:${suffix}`,
    linkedAt: '2026-10-01T00:04:00Z',
  });
}

function boundary(coverage: 'complete' | 'partial' = 'complete') {
  return createSettlementAllocationLineageBoundary({
    asOf: '2026-10-01T00:05:00Z',
    observedThrough: coverage === 'complete' ? '2026-10-01T00:05:00Z' : '2026-10-01T00:04:30Z',
    coverage,
    sourceRef: 'allocation-store:snapshot:1',
  });
}

describe('settlement allocation lineage', () => {
  it('accepts monotonic residual resolution by new deduction authority', () => {
    expect(() => assertSettlementAllocationLineageStep(initial, deductionResolved)).not.toThrow();
    const link = linkTo(deductionResolved, 'deduction');
    const resolved = resolveSettlementAllocationLineage([deductionResolved, initial], [link], boundary());
    expect(resolved.status).toBe('canonical');
    expect(resolved.initialAllocationRoot).toBe(initial.allocationRoot);
    expect(resolved.headAllocationRoot).toBe(deductionResolved.allocationRoot);
    expect(requireCanonicalSettlementAllocationHead(resolved).obligationSetDischarged).toBe(true);
    expect(resolved.depth).toBe(1);
  });

  it('accepts monotonic residual resolution by a larger creator rail payment only when receipt identity also advances', () => {
    expect(() => assertSettlementAllocationLineageStep(initial, paymentResolved)).not.toThrow();
  });

  it('rejects a timestamp-only/no-op successor', () => {
    const noop = allocation({
      allocationId: 'allocation:noop',
      creatorPaidMinor: 425n,
      receipt: 'rail:receipt:425',
      deductions: [deduction50],
      residualHeldMinor: 25n,
      allocatedAt: '2026-10-01T00:03:00Z',
    });
    expect(() => assertSettlementAllocationLineageStep(initial, noop)).toThrow(/strictly reduce residual held value/);
  });

  it('rejects residual growth even when both allocations conserve independently', () => {
    const regressed = allocation({
      allocationId: 'allocation:residual-regressed',
      creatorPaidMinor: 425n,
      receipt: 'rail:receipt:425',
      deductions: [deduction25],
      residualHeldMinor: 50n,
      allocatedAt: '2026-10-01T00:03:00Z',
    });
    expect(() => assertSettlementAllocationLineageStep(initial, regressed)).toThrow(/strictly reduce residual held value/);
  });

  it('rejects creator-paid regression', () => {
    const regressed = allocation({
      allocationId: 'allocation:creator-regressed',
      creatorPaidMinor: 400n,
      receipt: 'rail:receipt:400',
      deductions: [deduction50, extra50],
      residualHeldMinor: 0n,
      allocatedAt: '2026-10-01T00:03:00Z',
    });
    expect(() => assertSettlementAllocationLineageStep(initial, regressed)).toThrow(/cannot reduce creator-paid value/);
  });

  it('rejects disappearance of a prior deduction authority', () => {
    const rebound = allocation({
      allocationId: 'allocation:deduction-rebound',
      creatorPaidMinor: 425n,
      receipt: 'rail:receipt:425',
      deductions: [deduction75],
      residualHeldMinor: 0n,
      allocatedAt: '2026-10-01T00:03:00Z',
    });
    expect(() => assertSettlementAllocationLineageStep(initial, rebound)).toThrow(/cannot remove a prior deduction authority root/);
  });

  it('rejects rail receipt rebinding when creator-paid value did not change', () => {
    const rebound = allocation({
      allocationId: 'allocation:receipt-rebound',
      creatorPaidMinor: 425n,
      receipt: 'rail:receipt:425-other',
      deductions: [deduction50, deduction25],
      residualHeldMinor: 0n,
      allocatedAt: '2026-10-01T00:03:00Z',
    });
    expect(() => assertSettlementAllocationLineageStep(initial, rebound)).toThrow(/amount and durable rail receipt identity must advance together/);
  });

  it('rejects creator-paid growth under an unchanged receipt identity', () => {
    const rebound = allocation({
      allocationId: 'allocation:amount-rebound',
      creatorPaidMinor: 450n,
      receipt: 'rail:receipt:425',
      deductions: [deduction50],
      residualHeldMinor: 0n,
      allocatedAt: '2026-10-01T00:03:00Z',
    });
    expect(() => assertSettlementAllocationLineageStep(initial, rebound)).toThrow(/amount and durable rail receipt identity must advance together/);
  });

  it('forbids successors after economic discharge', () => {
    const later = allocation({
      allocationId: 'allocation:after-discharge',
      creatorPaidMinor: 450n,
      receipt: 'rail:receipt:after-discharge',
      deductions: [deduction50],
      residualHeldMinor: 0n,
      allocatedAt: '2026-10-01T00:05:00Z',
    });
    expect(() => assertSettlementAllocationLineageStep(deductionResolved, later)).toThrow(/discharged settlement allocation cannot be superseded/);
  });

  it('binds successor links cryptographically and rejects a link emitted before the successor', () => {
    expect(() => createSettlementAllocationSuccessorLink({
      linkId: 'allocation-link:early',
      predecessor: initial,
      successor: deductionResolved,
      supersessionEvidenceRef: 'reconciliation:early',
      linkedAt: '2026-10-01T00:02:59Z',
    })).toThrow(/cannot predate the successor allocation/);

    const link = linkTo(deductionResolved, 'bound');
    expect(() => assertSettlementAllocationSuccessorLink(
      { ...link, linkRoot: 'f'.repeat(64) },
      initial,
      deductionResolved,
    )).toThrow(/does not match authoritative reconstruction/);
  });

  it('binds source completeness into the lineage root and never promotes partial coverage', () => {
    const link = linkTo(deductionResolved, 'coverage');
    const complete = resolveSettlementAllocationLineage([initial, deductionResolved], [link], boundary('complete'));
    const partial = resolveSettlementAllocationLineage([initial, deductionResolved], [link], boundary('partial'));
    expect(complete.status).toBe('canonical');
    expect(partial.status).toBe('provisional');
    expect(partial.lineageRoot).not.toBe(complete.lineageRoot);
    expect(() => requireCanonicalSettlementAllocationHead(partial)).toThrow(/provisional/);
  });

  it('rejects a completeness claim whose source was not observed through asOf', () => {
    expect(() => createSettlementAllocationLineageBoundary({
      asOf: '2026-10-01T00:05:00Z',
      observedThrough: '2026-10-01T00:04:59Z',
      coverage: 'complete',
      sourceRef: 'allocation-store:stale',
    })).toThrow(/must be observed through asOf/);
  });

  it('rejects boundary-root tampering', () => {
    const valid = boundary();
    expect(() => assertSettlementAllocationLineageBoundary({ ...valid, boundaryRoot: 'f'.repeat(64) }))
      .toThrow(/does not match authoritative reconstruction/);
  });

  it('rejects allocation or link evidence later than the requested asOf', () => {
    const link = linkTo(deductionResolved, 'future-boundary');
    const tooEarly = createSettlementAllocationLineageBoundary({
      asOf: '2026-10-01T00:03:30Z',
      observedThrough: '2026-10-01T00:03:30Z',
      coverage: 'complete',
      sourceRef: 'allocation-store:early-snapshot',
    });
    expect(() => resolveSettlementAllocationLineage([initial, deductionResolved], [link], tooEarly))
      .toThrow(/successor link later than asOf/);
  });

  it('is deterministic and independent of caller input order', () => {
    const link = linkTo(deductionResolved, 'deterministic');
    const observationBoundary = boundary();
    const left = resolveSettlementAllocationLineage([initial, deductionResolved], [link], observationBoundary);
    const right = resolveSettlementAllocationLineage([deductionResolved, initial], [link], observationBoundary);
    expect(right.lineageRoot).toBe(left.lineageRoot);
    expect(right.allocationRoots).toEqual(left.allocationRoots);
  });

  it('fails closed on missing lineage links instead of choosing the newest timestamp', () => {
    expect(() => resolveSettlementAllocationLineage([initial, deductionResolved], [], boundary()))
      .toThrow(/exactly one successor link per non-initial allocation/);
  });

  it('fails closed on forks rather than selecting one discharged successor', () => {
    const deductionLink = linkTo(deductionResolved, 'fork-deduction');
    const paymentLink = linkTo(paymentResolved, 'fork-payment');
    expect(() => resolveSettlementAllocationLineage(
      [initial, deductionResolved, paymentResolved],
      [deductionLink, paymentLink],
      boundary(),
    )).toThrow(/fork detected/);
  });

  it('fails closed on joins from independent predecessors', () => {
    const alternateInitial = allocation({
      allocationId: 'allocation:alternate-initial',
      creatorPaidMinor: 425n,
      receipt: 'rail:receipt:425',
      deductions: [deduction50],
      residualHeldMinor: 25n,
      allocatedAt: '2026-10-01T00:02:30Z',
    });
    const firstLink = createSettlementAllocationSuccessorLink({
      linkId: 'allocation-link:join-first',
      predecessor: initial,
      successor: deductionResolved,
      supersessionEvidenceRef: 'reconciliation:join-first',
      linkedAt: '2026-10-01T00:04:00Z',
    });
    const secondLink = createSettlementAllocationSuccessorLink({
      linkId: 'allocation-link:join-second',
      predecessor: alternateInitial,
      successor: deductionResolved,
      supersessionEvidenceRef: 'reconciliation:join-second',
      linkedAt: '2026-10-01T00:04:01Z',
    });
    expect(() => resolveSettlementAllocationLineage(
      [initial, alternateInitial, deductionResolved],
      [firstLink, secondLink],
      boundary(),
    )).toThrow(/join detected/);
  });

  it('resolves a single immutable allocation only as strongly as its source coverage permits', () => {
    const complete = resolveSettlementAllocationLineage([initial], [], boundary('complete'));
    const partial = resolveSettlementAllocationLineage([initial], [], boundary('partial'));
    expect(complete.status).toBe('canonical');
    expect(complete.depth).toBe(0);
    expect(complete.headAllocationRoot).toBe(initial.allocationRoot);
    expect(partial.status).toBe('provisional');
  });
});
