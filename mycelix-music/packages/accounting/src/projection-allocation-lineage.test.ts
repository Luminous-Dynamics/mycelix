import { describe, expect, it } from 'vitest';
import { createRoyaltyDeductionAuthority } from './deduction-authority.js';
import { money } from './money.js';
import { buildDeterministicNettingBatches } from './netting.js';
import { createRoyaltyObligationAuthority } from './obligation-authority.js';
import { compileRoyaltyStatement } from './projection.js';
import { reconstructSettlementRecovery, SettlementAttemptState } from './recovery.js';
import {
  createSettlementAllocationLineageBoundary,
  createSettlementAllocationSuccessorLink,
  resolveSettlementAllocationLineage,
} from './settlement-allocation-lineage.js';
import { createSettlementAllocationAuthority } from './settlement-allocation.js';
import { SettlementEligibilityCode, type SettlementEpoch } from './settlement.js';
import { StatementKind } from './statements.js';

const obligation = createRoyaltyObligationAuthority({
  id: 'obl:statement-lineage',
  beneficiaryId: 'artist:statement-lineage',
  amount: money(500n, 'USD'),
  observedAt: '2026-09-10T00:00:00Z',
  provenance: {
    usageEvidenceRef: 'usage:statement-lineage',
    rightsResolutionRef: 'rights:statement-lineage',
    economicTermsRef: 'terms:statement-lineage',
  },
});
const epoch: SettlementEpoch = {
  id: 'epoch:statement-lineage',
  cutoff: '2026-09-30T23:59:59Z',
  eligibilityAsOf: '2026-10-01T00:00:00Z',
  minimumPayout: money(100n, 'USD'),
};
const eligibility = [{
  id: 'elig:statement-lineage',
  obligationId: obligation.id,
  code: SettlementEligibilityCode.Eligible,
  sourceRef: 'eligibility:statement-lineage',
  observedAt: '2026-09-30T00:00:00Z',
}] as const;
const batch = buildDeterministicNettingBatches([obligation], epoch, eligibility)[0]!;
const recovery = reconstructSettlementRecovery(batch, [{
  attemptId: 'attempt:statement-lineage',
  batchId: batch.batchId,
  obligationSetRoot: batch.obligationSetRoot,
  eligibilityAsOf: batch.eligibilityAsOf,
  eligibilityEvidenceRoot: batch.eligibilityEvidenceRoot,
  state: SettlementAttemptState.Finalized,
  observedAt: '2026-10-01T00:01:00Z',
  railReceiptRef: 'rail:receipt:statement-lineage',
  settledAmount: money(425n, 'USD'),
}]);
const tax50 = createRoyaltyDeductionAuthority({
  id: 'deduction:tax-50', beneficiaryId: batch.beneficiaryId, amount: money(50n, 'USD'),
  basis: 'tax:withholding:v1', authorityRef: 'tax:notice:50', observedAt: '2026-10-01T00:01:30Z',
});
const tax25 = createRoyaltyDeductionAuthority({
  id: 'deduction:tax-25', beneficiaryId: batch.beneficiaryId, amount: money(25n, 'USD'),
  basis: 'tax:withholding:supplement:v1', authorityRef: 'tax:notice:25', observedAt: '2026-10-01T00:02:30Z',
});
const initial = createSettlementAllocationAuthority({
  allocationId: 'allocation:statement-lineage:initial',
  batch,
  recovery,
  deductions: [tax50],
  residualHeld: money(25n, 'USD'),
  residualAuthorityRef: 'residual:statement-lineage',
  allocatedAt: '2026-10-01T00:02:00Z',
});
const successor = createSettlementAllocationAuthority({
  allocationId: 'allocation:statement-lineage:successor',
  batch,
  recovery,
  deductions: [tax50, tax25],
  residualHeld: money(0n, 'USD'),
  allocatedAt: '2026-10-01T00:03:00Z',
});
const link = createSettlementAllocationSuccessorLink({
  linkId: 'allocation-link:statement-lineage',
  predecessor: initial,
  successor,
  supersessionEvidenceRef: 'reconciliation:statement-lineage',
  linkedAt: '2026-10-01T00:03:30Z',
});
const asOf = '2026-10-02T00:00:00Z';
const boundary = createSettlementAllocationLineageBoundary({
  asOf,
  observedThrough: asOf,
  coverage: 'complete',
  sourceRef: 'allocation-store:snapshot:statement-lineage',
});
const lineage = resolveSettlementAllocationLineage([successor, initial], [link], boundary);

function compile(allocationLineage = lineage) {
  return compileRoyaltyStatement({
    statementId: 'statement:statement-lineage',
    kind: StatementKind.Periodic,
    beneficiaryId: batch.beneficiaryId,
    period: { startInclusive: '2026-09-01T00:00:00Z', endExclusive: '2026-10-01T00:00:00Z' },
    asOf,
    completeness: {
      kind: 'complete',
      through: {
        usageObservedThrough: '2026-10-01T00:00:00Z',
        rightsResolvedThrough: '2026-10-01T00:00:00Z',
        settlementsObservedThrough: asOf,
      },
    },
    settlementEpoch: epoch,
    obligations: [obligation],
    eligibilityObservations: eligibility,
    deductions: [tax50, tax25],
    settlements: [{ batch, recovery, allocationLineage }],
  });
}

describe('statement allocation-lineage authority', () => {
  it('uses the resolver-selected successor head and commits the lineage history', () => {
    expect(lineage.headAllocationRoot).toBe(successor.allocationRoot);
    expect(lineage.headAllocation.obligationSetDischarged).toBe(true);
    const statement = compile();
    expect(statement.paid.amountMinor).toBe(425n);
    expect(statement.deductions.amountMinor).toBe(75n);
    expect(statement.netPayable.amountMinor).toBe(425n);
  });

  it('changes settlement commitment when observed allocation history changes', () => {
    const predecessorBoundary = createSettlementAllocationLineageBoundary({
      asOf,
      observedThrough: asOf,
      coverage: 'complete',
      sourceRef: 'allocation-store:snapshot:predecessor-only',
    });
    const predecessorOnly = resolveSettlementAllocationLineage([initial], [], predecessorBoundary);
    const predecessorStatement = compile(predecessorOnly);
    const successorStatement = compile(lineage);
    expect(successorStatement.obligationRoot).toBe(predecessorStatement.obligationRoot);
    expect(successorStatement.adjustmentRoot).toBe(predecessorStatement.adjustmentRoot);
    expect(successorStatement.settlementRoot).not.toBe(predecessorStatement.settlementRoot);
  });

  it('rejects a valid lineage resolved for a different statement instant', () => {
    const wrongBoundary = createSettlementAllocationLineageBoundary({
      asOf: '2026-10-01T23:59:59Z',
      observedThrough: '2026-10-01T23:59:59Z',
      coverage: 'complete',
      sourceRef: 'allocation-store:snapshot:wrong-asof',
    });
    const wrongLineage = resolveSettlementAllocationLineage([initial, successor], [link], wrongBoundary);
    expect(() => compile(wrongLineage)).toThrow(/boundary asOf must equal statement asOf/);
  });

  it('rejects a structural clone of a genuine canonical lineage resolution', () => {
    const clonedLineage = { ...lineage };
    expect(() => compile(clonedLineage)).toThrow(/must be produced by authoritative resolver/);
  });
});
