import { describe, expect, it } from 'vitest';
import { money } from './money.js';
import { buildDeterministicNettingBatches } from './netting.js';
import { createRoyaltyObligationAuthority } from './obligation-authority.js';
import { reconstructSettlementRecovery, SettlementAttemptState } from './recovery.js';
import {
  createSettlementAllocationLineageBoundary,
  requireCanonicalSettlementAllocationHead,
  resolveSettlementAllocationLineage,
} from './settlement-allocation-lineage.js';
import { createSettlementAllocationAuthority } from './settlement-allocation.js';
import { SettlementEligibilityCode, type SettlementEpoch } from './settlement.js';

const obligation = createRoyaltyObligationAuthority({
  id: 'obl:seal',
  beneficiaryId: 'artist:seal',
  amount: money(500n, 'USD'),
  observedAt: '2026-09-10T00:00:00Z',
  provenance: {
    usageEvidenceRef: 'usage:seal',
    rightsResolutionRef: 'rights:seal',
    economicTermsRef: 'terms:seal',
  },
});
const epoch: SettlementEpoch = {
  id: 'epoch:seal',
  cutoff: '2026-09-30T23:59:59Z',
  eligibilityAsOf: '2026-10-01T00:00:00Z',
  minimumPayout: money(100n, 'USD'),
};
const eligibility = [{
  id: 'elig:seal',
  obligationId: obligation.id,
  code: SettlementEligibilityCode.Eligible,
  sourceRef: 'eligibility:seal',
  observedAt: '2026-09-30T00:00:00Z',
}] as const;
const batch = buildDeterministicNettingBatches([obligation], epoch, eligibility)[0]!;
const recovery = reconstructSettlementRecovery(batch, [{
  attemptId: 'attempt:seal',
  batchId: batch.batchId,
  obligationSetRoot: batch.obligationSetRoot,
  eligibilityAsOf: batch.eligibilityAsOf,
  eligibilityEvidenceRoot: batch.eligibilityEvidenceRoot,
  state: SettlementAttemptState.Finalized,
  observedAt: '2026-10-01T00:01:00Z',
  railReceiptRef: 'rail:receipt:seal',
  settledAmount: money(450n, 'USD'),
}]);
const allocation = createSettlementAllocationAuthority({
  allocationId: 'allocation:seal',
  batch,
  recovery,
  deductions: [],
  residualHeld: money(50n, 'USD'),
  residualAuthorityRef: 'residual:seal',
  allocatedAt: '2026-10-01T00:02:00Z',
});
const boundary = createSettlementAllocationLineageBoundary({
  asOf: '2026-10-01T00:05:00Z',
  observedThrough: '2026-10-01T00:05:00Z',
  coverage: 'complete',
  sourceRef: 'allocation-store:snapshot:seal',
});

describe('settlement allocation lineage runtime seal', () => {
  it('permits only the exact in-process resolution minted by the authoritative resolver', () => {
    const resolution = resolveSettlementAllocationLineage([allocation], [], boundary);
    expect(requireCanonicalSettlementAllocationHead(resolution).allocationRoot).toBe(allocation.allocationRoot);

    const structuralClone = { ...resolution };
    expect(() => requireCanonicalSettlementAllocationHead(structuralClone))
      .toThrow(/must be produced by authoritative resolver/);
  });

  it('rejects a structurally fabricated canonical claim even with a valid boundary and head', () => {
    const genuine = resolveSettlementAllocationLineage([allocation], [], boundary);
    const fabricated = {
      ...genuine,
      lineageRoot: 'f'.repeat(64),
    };
    expect(() => requireCanonicalSettlementAllocationHead(fabricated))
      .toThrow(/must be produced by authoritative resolver/);
  });
});
