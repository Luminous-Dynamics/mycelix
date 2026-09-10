import { describe, expect, it } from 'vitest';
import { money } from './money.js';
import { createRoyaltyObligationAuthority } from './obligation-authority.js';
import {
  projectDeductionRecord,
  projectEligibilityObservationRecord,
  projectRoyaltyObligationRecord,
  projectSettlementObservationRecord,
  projectStatementSnapshotRecord,
} from './persistence-projection.js';
import { SettlementAttemptState } from './recovery.js';
import {
  SettlementEligibilityCode,
  type SettlementEligibilityObservation,
} from './settlement.js';
import { createStatementSnapshot, StatementKind } from './statements.js';

function obligation(usageEvidenceRef: string) {
  return createRoyaltyObligationAuthority({
    id: 'obl:1',
    beneficiaryId: 'creator:alice',
    amount: money(500n, 'USD'),
    observedAt: '2026-09-10T00:00:00Z',
    provenance: {
      usageEvidenceRef,
      rightsResolutionRef: 'rights:resolution:1',
      economicTermsRef: 'terms:v1',
    },
  });
}

describe('canonical accounting persistence projections', () => {
  it('projects an obligation root from immutable economic provenance', () => {
    const first = projectRoyaltyObligationRecord(obligation('usage:epoch:a'));
    const second = projectRoyaltyObligationRecord(obligation('usage:epoch:b'));
    expect(first.amountMinor).toBe('500');
    expect(first.obligationRoot).not.toBe(second.obligationRoot);
  });

  it('derives eligibility evidence roots from source and observation time', () => {
    const base: SettlementEligibilityObservation = {
      id: 'eligibility:1',
      obligationId: 'obl:1',
      code: SettlementEligibilityCode.AwaitingPayeeRoute,
      sourceRef: 'payee-route-registry:v1',
      observedAt: '2026-09-10T00:00:00Z',
    };
    const first = projectEligibilityObservationRecord(base);
    const second = projectEligibilityObservationRecord({ ...base, sourceRef: 'payee-route-registry:v2' });
    expect(first.observationRoot).not.toBe(second.observationRoot);
  });

  it('refuses compiler-only eligibility results as source evidence', () => {
    const forged = {
      id: 'eligibility:forged',
      obligationId: 'obl:1',
      code: SettlementEligibilityCode.BelowThreshold,
      sourceRef: 'source:forged',
      observedAt: '2026-09-10T00:00:00Z',
    } as unknown as SettlementEligibilityObservation;
    expect(() => projectEligibilityObservationRecord(forged)).toThrow(/compiler-only code/);
  });

  it('derives deduction roots from both basis and authority reference', () => {
    const deduction = {
      id: 'deduction:1',
      beneficiaryId: 'creator:alice',
      amount: money(50n, 'USD'),
      basis: 'tax:withholding:v1',
      observedAt: '2026-09-10T00:00:00Z',
    } as const;
    const first = projectDeductionRecord(deduction, 'tax-authority:notice:1');
    const second = projectDeductionRecord(deduction, 'tax-authority:notice:2');
    expect(first.deductionRoot).not.toBe(second.deductionRoot);
  });

  it('binds persisted settlement evidence to the exact eligibility snapshot', () => {
    const base = {
      attemptId: 'attempt:1',
      batchId: 'batch:1',
      obligationSetRoot: 'a'.repeat(64),
      eligibilityAsOf: '2026-09-10T00:00:00Z',
      eligibilityEvidenceRoot: 'b'.repeat(64),
      state: SettlementAttemptState.Submitted,
      observedAt: '2026-09-10T00:01:00Z',
    } as const;
    const first = projectSettlementObservationRecord('settlement-observation:1', base);
    const second = projectSettlementObservationRecord('settlement-observation:1', {
      ...base,
      eligibilityEvidenceRoot: 'c'.repeat(64),
    });
    expect(first.observationRoot).not.toBe(second.observationRoot);
    expect(first.eligibilityAsOf).toBe('2026-09-10T00:00:00.000Z');
  });

  it('requires finalized rail observations to retain receipt identity', () => {
    expect(() => projectSettlementObservationRecord('settlement-observation:1', {
      attemptId: 'attempt:1',
      batchId: 'batch:1',
      obligationSetRoot: 'a'.repeat(64),
      eligibilityAsOf: '2026-09-10T00:00:00Z',
      eligibilityEvidenceRoot: 'b'.repeat(64),
      state: SettlementAttemptState.Finalized,
      observedAt: '2026-09-10T00:00:00Z',
    })).toThrow(/requires railReceiptRef/);
  });

  it('derives a stable statement snapshot root from the immutable projection', () => {
    const statement = createStatementSnapshot({
      statementId: 'statement:1',
      kind: StatementKind.Periodic,
      beneficiaryId: 'creator:alice',
      period: {
        startInclusive: '2026-09-01T00:00:00Z',
        endExclusive: '2026-10-01T00:00:00Z',
      },
      asOf: '2026-10-02T00:00:00Z',
      obligationRoot: 'a'.repeat(64),
      adjustmentRoot: 'b'.repeat(64),
      settlementRoot: 'c'.repeat(64),
      gross: money(500n, 'USD'),
      held: money(100n, 'USD'),
      deductions: money(50n, 'USD'),
      netPayable: money(350n, 'USD'),
      paid: money(300n, 'USD'),
      completeness: {
        kind: 'complete',
        through: {
          usageObservedThrough: '2026-10-01T00:00:00Z',
          rightsResolvedThrough: '2026-10-01T00:00:00Z',
          settlementsObservedThrough: '2026-10-01T00:00:00Z',
        },
      },
    });
    const first = projectStatementSnapshotRecord(statement);
    const second = projectStatementSnapshotRecord(statement);
    expect(first.snapshotRoot).toBe(second.snapshotRoot);
    expect(first.grossMinor).toBe('500');
  });
});
