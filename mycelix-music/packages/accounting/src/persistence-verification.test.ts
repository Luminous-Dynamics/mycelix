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
import {
  verifyPersistedDeduction,
  verifyPersistedEligibilityObservation,
  verifyPersistedRoyaltyObligation,
  verifyPersistedSettlementObservation,
  verifyPersistedStatementSnapshot,
} from './persistence-verification.js';
import { SettlementAttemptState } from './recovery.js';
import { SettlementEligibilityCode } from './settlement.js';
import { createStatementSnapshot, StatementKind } from './statements.js';

const obligation = createRoyaltyObligationAuthority({
  id: 'obl:1',
  beneficiaryId: 'creator:alice',
  amount: money(500n, 'USD'),
  observedAt: '2026-09-10T00:00:00Z',
  provenance: {
    usageEvidenceRef: 'usage:epoch:1',
    rightsResolutionRef: 'rights:resolution:1',
    economicTermsRef: 'terms:v1',
  },
});

describe('persisted accounting evidence verification', () => {
  it('reconstructs and verifies obligation authority from Prisma-style timestamps', () => {
    const persisted = projectRoyaltyObligationRecord(obligation);
    const verified = verifyPersistedRoyaltyObligation({
      ...persisted,
      observedAt: new Date(persisted.observedAt),
    });
    expect(verified.authorityRoot).toBe(obligation.authorityRoot);
  });

  it('detects obligation content changed without recomputing its authority root', () => {
    const persisted = projectRoyaltyObligationRecord(obligation);
    expect(() => verifyPersistedRoyaltyObligation({ ...persisted, amountMinor: '501' }))
      .toThrow(/canonical projection at obligationRoot/);
  });

  it('detects eligibility source provenance changed behind a stored evidence root', () => {
    const persisted = projectEligibilityObservationRecord({
      id: 'elig:1',
      obligationId: obligation.id,
      code: SettlementEligibilityCode.Eligible,
      sourceRef: 'eligibility-authority:v1',
      observedAt: '2026-09-10T00:01:00Z',
    });
    expect(() => verifyPersistedEligibilityObservation({ ...persisted, sourceRef: 'eligibility-authority:tampered' }))
      .toThrow(/canonical projection at observationRoot/);
  });

  it('detects deduction authority substitution', () => {
    const persisted = projectDeductionRecord({
      id: 'deduction:1',
      beneficiaryId: 'creator:alice',
      amount: money(50n, 'USD'),
      basis: 'tax:withholding:v1',
      observedAt: '2026-09-10T00:02:00Z',
    }, 'tax-authority:notice:1');
    expect(() => verifyPersistedDeduction({ ...persisted, authorityRef: 'tax-authority:notice:other' }))
      .toThrow(/canonical projection at deductionRoot/);
  });

  it('detects settlement eligibility-snapshot substitution', () => {
    const persisted = projectSettlementObservationRecord('settlement-observation:1', {
      attemptId: 'attempt:1',
      batchId: 'batch:1',
      obligationSetRoot: 'a'.repeat(64),
      eligibilityAsOf: '2026-09-10T00:03:00Z',
      eligibilityEvidenceRoot: 'b'.repeat(64),
      state: SettlementAttemptState.Submitted,
      observedAt: '2026-09-10T00:04:00Z',
    });
    expect(() => verifyPersistedSettlementObservation({
      ...persisted,
      eligibilityEvidenceRoot: 'c'.repeat(64),
    })).toThrow(/canonical projection at observationRoot/);
  });

  it('rejects persisted finality that lacks a durable rail receipt', () => {
    const persisted = projectSettlementObservationRecord('settlement-observation:final', {
      attemptId: 'attempt:1',
      batchId: 'batch:1',
      obligationSetRoot: 'a'.repeat(64),
      eligibilityAsOf: '2026-09-10T00:03:00Z',
      eligibilityEvidenceRoot: 'b'.repeat(64),
      state: SettlementAttemptState.Finalized,
      observedAt: '2026-09-10T00:04:00Z',
      railReceiptRef: 'rail:receipt:1',
    });
    const forged = { ...persisted } as Record<string, unknown>;
    delete forged.railReceiptRef;
    expect(() => verifyPersistedSettlementObservation(forged as unknown as typeof persisted))
      .toThrow(/requires railReceiptRef/);
  });

  it('rechecks statement arithmetic and immutable snapshot commitment', () => {
    const statement = createStatementSnapshot({
      statementId: 'statement:1',
      kind: StatementKind.Periodic,
      beneficiaryId: 'creator:alice',
      period: { startInclusive: '2026-09-01T00:00:00Z', endExclusive: '2026-10-01T00:00:00Z' },
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
    const persisted = projectStatementSnapshotRecord(statement);
    expect(verifyPersistedStatementSnapshot(persisted).statementId).toBe('statement:1');
    expect(() => verifyPersistedStatementSnapshot({ ...persisted, netPayableMinor: '349' }))
      .toThrow(/gross must equal held \+ deductions \+ netPayable/);
    expect(() => verifyPersistedStatementSnapshot({ ...persisted, snapshotRoot: 'f'.repeat(64) }))
      .toThrow(/canonical projection at snapshotRoot/);
  });
});
