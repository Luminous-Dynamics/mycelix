import { describe, expect, it } from 'vitest';
import { money } from './money.js';
import {
  StatementKind,
  assertCompletenessState,
  assertStatementArithmetic,
  createStatementSnapshot,
  createStatementSuccessor,
  type RoyaltyStatementProjection,
  type RoyaltyStatementSnapshot,
} from './statements.js';

const watermark = {
  usageObservedThrough: '2026-09-30T23:59:59Z',
  rightsResolvedThrough: '2026-09-30T23:59:59Z',
  settlementsObservedThrough: '2026-10-08T12:00:00Z',
} as const;

function statement(): RoyaltyStatementProjection {
  return {
    beneficiaryId: 'creator:alice',
    period: { startInclusive: '2026-09-01T00:00:00Z', endExclusive: '2026-10-01T00:00:00Z' },
    asOf: '2026-10-08T12:00:00Z',
    obligationRoot: 'obl-root',
    adjustmentRoot: 'adj-root',
    settlementRoot: 'set-root',
    gross: money(10000n),
    held: money(1000n),
    deductions: money(500n),
    netPayable: money(8500n),
    paid: money(4000n),
    completeness: { kind: 'partial', through: watermark, missingSources: ['dsp:fourth-source'] },
  };
}

function snapshot(): RoyaltyStatementSnapshot {
  return { ...statement(), statementId: 'statement:s1', kind: StatementKind.Periodic };
}

describe('statement completeness', () => {
  it('preserves missing-source uncertainty', () => {
    expect(() => assertStatementArithmetic(statement())).not.toThrow();
    expect(statement().completeness.kind).toBe('partial');
  });

  it('rejects partial statements that name no missing source', () => {
    expect(() => assertCompletenessState({ kind: 'partial', through: watermark, missingSources: [] }))
      .toThrow(/missing source/);
  });

  it('rejects arithmetic that would hide value', () => {
    const invalid = { ...statement(), netPayable: money(8499n) };
    expect(() => assertStatementArithmetic(invalid)).toThrow(/gross must equal/);
  });
});

describe('immutable statement lineage', () => {
  it('creates a supplement without rewriting the predecessor', () => {
    const original = createStatementSnapshot(snapshot());
    const supplement = createStatementSuccessor(original, {
      ...snapshot(),
      statementId: 'statement:s2',
      kind: StatementKind.Supplemental,
      asOf: '2026-10-10T12:00:00Z',
      gross: money(11000n),
      netPayable: money(9500n),
    });

    expect(supplement.predecessorStatementId).toBe('statement:s1');
    expect(original.gross.amountMinor).toBe(10000n);
    expect(supplement.gross.amountMinor).toBe(11000n);
    expect(Object.isFrozen(original)).toBe(true);
  });

  it('rejects a successor for a different beneficiary', () => {
    const original = createStatementSnapshot(snapshot());
    expect(() => createStatementSuccessor(original, {
      ...snapshot(),
      beneficiaryId: 'creator:bob',
      statementId: 'statement:s2',
      kind: StatementKind.Adjustment,
      asOf: '2026-10-10T12:00:00Z',
    })).toThrow(/beneficiary/);
  });
});
