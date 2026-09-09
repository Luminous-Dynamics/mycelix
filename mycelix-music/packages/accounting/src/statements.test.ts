import { describe, expect, it } from 'vitest';
import { money } from './money.js';
import {
  assertCompletenessState,
  assertStatementArithmetic,
  type RoyaltyStatementProjection,
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
