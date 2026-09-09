import { describe, expect, it } from 'vitest';
import { buildDeterministicNettingBatches } from './netting.js';
import { money } from './money.js';
import { compileRoyaltyStatement } from './projection.js';
import {
  SettlementAttemptState,
  reconstructSettlementRecovery,
} from './recovery.js';
import type { RoyaltyObligation, SettlementEpoch } from './settlement.js';
import { StatementKind } from './statements.js';

const period = {
  startInclusive: '2026-09-01T00:00:00Z',
  endExclusive: '2026-10-01T00:00:00Z',
};
const complete = {
  kind: 'complete' as const,
  through: {
    usageObservedThrough: '2026-10-01T00:00:00Z',
    rightsResolvedThrough: '2026-10-01T00:00:00Z',
    settlementsObservedThrough: '2026-10-01T00:00:00Z',
  },
};
const epoch: SettlementEpoch = {
  id: 'epoch:2026-09',
  cutoff: '2026-09-30T23:59:59Z',
  minimumPayout: money(100n, 'USD'),
};

function obligation(id: string, amountMinor: bigint, extra: Partial<RoyaltyObligation> = {}): RoyaltyObligation {
  return {
    id,
    beneficiaryId: 'artist:1',
    amount: money(amountMinor, 'USD'),
    observedAt: '2026-09-10T12:00:00Z',
    ...extra,
  };
}

function compile(obligations: readonly RoyaltyObligation[], extra: Partial<Parameters<typeof compileRoyaltyStatement>[0]> = {}) {
  return compileRoyaltyStatement({
    statementId: 'statement:2026-09:artist:1',
    kind: StatementKind.Periodic,
    beneficiaryId: 'artist:1',
    period,
    asOf: '2026-10-02T00:00:00Z',
    completeness: complete,
    settlementEpoch: epoch,
    obligations,
    ...extra,
  });
}

describe('royalty statement compiler', () => {
  it('keeps an unavailable payout route as held debt rather than erasing it', () => {
    const statement = compile([obligation('obl:1', 500n, { routeAvailable: false })]);
    expect(statement.gross.amountMinor).toBe(500n);
    expect(statement.held.amountMinor).toBe(500n);
    expect(statement.netPayable.amountMinor).toBe(0n);
    expect(statement.paid.amountMinor).toBe(0n);
  });

  it('carries sub-threshold royalties forward without creating a zero-value batch', () => {
    const statement = compile([obligation('obl:1', 99n)]);
    expect(statement.held.amountMinor).toBe(99n);
    expect(statement.netPayable.amountMinor).toBe(0n);
  });

  it('holds obligations observed after the settlement cutoff for the next epoch', () => {
    const statement = compile([
      obligation('obl:early', 300n, { observedAt: '2026-09-10T00:00:00Z' }),
      obligation('obl:late', 200n, { observedAt: '2026-09-20T00:00:00Z' }),
    ], {
      settlementEpoch: {
        ...epoch,
        cutoff: '2026-09-15T23:59:59Z',
      },
    });
    expect(statement.gross.amountMinor).toBe(500n);
    expect(statement.held.amountMinor).toBe(200n);
    expect(statement.netPayable.amountMinor).toBe(300n);
  });

  it('handles an all-post-cutoff statement without promoting any value to payable', () => {
    const statement = compile([
      obligation('obl:late', 500n, { observedAt: '2026-09-20T00:00:00Z' }),
    ], {
      settlementEpoch: {
        ...epoch,
        cutoff: '2026-09-15T23:59:59Z',
      },
    });
    expect(statement.gross.amountMinor).toBe(500n);
    expect(statement.held.amountMinor).toBe(500n);
    expect(statement.netPayable.amountMinor).toBe(0n);
  });

  it('rejects obligations observed after the immutable statement asOf', () => {
    expect(() => compile([
      obligation('obl:future', 500n, { observedAt: '2026-09-20T00:00:00Z' }),
    ], {
      asOf: '2026-09-15T00:00:00Z',
      completeness: {
        kind: 'partial',
        through: {
          usageObservedThrough: '2026-09-15T00:00:00Z',
          rightsResolvedThrough: '2026-09-15T00:00:00Z',
          settlementsObservedThrough: '2026-09-15T00:00:00Z',
        },
        missingSources: ['usage:future'],
      },
    })).toThrow(/observed after the statement asOf/);
  });

  it('counts value as paid only after settlement finality', () => {
    const obligations = [obligation('obl:1', 500n)];
    const batch = buildDeterministicNettingBatches(obligations, epoch)[0]!;
    const inFlight = reconstructSettlementRecovery(batch, [{
      attemptId: 'attempt:1',
      batchId: batch.batchId,
      obligationSetRoot: batch.obligationSetRoot,
      state: SettlementAttemptState.Submitted,
      observedAt: '2026-10-01T00:01:00Z',
    }]);
    expect(compile(obligations, {
      settlements: [{ batch, recovery: inFlight, settledAmount: money(500n, 'USD') }],
    }).paid.amountMinor).toBe(0n);

    const finalized = reconstructSettlementRecovery(batch, [
      {
        attemptId: 'attempt:1',
        batchId: batch.batchId,
        obligationSetRoot: batch.obligationSetRoot,
        state: SettlementAttemptState.Submitted,
        observedAt: '2026-10-01T00:01:00Z',
      },
      {
        attemptId: 'attempt:1',
        batchId: batch.batchId,
        obligationSetRoot: batch.obligationSetRoot,
        state: SettlementAttemptState.Finalized,
        observedAt: '2026-10-01T00:02:00Z',
        railReceiptRef: 'rail:receipt:1',
      },
    ]);
    expect(compile(obligations, {
      settlements: [{ batch, recovery: finalized, settledAmount: money(500n, 'USD') }],
    }).paid.amountMinor).toBe(500n);
  });

  it('rejects settlement evidence from a different epoch even for the same obligations', () => {
    const obligations = [obligation('obl:1', 500n)];
    const otherEpoch = { ...epoch, id: 'epoch:other' };
    const batch = buildDeterministicNettingBatches(obligations, otherEpoch)[0]!;
    const recovery = reconstructSettlementRecovery(batch, [{
      attemptId: 'attempt:1',
      batchId: batch.batchId,
      obligationSetRoot: batch.obligationSetRoot,
      state: SettlementAttemptState.Finalized,
      observedAt: '2026-10-01T00:02:00Z',
      railReceiptRef: 'rail:receipt:other',
    }]);
    expect(() => compile(obligations, {
      settlements: [{ batch, recovery, settledAmount: money(500n, 'USD') }],
    })).toThrow(/epoch mismatch/);
  });

  it('rejects settlement observations that occurred after statement asOf', () => {
    const obligations = [obligation('obl:1', 500n)];
    const batch = buildDeterministicNettingBatches(obligations, epoch)[0]!;
    const recovery = reconstructSettlementRecovery(batch, [{
      attemptId: 'attempt:1',
      batchId: batch.batchId,
      obligationSetRoot: batch.obligationSetRoot,
      state: SettlementAttemptState.Finalized,
      observedAt: '2026-10-03T00:00:00Z',
      railReceiptRef: 'rail:receipt:future',
    }]);
    expect(() => compile(obligations, {
      settlements: [{ batch, recovery, settledAmount: money(500n, 'USD') }],
    })).toThrow(/later than statement asOf/);
  });

  it('produces roots independent of source input order', () => {
    const a = compile([obligation('obl:b', 200n), obligation('obl:a', 300n)]);
    const b = compile([obligation('obl:a', 300n), obligation('obl:b', 200n)]);
    expect(a.obligationRoot).toBe(b.obligationRoot);
  });

  it('rejects obligations outside the statement period instead of silently dropping them', () => {
    expect(() => compile([
      obligation('obl:late', 500n, { observedAt: '2026-10-01T00:00:00Z' }),
    ])).toThrow(/outside the statement period/);
  });
});
