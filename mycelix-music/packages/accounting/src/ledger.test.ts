import { describe, expect, it } from 'vitest';
import {
  AccountingEvent,
  ValueClass,
  assertBalancedLedgerEntry,
  createLedgerEntry,
} from './ledger.js';
import { money } from './money.js';

describe('balanced accounting ledger', () => {
  it('accepts conservation across distinct value classes', () => {
    const entry = createLedgerEntry({
      id: 'entry-1',
      event: AccountingEvent.ObligationCreated,
      basis: 'authority:agreement-7',
      debits: [{
        accountId: 'economic-pool',
        valueClass: ValueClass.ThirdPartyCustody,
        amount: money(327n),
      }],
      credits: [{
        accountId: 'creator:alice',
        valueClass: ValueClass.AccruedObligation,
        amount: money(327n),
      }],
    });

    expect(entry.credits[0]?.valueClass).toBe(ValueClass.AccruedObligation);
  });

  it('rejects value creation or disappearance', () => {
    expect(() => assertBalancedLedgerEntry({
      id: 'entry-2',
      event: AccountingEvent.FeeApplied,
      basis: 'policy:fee-v1',
      debits: [{
        accountId: 'creator:alice',
        valueClass: ValueClass.AccruedObligation,
        amount: money(100n),
      }],
      credits: [{
        accountId: 'fees',
        valueClass: ValueClass.OwnedBalance,
        amount: money(99n),
      }],
    })).toThrow(/unbalanced ledger entry/);
  });

  it('rejects negative postings', () => {
    expect(() => assertBalancedLedgerEntry({
      id: 'entry-3',
      event: AccountingEvent.ObligationAdjusted,
      basis: 'adjustment:a1',
      debits: [{ accountId: 'a', valueClass: ValueClass.AccruedObligation, amount: money(-1n) }],
      credits: [{ accountId: 'b', valueClass: ValueClass.AccruedObligation, amount: money(-1n) }],
    })).toThrow(/non-negative/);
  });
});
