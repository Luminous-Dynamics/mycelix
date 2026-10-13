import { type Money } from './money.js';

export enum ValueClass {
  OwnedBalance = 'owned_balance',
  ThirdPartyCustody = 'third_party_custody',
  AccruedObligation = 'accrued_obligation',
  EscrowedValue = 'escrowed_value',
  SuspenseValue = 'suspense_value',
}

export enum AccountingEvent {
  ObligationCreated = 'obligation_created',
  ObligationAdjusted = 'obligation_adjusted',
  ValueHeld = 'value_held',
  ValueReleased = 'value_released',
  SettlementAuthorized = 'settlement_authorized',
  SettlementExecuted = 'settlement_executed',
  SettlementReversed = 'settlement_reversed',
  WithholdingApplied = 'withholding_applied',
  FeeApplied = 'fee_applied',
  FxConverted = 'fx_converted',
}

export interface LedgerPosting {
  readonly accountId: string;
  readonly valueClass: ValueClass;
  readonly amount: Money;
}

export interface LedgerEntry {
  readonly id: string;
  readonly event: AccountingEvent;
  readonly debits: readonly LedgerPosting[];
  readonly credits: readonly LedgerPosting[];
  readonly basis: string;
}

function validatePosting(posting: LedgerPosting): void {
  if (!posting.accountId.trim()) {
    throw new Error('ledger posting accountId must be non-empty');
  }
  if (posting.amount.amountMinor < 0n) {
    throw new Error('ledger posting amount must be non-negative');
  }
  if (!posting.amount.currency.trim()) {
    throw new Error('ledger posting currency must be non-empty');
  }
}

function totalsByCurrency(postings: readonly LedgerPosting[]): Map<string, bigint> {
  const totals = new Map<string, bigint>();
  for (const posting of postings) {
    validatePosting(posting);
    totals.set(
      posting.amount.currency,
      (totals.get(posting.amount.currency) ?? 0n) + posting.amount.amountMinor,
    );
  }
  return totals;
}

export function assertBalancedLedgerEntry(entry: LedgerEntry): void {
  if (!entry.id.trim()) throw new Error('ledger entry id must be non-empty');
  if (!entry.basis.trim()) throw new Error('ledger entry basis must be non-empty');
  if (entry.debits.length === 0 || entry.credits.length === 0) {
    throw new Error('ledger entry requires at least one debit and one credit');
  }

  const debits = totalsByCurrency(entry.debits);
  const credits = totalsByCurrency(entry.credits);
  const currencies = new Set([...debits.keys(), ...credits.keys()]);
  for (const currency of currencies) {
    const debit = debits.get(currency) ?? 0n;
    const credit = credits.get(currency) ?? 0n;
    if (debit !== credit) {
      throw new Error(`unbalanced ledger entry for ${currency}: ${debit} != ${credit}`);
    }
  }
}

export function createLedgerEntry(entry: LedgerEntry): Readonly<LedgerEntry> {
  assertBalancedLedgerEntry(entry);
  return Object.freeze({
    ...entry,
    debits: Object.freeze([...entry.debits]),
    credits: Object.freeze([...entry.credits]),
  });
}
