import { addMoney, assertSameCurrency, type Money } from './money.js';

export interface AccountingPeriod {
  readonly startInclusive: string;
  readonly endExclusive: string;
}

export interface DataWatermark {
  readonly usageObservedThrough: string;
  readonly rightsResolvedThrough: string;
  readonly settlementsObservedThrough: string;
}

export type CompletenessState =
  | {
      readonly kind: 'complete';
      readonly through: DataWatermark;
    }
  | {
      readonly kind: 'partial';
      readonly through: DataWatermark;
      readonly missingSources: readonly string[];
    }
  | {
      readonly kind: 'indeterminate';
      readonly reason: string;
    };

export interface RoyaltyStatementProjection {
  readonly beneficiaryId: string;
  readonly period: AccountingPeriod;
  readonly asOf: string;
  readonly obligationRoot: string;
  readonly adjustmentRoot: string;
  readonly settlementRoot: string;
  readonly gross: Money;
  readonly held: Money;
  readonly deductions: Money;
  readonly netPayable: Money;
  readonly paid: Money;
  readonly completeness: CompletenessState;
}

export function assertAccountingPeriod(period: AccountingPeriod): void {
  const start = Date.parse(period.startInclusive);
  const end = Date.parse(period.endExclusive);
  if (!Number.isFinite(start) || !Number.isFinite(end) || start >= end) {
    throw new Error('accounting period must have valid startInclusive < endExclusive');
  }
}

export function assertCompletenessState(state: CompletenessState): void {
  if (state.kind === 'indeterminate') {
    if (!state.reason.trim()) throw new Error('indeterminate completeness requires a reason');
    return;
  }

  const values = [
    state.through.usageObservedThrough,
    state.through.rightsResolvedThrough,
    state.through.settlementsObservedThrough,
  ];
  if (values.some(value => !Number.isFinite(Date.parse(value)))) {
    throw new Error('data watermarks must be valid timestamps');
  }

  if (state.kind === 'partial' && state.missingSources.length === 0) {
    throw new Error('partial completeness requires at least one missing source');
  }
}

export function assertStatementArithmetic(statement: RoyaltyStatementProjection): void {
  assertAccountingPeriod(statement.period);
  assertCompletenessState(statement.completeness);
  if (!statement.beneficiaryId.trim()) throw new Error('beneficiaryId must be non-empty');
  if (!Number.isFinite(Date.parse(statement.asOf))) throw new Error('asOf must be a valid timestamp');
  if (!statement.obligationRoot.trim() || !statement.adjustmentRoot.trim() || !statement.settlementRoot.trim()) {
    throw new Error('statement roots must be non-empty');
  }

  assertSameCurrency(statement.gross, statement.held);
  assertSameCurrency(statement.gross, statement.deductions);
  assertSameCurrency(statement.gross, statement.netPayable);
  assertSameCurrency(statement.gross, statement.paid);

  const accounted = addMoney(addMoney(statement.held, statement.deductions), statement.netPayable);
  if (accounted.amountMinor !== statement.gross.amountMinor) {
    throw new Error('statement gross must equal held + deductions + netPayable');
  }
  if (statement.paid.amountMinor < 0n || statement.paid.amountMinor > statement.netPayable.amountMinor) {
    throw new Error('paid must be between zero and netPayable');
  }
}
