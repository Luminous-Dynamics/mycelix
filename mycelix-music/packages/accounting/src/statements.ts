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

export enum StatementKind {
  Periodic = 'periodic',
  Supplemental = 'supplemental',
  Adjustment = 'adjustment',
  Reconciliation = 'reconciliation',
}

export interface RoyaltyStatementSnapshot extends RoyaltyStatementProjection {
  readonly statementId: string;
  readonly kind: StatementKind;
  readonly predecessorStatementId?: string;
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

  if (state.kind === 'partial') {
    if (state.missingSources.length === 0) {
      throw new Error('partial completeness requires at least one missing source');
    }
    const normalized = state.missingSources.map(source => source.trim());
    if (normalized.some(source => !source)) {
      throw new Error('partial completeness missing sources must be non-empty');
    }
    if (new Set(normalized).size !== normalized.length) {
      throw new Error('partial completeness missing sources must be unique');
    }
  }
}

function assertCompletenessCoverage(statement: RoyaltyStatementProjection): void {
  if (statement.completeness.kind === 'indeterminate') return;

  const asOf = Date.parse(statement.asOf);
  const periodEnd = Date.parse(statement.period.endExclusive);
  const watermarks = [
    statement.completeness.through.usageObservedThrough,
    statement.completeness.through.rightsResolvedThrough,
    statement.completeness.through.settlementsObservedThrough,
  ].map(Date.parse);

  if (watermarks.some(watermark => watermark > asOf)) {
    throw new Error('statement data watermark cannot be later than asOf');
  }
  if (statement.completeness.kind === 'complete' && watermarks.some(watermark => watermark < periodEnd)) {
    throw new Error('complete statement requires every data watermark through the period end');
  }
}

export function assertStatementArithmetic(statement: RoyaltyStatementProjection): void {
  assertAccountingPeriod(statement.period);
  assertCompletenessState(statement.completeness);
  if (!statement.beneficiaryId.trim()) throw new Error('beneficiaryId must be non-empty');
  if (!Number.isFinite(Date.parse(statement.asOf))) throw new Error('asOf must be a valid timestamp');
  assertCompletenessCoverage(statement);
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

function freezeCompleteness(state: CompletenessState): CompletenessState {
  if (state.kind === 'indeterminate') {
    return Object.freeze({ ...state });
  }
  const through = Object.freeze({ ...state.through });
  if (state.kind === 'complete') return Object.freeze({ kind: 'complete', through });
  return Object.freeze({
    kind: 'partial',
    through,
    missingSources: Object.freeze([...state.missingSources]),
  });
}

export function createStatementSnapshot(
  input: RoyaltyStatementSnapshot,
): Readonly<RoyaltyStatementSnapshot> {
  if (!input.statementId.trim()) throw new Error('statementId must be non-empty');
  if (input.kind === StatementKind.Periodic && input.predecessorStatementId) {
    throw new Error('periodic statement cannot name a predecessor');
  }
  if (input.kind !== StatementKind.Periodic && !input.predecessorStatementId?.trim()) {
    throw new Error('non-periodic statement must name a predecessor');
  }
  assertStatementArithmetic(input);

  return Object.freeze({
    ...input,
    period: Object.freeze({ ...input.period }),
    gross: Object.freeze({ ...input.gross }),
    held: Object.freeze({ ...input.held }),
    deductions: Object.freeze({ ...input.deductions }),
    netPayable: Object.freeze({ ...input.netPayable }),
    paid: Object.freeze({ ...input.paid }),
    completeness: freezeCompleteness(input.completeness),
  });
}

export function createStatementSuccessor(
  predecessor: RoyaltyStatementSnapshot,
  successor: Omit<RoyaltyStatementSnapshot, 'predecessorStatementId'>,
): Readonly<RoyaltyStatementSnapshot> {
  if (successor.kind === StatementKind.Periodic) {
    throw new Error('statement successor must be supplemental, adjustment, or reconciliation');
  }
  if (successor.beneficiaryId !== predecessor.beneficiaryId) {
    throw new Error('statement successor beneficiary must match predecessor');
  }
  if (
    successor.period.startInclusive !== predecessor.period.startInclusive ||
    successor.period.endExclusive !== predecessor.period.endExclusive
  ) {
    throw new Error('statement successor period must match predecessor');
  }
  if (Date.parse(successor.asOf) <= Date.parse(predecessor.asOf)) {
    throw new Error('statement successor asOf must be later than predecessor');
  }

  return createStatementSnapshot({
    ...successor,
    predecessorStatementId: predecessor.statementId,
  });
}
