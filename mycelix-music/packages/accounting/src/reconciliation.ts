import { addMoney, assertSameCurrency, type Money } from './money.js';

export interface ConservationSummary {
  readonly inputTotal: Money;
  readonly distributedTotal: Money;
  readonly heldTotal: Money;
  readonly deductionTotal: Money;
  readonly residualTotal: Money;
}

export interface ReconciliationReceipt {
  readonly inputRoot: string;
  readonly outputRoot: string;
  readonly inputTotal: Money;
  readonly outputTotal: Money;
  readonly heldTotal: Money;
  /** Signed adjustments already reflected in the classified output buckets. */
  readonly adjustmentTotal: Money;
  readonly deductionTotal: Money;
  readonly residualTotal: Money;
  readonly policyDigest: string;
}

function assertNonNegative(label: string, value: Money): void {
  if (value.amountMinor < 0n) throw new Error(`${label} must be non-negative`);
}

export function assertConservation(summary: ConservationSummary): void {
  assertSameCurrency(summary.inputTotal, summary.distributedTotal);
  assertSameCurrency(summary.inputTotal, summary.heldTotal);
  assertSameCurrency(summary.inputTotal, summary.deductionTotal);
  assertSameCurrency(summary.inputTotal, summary.residualTotal);

  assertNonNegative('inputTotal', summary.inputTotal);
  assertNonNegative('distributedTotal', summary.distributedTotal);
  assertNonNegative('heldTotal', summary.heldTotal);
  assertNonNegative('deductionTotal', summary.deductionTotal);
  assertNonNegative('residualTotal', summary.residualTotal);

  const classified = addMoney(
    addMoney(summary.distributedTotal, summary.heldTotal),
    addMoney(summary.deductionTotal, summary.residualTotal),
  );
  if (classified.amountMinor !== summary.inputTotal.amountMinor) {
    throw new Error(
      `economic value does not conserve: ${summary.inputTotal.amountMinor} != ${classified.amountMinor}`,
    );
  }
}

export function assertReconciliationReceipt(receipt: ReconciliationReceipt): void {
  if (!receipt.inputRoot.trim() || !receipt.outputRoot.trim() || !receipt.policyDigest.trim()) {
    throw new Error('reconciliation roots and policyDigest must be non-empty');
  }
  assertSameCurrency(receipt.inputTotal, receipt.adjustmentTotal);
  assertConservation({
    inputTotal: receipt.inputTotal,
    distributedTotal: receipt.outputTotal,
    heldTotal: receipt.heldTotal,
    deductionTotal: receipt.deductionTotal,
    residualTotal: receipt.residualTotal,
  });
}

export function createReconciliationReceipt(
  receipt: ReconciliationReceipt,
): Readonly<ReconciliationReceipt> {
  assertReconciliationReceipt(receipt);
  return Object.freeze({
    ...receipt,
    inputTotal: Object.freeze({ ...receipt.inputTotal }),
    outputTotal: Object.freeze({ ...receipt.outputTotal }),
    heldTotal: Object.freeze({ ...receipt.heldTotal }),
    adjustmentTotal: Object.freeze({ ...receipt.adjustmentTotal }),
    deductionTotal: Object.freeze({ ...receipt.deductionTotal }),
    residualTotal: Object.freeze({ ...receipt.residualTotal }),
  });
}
