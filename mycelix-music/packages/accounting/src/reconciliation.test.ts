import { describe, expect, it } from 'vitest';
import { money } from './money.js';
import { assertConservation, createReconciliationReceipt } from './reconciliation.js';

describe('economic conservation and reconciliation', () => {
  it('proves every unit is distributed, held, deducted, or residual', () => {
    expect(() => assertConservation({
      inputTotal: money(10000n),
      distributedTotal: money(8100n),
      heldTotal: money(1000n),
      deductionTotal: money(500n),
      residualTotal: money(400n),
    })).not.toThrow();
  });

  it('refuses a one-unit disappearance', () => {
    expect(() => assertConservation({
      inputTotal: money(10000n),
      distributedTotal: money(8099n),
      heldTotal: money(1000n),
      deductionTotal: money(500n),
      residualTotal: money(400n),
    })).toThrow(/does not conserve/);
  });

  it('keeps adjustments visible without counting them twice', () => {
    const receipt = createReconciliationReceipt({
      inputRoot: 'usage-economic-root',
      outputRoot: 'obligation-root',
      inputTotal: money(10000n),
      outputTotal: money(8100n),
      heldTotal: money(1000n),
      adjustmentTotal: money(-50n),
      deductionTotal: money(500n),
      residualTotal: money(400n),
      policyDigest: 'policy:v7:digest',
    });
    expect(receipt.adjustmentTotal.amountMinor).toBe(-50n);
  });
});
