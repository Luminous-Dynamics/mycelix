// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

import test from 'node:test';
import { readFile } from 'node:fs/promises';
import { fileURLToPath } from 'node:url';
import { dirname, resolve } from 'node:path';
import assert from 'node:assert/strict';
import { analyzeBusinessModel, compareCashPlanToActuals, compareCashRunwayScenarios, DEFAULT_CASH_TIMING_SCHEDULE, formatMinorUnits, projectCashRunway, projectCashRunwayWithTimingSchedule } from '../src/index.js';

const completeModel = (overrides = {}) => ({
  currency: { code: 'ZAR', minorUnitExponent: 2 },
  sellingPricePerUnitMinor: 10_000,
  variableCostPerUnitMinor: 6_000,
  fixedMonthlyCostsMinor: 100_000,
  plannedUnitsPerMonth: 300,
  setupCostsMinor: 400_000,
  initialWorkingCapitalBufferMinor: 100_000,
  availableCashForStartupMinor: 300_000,
  costCoverage: { level: 'complete_for_declared_scope', omittedCategories: [] },
  ...overrides
});

test('calculates contribution, break-even, monthly result and startup funding gap with integer minor units', () => {
  const result = analyzeBusinessModel(completeModel());
  assert.equal(result.ok, true);
  assert.deepEqual(result.report.unitEconomics, {
    sellingPricePerUnitMinor: 10_000,
    variableCostPerUnitMinor: 6_000,
    contributionPerUnitMinor: 4_000
  });
  assert.deepEqual(result.report.monthlyScenario, {
    revenueMinor: 3_000_000,
    variableCostsMinor: 1_800_000,
    contributionMinor: 1_200_000,
    fixedCostsMinor: 100_000,
    operatingResultBeforeExcludedItemsMinor: 1_100_000
  });
  assert.deepEqual(result.report.breakEven, { status: 'reachable', unitsPerMonth: 25, reason: null });
  assert.deepEqual(result.report.startup, {
    setupCostsMinor: 400_000,
    workingCapitalBufferMinor: 100_000,
    requiredCashMinor: 500_000,
    availableCashMinor: 300_000,
    fundingGapMinor: 200_000
  });
  assert.ok(
    result.report.scopeLimitations.some(message => message.includes('single-offer model')),
    'the report must disclose that product/service mix is not modeled'
  );
  assert.ok(
    result.report.scopeLimitations.some(message => message.includes('extra runway for slow sales or operating losses')),
    'the report must not imply its funding gap calculates runway'
  );
});

test('rounds break-even units upward instead of understating required sales', () => {
  const result = analyzeBusinessModel(completeModel({ fixedMonthlyCostsMinor: 100_001 }));
  assert.equal(result.ok, true);
  assert.equal(result.report.breakEven.unitsPerMonth, 26);
});

test('returns explicit unreachable break-even when per-unit contribution is zero', () => {
  const result = analyzeBusinessModel(completeModel({ variableCostPerUnitMinor: 10_000 }));
  assert.equal(result.ok, true);
  assert.deepEqual(result.report.breakEven, { status: 'unreachable', unitsPerMonth: null, reason: 'unit_contribution_not_positive' });
  assert.equal(result.report.monthlyScenario.operatingResultBeforeExcludedItemsMinor, -100_000);
});

test('returns explicit unreachable break-even when each sale loses money', () => {
  const result = analyzeBusinessModel(completeModel({ variableCostPerUnitMinor: 12_000 }));
  assert.equal(result.ok, true);
  assert.equal(result.report.unitEconomics.contributionPerUnitMinor, -2_000);
  assert.equal(result.report.breakEven.status, 'unreachable');
  assert.equal(result.report.monthlyScenario.operatingResultBeforeExcludedItemsMinor, -700_000);
});

test('does not assume an incomplete cost model is complete', () => {
  const result = analyzeBusinessModel(completeModel({
    costCoverage: { level: 'partial', omittedCategories: ['owner pay', 'insurance'] }
  }));
  assert.equal(result.ok, true);
  assert.equal(result.report.costCoverage.level, 'partial');
  assert.deepEqual(result.report.costCoverage.omittedCategories, ['owner pay', 'insurance']);
});

test('rejects partial costs when omitted categories are not named', () => {
  const result = analyzeBusinessModel(completeModel({
    costCoverage: { level: 'partial', omittedCategories: [] }
  }));
  assert.equal(result.ok, false);
  assert.equal(result.error.code, 'incomplete_costs_not_described');
});

test('rejects contradictory complete-cost declaration', () => {
  const result = analyzeBusinessModel(completeModel({
    costCoverage: { level: 'complete_for_declared_scope', omittedCategories: ['insurance'] }
  }));
  assert.equal(result.ok, false);
  assert.equal(result.error.code, 'contradictory_cost_coverage');
});

test('does not claim break-even from zero price and zero variable cost', () => {
  const result = analyzeBusinessModel(completeModel({ sellingPricePerUnitMinor: 0, variableCostPerUnitMinor: 0 }));
  assert.equal(result.ok, true);
  assert.equal(result.report.breakEven.status, 'unreachable');
});

test('reports no upfront funding gap when available cash meets requirement', () => {
  const result = analyzeBusinessModel(completeModel({ availableCashForStartupMinor: 500_000 }));
  assert.equal(result.ok, true);
  assert.equal(result.report.startup.fundingGapMinor, 0);
});

test('fixed costs of zero and positive contribution require zero units to cover fixed costs', () => {
  const result = analyzeBusinessModel(completeModel({ fixedMonthlyCostsMinor: 0 }));
  assert.equal(result.ok, true);
  assert.equal(result.report.breakEven.status, 'reachable');
  assert.equal(result.report.breakEven.unitsPerMonth, 0);
});

test('rejects malformed currency code instead of asserting it is valid', () => {
  const result = analyzeBusinessModel(completeModel({ currency: { code: 'zar', minorUnitExponent: 2 } }));
  assert.equal(result.ok, false);
  assert.equal(result.error.code, 'invalid_currency_code');
});

test('rejects unsafe numeric inputs', () => {
  const result = analyzeBusinessModel(completeModel({ sellingPricePerUnitMinor: Number.MAX_SAFE_INTEGER + 1 }));
  assert.equal(result.ok, false);
  assert.equal(result.error.code, 'invalid_amount');
});

test('fails closed when products exceed exact JavaScript integer range', () => {
  const result = analyzeBusinessModel(completeModel({
    sellingPricePerUnitMinor: 9_000_000_000_000_000,
    variableCostPerUnitMinor: 0,
    plannedUnitsPerMonth: 2
  }));
  assert.equal(result.ok, false);
  assert.equal(result.error.code, 'calculation_overflow');
});

test('projects independently timed cash receipts and operating payments', () => {
  const result = projectCashRunway({
    model: completeModel(),
    monthlyPlan: [
      { salesUnits: 0, salesCashReceiptsMinor: 0, operatingCashOutflowsMinor: 100_000, otherCashInflowsMinor: 0, otherCashOutflowsMinor: 0 },
      { salesUnits: 50, salesCashReceiptsMinor: 500_000, operatingCashOutflowsMinor: 400_000, otherCashInflowsMinor: 0, otherCashOutflowsMinor: 0 },
      { salesUnits: 100, salesCashReceiptsMinor: 1_000_000, operatingCashOutflowsMinor: 700_000, otherCashInflowsMinor: 25_000, otherCashOutflowsMinor: 10_000 }
    ]
  });
  assert.equal(result.ok, true);
  assert.equal(result.report.kind, 'business-launchpad-cash-runway-report');
  assert.equal(result.report.startup.cashAfterSetupBeforeMonthOneMinor, -100_000);
  assert.deepEqual(result.report.periods.map(period => period.closingCashMinor), [-200_000, -100_000, 215_000]);
  assert.equal(result.report.periods[0].salesCashReceiptsMinor, 0);
  assert.equal(result.report.periods[0].operatingCashOutflowsMinor, 100_000);
  assert.equal(result.report.periods[1].salesUnitsScenario, 50);
  assert.equal(result.report.summary.minimumProjectedCashMinor, -200_000);
  assert.equal(result.report.summary.minimumCashPeriod, 1);
  assert.equal(result.report.summary.firstNegativeCashPeriod, 0);
  assert.equal(result.report.summary.firstBelowReservePeriod, 0);
  assert.equal(result.report.summary.additionalCashNeededToMaintainReserveMinor, 300_000);
  assert.deepEqual(result.report.summary.periodsBelowReserve, [0, 1, 2]);
  assert.deepEqual(result.report.summary.periodsWithNegativeCash, [0, 1, 2]);
});

test('sales activity does not automatically become cash receipts, and cash payments can precede collections', () => {
  const result = projectCashRunway({
    model: completeModel({
      setupCostsMinor: 0,
      availableCashForStartupMinor: 200_000,
      initialWorkingCapitalBufferMinor: 50_000
    }),
    monthlyPlan: [
      {
        salesUnits: 100,
        salesCashReceiptsMinor: 0,
        operatingCashOutflowsMinor: 160_000,
        otherCashInflowsMinor: 0,
        otherCashOutflowsMinor: 0
      },
      {
        salesUnits: 0,
        salesCashReceiptsMinor: 1_000_000,
        operatingCashOutflowsMinor: 0,
        otherCashInflowsMinor: 0,
        otherCashOutflowsMinor: 0
      }
    ]
  });
  assert.equal(result.ok, true);
  assert.equal(result.report.periods[0].salesUnitsScenario, 100);
  assert.equal(result.report.periods[0].salesCashReceiptsMinor, 0);
  assert.equal(result.report.periods[0].operatingCashOutflowsMinor, 160_000);
  assert.equal(result.report.periods[0].closingCashMinor, 40_000);
  assert.equal(result.report.periods[1].salesUnitsScenario, 0);
  assert.equal(result.report.periods[1].salesCashReceiptsMinor, 1_000_000);
  assert.equal(result.report.periods[1].closingCashMinor, 1_040_000);
  assert.equal(result.report.summary.firstBelowReservePeriod, 1);
});

test('separates operating, investing and financing cash flows and reconciles to closing cash', () => {
  const result = projectCashRunway({
    model: completeModel({
      setupCostsMinor: 0,
      availableCashForStartupMinor: 1_000,
      initialWorkingCapitalBufferMinor: 0
    }),
    monthlyPlan: [
      {
        salesUnits: 0,
        salesCashReceiptsMinor: 100,
        operatingCashOutflowsMinor: 40,
        otherCashInflowsMinor: 5,
        otherCashOutflowsMinor: 10,
        investingCashInflowsMinor: 20,
        investingCashOutflowsMinor: 50,
        financingCashInflowsMinor: 300,
        financingCashOutflowsMinor: 25
      },
      {
        salesUnits: 0,
        salesCashReceiptsMinor: 0,
        operatingCashOutflowsMinor: 0,
        otherCashInflowsMinor: 0,
        otherCashOutflowsMinor: 0,
        investingCashInflowsMinor: 80,
        investingCashOutflowsMinor: 0,
        financingCashInflowsMinor: 0,
        financingCashOutflowsMinor: 100
      }
    ]
  });
  assert.equal(result.ok, true);
  assert.equal(result.report.schemaVersion, 2);
  assert.equal(result.report.periods[0].totalOperatingCashReceiptsMinor, 100);
  assert.equal(result.report.periods[0].totalOperatingCashOutflowsMinor, 40);
  assert.equal(result.report.periods[0].netOperatingCashFlowMinor, 60);
  assert.equal(result.report.periods[0].netUnclassifiedCashFlowMinor, -5);
  assert.equal(result.report.periods[0].netInvestingCashFlowMinor, -30);
  assert.equal(result.report.periods[0].netFinancingCashFlowMinor, 275);
  assert.equal(result.report.periods[0].totalCashReceiptsMinor, 425);
  assert.equal(result.report.periods[0].totalCashOutflowsMinor, 125);
  assert.equal(result.report.periods[0].netCashChangeMinor, 300);
  assert.equal(result.report.periods[0].closingCashMinor, 1_300);
  assert.equal(result.report.periods[1].netInvestingCashFlowMinor, 80);
  assert.equal(result.report.periods[1].netFinancingCashFlowMinor, -100);
  assert.equal(result.report.periods[1].netCashChangeMinor, -20);
  assert.equal(result.report.periods[1].closingCashMinor, 1_280);
});

test('rejects invalid investing or financing cash amounts rather than silently coercing them', () => {
  const model = completeModel();
  for (const field of [
    'investingCashInflowsMinor',
    'investingCashOutflowsMinor',
    'financingCashInflowsMinor',
    'financingCashOutflowsMinor'
  ]) {
    const result = projectCashRunway({
      model,
      monthlyPlan: [{
        salesUnits: 0,
        otherCashInflowsMinor: 0,
        otherCashOutflowsMinor: 0,
        [field]: -1
      }]
    });
    assert.equal(result.ok, false, field);
    assert.equal(result.error.code, 'invalid_cash_runway_amount', field);
    assert.match(result.error.field, new RegExp(field), field);
  }
});

test('compares base, downside and upside cash scenarios with explicit assumptions', () => {
  const model = completeModel({
    setupCostsMinor: 0,
    availableCashForStartupMinor: 5_000,
    initialWorkingCapitalBufferMinor: 1_000
  });
  const monthlyPlan = [
    {
      salesUnits: 10,
      salesCashReceiptsMinor: 1_000,
      operatingCashOutflowsMinor: 1_000,
      otherCashInflowsMinor: 0,
      otherCashOutflowsMinor: 0,
      investingCashInflowsMinor: 0,
      investingCashOutflowsMinor: 0,
      financingCashInflowsMinor: 0,
      financingCashOutflowsMinor: 0
    },
    {
      salesUnits: 10,
      salesCashReceiptsMinor: 1_000,
      operatingCashOutflowsMinor: 1_000,
      otherCashInflowsMinor: 0,
      otherCashOutflowsMinor: 0,
      investingCashInflowsMinor: 0,
      investingCashOutflowsMinor: 0,
      financingCashInflowsMinor: 0,
      financingCashOutflowsMinor: 0
    }
  ];
  const result = compareCashRunwayScenarios({ model, monthlyPlan });
  assert.equal(result.ok, true);
  assert.equal(result.report.kind, 'business-launchpad-cash-scenario-comparison');
  assert.deepEqual(result.report.scenarios.map(scenario => scenario.id), ['downside', 'base', 'upside']);
  assert.equal(result.report.scenarios[0].report.periods[0].salesCashReceiptsMinor, 0);
  assert.equal(result.report.scenarios[0].report.periods[0].operatingCashOutflowsMinor, 1_150);
  assert.equal(result.report.scenarios[0].report.periods[1].salesCashReceiptsMinor, 800);
  assert.equal(result.report.scenarios[0].deferredSalesCashReceiptsAfterHorizonMinor, 800);
  assert.equal(result.report.scenarios[1].report.summary.endingCashMinor, 5_000);
  assert.equal(result.report.scenarios[2].report.periods[0].salesCashReceiptsMinor, 2_200);
  assert.equal(result.report.scenarios[2].report.periods[0].operatingCashOutflowsMinor, 950);
  assert.equal(result.report.scenarios[2].report.summary.endingCashMinor, 5_300);
  assert.match(result.report.basis, /no scenario is a prediction/);
});

test('zero stress assumptions reproduce the base cash projection exactly', () => {
  const model = completeModel({ setupCostsMinor: 0, availableCashForStartupMinor: 10_000 });
  const monthlyPlan = [{
    salesUnits: 1,
    salesCashReceiptsMinor: 123,
    operatingCashOutflowsMinor: 45,
    otherCashInflowsMinor: 2,
    otherCashOutflowsMinor: 3,
    investingCashInflowsMinor: 4,
    investingCashOutflowsMinor: 5,
    financingCashInflowsMinor: 6,
    financingCashOutflowsMinor: 7
  }];
  const assumptions = {
    downside: {
      salesReceiptReductionPercent: 0,
      salesReceiptDelayMonths: 0,
      operatingCashOutflowIncreasePercent: 0
    },
    upside: {
      salesReceiptIncreasePercent: 0,
      salesReceiptAdvanceMonths: 0,
      operatingCashOutflowReductionPercent: 0
    }
  };
  const result = compareCashRunwayScenarios({ model, monthlyPlan, assumptions });
  assert.equal(result.ok, true);
  const base = result.report.scenarios.find(scenario => scenario.id === 'base').report;
  for (const id of ['downside', 'upside']) {
    const stress = result.report.scenarios.find(scenario => scenario.id === id).report;
    assert.equal(stress.summary.endingCashMinor, base.summary.endingCashMinor, id);
    assert.equal(stress.summary.minimumProjectedCashMinor, base.summary.minimumProjectedCashMinor, id);
    assert.equal(stress.periods[0].netCashChangeMinor, base.periods[0].netCashChangeMinor, id);
    assert.equal(stress.periods[0].closingCashMinor, base.periods[0].closingCashMinor, id);
  }
});

test('stress and terms-based cash comparison preserve reconciliation across a fixed-seed corpus', () => {
  let seed = 0x5ce7e55;
  const next = maxExclusive => {
    seed = (Math.imul(seed, 1664525) + 1013904223) >>> 0;
    return seed % maxExclusive;
  };

  for (let caseIndex = 0; caseIndex < 100; caseIndex += 1) {
    const model = completeModel({
      setupCostsMinor: next(20_000),
      availableCashForStartupMinor: next(50_000),
      initialWorkingCapitalBufferMinor: next(10_000)
    });
    const horizon = 1 + next(8);
    const monthlyPlan = Array.from({ length: horizon }, () => ({
      salesUnits: next(300),
      salesCashReceiptsMinor: next(30_000),
      operatingCashOutflowsMinor: next(25_000),
      otherCashInflowsMinor: next(3_000),
      otherCashOutflowsMinor: next(4_000),
      investingCashInflowsMinor: next(5_000),
      investingCashOutflowsMinor: next(6_000),
      financingCashInflowsMinor: next(8_000),
      financingCashOutflowsMinor: next(7_000)
    }));
    const receiptCurrent = next(101);
    const receiptNext = next(101 - receiptCurrent);
    const supplierCurrent = next(101);
    const supplierNext = next(101 - supplierCurrent);
    const timingSchedule = {
      salesReceiptsPercent: [receiptCurrent, receiptNext, 100 - receiptCurrent - receiptNext],
      variableSupplierPaymentsPercent: [supplierCurrent, supplierNext, 100 - supplierCurrent - supplierNext]
    };
    const assumptions = {
      downside: {
        salesReceiptReductionPercent: next(101),
        salesReceiptDelayMonths: next(4),
        operatingCashOutflowIncreasePercent: next(101)
      },
      upside: {
        salesReceiptIncreasePercent: next(101),
        salesReceiptAdvanceMonths: next(4),
        operatingCashOutflowReductionPercent: next(101)
      }
    };
    const result = compareCashRunwayScenarios({ model, monthlyPlan, assumptions, timingSchedule });
    assert.equal(result.ok, true, 'stress corpus case ' + caseIndex);
    assert.deepEqual(result.report.scenarios.map(item => item.id), ['downside', 'base', 'upside', 'terms-based']);

    for (const scenario of result.report.scenarios) {
      let expectedCash = BigInt(scenario.report.startup.cashAfterSetupBeforeMonthOneMinor);
      for (let index = 0; index < horizon; index += 1) {
        const period = scenario.report.periods[index];
        const expectedNet =
          BigInt(period.netOperatingCashFlowMinor) +
          BigInt(period.netUnclassifiedCashFlowMinor) +
          BigInt(period.netInvestingCashFlowMinor) +
          BigInt(period.netFinancingCashFlowMinor);
        assert.equal(period.netCashChangeMinor, Number(expectedNet),
          'category-to-total identity case ' + caseIndex + ' period ' + index + ' scenario ' + scenario.id);
        assert.equal(period.totalCashReceiptsMinor,
          period.salesCashReceiptsMinor + period.otherCashInflowsMinor +
          period.investingCashInflowsMinor + period.financingCashInflowsMinor,
          'receipt total case ' + caseIndex + ' period ' + index + ' scenario ' + scenario.id);
        assert.equal(period.totalCashOutflowsMinor,
          period.operatingCashOutflowsMinor + period.otherCashOutflowsMinor +
          period.investingCashOutflowsMinor + period.financingCashOutflowsMinor,
          'payment total case ' + caseIndex + ' period ' + index + ' scenario ' + scenario.id);
        expectedCash += expectedNet;
        assert.equal(period.closingCashMinor, Number(expectedCash),
          'closing cash recurrence case ' + caseIndex + ' period ' + index + ' scenario ' + scenario.id);
        assert.equal(period.salesUnitsScenario, monthlyPlan[index].salesUnits,
          'stress must not fabricate a change in activity case ' + caseIndex + ' period ' + index + ' scenario ' + scenario.id);
      }
      assert.equal(scenario.report.summary.endingCashMinor, Number(expectedCash),
        'ending cash case ' + caseIndex + ' scenario ' + scenario.id);
    }

    const terms = result.report.scenarios[3];
    const allocate = (amount, percentages) => {
      const buckets = percentages.map((percent, index) => ({
        index,
        amount: Math.floor(amount * percent / 100),
        remainder: (amount * percent) % 100
      }));
      let remaining = amount - buckets.reduce((sum, bucket) => sum + bucket.amount, 0);
      const priority = [...buckets].sort((left, right) =>
        left.remainder === right.remainder
          ? left.index - right.index
          : right.remainder - left.remainder
      );
      for (let index = 0; remaining > 0; index += 1, remaining -= 1) {
        priority[index % priority.length].amount += 1;
      }
      return buckets.sort((left, right) => left.index - right.index).map(bucket => bucket.amount);
    };
    const expectedScheduledReceipts = Array(horizon).fill(0);
    const expectedScheduledSupplierPayments = Array(horizon).fill(0);
    let deferredScheduledReceipts = 0;
    let deferredScheduledSupplierPayments = 0;
    monthlyPlan.forEach((period, cohort) => {
      const receiptBuckets = allocate(model.sellingPricePerUnitMinor * period.salesUnits, timingSchedule.salesReceiptsPercent);
      const supplierBuckets = allocate(model.variableCostPerUnitMinor * period.salesUnits, timingSchedule.variableSupplierPaymentsPercent);
      for (let offset = 0; offset < 3; offset += 1) {
        if (cohort + offset >= horizon) {
          deferredScheduledReceipts += receiptBuckets[offset];
          deferredScheduledSupplierPayments += supplierBuckets[offset];
        } else {
          expectedScheduledReceipts[cohort + offset] += receiptBuckets[offset];
          expectedScheduledSupplierPayments[cohort + offset] += supplierBuckets[offset];
        }
      }
    });
    assert.equal(
      expectedScheduledReceipts.reduce((sum, amount) => sum + amount, 0) + deferredScheduledReceipts,
      monthlyPlan.reduce((sum, period) => sum + model.sellingPricePerUnitMinor * period.salesUnits, 0),
      'scheduled gross receipts conserve every sale cohort in case ' + caseIndex
    );
    assert.equal(
      expectedScheduledSupplierPayments.reduce((sum, amount) => sum + amount, 0) + deferredScheduledSupplierPayments,
      monthlyPlan.reduce((sum, period) => sum + model.variableCostPerUnitMinor * period.salesUnits, 0),
      'scheduled variable costs conserve every payable cohort in case ' + caseIndex
    );
    assert.equal(terms.deferredSalesCashReceiptsAfterHorizonMinor, deferredScheduledReceipts,
      'terms-based receipts after horizon case ' + caseIndex);
    assert.equal(terms.deferredVariableSupplierPaymentsAfterHorizonMinor, deferredScheduledSupplierPayments,
      'terms-based supplier payments after horizon case ' + caseIndex);
    terms.report.periods.forEach((period, index) => {
      assert.equal(period.salesCashReceiptsMinor, expectedScheduledReceipts[index],
        'terms-based collections case ' + caseIndex + ' month ' + index);
      assert.equal(period.operatingCashOutflowsMinor, model.fixedMonthlyCostsMinor + expectedScheduledSupplierPayments[index],
        'terms-based supplier payments case ' + caseIndex + ' month ' + index);
    });

    const reduction = assumptions.downside.salesReceiptReductionPercent;
    const delay = assumptions.downside.salesReceiptDelayMonths;
    const expectedDeferred = monthlyPlan.reduce((total, period, index) => {
      if (index + delay < horizon) return total;
      return total + Math.floor(period.salesCashReceiptsMinor * (100 - reduction) / 100);
    }, 0);
    assert.equal(
      result.report.scenarios[0].deferredSalesCashReceiptsAfterHorizonMinor,
      expectedDeferred,
      'out-of-horizon receipts remain visible in case ' + caseIndex
    );
  }
});

test('terms-based schedule allocates customer receipts and supplier payments by month and exposes horizon residuals', () => {
  const model = completeModel({
    sellingPricePerUnitMinor: 10_000,
    variableCostPerUnitMinor: 6_000,
    fixedMonthlyCostsMinor: 100,
    setupCostsMinor: 0,
    availableCashForStartupMinor: 500_000,
    initialWorkingCapitalBufferMinor: 0
  });
  const monthlyPlan = [10, 20, 30].map(salesUnits => ({
    salesUnits,
    salesCashReceiptsMinor: 0,
    operatingCashOutflowsMinor: 0,
    otherCashInflowsMinor: 0,
    otherCashOutflowsMinor: 0,
    investingCashInflowsMinor: 0,
    investingCashOutflowsMinor: 0,
    financingCashInflowsMinor: 0,
    financingCashOutflowsMinor: 0
  }));
  const timingSchedule = {
    salesReceiptsPercent: [50, 30, 20],
    variableCostCategories: [
      { id: 'category-1', label: 'Materials', costSharePercent: 60, paymentPercentages: [60, 30, 10] },
      { id: 'category-2', label: 'Labor', costSharePercent: 25, paymentPercentages: [100, 0, 0] },
      { id: 'category-3', label: 'Fulfillment', costSharePercent: 15, paymentPercentages: [0, 100, 0] }
    ]
  };
  const result = projectCashRunwayWithTimingSchedule({ model, monthlyPlan, timingSchedule });
  assert.equal(result.ok, true);
  assert.equal(result.report.kind, 'business-launchpad-terms-based-cash-projection');
  assert.deepEqual(result.report.monthlyPlan.map(period => period.salesCashReceiptsMinor), [50_000, 130_000, 230_000]);
  assert.deepEqual(result.report.monthlyPlan.map(period => period.operatingCashOutflowsMinor), [36_700, 93_100, 153_100]);
  assert.equal(result.report.deferredSalesCashReceiptsAfterHorizonMinor, 190_000);
  assert.equal(result.report.deferredVariableSupplierPaymentsAfterHorizonMinor, 77_400);
  assert.deepEqual(result.report.variableCostCategoryBreakdown.map(category => category.variableCostTotalMinor), [216_000, 90_000, 54_000]);
  assert.deepEqual(result.report.variableCostCategoryBreakdown.map(category => category.deferredPaymentsAfterHorizonMinor), [50_400, 0, 27_000]);
  assert.deepEqual(result.report.projection.periods.map(period => period.closingCashMinor), [513_300, 550_200, 627_100]);
  assert.deepEqual(monthlyPlan.map(period => period.salesCashReceiptsMinor), [0, 0, 0], 'terms case must not overwrite the entered plan');
});


test('small minor-unit costs are apportioned by sales cohort without losing or inventing category cost', () => {
  const model = completeModel({
    sellingPricePerUnitMinor: 1,
    variableCostPerUnitMinor: 1,
    fixedMonthlyCostsMinor: 0,
    setupCostsMinor: 0,
    availableCashForStartupMinor: 100,
    initialWorkingCapitalBufferMinor: 0
  });
  const monthlyPlan = [{
    salesUnits: 3,
    salesCashReceiptsMinor: 0,
    operatingCashOutflowsMinor: 0,
    otherCashInflowsMinor: 0,
    otherCashOutflowsMinor: 0,
    investingCashInflowsMinor: 0,
    investingCashOutflowsMinor: 0,
    financingCashInflowsMinor: 0,
    financingCashOutflowsMinor: 0
  }];
  const timingSchedule = {
    salesReceiptsPercent: [100, 0, 0],
    variableCostCategories: [
      { id: 'category-1', label: 'Materials', costSharePercent: 60, paymentPercentages: [0, 100, 0] },
      { id: 'category-2', label: 'Labor', costSharePercent: 25, paymentPercentages: [100, 0, 0] },
      { id: 'category-3', label: 'Fulfillment', costSharePercent: 15, paymentPercentages: [100, 0, 0] }
    ]
  };
  const result = projectCashRunwayWithTimingSchedule({ model, monthlyPlan, timingSchedule });
  assert.equal(result.ok, true);
  assert.deepEqual(result.report.variableCostCategoryBreakdown.map(category => category.variableCostTotalMinor), [2, 1, 0]);
  assert.deepEqual(result.report.monthlyPlan.map(period => period.operatingCashOutflowsMinor), [1]);
  assert.equal(result.report.deferredVariableSupplierPaymentsAfterHorizonMinor, 2);
  assert.deepEqual(result.report.variableCostCategoryBreakdown.map(category => category.deferredPaymentsAfterHorizonMinor), [2, 0, 0]);
  assert.equal(
    result.report.variableCostCategoryBreakdown.reduce((sum, category) => sum + category.variableCostTotalMinor, 0),
    3,
    'category totals must conserve the full one-minor-unit-per-sale cost'
  );
});

test('timing schedules require exact 100 percent allocations and terms-based comparison is explicit', () => {
  const model = completeModel({
    sellingPricePerUnitMinor: 101,
    variableCostPerUnitMinor: 37,
    fixedMonthlyCostsMinor: 10,
    setupCostsMinor: 0,
    availableCashForStartupMinor: 1_000
  });
  const monthlyPlan = [{
    salesUnits: 1,
    salesCashReceiptsMinor: 1,
    operatingCashOutflowsMinor: 2,
    otherCashInflowsMinor: 0,
    otherCashOutflowsMinor: 0,
    investingCashInflowsMinor: 0,
    investingCashOutflowsMinor: 0,
    financingCashInflowsMinor: 0,
    financingCashOutflowsMinor: 0
  }];
  const invalid = projectCashRunwayWithTimingSchedule({
    model,
    monthlyPlan,
    timingSchedule: { ...DEFAULT_CASH_TIMING_SCHEDULE, salesReceiptsPercent: [70, 20, 5] }
  });
  assert.equal(invalid.ok, false);
  assert.equal(invalid.error.code, 'invalid_cash_timing_schedule');
  const invalidShares = projectCashRunwayWithTimingSchedule({
    model,
    monthlyPlan,
    timingSchedule: {
      ...DEFAULT_CASH_TIMING_SCHEDULE,
      variableCostCategories: DEFAULT_CASH_TIMING_SCHEDULE.variableCostCategories.map(category => ({
        ...category,
        costSharePercent: 50
      }))
    }
  });
  assert.equal(invalidShares.ok, false);
  assert.equal(invalidShares.error.code, 'invalid_cash_timing_schedule');

  const schedule = {
    salesReceiptsPercent: [70, 25, 5],
    variableSupplierPaymentsPercent: [80, 15, 5]
  };
  const comparison = compareCashRunwayScenarios({ model, monthlyPlan, timingSchedule: schedule });
  assert.equal(comparison.ok, true);
  assert.deepEqual(comparison.report.scenarios.map(scenario => scenario.id), ['downside', 'base', 'upside', 'terms-based']);
  const terms = comparison.report.scenarios[3];
  assert.equal(terms.deferredSalesCashReceiptsAfterHorizonMinor, 30);
  assert.equal(terms.deferredVariableSupplierPaymentsAfterHorizonMinor, 7);
  assert.deepEqual(comparison.report.timingSchedule, {
    salesReceiptsPercent: [70, 25, 5],
    variableCostCategories: DEFAULT_CASH_TIMING_SCHEDULE.variableCostCategories.map(category => ({
      ...category,
      paymentPercentages: [80, 15, 5]
    }))
  });
});

test('cash scenario comparison validates stress ranges and exact arithmetic limits', () => {
  const model = completeModel({ setupCostsMinor: 0, availableCashForStartupMinor: 1 });
  const monthlyPlan = [{
    salesUnits: 1,
    salesCashReceiptsMinor: 1,
    operatingCashOutflowsMinor: 1,
    otherCashInflowsMinor: 0,
    otherCashOutflowsMinor: 0,
    investingCashInflowsMinor: 0,
    investingCashOutflowsMinor: 0,
    financingCashInflowsMinor: 0,
    financingCashOutflowsMinor: 0
  }];
  const invalid = compareCashRunwayScenarios({
    model,
    monthlyPlan,
    assumptions: { downside: { salesReceiptReductionPercent: 101 } }
  });
  assert.equal(invalid.ok, false);
  assert.equal(invalid.error.code, 'invalid_cash_scenario_assumptions');

  const overflow = compareCashRunwayScenarios({
    model,
    monthlyPlan: [{
      ...monthlyPlan[0],
      salesCashReceiptsMinor: Number.MAX_SAFE_INTEGER
    }],
    assumptions: {
      downside: {
        salesReceiptReductionPercent: 0,
        salesReceiptDelayMonths: 0,
        operatingCashOutflowIncreasePercent: 0
      },
      upside: {
        salesReceiptIncreasePercent: 500,
        salesReceiptAdvanceMonths: 0,
        operatingCashOutflowReductionPercent: 0
      }
    }
  });
  assert.equal(overflow.ok, false);
  assert.equal(overflow.error.code, 'calculation_overflow');
});

test('legacy cash plans derive only missing receipt and payment fields', () => {
  const model = completeModel();
  const legacy = projectCashRunway({
    model,
    monthlyPlan: [{ salesUnits: 10, otherCashInflowsMinor: 0, otherCashOutflowsMinor: 0 }]
  });
  assert.equal(legacy.ok, true);
  assert.equal(legacy.report.periods[0].salesCashReceiptsMinor, 100_000);
  assert.equal(legacy.report.periods[0].operatingCashOutflowsMinor, 160_000);

  const mixedVersion = projectCashRunway({
    model,
    monthlyPlan: [{
      salesUnits: 10,
      salesCashReceiptsMinor: 0,
      otherCashInflowsMinor: 0,
      otherCashOutflowsMinor: 0
    }]
  });
  assert.equal(mixedVersion.ok, true);
  assert.equal(mixedVersion.report.periods[0].salesCashReceiptsMinor, 0);
  assert.equal(mixedVersion.report.periods[0].operatingCashOutflowsMinor, 160_000);
});

test('cash-runway projection rejects missing, oversized and invalid monthly plans', () => {
  const model = completeModel();
  assert.equal(projectCashRunway({ model, monthlyPlan: [] }).error.code, 'invalid_cash_runway_horizon');
  assert.equal(projectCashRunway({ model, monthlyPlan: Array.from({ length: 25 }, () => ({
    salesUnits: 0, otherCashInflowsMinor: 0, otherCashOutflowsMinor: 0
  })) }).error.code, 'invalid_cash_runway_horizon');
  assert.equal(projectCashRunway({ model, monthlyPlan: [{ salesUnits: -1, otherCashInflowsMinor: 0, otherCashOutflowsMinor: 0 }] }).error.code, 'invalid_cash_runway_amount');
  assert.equal(projectCashRunway({ model, monthlyPlan: [{ salesUnits: 1, salesCashReceiptsMinor: -1, otherCashInflowsMinor: 0, otherCashOutflowsMinor: 0 }] }).error.code, 'invalid_cash_runway_amount');
  assert.equal(projectCashRunway({ model, monthlyPlan: [{ salesUnits: 1, operatingCashOutflowsMinor: Number.MAX_SAFE_INTEGER + 1, otherCashInflowsMinor: 0, otherCashOutflowsMinor: 0 }] }).error.code, 'invalid_cash_runway_amount');
  assert.equal(projectCashRunway({ model, monthlyPlan: [{ salesUnits: 1, otherCashInflowsMinor: Number.MAX_SAFE_INTEGER + 1, otherCashOutflowsMinor: 0 }] }).error.code, 'invalid_cash_runway_amount');
  assert.equal(projectCashRunway({ model: null, monthlyPlan: [{ salesUnits: 0, otherCashInflowsMinor: 0, otherCashOutflowsMinor: 0 }] }).error.code, 'invalid_input');
});

test('cash-runway projection fails closed when a period cash amount exceeds exact integer range', () => {
  const result = projectCashRunway({
    model: completeModel(),
    monthlyPlan: [{ salesUnits: 2, salesCashReceiptsMinor: Number.MAX_SAFE_INTEGER, operatingCashOutflowsMinor: 0, otherCashInflowsMinor: 1, otherCashOutflowsMinor: 0 }]
  });
  assert.equal(result.ok, false);
  assert.equal(result.error.code, 'calculation_overflow');
});

test('cash balance recurrence holds for 200 fixed-seed scenarios with independent receipt/payment timing', () => {
  let seed = 0x51a7cafe;
  const next = (maxExclusive) => {
    seed = (Math.imul(seed, 1664525) + 1013904223) >>> 0;
    return seed % maxExclusive;
  };
  for (let caseIndex = 0; caseIndex < 200; caseIndex += 1) {
    const model = completeModel({
      setupCostsMinor: next(100_000),
      initialWorkingCapitalBufferMinor: next(40_000),
      availableCashForStartupMinor: next(150_000)
    });
    const monthlyPlan = Array.from({ length: 1 + next(12) }, () => ({
      salesUnits: next(100),
      salesCashReceiptsMinor: next(1_000_000),
      operatingCashOutflowsMinor: next(900_000),
      otherCashInflowsMinor: next(10_000),
      otherCashOutflowsMinor: next(15_000)
    }));
    const result = projectCashRunway({ model, monthlyPlan });
    assert.equal(result.ok, true, 'result must be valid in bounded corpus ' + caseIndex);
    let expectedCash = BigInt(model.availableCashForStartupMinor) - BigInt(model.setupCostsMinor);
    let expectedMinimum = expectedCash;
    let expectedMinimumPeriod = 0;
    const expectedBelowReserve = expectedCash < BigInt(model.initialWorkingCapitalBufferMinor) ? [0] : [];
    const expectedNegative = expectedCash < 0n ? [0] : [];
    for (let index = 0; index < monthlyPlan.length; index += 1) {
      const period = monthlyPlan[index];
      expectedCash += BigInt(period.salesCashReceiptsMinor) + BigInt(period.otherCashInflowsMinor)
        - BigInt(period.operatingCashOutflowsMinor) - BigInt(period.otherCashOutflowsMinor);
      const projected = result.report.periods[index];
      assert.equal(projected.closingCashMinor, Number(expectedCash), 'closing cash case ' + caseIndex + ' period ' + (index + 1));
      assert.equal(
        projected.totalCashReceiptsMinor,
        projected.salesCashReceiptsMinor + projected.otherCashInflowsMinor,
        'receipt identity case ' + caseIndex + ' period ' + (index + 1)
      );
      assert.equal(
        projected.totalCashOutflowsMinor,
        projected.operatingCashOutflowsMinor + projected.otherCashOutflowsMinor,
        'payment identity case ' + caseIndex + ' period ' + (index + 1)
      );
      assert.equal(
        projected.netCashChangeMinor,
        projected.totalCashReceiptsMinor - projected.totalCashOutflowsMinor,
        'net identity case ' + caseIndex + ' period ' + (index + 1)
      );
      const p = index + 1;
      if (expectedCash < expectedMinimum) {
        expectedMinimum = expectedCash;
        expectedMinimumPeriod = p;
      }
      if (expectedCash < BigInt(model.initialWorkingCapitalBufferMinor)) expectedBelowReserve.push(p);
      if (expectedCash < 0n) expectedNegative.push(p);
    }
    assert.equal(result.report.summary.endingCashMinor, Number(expectedCash), 'ending cash case ' + caseIndex);
    assert.equal(result.report.summary.minimumProjectedCashMinor, Number(expectedMinimum), 'minimum cash case ' + caseIndex);
    assert.equal(result.report.summary.minimumCashPeriod, expectedMinimumPeriod, 'minimum period case ' + caseIndex);
    assert.deepEqual(result.report.summary.periodsBelowReserve, expectedBelowReserve, 'reserve breaches case ' + caseIndex);
    assert.deepEqual(result.report.summary.periodsWithNegativeCash, expectedNegative, 'negative balances case ' + caseIndex);
  }
});

test('formats signed values deterministically without floating point', () => {
  assert.equal(formatMinorUnits(1_999, { code: 'ZAR', minorUnitExponent: 2 }), 'ZAR 19.99');
  assert.equal(formatMinorUnits(-5, { code: 'USD', minorUnitExponent: 2 }), 'USD -0.05');
  assert.equal(formatMinorUnits(123, { code: 'JPY', minorUnitExponent: 0 }), 'JPY 123');
});

test('declares limitations so the worksheet cannot be mistaken for net profit or legal advice', () => {
  const result = analyzeBusinessModel(completeModel());
  assert.equal(result.ok, true);
  assert.ok(result.report.scopeLimitations.some(item => item.includes('not a full accounting ledger')));
  assert.ok(result.report.scopeLimitations.some(item => item.includes('jurisdictional rules')));
});

test('service, food, and retail fixture scenarios produce their frozen arithmetic results', async (t) => {
  const here = dirname(fileURLToPath(import.meta.url));
  const fixturePath = resolve(here, 'fixtures/business-scenarios.v1.json');
  const fixtures = JSON.parse(await readFile(fixturePath, 'utf8'));
  assert.equal(fixtures.schemaVersion, 1);
  assert.equal(fixtures.currency.code, 'ZAR');
  assert.equal(fixtures.scenarios.length, 3);

  for (const scenario of fixtures.scenarios) {
    await t.test(scenario.id, () => {
      const result = analyzeBusinessModel({
        currency: fixtures.currency,
        ...scenario.input
      });
      assert.equal(result.ok, true, scenario.id);
      assert.equal(result.report.costCoverage.level, 'partial');
      assert.deepEqual(result.report.costCoverage.omittedCategories, scenario.input.costCoverage.omittedCategories);
      assert.equal(result.report.unitEconomics.contributionPerUnitMinor, scenario.expected.contributionPerUnitMinor);
      assert.equal(result.report.monthlyScenario.revenueMinor, scenario.expected.revenueMinor);
      assert.equal(result.report.monthlyScenario.variableCostsMinor, scenario.expected.variableCostsMinor);
      assert.equal(result.report.monthlyScenario.contributionMinor, scenario.expected.contributionMinor);
      assert.equal(result.report.monthlyScenario.fixedCostsMinor, scenario.expected.fixedCostsMinor);
      assert.equal(result.report.monthlyScenario.operatingResultBeforeExcludedItemsMinor, scenario.expected.operatingResultBeforeExcludedItemsMinor);
      assert.equal(result.report.breakEven.unitsPerMonth, scenario.expected.breakEvenUnitsPerMonth);
      assert.equal(result.report.startup.requiredCashMinor, scenario.expected.requiredCashMinor);
      assert.equal(result.report.startup.fundingGapMinor, scenario.expected.fundingGapMinor);
    });
  }
});


test('parses decimal text to minor units without binary floating-point rounding', async () => {
  const { parseMajorToMinorUnits } = await import('../src/index.js');
  assert.deepEqual(parseMajorToMinorUnits('180.00', 2), { ok: true, amountMinor: 18_000 });
  assert.deepEqual(parseMajorToMinorUnits('0.05', 2), { ok: true, amountMinor: 5 });
  assert.deepEqual(parseMajorToMinorUnits('12', 2), { ok: true, amountMinor: 1_200 });
  assert.deepEqual(parseMajorToMinorUnits('123', 0), { ok: true, amountMinor: 123 });
});

test('rejects invalid, negative, over-precise, and unsafe decimal text', async () => {
  const { parseMajorToMinorUnits } = await import('../src/index.js');
  assert.equal(parseMajorToMinorUnits('-1.00', 2).error.code, 'invalid_money_text');
  assert.equal(parseMajorToMinorUnits('1,000.00', 2).error.code, 'invalid_money_text');
  assert.equal(parseMajorToMinorUnits('1.001', 2).error.code, 'excess_fractional_digits');
  assert.equal(parseMajorToMinorUnits('90071992547409.92', 2).error.code, 'calculation_overflow');
  assert.equal(parseMajorToMinorUnits('1.00', 8).error.code, 'invalid_currency_precision');
});

test('deterministic generated scenarios preserve arithmetic identities across a bounded input corpus', () => {
  let seed = 0x51f15e;
  const next = ceiling => {
    seed = (Math.imul(seed, 1664525) + 1013904223) >>> 0;
    return seed % (ceiling + 1);
  };

  for (let index = 0; index < 500; index += 1) {
    const sellingPrice = next(100_000);
    const variableCost = next(100_000);
    const fixedCosts = next(10_000_000);
    const plannedUnits = next(2_000);
    const setupCosts = next(10_000_000);
    const workingCapital = next(10_000_000);
    const availableCash = next(10_000_000);

    const result = analyzeBusinessModel(completeModel({
      sellingPricePerUnitMinor: sellingPrice,
      variableCostPerUnitMinor: variableCost,
      fixedMonthlyCostsMinor: fixedCosts,
      plannedUnitsPerMonth: plannedUnits,
      setupCostsMinor: setupCosts,
      initialWorkingCapitalBufferMinor: workingCapital,
      availableCashForStartupMinor: availableCash
    }));
    assert.equal(result.ok, true, 'valid generated case ' + index);
    const report = result.report;

    const unitContribution = sellingPrice - variableCost;
    const revenue = sellingPrice * plannedUnits;
    const variableCosts = variableCost * plannedUnits;
    const contribution = revenue - variableCosts;
    const operatingResult = contribution - fixedCosts;
    const requiredCash = setupCosts + workingCapital;
    const fundingGap = Math.max(0, requiredCash - availableCash);

    assert.equal(report.unitEconomics.contributionPerUnitMinor, unitContribution, 'unit contribution case ' + index);
    assert.equal(report.monthlyScenario.revenueMinor, revenue, 'revenue case ' + index);
    assert.equal(report.monthlyScenario.variableCostsMinor, variableCosts, 'variable costs case ' + index);
    assert.equal(report.monthlyScenario.contributionMinor, contribution, 'contribution case ' + index);
    assert.equal(report.monthlyScenario.operatingResultBeforeExcludedItemsMinor, operatingResult, 'operating result case ' + index);
    assert.equal(report.startup.requiredCashMinor, requiredCash, 'required cash case ' + index);
    assert.equal(report.startup.fundingGapMinor, fundingGap, 'funding gap case ' + index);

    if (unitContribution > 0) {
      const expectedBreakEven = Math.floor((fixedCosts + unitContribution - 1) / unitContribution);
      assert.equal(report.breakEven.status, 'reachable', 'break-even status case ' + index);
      assert.equal(report.breakEven.unitsPerMonth, expectedBreakEven, 'break-even units case ' + index);

      const contributionPerUnitBig = BigInt(unitContribution);
      const fixedCostsBig = BigInt(fixedCosts);
      const breakEvenBig = BigInt(report.breakEven.unitsPerMonth);
      assert.ok(breakEvenBig * contributionPerUnitBig >= fixedCostsBig, 'break-even must cover fixed costs, case ' + index);
      if (breakEvenBig > 0n) {
        assert.ok((breakEvenBig - 1n) * contributionPerUnitBig < fixedCostsBig, 'one fewer unit must not cover fixed costs, case ' + index);
      }
    } else {
      assert.equal(report.breakEven.status, 'unreachable', 'break-even status for non-positive contribution, case ' + index);
      assert.equal(report.breakEven.unitsPerMonth, null, 'no finite non-negative break-even units, case ' + index);
    }
  }
});

test('rejects extremely long amount text before BigInt conversion', async () => {
  const { parseMajorToMinorUnits } = await import('../src/index.js');
  const result = parseMajorToMinorUnits('9'.repeat(65), 2);
  assert.equal(result.ok, false);
  assert.equal(result.error.code, 'money_text_too_long');
});


function actualsFixture() {
  return {
    schemaVersion: 1, kind: 'business-launchpad-actuals-source',
    currency: { code: 'ZAR', minorUnitExponent: 2 },
    source: {
      type: 'bank_statement_export', label: 'Synthetic test bank statement export',
      evidenceReference: 'Synthetic fixture reference only',
      coverageStartDate: '2026-01-01', coverageEndDate: '2026-02-28',
      completeMonthlyCoverage: true, openingCashBalanceMinor: 5000, closingCashBalanceMinor: 7050
    },
    transactions: [
      { transactionId: 'tx-001', date: '2026-01-10', currencyCode: 'ZAR', category: 'sales_receipt', amountMinor: 900, sourceReference: 'statement line 1' },
      { transactionId: 'tx-002', date: '2026-01-14', currencyCode: 'ZAR', category: 'operating_outflow', amountMinor: 450, sourceReference: 'statement line 2' },
      { transactionId: 'tx-003', date: '2026-02-10', currencyCode: 'ZAR', category: 'sales_receipt', amountMinor: 2200, sourceReference: 'statement line 3' },
      { transactionId: 'tx-004', date: '2026-02-14', currencyCode: 'ZAR', category: 'operating_outflow', amountMinor: 600, sourceReference: 'statement line 4' }
    ]
  };
}
function actualsPlanFixture() {
  const model = completeModel({
    sellingPricePerUnitMinor: 100, variableCostPerUnitMinor: 60, fixedMonthlyCostsMinor: 10,
    setupCostsMinor: 0, availableCashForStartupMinor: 5000, initialWorkingCapitalBufferMinor: 0
  });
  const monthlyPlan = [
    { salesUnits: 10, salesCashReceiptsMinor: 1000, operatingCashOutflowsMinor: 400, otherCashInflowsMinor: 0, otherCashOutflowsMinor: 0, investingCashInflowsMinor: 0, investingCashOutflowsMinor: 0, financingCashInflowsMinor: 0, financingCashOutflowsMinor: 0 },
    { salesUnits: 10, salesCashReceiptsMinor: 2000, operatingCashOutflowsMinor: 500, otherCashInflowsMinor: 0, otherCashOutflowsMinor: 0, investingCashInflowsMinor: 0, investingCashOutflowsMinor: 0, financingCashInflowsMinor: 0, financingCashOutflowsMinor: 0 }
  ];
  return { model, monthlyPlan };
}

test('actuals comparison reconciles balances and reports category, net-movement, and closing-cash variance', () => {
  const { model, monthlyPlan } = actualsPlanFixture();
  const result = compareCashPlanToActuals({ model, monthlyPlan, forecastStartMonth: '2026-01', actualsSource: actualsFixture(), sourceSha256: 'a'.repeat(64) });
  assert.equal(result.ok, true);
  assert.equal(result.report.kind, 'business-launchpad-actuals-variance-report');
  assert.equal(result.report.source.authenticity, 'not_independently_verified');
  assert.equal(result.report.source.transactionCount, 4);
  assert.deepEqual(result.report.reconciliation, {
    status: 'passed', openingCashBalanceMinor: 5000, closingCashBalanceMinor: 7050,
    transactionNetMovementMinor: 2050, balanceDerivedNetMovementMinor: 2050, differenceMinor: 0
  });
  assert.equal(result.report.periods[0].month, '2026-01');
  assert.equal(result.report.periods[0].categories[0].actualMinusPlanMinor, -100);
  assert.equal(result.report.periods[0].categories[1].actualMinusPlanMinor, 50);
  assert.equal(result.report.periods[0].netCashMovementVarianceMinor, -150);
  assert.equal(result.report.periods[0].actualClosingCashMinor, 5450);
  assert.equal(result.report.periods[0].closingCashVarianceMinor, -150);
  assert.equal(result.report.periods[1].actualClosingCashMinor, 7050);
  assert.equal(result.report.periods[1].closingCashVarianceMinor, -50);
});

test('actuals comparison fails closed on duplicate IDs, incomplete coverage, and unreconciled balances', () => {
  const { model, monthlyPlan } = actualsPlanFixture();
  const input = { model, monthlyPlan, forecastStartMonth: '2026-01', actualsSource: actualsFixture(), sourceSha256: 'b'.repeat(64) };
  const duplicate = actualsFixture();
  duplicate.transactions[1].transactionId = duplicate.transactions[0].transactionId;
  assert.equal(compareCashPlanToActuals({ ...input, actualsSource: duplicate }).error.code, 'duplicate_or_invalid_actuals_transaction_id');
  const incomplete = actualsFixture();
  incomplete.source.completeMonthlyCoverage = false;
  assert.equal(compareCashPlanToActuals({ ...input, actualsSource: incomplete }).error.code, 'incomplete_actuals_coverage');
  const unreconciled = actualsFixture();
  unreconciled.source.closingCashBalanceMinor += 1;
  assert.equal(compareCashPlanToActuals({ ...input, actualsSource: unreconciled }).error.code, 'actuals_balance_reconciliation_failed');
});

test('actuals comparison rejects currency mismatch, out-of-horizon months, and missing row references', () => {
  const { model, monthlyPlan } = actualsPlanFixture();
  const input = { model, monthlyPlan, forecastStartMonth: '2026-01', actualsSource: actualsFixture(), sourceSha256: 'c'.repeat(64) };
  const mismatch = actualsFixture();
  mismatch.transactions[0].currencyCode = 'USD';
  assert.equal(compareCashPlanToActuals({ ...input, actualsSource: mismatch }).error.code, 'actuals_currency_mismatch');
  assert.equal(compareCashPlanToActuals({ ...input, forecastStartMonth: '2025-12' }).error.code, 'actuals_coverage_outside_plan');
  assert.equal(compareCashPlanToActuals({ ...input, forecastStartMonth: '2026-13' }).error.code, 'invalid_forecast_start_month');

  const invalidCoverageDate = actualsFixture();
  invalidCoverageDate.source.coverageStartDate = '2026-02-30';
  assert.equal(compareCashPlanToActuals({ ...input, actualsSource: invalidCoverageDate }).error.code, 'invalid_actuals_coverage_dates');

  const invalidTransactionDate = actualsFixture();
  invalidTransactionDate.transactions[0].date = '2026-02-30';
  assert.equal(compareCashPlanToActuals({ ...input, actualsSource: invalidTransactionDate }).error.code, 'actuals_transaction_outside_coverage');

  const noReference = actualsFixture();
  noReference.transactions[0].sourceReference = '';
  assert.equal(compareCashPlanToActuals({ ...input, actualsSource: noReference }).error.code, 'missing_actuals_row_reference');
  assert.equal(compareCashPlanToActuals({ ...input, sourceSha256: 'not-a-digest' }).error.code, 'invalid_actuals_source_digest');
});
