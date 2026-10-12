// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

import test from 'node:test';
import { readFile } from 'node:fs/promises';
import { fileURLToPath } from 'node:url';
import { dirname, resolve } from 'node:path';
import assert from 'node:assert/strict';
import { analyzeBusinessModel, formatMinorUnits, projectCashRunway } from '../src/index.js';

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
  assert.deepEqual(result.report.summary.periodsWithNegativeCash, [0, 1]);
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
