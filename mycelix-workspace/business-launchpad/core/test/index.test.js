// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

import test from 'node:test';
import { readFile } from 'node:fs/promises';
import { fileURLToPath } from 'node:url';
import { dirname, resolve } from 'node:path';
import assert from 'node:assert/strict';
import { analyzeBusinessModel, formatMinorUnits } from '../src/index.js';

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
