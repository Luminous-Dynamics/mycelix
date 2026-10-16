// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//
// This module is a scenario calculator, not a ledger, forecast, tax engine,
// lending decision, legal-compliance determination, or guarantee of profit.

const MAX_SAFE = BigInt(Number.MAX_SAFE_INTEGER);
const CURRENCY_CODE = /^[A-Z]{3}$/;
const MAX_MINOR_UNIT_EXPONENT = 6;
const MAX_MONEY_TEXT_LENGTH = 64;

const SCOPE_LIMITATIONS = Object.freeze([
  'This is a contribution and fixed-cost worksheet, not a full accounting ledger.',
  'The monthly result is before tax, financing/debt service, depreciation, and owner draw unless those items are explicitly included in the supplied costs.',
  'The funding gap is setup costs plus the owner-entered working-capital buffer less available startup cash; it does not calculate extra runway for slow sales or operating losses and is not a lending decision.',
  'Currency-code shape is checked, but currency validity, tax treatment, and jurisdictional rules must come from a separately verified jurisdiction pack.',
  'This is a single-offer model with one average selling price and one average variable cost per unit; product/service mix, inventory-specific costing, and capacity constraints are not modeled.',
  'Forecast units are an owner-supplied scenario, not a demand prediction or guarantee of sales.'
]);

function fail(code, message, field = null) {
  return { ok: false, error: { code, message, field } };
}

function isRecord(value) {
  return value !== null && typeof value === 'object' && !Array.isArray(value);
}

function isNonNegativeSafeInteger(value) {
  return Number.isSafeInteger(value) && value >= 0;
}

function outputInteger(value, field) {
  if (value > MAX_SAFE || value < -MAX_SAFE) {
    const error = new RangeError('Calculated ' + field + ' exceeds the supported exact integer range.');
    error.code = 'calculation_overflow';
    error.field = field;
    throw error;
  }
  return Number(value);
}

function validateModel(input) {
  if (!isRecord(input)) {
    return { error: fail('invalid_input', 'A business model object is required.') };
  }
  if (!isRecord(input.currency)) {
    return { error: fail('invalid_currency', 'Currency metadata is required.', 'currency') };
  }

  const { code, minorUnitExponent } = input.currency;
  if (typeof code !== 'string' || !CURRENCY_CODE.test(code)) {
    return { error: fail('invalid_currency_code', 'Currency code must be three uppercase ASCII letters. This shape check does not verify that it is a valid or supported currency.', 'currency.code') };
  }
  if (!Number.isInteger(minorUnitExponent) || minorUnitExponent < 0 || minorUnitExponent > MAX_MINOR_UNIT_EXPONENT) {
    return { error: fail('invalid_currency_precision', 'Currency minor-unit exponent must be an integer from 0 to ' + MAX_MINOR_UNIT_EXPONENT + '.', 'currency.minorUnitExponent') };
  }

  const numericFields = [
    'sellingPricePerUnitMinor',
    'variableCostPerUnitMinor',
    'fixedMonthlyCostsMinor',
    'plannedUnitsPerMonth',
    'setupCostsMinor',
    'initialWorkingCapitalBufferMinor',
    'availableCashForStartupMinor'
  ];
  for (const field of numericFields) {
    if (!isNonNegativeSafeInteger(input[field])) {
      return { error: fail('invalid_amount', field + ' must be a non-negative safe integer expressed in the currency minor units.', field) };
    }
  }

  if (!isRecord(input.costCoverage)) {
    return { error: fail('missing_cost_coverage', 'Explicit cost-coverage status is required; unknown costs must not silently become zero.', 'costCoverage') };
  }
  const { level, omittedCategories } = input.costCoverage;
  if (!['complete_for_declared_scope', 'partial'].includes(level)) {
    return { error: fail('invalid_cost_coverage', 'Cost coverage level must be complete_for_declared_scope or partial.', 'costCoverage.level') };
  }
  if (!Array.isArray(omittedCategories) || omittedCategories.some(item => typeof item !== 'string' || item.trim().length === 0)) {
    return { error: fail('invalid_omitted_categories', 'omittedCategories must be an array of non-empty strings.', 'costCoverage.omittedCategories') };
  }
  const cleanedOmissions = omittedCategories.map(item => item.trim());
  if (level === 'complete_for_declared_scope' && cleanedOmissions.length > 0) {
    return { error: fail('contradictory_cost_coverage', 'A complete cost-coverage declaration cannot list omitted cost categories.', 'costCoverage') };
  }
  if (level === 'partial' && cleanedOmissions.length === 0) {
    return { error: fail('incomplete_costs_not_described', 'Partial cost coverage must name the omitted or unknown cost categories.', 'costCoverage.omittedCategories') };
  }

  return {
    model: {
      ...input,
      currency: { code, minorUnitExponent },
      costCoverage: { level, omittedCategories: cleanedOmissions }
    }
  };
}

/**
 * Analyze a single-offer business scenario using integer minor currency units.
 * BigInt intermediates prevent rounding drift; outputs that cannot be
 * represented exactly as JavaScript safe integers fail closed.
 *
 * @param {object} input
 * @returns {{ok: true, report: object} | {ok: false, error: {code: string, message: string, field: string|null}}}
 */
export function analyzeBusinessModel(input) {
  const validated = validateModel(input);
  if (validated.error) return validated.error;
  const model = validated.model;

  try {
    const price = BigInt(model.sellingPricePerUnitMinor);
    const unitVariableCost = BigInt(model.variableCostPerUnitMinor);
    const fixed = BigInt(model.fixedMonthlyCostsMinor);
    const units = BigInt(model.plannedUnitsPerMonth);
    const setup = BigInt(model.setupCostsMinor);
    const workingCapital = BigInt(model.initialWorkingCapitalBufferMinor);
    const availableCash = BigInt(model.availableCashForStartupMinor);

    const unitContribution = price - unitVariableCost;
    const revenue = price * units;
    const variableCosts = unitVariableCost * units;
    const plannedContribution = revenue - variableCosts;
    const operatingResult = plannedContribution - fixed;
    const requiredStartupCash = setup + workingCapital;
    const fundingGap = requiredStartupCash > availableCash ? requiredStartupCash - availableCash : 0n;

    let breakEvenUnitsPerMonth = null;
    let breakEvenStatus = 'unreachable';
    let breakEvenReason = 'unit_contribution_not_positive';
    if (unitContribution > 0n) {
      const needed = fixed === 0n ? 0n : (fixed + unitContribution - 1n) / unitContribution;
      breakEvenUnitsPerMonth = outputInteger(needed, 'breakEvenUnitsPerMonth');
      breakEvenStatus = 'reachable';
      breakEvenReason = null;
    }

    const report = {
      schemaVersion: 1,
      currency: { ...model.currency },
      units: {
        period: 'month',
        plannedSales: model.plannedUnitsPerMonth
      },
      unitEconomics: {
        sellingPricePerUnitMinor: model.sellingPricePerUnitMinor,
        variableCostPerUnitMinor: model.variableCostPerUnitMinor,
        contributionPerUnitMinor: outputInteger(unitContribution, 'contributionPerUnitMinor')
      },
      monthlyScenario: {
        revenueMinor: outputInteger(revenue, 'revenueMinor'),
        variableCostsMinor: outputInteger(variableCosts, 'variableCostsMinor'),
        contributionMinor: outputInteger(plannedContribution, 'contributionMinor'),
        fixedCostsMinor: model.fixedMonthlyCostsMinor,
        operatingResultBeforeExcludedItemsMinor: outputInteger(operatingResult, 'operatingResultBeforeExcludedItemsMinor')
      },
      breakEven: {
        status: breakEvenStatus,
        unitsPerMonth: breakEvenUnitsPerMonth,
        reason: breakEvenReason
      },
      startup: {
        setupCostsMinor: model.setupCostsMinor,
        workingCapitalBufferMinor: model.initialWorkingCapitalBufferMinor,
        requiredCashMinor: outputInteger(requiredStartupCash, 'requiredCashMinor'),
        availableCashMinor: model.availableCashForStartupMinor,
        fundingGapMinor: outputInteger(fundingGap, 'fundingGapMinor')
      },
      costCoverage: {
        level: model.costCoverage.level,
        omittedCategories: [...model.costCoverage.omittedCategories]
      },
      scopeLimitations: [...SCOPE_LIMITATIONS]
    };

    return { ok: true, report };
  } catch (error) {
    if (error instanceof RangeError && error.code === 'calculation_overflow') {
      return fail(error.code, error.message, error.field ?? null);
    }
    return fail('calculation_error', 'The business model could not be analyzed safely. No result has been produced.');
  }
}

/**
 * Project a bounded month-by-month cash scenario from the single-offer model.
 *
 * Sales activity, sales cash receipts, operating payments, investing cash flows,
 * and financing cash flows are separate inputs. The legacy generic "other" amounts
 * remain unclassified rather than being silently relabeled as operating cash. Same-
 * month sales/cost defaults are only conveniences. Setup is deducted before month 1;
 * the working-capital buffer is a reserve threshold, not a second payment.
 *
 * Backward compatibility: legacy plans missing receipt/payment fields receive the
 * prior same-month defaults; missing investing/financing fields default to zero.
 *
 * @param {{model:object, monthlyPlan:Array<{salesUnits:number,salesCashReceiptsMinor?:number,operatingCashOutflowsMinor?:number,otherCashInflowsMinor:number,otherCashOutflowsMinor:number,investingCashInflowsMinor?:number,investingCashOutflowsMinor?:number,financingCashInflowsMinor?:number,financingCashOutflowsMinor?:number}>}} input
 * @returns {{ok:true, report:object} | {ok:false, error:object}}
 */
export function projectCashRunway(input) {
  if (!isRecord(input)) {
    return fail('invalid_cash_runway_input', 'A cash-runway scenario object is required.');
  }
  const validation = validateModel(input.model);
  if (validation.error) return validation.error;
  const model = validation.model;
  const plan = input.monthlyPlan;

  if (!Array.isArray(plan) || plan.length < 1 || plan.length > 24) {
    return fail('invalid_cash_runway_horizon', 'The monthly cash plan must contain between 1 and 24 months.', 'monthlyPlan');
  }

  const normalizedPlan = [];
  const requiredFields = ['salesUnits', 'otherCashInflowsMinor', 'otherCashOutflowsMinor'];
  for (let index = 0; index < plan.length; index += 1) {
    const period = plan[index];
    if (!isRecord(period)) {
      return fail('invalid_cash_runway_period', 'Each monthly cash-plan entry must be an object.', 'monthlyPlan[' + index + ']');
    }
    for (const field of requiredFields) {
      if (!isNonNegativeSafeInteger(period[field])) {
        return fail(
          'invalid_cash_runway_amount',
          field + ' must be a non-negative safe integer in the stated units/minor currency units.',
          'monthlyPlan[' + index + '].' + field
        );
      }
    }

    const periodSales = BigInt(period.salesUnits);
    const calculatedReceipts = BigInt(model.sellingPricePerUnitMinor) * periodSales;
    const calculatedOperatingOutflows =
      BigInt(model.variableCostPerUnitMinor) * periodSales + BigInt(model.fixedMonthlyCostsMinor);
    // Compatibility defaults are calculated as BigInt, range-checked, then
    // converted to Number before safe-integer validation and report serialization.
    if (
      (period.salesCashReceiptsMinor === undefined && calculatedReceipts > MAX_SAFE) ||
      (period.operatingCashOutflowsMinor === undefined && calculatedOperatingOutflows > MAX_SAFE)
    ) {
      return fail('calculation_overflow', 'A default monthly cash amount exceeds the supported exact integer range.', 'monthlyPlan[' + index + ']');
    }
    const salesCashReceiptsMinor = period.salesCashReceiptsMinor === undefined
      ? Number(calculatedReceipts)
      : period.salesCashReceiptsMinor;
    const operatingCashOutflowsMinor = period.operatingCashOutflowsMinor === undefined
      ? Number(calculatedOperatingOutflows)
      : period.operatingCashOutflowsMinor;

    if (!isNonNegativeSafeInteger(salesCashReceiptsMinor)) {
      return fail(
        'invalid_cash_runway_amount',
        'salesCashReceiptsMinor must be a non-negative safe integer representing cash expected to be collected this month.',
        'monthlyPlan[' + index + '].salesCashReceiptsMinor'
      );
    }
    if (!isNonNegativeSafeInteger(operatingCashOutflowsMinor)) {
      return fail(
        'invalid_cash_runway_amount',
        'operatingCashOutflowsMinor must be a non-negative safe integer representing operating cash payments expected this month.',
        'monthlyPlan[' + index + '].operatingCashOutflowsMinor'
      );
    }

    const additionalCashFields = [
      'investingCashInflowsMinor',
      'investingCashOutflowsMinor',
      'financingCashInflowsMinor',
      'financingCashOutflowsMinor'
    ];
    const additionalCash = {};
    for (const field of additionalCashFields) {
      const value = period[field] === undefined ? 0 : period[field];
      if (!isNonNegativeSafeInteger(value)) {
        return fail(
          'invalid_cash_runway_amount',
          field + ' must be a non-negative safe integer representing cash expected to move in this month.',
          'monthlyPlan[' + index + '].' + field
        );
      }
      additionalCash[field] = value;
    }

    normalizedPlan.push({
      salesUnits: period.salesUnits,
      salesCashReceiptsMinor,
      operatingCashOutflowsMinor,
      otherCashInflowsMinor: period.otherCashInflowsMinor,
      otherCashOutflowsMinor: period.otherCashOutflowsMinor,
      ...additionalCash
    });
  }

  try {
    const setupOutflow = BigInt(model.setupCostsMinor);
    const openingCash = BigInt(model.availableCashForStartupMinor);
    const reserveTarget = BigInt(model.initialWorkingCapitalBufferMinor);

    // Setup is paid once before the first projected month. Do not subtract the
    // working-capital reserve: it is the threshold against which liquidity is judged.
    let cash = openingCash - setupOutflow;
    let minimumCash = cash;
    let minimumCashPeriod = 0;
    let firstNegativeCashPeriod = cash < 0n ? 0 : null;
    let firstBelowReservePeriod = cash < reserveTarget ? 0 : null;
    const periods = [];

    for (let index = 0; index < normalizedPlan.length; index += 1) {
      const period = normalizedPlan[index];
      const salesCashReceipts = BigInt(period.salesCashReceiptsMinor);
      const operatingCashOutflows = BigInt(period.operatingCashOutflowsMinor);
      const otherInflows = BigInt(period.otherCashInflowsMinor);
      const otherOutflows = BigInt(period.otherCashOutflowsMinor);
      const investingInflows = BigInt(period.investingCashInflowsMinor);
      const investingOutflows = BigInt(period.investingCashOutflowsMinor);
      const financingInflows = BigInt(period.financingCashInflowsMinor);
      const financingOutflows = BigInt(period.financingCashOutflowsMinor);
      const totalOperatingCashReceipts = salesCashReceipts;
      const totalOperatingCashOutflows = operatingCashOutflows;
      const netOperatingCashFlow = totalOperatingCashReceipts - totalOperatingCashOutflows;
      const netUnclassifiedCashFlow = otherInflows - otherOutflows;
      const netInvestingCashFlow = investingInflows - investingOutflows;
      const netFinancingCashFlow = financingInflows - financingOutflows;
      const totalCashReceipts = salesCashReceipts + otherInflows + investingInflows + financingInflows;
      const totalCashOutflows = operatingCashOutflows + otherOutflows + investingOutflows + financingOutflows;
      const netCashChange = netOperatingCashFlow + netUnclassifiedCashFlow + netInvestingCashFlow + netFinancingCashFlow;
      cash += netCashChange;
      const periodNumber = index + 1;

      if (cash < minimumCash) {
        minimumCash = cash;
        minimumCashPeriod = periodNumber;
      }
      if (firstNegativeCashPeriod === null && cash < 0n) {
        firstNegativeCashPeriod = periodNumber;
      }
      if (firstBelowReservePeriod === null && cash < reserveTarget) {
        firstBelowReservePeriod = periodNumber;
      }

      periods.push({
        period: periodNumber,
        salesUnitsScenario: period.salesUnits,
        salesCashReceiptsMinor: period.salesCashReceiptsMinor,
        otherCashInflowsMinor: period.otherCashInflowsMinor,
        totalOperatingCashReceiptsMinor: outputInteger(totalOperatingCashReceipts, 'period' + periodNumber + '.totalOperatingCashReceiptsMinor'),
        totalCashReceiptsMinor: outputInteger(totalCashReceipts, 'period' + periodNumber + '.totalCashReceiptsMinor'),
        operatingCashOutflowsMinor: period.operatingCashOutflowsMinor,
        otherCashOutflowsMinor: period.otherCashOutflowsMinor,
        totalOperatingCashOutflowsMinor: outputInteger(totalOperatingCashOutflows, 'period' + periodNumber + '.totalOperatingCashOutflowsMinor'),
        netOperatingCashFlowMinor: outputInteger(netOperatingCashFlow, 'period' + periodNumber + '.netOperatingCashFlowMinor'),
        netUnclassifiedCashFlowMinor: outputInteger(netUnclassifiedCashFlow, 'period' + periodNumber + '.netUnclassifiedCashFlowMinor'),
        investingCashInflowsMinor: period.investingCashInflowsMinor,
        investingCashOutflowsMinor: period.investingCashOutflowsMinor,
        netInvestingCashFlowMinor: outputInteger(netInvestingCashFlow, 'period' + periodNumber + '.netInvestingCashFlowMinor'),
        financingCashInflowsMinor: period.financingCashInflowsMinor,
        financingCashOutflowsMinor: period.financingCashOutflowsMinor,
        netFinancingCashFlowMinor: outputInteger(netFinancingCashFlow, 'period' + periodNumber + '.netFinancingCashFlowMinor'),
        totalCashOutflowsMinor: outputInteger(totalCashOutflows, 'period' + periodNumber + '.totalCashOutflowsMinor'),
        netCashChangeMinor: outputInteger(netCashChange, 'period' + periodNumber + '.netCashChangeMinor'),
        closingCashMinor: outputInteger(cash, 'period' + periodNumber + '.closingCashMinor'),
        belowReserve: cash < reserveTarget,
        negativeCash: cash < 0n
      });
    }

    const minimumCashNumber = outputInteger(minimumCash, 'minimumCashMinor');
    const report = {
      schemaVersion: 2,
      kind: 'business-launchpad-cash-runway-report',
      currency: { ...model.currency },
      basis: 'Owner-supplied monthly cash receipts and payments in operating, investing and financing planning buckets plus an explicitly unclassified residual; not a demand forecast or accounting statement.',
      horizonMonths: normalizedPlan.length,
      startup: {
        openingCashBeforeSetupMinor: model.availableCashForStartupMinor,
        oneTimeSetupCashOutflowMinor: model.setupCostsMinor,
        cashAfterSetupBeforeMonthOneMinor: outputInteger(openingCash - setupOutflow, 'cashAfterSetupBeforeMonthOneMinor'),
        minimumCashReserveTargetMinor: model.initialWorkingCapitalBufferMinor
      },
      periods,
      summary: {
        endingCashMinor: outputInteger(cash, 'endingCashMinor'),
        minimumProjectedCashMinor: minimumCashNumber,
        minimumCashPeriod,
        firstNegativeCashPeriod,
        firstBelowReservePeriod,
        additionalCashNeededToMaintainReserveMinor: outputInteger(
          minimumCash < reserveTarget ? reserveTarget - minimumCash : 0n,
          'additionalCashNeededToMaintainReserveMinor'
        ),
        periodsBelowReserve: [
          ...(openingCash - setupOutflow < reserveTarget ? [0] : []),
          ...periods.filter(period => period.belowReserve).map(period => period.period)
        ],
        periodsWithNegativeCash: [
          ...(openingCash - setupOutflow < 0n ? [0] : []),
          ...periods.filter(period => period.negativeCash).map(period => period.period)
        ]
      },
      costCoverage: {
        level: model.costCoverage.level,
        omittedCategories: [...model.costCoverage.omittedCategories]
      },
      scopeLimitations: [
        'This is a deterministic owner-supplied cash scenario, not a demand prediction or guarantee of sales.',
        'Sales units are activity assumptions, not proof of cash collection. Enter salesCashReceiptsMinor for the month cash is expected to arrive; by default only, legacy plans derive receipts as if each sale is collected in the same month.',
        'Enter operatingCashOutflowsMinor for the month operating payments are expected to leave cash. By default only, legacy plans derive variable and fixed operating cash costs from same-month sales activity.',
        'The model does not maintain an accounts-receivable/accounts-payable ledger, age invoices, reconcile actual settlement, or model supplier-credit balances. User-supplied monthly cash timing is a scenario assumption.',
        'Setup costs are deducted once before month one. The working-capital amount is a reserve threshold, not an additional cash outflow.',
        'The legacy otherCashInflowsMinor/otherCashOutflowsMinor fields are unclassified residuals. They affect total cash, but do not contribute to the operating/investing/financing net-flow subtotals until an owner reclassifies the underlying item. This preserves ambiguous values imported from old scenario files rather than guessing their accounting category. Do not count one transaction in more than one bucket.',
        'Investing fields capture cash from asset/investment disposals and cash paid for long-term assets; financing fields capture cash from capital/borrowing and cash paid for owner distributions or principal repayments. Enter only cash movements, not non-cash transactions.',
        'These categories are planning labels, not accounting classification advice. Actual classification can depend on the transaction, applicable reporting framework and jurisdiction; this prototype is not an IAS 7 statement.',
        'Missing investing/financing fields in legacy plans are treated as zero for compatibility, which may leave their runway understated if a real capital or financing event was omitted. The generic other fields are not automatically assigned to a category.',
        'Each period uses one average offer for its activity assumption; product mix, seasonality, capacity and operational disruptions are not modeled.'
      ]
    };
    return { ok: true, report };
  } catch (error) {
    if (error instanceof RangeError && error.code === 'calculation_overflow') {
      return fail(error.code, error.message, error.field ?? null);
    }
    return fail('calculation_error', 'The cash-runway scenario could not be analyzed safely. No projection has been produced.');
  }
}

/**
 * Illustrative, owner-editable stress assumptions. These are not industry
 * benchmarks, probabilities, or a forecast.
 */
export const DEFAULT_CASH_SCENARIO_ASSUMPTIONS = Object.freeze({
  downside: Object.freeze({
    salesReceiptReductionPercent: 20,
    salesReceiptDelayMonths: 1,
    operatingCashOutflowIncreasePercent: 15
  }),
  upside: Object.freeze({
    salesReceiptIncreasePercent: 10,
    salesReceiptAdvanceMonths: 1,
    operatingCashOutflowReductionPercent: 5
  })
});

function isIntegerInRange(value, min, max) {
  return Number.isSafeInteger(value) && value >= min && value <= max;
}

function normalizeCashScenarioAssumptions(value) {
  const source = value === undefined ? DEFAULT_CASH_SCENARIO_ASSUMPTIONS : value;
  if (!isRecord(source) || !isRecord(source.downside) || !isRecord(source.upside)) {
    return { error: fail('invalid_cash_scenario_assumptions', 'Downside and upside cash-stress assumptions are required.', 'cashScenarioAssumptions') };
  }
  const fields = [
    ['downside', 'salesReceiptReductionPercent', 0, 100],
    ['downside', 'salesReceiptDelayMonths', 0, 24],
    ['downside', 'operatingCashOutflowIncreasePercent', 0, 500],
    ['upside', 'salesReceiptIncreasePercent', 0, 500],
    ['upside', 'salesReceiptAdvanceMonths', 0, 24],
    ['upside', 'operatingCashOutflowReductionPercent', 0, 100]
  ];
  const normalized = { downside: {}, upside: {} };
  for (const [group, field, min, max] of fields) {
    const amount = source[group][field];
    if (!isIntegerInRange(amount, min, max)) {
      return { error: fail(
        'invalid_cash_scenario_assumptions',
        group + '.' + field + ' must be an integer from ' + min + ' through ' + max + '.',
        'cashScenarioAssumptions.' + group + '.' + field
      ) };
    }
    normalized[group][field] = amount;
  }
  return { assumptions: normalized };
}

function scaleCashAmount(amount, factorPercent, roundUp, field) {
  const numerator = BigInt(amount) * BigInt(factorPercent);
  const scaled = roundUp ? (numerator + 99n) / 100n : numerator / 100n;
  if (scaled > MAX_SAFE) {
    const error = new RangeError('Calculated ' + field + ' exceeds the supported exact integer range.');
    error.code = 'calculation_overflow';
    error.field = field;
    throw error;
  }
  return Number(scaled);
}

function transformCashPlanForStress(basePeriods, config, mode) {
  const plans = basePeriods.map(period => ({
    salesUnits: period.salesUnitsScenario,
    salesCashReceiptsMinor: 0,
    operatingCashOutflowsMinor: period.operatingCashOutflowsMinor,
    otherCashInflowsMinor: period.otherCashInflowsMinor,
    otherCashOutflowsMinor: period.otherCashOutflowsMinor,
    investingCashInflowsMinor: period.investingCashInflowsMinor,
    investingCashOutflowsMinor: period.investingCashOutflowsMinor,
    financingCashInflowsMinor: period.financingCashInflowsMinor,
    financingCashOutflowsMinor: period.financingCashOutflowsMinor
  }));
  let deferredSalesCashReceipts = 0n;

  for (let index = 0; index < basePeriods.length; index += 1) {
    const period = basePeriods[index];
    const isDownside = mode === 'downside';
    const receiptFactor = isDownside
      ? 100 - config.salesReceiptReductionPercent
      : 100 + config.salesReceiptIncreasePercent;
    const receipts = scaleCashAmount(
      period.salesCashReceiptsMinor,
      receiptFactor,
      !isDownside,
      mode + '.month' + (index + 1) + '.salesCashReceiptsMinor'
    );
    const targetIndex = isDownside
      ? index + config.salesReceiptDelayMonths
      : Math.max(0, index - config.salesReceiptAdvanceMonths);
    if (targetIndex >= plans.length) {
      deferredSalesCashReceipts += BigInt(receipts);
    } else {
      const combined = BigInt(plans[targetIndex].salesCashReceiptsMinor) + BigInt(receipts);
      if (combined > MAX_SAFE) {
        const error = new RangeError('Combined stressed receipts exceed the supported exact integer range.');
        error.code = 'calculation_overflow';
        error.field = mode + '.month' + (targetIndex + 1) + '.salesCashReceiptsMinor';
        throw error;
      }
      plans[targetIndex].salesCashReceiptsMinor = Number(combined);
    }

    const outflowFactor = isDownside
      ? 100 + config.operatingCashOutflowIncreasePercent
      : 100 - config.operatingCashOutflowReductionPercent;
    plans[index].operatingCashOutflowsMinor = scaleCashAmount(
      period.operatingCashOutflowsMinor,
      outflowFactor,
      isDownside,
      mode + '.month' + (index + 1) + '.operatingCashOutflowsMinor'
    );
  }

  if (deferredSalesCashReceipts > MAX_SAFE) {
    const error = new RangeError('Deferred stressed receipts exceed the supported exact integer range.');
    error.code = 'calculation_overflow';
    error.field = mode + '.deferredSalesCashReceiptsAfterHorizonMinor';
    throw error;
  }
  return {
    monthlyPlan: plans,
    deferredSalesCashReceiptsAfterHorizonMinor: Number(deferredSalesCashReceipts)
  };
}

/**
 * Default terms-based timing schedule, expressed as integer percentages.
 * These are illustrative starting values rather than industry defaults.
 */
export const DEFAULT_CASH_TIMING_SCHEDULE = Object.freeze({
  salesReceiptsPercent: Object.freeze([70, 25, 5]),
  variableCostCategories: Object.freeze([
    Object.freeze({ id: 'category-1', label: 'Materials / components', costSharePercent: 60, paymentPercentages: Object.freeze([80, 15, 5]) }),
    Object.freeze({ id: 'category-2', label: 'Direct labor / subcontractors', costSharePercent: 25, paymentPercentages: Object.freeze([100, 0, 0]) }),
    Object.freeze({ id: 'category-3', label: 'Fulfillment / other variable costs', costSharePercent: 15, paymentPercentages: Object.freeze([70, 25, 5]) })
  ])
});

function normalizeCashTimingSchedule(value) {
  const source = value === undefined ? DEFAULT_CASH_TIMING_SCHEDULE : value;
  if (!isRecord(source)) {
    return { error: fail('invalid_cash_timing_schedule', 'A collection and supplier-payment timing schedule is required.', 'timingSchedule') };
  }
  const sales = source.salesReceiptsPercent;
  if (!Array.isArray(sales) || sales.length !== 3 ||
      sales.some(value => !isIntegerInRange(value, 0, 100)) ||
      sales.reduce((sum, value) => sum + value, 0) !== 100) {
    return { error: fail(
      'invalid_cash_timing_schedule',
      'salesReceiptsPercent must contain three integer percentages for this month, next month, and the month after that; the values must total exactly 100.',
      'timingSchedule.salesReceiptsPercent'
    ) };
  }

  let categories = source.variableCostCategories;
  if (categories === undefined) {
    // Legacy schema-v5 and earlier: applying the prior aggregate payment curve
    // to every category preserves the old total timing during migration.
    const legacy = source.variableSupplierPaymentsPercent;
    if (!Array.isArray(legacy) || legacy.length !== 3 ||
        legacy.some(value => !isIntegerInRange(value, 0, 100)) ||
        legacy.reduce((sum, value) => sum + value, 0) !== 100) {
      return { error: fail(
        'invalid_cash_timing_schedule',
        'Provide three variable-cost categories or a legacy variableSupplierPaymentsPercent allocation that totals exactly 100.',
        'timingSchedule.variableCostCategories'
      ) };
    }
    categories = DEFAULT_CASH_TIMING_SCHEDULE.variableCostCategories.map(category => ({
      ...category,
      paymentPercentages: [...legacy]
    }));
  }
  if (!Array.isArray(categories) || categories.length !== 3) {
    return { error: fail(
      'invalid_cash_timing_schedule',
      'variableCostCategories must contain exactly three categories so their cost shares and payment timing are explicit.',
      'timingSchedule.variableCostCategories'
    ) };
  }

  const normalizedCategories = [];
  const labels = new Set();
  let shareTotal = 0;
  for (let index = 0; index < categories.length; index += 1) {
    const category = categories[index];
    if (!isRecord(category) || category.id !== 'category-' + (index + 1)) {
      return { error: fail(
        'invalid_cash_timing_schedule',
        'Variable-cost category IDs must be category-1, category-2, and category-3 in their defined order.',
        'timingSchedule.variableCostCategories[' + index + '].id'
      ) };
    }
    const label = typeof category.label === 'string' ? category.label.trim() : '';
    if (!label || label.length > 80 || /[\u0000-\u001f\u007f]/.test(label) || labels.has(label.toLowerCase())) {
      return { error: fail(
        'invalid_cash_timing_schedule',
        'Each variable-cost category needs a unique, non-empty label of at most 80 characters.',
        'timingSchedule.variableCostCategories[' + index + '].label'
      ) };
    }
    if (!isIntegerInRange(category.costSharePercent, 0, 100)) {
      return { error: fail(
        'invalid_cash_timing_schedule',
        'Each variable-cost share must be a whole percentage from 0 through 100.',
        'timingSchedule.variableCostCategories[' + index + '].costSharePercent'
      ) };
    }
    const payments = category.paymentPercentages;
    if (!Array.isArray(payments) || payments.length !== 3 ||
        payments.some(value => !isIntegerInRange(value, 0, 100)) ||
        payments.reduce((sum, value) => sum + value, 0) !== 100) {
      return { error: fail(
        'invalid_cash_timing_schedule',
        'Each category payment allocation must contain three integer percentages that total exactly 100.',
        'timingSchedule.variableCostCategories[' + index + '].paymentPercentages'
      ) };
    }
    labels.add(label.toLocaleLowerCase());
    shareTotal += category.costSharePercent;
    normalizedCategories.push({
      id: category.id,
      label,
      costSharePercent: category.costSharePercent,
      paymentPercentages: [...payments]
    });
  }
  if (shareTotal !== 100) {
    return { error: fail(
      'invalid_cash_timing_schedule',
      'Variable-cost category shares must total exactly 100%.',
      'timingSchedule.variableCostCategories'
    ) };
  }

  return {
    schedule: {
      salesReceiptsPercent: [...sales],
      variableCostCategories: normalizedCategories
    },
    // Preserve the old aggregate allocation only when importing the legacy
    // single-curve shape. Schema-v6 category schedules should be honored
    // individually, even when the three curves happen to match.
    aggregatePaymentTiming: source.variableCostCategories === undefined
  };
}

function distributeMinorUnits(totalMinor, percentages) {
  const total = BigInt(totalMinor);
  const buckets = percentages.map((percent, index) => {
    const numerator = total * BigInt(percent);
    return {
      index,
      amount: numerator / 100n,
      remainder: numerator % 100n
    };
  });
  let remaining = total - buckets.reduce((sum, bucket) => sum + bucket.amount, 0n);
  const priority = [...buckets].sort((left, right) =>
    left.remainder === right.remainder
      ? left.index - right.index
      : left.remainder > right.remainder ? -1 : 1
  );
  for (let index = 0; remaining > 0n; index += 1, remaining -= 1n) {
    priority[index % priority.length].amount += 1n;
  }
  return buckets.sort((left, right) => left.index - right.index).map(bucket => Number(bucket.amount));
}

/**
 * Build an alternative cash plan from monthly sales activity and an explicit
 * three-bucket customer/supplier timing schedule. This must not overwrite the
 * owner's original receipts/payments. It models gross sales receipts and
 * variable supplier costs from the unit assumptions, keeps fixed monthly costs
 * due in their originating month, and carries other/investing/financing flows
 * from the supplied plan without recategorizing them.
 *
 * @param {{model:object,monthlyPlan:Array,timingSchedule?:object}} input
 * @returns {{ok:true,report:object}|{ok:false,error:object}}
 */
export function projectCashRunwayWithTimingSchedule(input) {
  if (!isRecord(input)) {
    return fail('invalid_cash_timing_input', 'A timing-schedule projection input object is required.');
  }
  const base = projectCashRunway({ model: input.model, monthlyPlan: input.monthlyPlan });
  if (!base.ok) return base;
  const normalized = normalizeCashTimingSchedule(input.timingSchedule);
  if (normalized.error) return normalized.error;

  const horizon = base.report.periods.length;
  const schedule = normalized.schedule;
  const categories = schedule.variableCostCategories;
  const aggregatePaymentTiming = normalized.aggregatePaymentTiming;
  const monthlyPlan = base.report.periods.map(period => ({
    salesUnits: period.salesUnitsScenario,
    salesCashReceiptsMinor: 0,
    operatingCashOutflowsMinor: input.model.fixedMonthlyCostsMinor,
    otherCashInflowsMinor: period.otherCashInflowsMinor,
    otherCashOutflowsMinor: period.otherCashOutflowsMinor,
    investingCashInflowsMinor: period.investingCashInflowsMinor,
    investingCashOutflowsMinor: period.investingCashOutflowsMinor,
    financingCashInflowsMinor: period.financingCashInflowsMinor,
    financingCashOutflowsMinor: period.financingCashOutflowsMinor
  }));
  let deferredSalesReceipts = 0n;
  let deferredSupplierPayments = 0n;
  const deferredByCategory = categories.map(() => 0n);
  const totalCostByCategory = categories.map(() => 0n);

  try {
    for (let cohort = 0; cohort < horizon; cohort += 1) {
      const activity = base.report.periods[cohort].salesUnitsScenario;
      const grossSales = BigInt(input.model.sellingPricePerUnitMinor) * BigInt(activity);
      const totalVariableCost = BigInt(input.model.variableCostPerUnitMinor) * BigInt(activity);
      if (grossSales > MAX_SAFE || totalVariableCost > MAX_SAFE) {
        return fail('calculation_overflow', 'Sales or variable supplier costs exceed the supported exact integer range.', 'monthlyPlan[' + cohort + ']');
      }
      const categoryCohortCosts = distributeMinorUnits(
        Number(totalVariableCost),
        categories.map(category => category.costSharePercent)
      );
      for (let categoryIndex = 0; categoryIndex < categoryCohortCosts.length; categoryIndex += 1) {
        const categoryTotal = totalCostByCategory[categoryIndex] + BigInt(categoryCohortCosts[categoryIndex]);
        if (categoryTotal > MAX_SAFE) {
          return fail('calculation_overflow', 'A variable-cost category total exceeds the supported exact integer range.', 'timingSchedule.variableCostCategories[' + categoryIndex + ']');
        }
        totalCostByCategory[categoryIndex] = categoryTotal;
      }
      const receipts = distributeMinorUnits(Number(grossSales), schedule.salesReceiptsPercent);
      for (let offset = 0; offset < 3; offset += 1) {
        const receiptMonth = cohort + offset;
        if (receiptMonth >= horizon) {
          deferredSalesReceipts += BigInt(receipts[offset]);
        } else {
          const sum = BigInt(monthlyPlan[receiptMonth].salesCashReceiptsMinor) + BigInt(receipts[offset]);
          if (sum > MAX_SAFE) {
            return fail('calculation_overflow', 'Scheduled sales receipts exceed the supported exact integer range.', 'monthlyPlan[' + receiptMonth + '].salesCashReceiptsMinor');
          }
          monthlyPlan[receiptMonth].salesCashReceiptsMinor = Number(sum);
        }
      }

      if (aggregatePaymentTiming) {
        // Legacy/equal curves are allocated once at aggregate level so rounding
        // is byte-for-byte equivalent to the former single-cohort schedule.
        const supplierPayments = distributeMinorUnits(Number(totalVariableCost), categories[0].paymentPercentages);
        for (let offset = 0; offset < 3; offset += 1) {
          const paymentMonth = cohort + offset;
          if (paymentMonth >= horizon) {
            const amount = supplierPayments[offset];
            deferredSupplierPayments += BigInt(amount);
            // A legacy aggregate schedule does not contain category-level due
            // dates. Do not manufacture per-category deferral amounts here.
          } else {
            const sum = BigInt(monthlyPlan[paymentMonth].operatingCashOutflowsMinor) + BigInt(supplierPayments[offset]);
            if (sum > MAX_SAFE) {
              return fail('calculation_overflow', 'Scheduled operating payments exceed the supported exact integer range.', 'monthlyPlan[' + paymentMonth + '].operatingCashOutflowsMinor');
            }
            monthlyPlan[paymentMonth].operatingCashOutflowsMinor = Number(sum);
          }
        }
      } else {
        for (let categoryIndex = 0; categoryIndex < categories.length; categoryIndex += 1) {
          const category = categories[categoryIndex];
          const categoryCohortCost = categoryCohortCosts[categoryIndex];
          const supplierPayments = distributeMinorUnits(categoryCohortCost, category.paymentPercentages);
          for (let offset = 0; offset < 3; offset += 1) {
            const paymentMonth = cohort + offset;
            if (paymentMonth >= horizon) {
              const amount = BigInt(supplierPayments[offset]);
              deferredSupplierPayments += amount;
              deferredByCategory[categoryIndex] += amount;
            } else {
              const sum = BigInt(monthlyPlan[paymentMonth].operatingCashOutflowsMinor) + BigInt(supplierPayments[offset]);
              if (sum > MAX_SAFE) {
                return fail('calculation_overflow', 'Scheduled operating payments exceed the supported exact integer range.', 'monthlyPlan[' + paymentMonth + '].operatingCashOutflowsMinor');
              }
              monthlyPlan[paymentMonth].operatingCashOutflowsMinor = Number(sum);
            }
          }
        }
      }
    }
    if (deferredSalesReceipts > MAX_SAFE || deferredSupplierPayments > MAX_SAFE ||
        deferredByCategory.some(amount => amount > MAX_SAFE)) {
      return fail('calculation_overflow', 'The timing-schedule amounts beyond the forecast horizon exceed the supported exact integer range.', 'timingSchedule');
    }
  } catch (error) {
    if (error instanceof RangeError) {
      return fail('calculation_overflow', 'The timing schedule could not be applied within the exact integer range.', 'timingSchedule');
    }
    return fail('calculation_error', 'The timing schedule could not be applied safely.', 'timingSchedule');
  }

  const projection = projectCashRunway({ model: input.model, monthlyPlan });
  if (!projection.ok) return projection;
  return {
    ok: true,
    report: {
      schemaVersion: 2,
      kind: 'business-launchpad-terms-based-cash-projection',
      currency: { ...projection.report.currency },
      horizonMonths: horizon,
      timingSchedule: schedule,
      variableCostCategoryBreakdown: categories.map((category, index) => ({
        ...category,
        variableCostTotalMinor: Number(totalCostByCategory[index]),
        deferredPaymentsAfterHorizonMinor: aggregatePaymentTiming ? null : Number(deferredByCategory[index])
      })),
      basis: 'Alternative terms-based scenario derived from sales-unit activity; it does not replace the owner-entered base plan. Variable cost per sale is allocated among three explicitly named categories; each category has its own payment timing. Fixed monthly costs remain due in their originating month.',
      assumptions: [
        'Sales cash receipts are derived from units multiplied by the entered selling price, then allocated to the sale month, next month and following month.',
        'Each sales cohort variable-cost total is split across three categories using owner-entered whole-percent shares; shares must total exactly 100%, and category costs reconcile to the entered per-sale variable cost multiplied by units. Category cash payments are then allocated independently to the originating month, next month and following month.',
        'Fixed monthly costs remain paid in their originating month; setup costs remain paid before Month 1.',
        'Other/unclassified, investing and financing movements are copied from the supplied plan without reclassification.',
        'This schedule does not model opening accounts receivable/payable, taxes, inventory purchases that precede sales, deposit/refund behavior, bad debt, invoice-level terms or day-level due dates. Category labels and distributions are user assumptions, not itemized supplier balances or a ledger.',
        ...(aggregatePaymentTiming ? [
          'This projection was migrated from a legacy single supplier-payment curve. The category names and cost shares are illustrative defaults; the aggregate timing is preserved, and per-category horizon deferrals are intentionally not assigned.'
        ] : []),
        'Percentage schedules are sensitivity assumptions, not a ledger or a forecast of actual settlement.'
      ],
      deferredSalesCashReceiptsAfterHorizonMinor: Number(deferredSalesReceipts),
      deferredVariableSupplierPaymentsAfterHorizonMinor: Number(deferredSupplierPayments),
      monthlyPlan,
      projection: projection.report
    }
  };
}

/**
 * Compare the owner-entered cash plan with explicit downside and upside stress
 * cases. This changes only sales receipts and operating cash payments according
 * to the accompanying visible assumptions. It does not alter sales activity,
 * other cash, investing flows, or financing flows.
 *
 * The downside case reduces expected sales receipts then delays them; receipts
 * shifted outside the selected horizon are reported instead of silently erased.
 * The upside case increases expected receipts and advances later-period receipts
 * toward earlier periods, leaving Month 1 in Month 1 because there is no
 * pre-horizon cash period in this contract.
 *
 * @param {{model:object,monthlyPlan:Array,assumptions?:object}} input
 * @returns {{ok:true,report:object}|{ok:false,error:object}}
 */
export function compareCashRunwayScenarios(input) {
  if (!isRecord(input)) {
    return fail('invalid_cash_scenario_input', 'A cash scenario comparison input object is required.');
  }
  const base = projectCashRunway({ model: input.model, monthlyPlan: input.monthlyPlan });
  if (!base.ok) return base;
  const normalized = normalizeCashScenarioAssumptions(input.assumptions);
  if (normalized.error) return normalized.error;

  try {
    const down = transformCashPlanForStress(base.report.periods, normalized.assumptions.downside, 'downside');
    const up = transformCashPlanForStress(base.report.periods, normalized.assumptions.upside, 'upside');
    const downsideReport = projectCashRunway({ model: input.model, monthlyPlan: down.monthlyPlan });
    if (!downsideReport.ok) return downsideReport;
    const upsideReport = projectCashRunway({ model: input.model, monthlyPlan: up.monthlyPlan });
    if (!upsideReport.ok) return upsideReport;

    const scenarios = [
      {
        id: 'downside',
        label: 'Downside stress',
        assumptions: { ...normalized.assumptions.downside },
        deferredSalesCashReceiptsAfterHorizonMinor: down.deferredSalesCashReceiptsAfterHorizonMinor,
        deferredVariableSupplierPaymentsAfterHorizonMinor: 0,
        report: downsideReport.report
      },
      {
        id: 'base',
        label: 'Base — entered plan',
        assumptions: null,
        deferredSalesCashReceiptsAfterHorizonMinor: 0,
        deferredVariableSupplierPaymentsAfterHorizonMinor: 0,
        report: base.report
      },
      {
        id: 'upside',
        label: 'Upside stress',
        assumptions: { ...normalized.assumptions.upside },
        deferredSalesCashReceiptsAfterHorizonMinor: 0,
        deferredVariableSupplierPaymentsAfterHorizonMinor: 0,
        report: upsideReport.report
      }
    ];
    let timingSchedule = null;
    if (input.timingSchedule !== undefined) {
      const termsBased = projectCashRunwayWithTimingSchedule({
        model: input.model,
        monthlyPlan: input.monthlyPlan,
        timingSchedule: input.timingSchedule
      });
      if (!termsBased.ok) return termsBased;
      timingSchedule = termsBased.report.timingSchedule;
      scenarios.push({
        id: 'terms-based',
        label: 'Terms-based timing',
        assumptions: termsBased.report.timingSchedule,
        deferredSalesCashReceiptsAfterHorizonMinor: termsBased.report.deferredSalesCashReceiptsAfterHorizonMinor,
        deferredVariableSupplierPaymentsAfterHorizonMinor: termsBased.report.deferredVariableSupplierPaymentsAfterHorizonMinor,
        report: termsBased.report.projection
      });
    }
    return {
      ok: true,
      report: {
        schemaVersion: 2,
        kind: 'business-launchpad-cash-scenario-comparison',
        currency: { ...base.report.currency },
        horizonMonths: base.report.horizonMonths,
        basis: 'Owner-authored base plan plus deterministic downside/upside stress transformations; no scenario is a prediction, probability or industry benchmark.',
        assumptions: normalized.assumptions,
        timingSchedule,
        scenarios: scenarios.map(scenario => ({
          id: scenario.id,
          label: scenario.label,
          assumptions: scenario.assumptions,
          deferredSalesCashReceiptsAfterHorizonMinor: scenario.deferredSalesCashReceiptsAfterHorizonMinor,
          deferredVariableSupplierPaymentsAfterHorizonMinor: scenario.deferredVariableSupplierPaymentsAfterHorizonMinor,
          summary: {
            endingCashMinor: scenario.report.summary.endingCashMinor,
            minimumProjectedCashMinor: scenario.report.summary.minimumProjectedCashMinor,
            minimumCashPeriod: scenario.report.summary.minimumCashPeriod,
            firstNegativeCashPeriod: scenario.report.summary.firstNegativeCashPeriod,
            firstBelowReservePeriod: scenario.report.summary.firstBelowReservePeriod,
            additionalCashNeededToMaintainReserveMinor: scenario.report.summary.additionalCashNeededToMaintainReserveMinor
          },
          report: scenario.report
        })),
        limitations: [
          'The base case is the owner-entered monthly cash plan; the stress cases apply only the displayed receipt and operating-outflow assumptions.',
          'Scenario parameters are illustrative until an owner replaces them with documented business-specific assumptions. They are not measured likelihoods, benchmarks or forecasts.',
          'The downside reduces sales receipts before delaying them. Receipts delayed beyond the horizon are reported separately and do not appear as cash collected within the horizon.',
          'The upside case advances receipts from later months toward earlier months, but leaves Month 1 receipts in Month 1 because this comparison has no pre-horizon period.',
          'Stress transformations change sales cash receipts and operating cash outflows only; sales activity, unclassified amounts, investing movements, financing movements and setup cash remain as entered.',
          'When supplied, the terms-based case is separate from the manually entered base plan: it derives receipts from unit activity and selling price, schedules variable supplier cost using the visible percentages, keeps fixed monthly costs due in their originating month, and carries other/investing/financing movements unchanged.',
          'Receipts and variable supplier payments moved beyond the horizon are reported separately; the comparison does not model opening receivables/payables, pre-sales inventory purchases, invoice aging, taxes or day-level due dates.',
          'No scenario establishes solvency, profitability, financing eligibility, demand, tax treatment or accounting compliance.'
        ]
      }
    };
  } catch (error) {
    if (error instanceof RangeError && error.code === 'calculation_overflow') {
      return fail(error.code, error.message, error.field ?? null);
    }
    return fail('calculation_error', 'The cash scenarios could not be compared safely. No comparison report has been produced.');
  }
}

/**
 * Format a signed minor-unit amount deterministically (not locale-specific).
 * The UI may localize display separately.
 *
 * @param {number} amountMinor Signed safe integer in minor units.
 * @param {{code: string, minorUnitExponent: number}} currency
 * @returns {string}
 */
export function formatMinorUnits(amountMinor, currency) {
  if (!Number.isSafeInteger(amountMinor)) throw new TypeError('amountMinor must be a signed safe integer.');
  if (!isRecord(currency) || typeof currency.code !== 'string' || !CURRENCY_CODE.test(currency.code)) {
    throw new TypeError('currency.code must be three uppercase ASCII letters.');
  }
  if (!Number.isInteger(currency.minorUnitExponent) || currency.minorUnitExponent < 0 || currency.minorUnitExponent > MAX_MINOR_UNIT_EXPONENT) {
    throw new TypeError('currency.minorUnitExponent must be an integer from 0 to ' + MAX_MINOR_UNIT_EXPONENT + '.');
  }

  const amount = BigInt(amountMinor);
  const negative = amount < 0n;
  const absolute = negative ? -amount : amount;
  const scale = 10n ** BigInt(currency.minorUnitExponent);
  const whole = absolute / scale;
  const remainder = absolute % scale;
  const suffix = currency.minorUnitExponent === 0
    ? ''
    : '.' + remainder.toString().padStart(currency.minorUnitExponent, '0');
  return currency.code + ' ' + (negative ? '-' : '') + whole.toString() + suffix;
}


/**
 * Parse a non-negative major-unit decimal string into exact integer minor units.
 * Designed for text input; never round a binary floating-point number.
 *
 * @param {string} value Decimal text such as "123.45".
 * @param {number} minorUnitExponent Number of fractional digits for the chosen currency.
 * @returns {{ok: true, amountMinor: number} | {ok: false, error: {code: string, message: string, field: string|null}}}
 */
export function parseMajorToMinorUnits(value, minorUnitExponent) {
  if (typeof value !== 'string') {
    return fail('invalid_money_text', 'Amount must be supplied as decimal text.');
  }
  if (value.length > MAX_MONEY_TEXT_LENGTH) {
    return fail('money_text_too_long', 'Amount text must be at most ' + MAX_MONEY_TEXT_LENGTH + ' characters; shorten it and retry.', 'amount');
  }
  if (!Number.isInteger(minorUnitExponent) || minorUnitExponent < 0 || minorUnitExponent > MAX_MINOR_UNIT_EXPONENT) {
    return fail('invalid_currency_precision', 'Currency minor-unit exponent must be an integer from 0 to ' + MAX_MINOR_UNIT_EXPONENT + '.', 'minorUnitExponent');
  }

  const text = value.trim();
  if (!/^\d+(?:\.\d+)?$/.test(text)) {
    return fail('invalid_money_text', 'Enter a non-negative decimal amount without a currency symbol or thousands separators.', 'amount');
  }

  const [wholeText, fractionText = ''] = text.split('.');
  if (fractionText.length > minorUnitExponent) {
    return fail('excess_fractional_digits', 'Amount has more fractional digits than the selected currency precision permits.', 'amount');
  }

  try {
    const scale = 10n ** BigInt(minorUnitExponent);
    const fraction = fractionText.padEnd(minorUnitExponent, '0');
    const amount = BigInt(wholeText) * scale + BigInt(fraction || '0');
    if (amount > MAX_SAFE) {
      return fail('calculation_overflow', 'Amount exceeds the supported exact integer range.', 'amount');
    }
    return { ok: true, amountMinor: Number(amount) };
  } catch {
    return fail('invalid_money_text', 'Amount could not be parsed exactly.', 'amount');
  }
}


const ACTUALS_CATEGORY_FIELDS = Object.freeze([
  Object.freeze({ category: 'sales_receipt', field: 'salesCashReceiptsMinor', direction: 'inflow', label: 'Sales cash receipts' }),
  Object.freeze({ category: 'operating_outflow', field: 'operatingCashOutflowsMinor', direction: 'outflow', label: 'Operating cash payments' }),
  Object.freeze({ category: 'other_inflow', field: 'otherCashInflowsMinor', direction: 'inflow', label: 'Other/unclassified inflows' }),
  Object.freeze({ category: 'other_outflow', field: 'otherCashOutflowsMinor', direction: 'outflow', label: 'Other/unclassified outflows' }),
  Object.freeze({ category: 'investing_inflow', field: 'investingCashInflowsMinor', direction: 'inflow', label: 'Investing inflows' }),
  Object.freeze({ category: 'investing_outflow', field: 'investingCashOutflowsMinor', direction: 'outflow', label: 'Investing outflows' }),
  Object.freeze({ category: 'financing_inflow', field: 'financingCashInflowsMinor', direction: 'inflow', label: 'Financing inflows' }),
  Object.freeze({ category: 'financing_outflow', field: 'financingCashOutflowsMinor', direction: 'outflow', label: 'Financing outflows' })
]);

function actualsMonthOrdinal(value) {
  if (typeof value !== 'string') return null;
  const match = /^(\d{4})-(0[1-9]|1[0-2])$/.exec(value);
  if (!match) return null;
  const year = Number(match[1]);
  return year < 1900 || year > 9999 ? null : year * 12 + Number(match[2]) - 1;
}

function validActualsDate(value) {
  if (typeof value !== 'string') return false;
  const match = /^(\d{4})-(0[1-9]|1[0-2])-(0[1-9]|[12]\d|3[01])$/.exec(value);
  if (!match) return false;
  const year = Number(match[1]), month = Number(match[2]), day = Number(match[3]);
  if (year < 1900 || year > 9999) return false;
  const leap = year % 4 === 0 && (year % 100 !== 0 || year % 400 === 0);
  const days = [31, leap ? 29 : 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31];
  return day <= days[month - 1];
}

function actualsDateMonthOrdinal(value) {
  return Number(value.slice(0, 4)) * 12 + Number(value.slice(5, 7)) - 1;
}

function actualsLastDayOfMonth(value) {
  const year = Number(value.slice(0, 4)), month = Number(value.slice(5, 7));
  const leap = year % 4 === 0 && (year % 100 !== 0 || year % 400 === 0);
  return String([31, leap ? 29 : 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31][month - 1]).padStart(2, '0');
}

/**
 * Compare dated owner-imported cash records with a monthly plan.
 * A complete month-range and exact opening/closing balance reconciliation are
 * required. The caller hashes the source bytes; a hash does not authenticate them.
 */
export function compareCashPlanToActuals(input) {
  if (!isRecord(input)) return fail('invalid_actuals_comparison_input', 'An actuals comparison input object is required.');
  const projection = projectCashRunway({ model: input.model, monthlyPlan: input.monthlyPlan });
  if (!projection.ok) return projection;
  const source = input.actualsSource;
  if (!isRecord(source) || source.schemaVersion !== 1 || source.kind !== 'business-launchpad-actuals-source') {
    return fail('invalid_actuals_source_schema', 'Actuals must use Business Launchpad actuals-source schema version 1.', 'actualsSource');
  }
  if (typeof input.sourceSha256 !== 'string' || !/^[a-f0-9]{64}$/.test(input.sourceSha256)) {
    return fail('invalid_actuals_source_digest', 'The source needs a lowercase 64-character SHA-256 digest.', 'sourceSha256');
  }
  if (!isRecord(source.currency) || source.currency.code !== projection.report.currency.code ||
      source.currency.minorUnitExponent !== projection.report.currency.minorUnitExponent) {
    return fail('actuals_currency_mismatch', 'Actuals currency and minor-unit precision must exactly match the cash plan.', 'actualsSource.currency');
  }
  const forecastStartOrdinal = actualsMonthOrdinal(input.forecastStartMonth);
  if (forecastStartOrdinal === null) return fail('invalid_forecast_start_month', 'Set Month 1 to a valid calendar month in YYYY-MM format.', 'forecastStartMonth');
  if (!isRecord(source.source)) return fail('invalid_actuals_source_metadata', 'Actuals source metadata is required.', 'actualsSource.source');
  const metadata = source.source;
  if (!['bank_statement_export', 'reconciled_cash_ledger_export'].includes(metadata.type)) {
    return fail('unsupported_actuals_source_type', 'Use a bank-statement export or reconciled cash-ledger export.', 'actualsSource.source.type');
  }
  if (typeof metadata.label !== 'string' || !metadata.label.trim() || metadata.label.trim().length > 120) {
    return fail('invalid_actuals_source_label', 'Describe the source file in 1–120 characters.', 'actualsSource.source.label');
  }
  if (typeof metadata.evidenceReference !== 'string' || !metadata.evidenceReference.trim() || metadata.evidenceReference.trim().length > 240) {
    return fail('invalid_actuals_evidence_reference', 'Provide a source-document reference of at most 240 characters.', 'actualsSource.source.evidenceReference');
  }
  if (metadata.completeMonthlyCoverage !== true) {
    return fail('incomplete_actuals_coverage', 'Confirm the source contains complete transactions for every full month in its coverage.', 'actualsSource.source.completeMonthlyCoverage');
  }
  const coverageStart = metadata.coverageStartDate, coverageEnd = metadata.coverageEndDate;
  if (!validActualsDate(coverageStart) || coverageStart.slice(8) !== '01' ||
      !validActualsDate(coverageEnd) || coverageEnd.slice(8) !== actualsLastDayOfMonth(coverageEnd)) {
    return fail('invalid_actuals_coverage_dates', 'Coverage must begin on the first day and end on the last day of a calendar month.', 'actualsSource.source');
  }
  const firstOrdinal = actualsDateMonthOrdinal(coverageStart), lastOrdinal = actualsDateMonthOrdinal(coverageEnd);
  const lastForecastOrdinal = forecastStartOrdinal + projection.report.horizonMonths - 1;
  if (firstOrdinal < forecastStartOrdinal || lastOrdinal > lastForecastOrdinal || lastOrdinal < firstOrdinal) {
    return fail('actuals_coverage_outside_plan', 'Actuals coverage must fall within the dated forecast horizon; map Month 1 correctly.', 'actualsSource.source.coverageStartDate');
  }
  if (!Number.isSafeInteger(metadata.openingCashBalanceMinor) || !Number.isSafeInteger(metadata.closingCashBalanceMinor)) {
    return fail('invalid_actuals_statement_balance', 'Opening and closing balances must be signed safe integers in currency minor units.', 'actualsSource.source');
  }
  if (!Array.isArray(source.transactions) || source.transactions.length > 50000) {
    return fail('invalid_actuals_transactions', 'Actuals must contain an array of at most 50,000 records.', 'actualsSource.transactions');
  }

  const categoryMap = new Map(ACTUALS_CATEGORY_FIELDS.map(item => [item.category, item]));
  const monthCount = lastOrdinal - firstOrdinal + 1;
  const totals = Array.from({ length: monthCount }, () => Object.fromEntries(ACTUALS_CATEGORY_FIELDS.map(item => [item.field, 0n])));
  const ids = new Set();

  try {
    for (let index = 0; index < source.transactions.length; index += 1) {
      const row = source.transactions[index], field = 'actualsSource.transactions[' + index + ']';
      if (!isRecord(row)) return fail('invalid_actuals_transaction', 'Each actuals row must be an object.', field);
      if (typeof row.transactionId !== 'string' || !row.transactionId.trim() || row.transactionId.length > 160 || ids.has(row.transactionId)) {
        return fail('duplicate_or_invalid_actuals_transaction_id', 'Each transaction needs a unique non-empty ID of at most 160 characters.', field + '.transactionId');
      }
      if (!validActualsDate(row.date) || row.date < coverageStart || row.date > coverageEnd) {
        return fail('actuals_transaction_outside_coverage', 'Every transaction date must be valid and inside declared coverage.', field + '.date');
      }
      if (row.currencyCode !== projection.report.currency.code) {
        return fail('actuals_currency_mismatch', 'Every transaction must use the plan currency; conversion is outside this prototype.', field + '.currencyCode');
      }
      const descriptor = categoryMap.get(row.category);
      if (!descriptor) return fail('invalid_actuals_category', 'Choose one of the eight supported cash-flow categories.', field + '.category');
      if (!isNonNegativeSafeInteger(row.amountMinor) || row.amountMinor === 0) {
        return fail('invalid_actuals_amount', 'Transaction amount must be a positive safe integer in currency minor units.', field + '.amountMinor');
      }
      if (typeof row.sourceReference !== 'string' || !row.sourceReference.trim() || row.sourceReference.length > 240) {
        return fail('missing_actuals_row_reference', 'Each transaction needs a statement, receipt, or ledger-entry reference.', field + '.sourceReference');
      }
      ids.add(row.transactionId);
      const monthIndex = actualsDateMonthOrdinal(row.date) - firstOrdinal;
      const sum = totals[monthIndex][descriptor.field] + BigInt(row.amountMinor);
      if (sum > MAX_SAFE) return fail('calculation_overflow', 'A monthly category total exceeds the supported exact integer range.', field + '.amountMinor');
      totals[monthIndex][descriptor.field] = sum;
    }

    let netMovement = 0n, actualOpeningCash = BigInt(metadata.openingCashBalanceMinor);
    const balanceDerivedMovement = BigInt(metadata.closingCashBalanceMinor) - actualOpeningCash;
    const periods = [];
    for (let index = 0; index < monthCount; index += 1) {
      const ordinal = firstOrdinal + index, forecastIndex = ordinal - forecastStartOrdinal;
      const year = Math.floor(ordinal / 12), monthNumber = ordinal - year * 12 + 1;
      const month = String(year).padStart(4, '0') + '-' + String(monthNumber).padStart(2, '0');
      const plan = projection.report.periods[forecastIndex], planned = {}, actual = {}, variance = {}, categories = [];
      let inflows = 0n, outflows = 0n;
      for (const descriptor of ACTUALS_CATEGORY_FIELDS) {
        const plannedMinor = plan[descriptor.field];
        const actualMinorBig = totals[index][descriptor.field];
        const actualMinor = outputInteger(actualMinorBig, month + '.' + descriptor.field + '.actualMinor');
        const difference = BigInt(actualMinor) - BigInt(plannedMinor);
        planned[descriptor.field] = plannedMinor;
        actual[descriptor.field] = actualMinor;
        variance[descriptor.field] = outputInteger(difference, month + '.' + descriptor.field + '.varianceMinor');
        if (descriptor.direction === 'inflow') inflows += actualMinorBig; else outflows += actualMinorBig;
        categories.push({
          category: descriptor.category, label: descriptor.label, direction: descriptor.direction,
          plannedMinor, actualMinor,
          actualMinusPlanMinor: outputInteger(difference, month + '.' + descriptor.field + '.varianceMinor')
        });
      }
      const actualNet = inflows - outflows, plannedNet = BigInt(plan.netCashChangeMinor);
      const actualClosingCash = actualOpeningCash + actualNet;
      const plannedOpeningCash = forecastIndex === 0
        ? BigInt(projection.report.startup.cashAfterSetupBeforeMonthOneMinor)
        : BigInt(projection.report.periods[forecastIndex - 1].closingCashMinor);
      const plannedClosingCash = BigInt(plan.closingCashMinor);
      netMovement += actualNet;
      periods.push({
        month, forecastPeriod: forecastIndex + 1, planned, actual, variance, categories,
        plannedNetCashMovementMinor: outputInteger(plannedNet, month + '.plannedNetCashMovementMinor'),
        actualNetCashMovementMinor: outputInteger(actualNet, month + '.actualNetCashMovementMinor'),
        netCashMovementVarianceMinor: outputInteger(actualNet - plannedNet, month + '.netCashMovementVarianceMinor'),
        plannedOpeningCashMinor: outputInteger(plannedOpeningCash, month + '.plannedOpeningCashMinor'),
        actualOpeningCashMinor: outputInteger(actualOpeningCash, month + '.actualOpeningCashMinor'),
        openingCashVarianceMinor: outputInteger(actualOpeningCash - plannedOpeningCash, month + '.openingCashVarianceMinor'),
        plannedClosingCashMinor: outputInteger(plannedClosingCash, month + '.plannedClosingCashMinor'),
        actualClosingCashMinor: outputInteger(actualClosingCash, month + '.actualClosingCashMinor'),
        closingCashVarianceMinor: outputInteger(actualClosingCash - plannedClosingCash, month + '.closingCashVarianceMinor')
      });
      actualOpeningCash = actualClosingCash;
    }

    const actualNetMovement = outputInteger(netMovement, 'source.actualNetCashMovementMinor');
    if (BigInt(metadata.closingCashBalanceMinor) - BigInt(metadata.openingCashBalanceMinor) !== netMovement) {
      return fail('actuals_balance_reconciliation_failed', 'Imported transactions do not reconcile to source opening and closing balances; review coverage, duplicates, omissions, and categories.', 'actualsSource.source.closingCashBalanceMinor');
    }
    if (outputInteger(actualOpeningCash, 'source.derivedClosingCashBalanceMinor') !== metadata.closingCashBalanceMinor) {
      return fail('actuals_balance_reconciliation_failed', 'Derived ending cash does not match source ending cash.', 'actualsSource.source.closingCashBalanceMinor');
    }
    return {
      ok: true,
      report: {
        schemaVersion: 1, kind: 'business-launchpad-actuals-variance-report',
        currency: { ...projection.report.currency }, forecastStartMonth: input.forecastStartMonth,
        source: {
          type: metadata.type, label: metadata.label.trim(), evidenceReference: metadata.evidenceReference.trim(),
          coverageStartDate: coverageStart, coverageEndDate: coverageEnd, completeMonthlyCoverage: true,
          openingCashBalanceMinor: metadata.openingCashBalanceMinor, closingCashBalanceMinor: metadata.closingCashBalanceMinor,
          transactionCount: source.transactions.length, sha256: input.sourceSha256, authenticity: 'not_independently_verified'
        },
        reconciliation: {
          status: 'passed', openingCashBalanceMinor: metadata.openingCashBalanceMinor,
          closingCashBalanceMinor: metadata.closingCashBalanceMinor, transactionNetMovementMinor: actualNetMovement,
          balanceDerivedNetMovementMinor: outputInteger(balanceDerivedMovement, 'source.balanceDerivedNetMovementMinor'), differenceMinor: 0
        },
        periods,
        interpretation: [
          'Category variance is actual minus plan. For outflows, positive means spending exceeded plan; for inflows, positive means receipts exceeded plan.',
          'Net cash movement variance is actual movement minus planned movement. Closing-balance variance also includes any opening-balance difference.',
          'The fingerprint identifies the imported bytes but does not authenticate the bank, ledger, records, or person supplying them.',
          'Source opening/closing balances reconcile over the selected range. Intermediate month-end actual balances are derived from transactions; only the range-end balance is separately supplied.',
          'This is a cash-timing comparison, not accounting recognition, tax reporting, profit analysis, invoice aging, or proof that the source export is complete.'
        ]
      }
    };
  } catch (error) {
    if (error instanceof RangeError && error.code === 'calculation_overflow') return fail(error.code, error.message, error.field ?? null);
    return fail('calculation_error', 'The actuals comparison could not be calculated safely. No report has been produced.');
  }
}
