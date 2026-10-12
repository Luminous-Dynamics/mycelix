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
  variableSupplierPaymentsPercent: Object.freeze([80, 15, 5])
});

function normalizeCashTimingSchedule(value) {
  const source = value === undefined ? DEFAULT_CASH_TIMING_SCHEDULE : value;
  if (!isRecord(source)) {
    return { error: fail('invalid_cash_timing_schedule', 'A collection and supplier-payment timing schedule is required.', 'timingSchedule') };
  }
  const normalized = {};
  for (const [field, label] of [
    ['salesReceiptsPercent', 'salesReceiptsPercent'],
    ['variableSupplierPaymentsPercent', 'variableSupplierPaymentsPercent']
  ]) {
    const values = source[field];
    if (!Array.isArray(values) || values.length !== 3 ||
        values.some(value => !isIntegerInRange(value, 0, 100)) ||
        values.reduce((sum, value) => sum + value, 0) !== 100) {
      return { error: fail(
        'invalid_cash_timing_schedule',
        label + ' must contain three integer percentages for this month, next month, and the month after that; the values must total exactly 100.',
        'timingSchedule.' + field
      ) };
    }
    normalized[field] = [...values];
  }
  return { schedule: normalized };
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

  try {
    for (let cohort = 0; cohort < horizon; cohort += 1) {
      const activity = base.report.periods[cohort].salesUnitsScenario;
      const grossSales = BigInt(input.model.sellingPricePerUnitMinor) * BigInt(activity);
      const variableSupplierCost = BigInt(input.model.variableCostPerUnitMinor) * BigInt(activity);
      if (grossSales > MAX_SAFE || variableSupplierCost > MAX_SAFE) {
        return fail('calculation_overflow', 'Sales or variable supplier costs exceed the supported exact integer range.', 'monthlyPlan[' + cohort + ']');
      }
      const receipts = distributeMinorUnits(Number(grossSales), normalized.schedule.salesReceiptsPercent);
      const supplierPayments = distributeMinorUnits(Number(variableSupplierCost), normalized.schedule.variableSupplierPaymentsPercent);

      for (let offset = 0; offset < 3; offset += 1) {
        const receiptMonth = cohort + offset;
        const paymentMonth = cohort + offset;
        if (receiptMonth >= horizon) {
          deferredSalesReceipts += BigInt(receipts[offset]);
        } else {
          const sum = BigInt(monthlyPlan[receiptMonth].salesCashReceiptsMinor) + BigInt(receipts[offset]);
          if (sum > MAX_SAFE) return fail('calculation_overflow', 'Scheduled sales receipts exceed the supported exact integer range.', 'monthlyPlan[' + receiptMonth + '].salesCashReceiptsMinor');
          monthlyPlan[receiptMonth].salesCashReceiptsMinor = Number(sum);
        }

        if (paymentMonth >= horizon) {
          deferredSupplierPayments += BigInt(supplierPayments[offset]);
        } else {
          const sum = BigInt(monthlyPlan[paymentMonth].operatingCashOutflowsMinor) + BigInt(supplierPayments[offset]);
          if (sum > MAX_SAFE) return fail('calculation_overflow', 'Scheduled operating payments exceed the supported exact integer range.', 'monthlyPlan[' + paymentMonth + '].operatingCashOutflowsMinor');
          monthlyPlan[paymentMonth].operatingCashOutflowsMinor = Number(sum);
        }
      }
    }
    if (deferredSalesReceipts > MAX_SAFE || deferredSupplierPayments > MAX_SAFE) {
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
      schemaVersion: 1,
      kind: 'business-launchpad-terms-based-cash-projection',
      currency: { ...projection.report.currency },
      horizonMonths: horizon,
      timingSchedule: normalized.schedule,
      basis: 'Alternative terms-based scenario derived from sales-unit activity; it does not replace the owner-entered base plan. Variable supplier costs are scheduled by cohort; fixed monthly costs remain due in their originating month.',
      assumptions: [
        'Sales cash receipts are derived from units multiplied by the entered selling price, then allocated to the sale month, next month and following month.',
        'Variable supplier payments are derived from units multiplied by the entered variable cost, then allocated to the purchase/sale cohort month, next month and following month.',
        'Fixed monthly costs remain paid in their originating month; setup costs remain paid before Month 1.',
        'Other/unclassified, investing and financing movements are copied from the owner's plan without reclassification.',
        'This schedule does not model opening accounts receivable/payable, taxes, inventory purchases that precede sales, deposit/refund behavior, bad debt, invoice-level terms or day-level due dates.',
        'The percentage schedule is an owner-supplied sensitivity assumption, not a ledger or forecast of actual settlement.'
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
        report: downsideReport.report
      },
      {
        id: 'base',
        label: 'Base — entered plan',
        assumptions: null,
        deferredSalesCashReceiptsAfterHorizonMinor: 0,
        report: base.report
      },
      {
        id: 'upside',
        label: 'Upside stress',
        assumptions: { ...normalized.assumptions.upside },
        deferredSalesCashReceiptsAfterHorizonMinor: 0,
        report: upsideReport.report
      }
    ];
    return {
      ok: true,
      report: {
        schemaVersion: 1,
        kind: 'business-launchpad-cash-scenario-comparison',
        currency: { ...base.report.currency },
        horizonMonths: base.report.horizonMonths,
        basis: 'Owner-authored base plan plus deterministic downside/upside stress transformations; no scenario is a prediction, probability or industry benchmark.',
        assumptions: normalized.assumptions,
        scenarios: scenarios.map(scenario => ({
          id: scenario.id,
          label: scenario.label,
          assumptions: scenario.assumptions,
          deferredSalesCashReceiptsAfterHorizonMinor: scenario.deferredSalesCashReceiptsAfterHorizonMinor,
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
