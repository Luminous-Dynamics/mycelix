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
 * Sales activity, sales cash receipts, and operating cash payments are separate
 * inputs. The browser generates same-month defaults for convenience, but callers
 * can independently model delayed customer collection or advance supplier payments.
 * Setup cost is deducted before month one; the working-capital buffer is a reserve
 * threshold rather than a second cash payment.
 *
 * Backward compatibility: legacy plan periods with only salesUnits and "other"
 * cash fields receive generated same-month sales receipts and operating payments.
 *
 * @param {{model:object, monthlyPlan:Array<{salesUnits:number,salesCashReceiptsMinor?:number,operatingCashOutflowsMinor?:number,otherCashInflowsMinor:number,otherCashOutflowsMinor:number}>}} input
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
    const salesCashReceiptsMinor = period.salesCashReceiptsMinor === undefined
      ? calculatedReceipts
      : period.salesCashReceiptsMinor;
    const operatingCashOutflowsMinor = period.operatingCashOutflowsMinor === undefined
      ? calculatedOperatingOutflows
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

    // A missing legacy cash field is derived from the same-month unit model. Fail
    // closed if this compatibility path would exceed the exact output range.
    if (
      (period.salesCashReceiptsMinor === undefined && calculatedReceipts > MAX_SAFE) ||
      (period.operatingCashOutflowsMinor === undefined && calculatedOperatingOutflows > MAX_SAFE)
    ) {
      return fail('calculation_overflow', 'A default monthly cash amount exceeds the supported exact integer range.', 'monthlyPlan[' + index + ']');
    }

    normalizedPlan.push({
      salesUnits: period.salesUnits,
      salesCashReceiptsMinor,
      operatingCashOutflowsMinor,
      otherCashInflowsMinor: period.otherCashInflowsMinor,
      otherCashOutflowsMinor: period.otherCashOutflowsMinor
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
      const totalCashReceipts = salesCashReceipts + otherInflows;
      const totalCashOutflows = operatingCashOutflows + otherOutflows;
      const netCashChange = totalCashReceipts - totalCashOutflows;
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
        totalCashReceiptsMinor: outputInteger(totalCashReceipts, 'period' + periodNumber + '.totalCashReceiptsMinor'),
        operatingCashOutflowsMinor: period.operatingCashOutflowsMinor,
        otherCashOutflowsMinor: period.otherCashOutflowsMinor,
        totalCashOutflowsMinor: outputInteger(totalCashOutflows, 'period' + periodNumber + '.totalCashOutflowsMinor'),
        netCashChangeMinor: outputInteger(netCashChange, 'period' + periodNumber + '.netCashChangeMinor'),
        closingCashMinor: outputInteger(cash, 'period' + periodNumber + '.closingCashMinor'),
        belowReserve: cash < reserveTarget,
        negativeCash: cash < 0n
      });
    }

    const minimumCashNumber = outputInteger(minimumCash, 'minimumCashMinor');
    const report = {
      schemaVersion: 1,
      kind: 'business-launchpad-cash-runway-report',
      currency: { ...model.currency },
      basis: 'Owner-supplied monthly cash receipts and payments; not a demand forecast.',
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
        'Taxes, owner draw, financing, debt service and other costs must be entered as monthly other cash outflows if they are to be included; omitted/unknown costs remain a limitation.',
        'Other monthly inflows/outflows are additive to sales cash receipts and operating cash payments; they do not replace or re-time those fields. Do not count an expense in both operatingCashOutflowsMinor and otherCashOutflowsMinor.',
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
