// Copyright (C) 2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

import { analyzeBusinessModel, compareCashPlanToActuals, compareCashRunwayScenarios, DEFAULT_CASH_SCENARIO_ASSUMPTIONS, DEFAULT_CASH_TIMING_SCHEDULE, formatMinorUnits, parseMajorToMinorUnits, projectCashRunway } from '../core/src/index.js';

const form = document.querySelector('#model-form');
const errorBox = document.querySelector('#input-error');
const resultRoot = document.querySelector('#result');
const resultStatus = document.querySelector('#result-status');
const coverageSelect = document.querySelector('#cost-coverage');
const omittedField = document.querySelector('#omitted-costs');
const downloadScenarioButton = document.querySelector('#download-scenario');
const loadScenarioInput = document.querySelector('#load-scenario-file');
const downloadReportButton = document.querySelector('#download-report');
const cashRunwayPeriodsRoot = document.querySelector('#cash-runway-periods');
const resetCashRunwayButton = document.querySelector('#reset-cash-runway');
const actualsForecastStartMonth = document.querySelector('#actuals-forecast-start-month');
const loadActualsInput = document.querySelector('#load-actuals-file');
const downloadActualsTemplateButton = document.querySelector('#download-actuals-template');
const downloadVarianceReportButton = document.querySelector('#download-variance-report');
const actualsStatus = document.querySelector('#actuals-status');
const actualsError = document.querySelector('#actuals-error');
const actualsResultRoot = document.querySelector('#actuals-result');
const CASH_RUNWAY_MONTHS = 12;

let latestModel = null;
let latestReport = null;
let latestCashRunwayPlan = null;
let latestCashRunway = null;
let latestCashScenarioAssumptions = null;
let latestCashScenarioComparison = null;
let latestCashTimingSchedule = null;
let latestActualsComparison = null;
let latestActualsSourcePayload = null;
let latestActualsSourceRawText = null;
let latestActualsSourceSha256 = null;
let latestActualsForecastStartMonth = null;

const moneyFields = [
  ['selling-price', 'sellingPricePerUnitMinor', 'Price per sale'],
  ['variable-cost', 'variableCostPerUnitMinor', 'Variable cost per sale'],
  ['fixed-monthly', 'fixedMonthlyCostsMinor', 'Fixed costs per month'],
  ['setup-costs', 'setupCostsMinor', 'One-time setup costs'],
  ['working-capital', 'initialWorkingCapitalBufferMinor', 'Initial working-capital buffer'],
  ['available-cash', 'availableCashForStartupMinor', 'Cash available for startup']
];

function element(tag, className, text) {
  const node = document.createElement(tag);
  if (className) node.className = className;
  if (text !== undefined) node.textContent = text;
  return node;
}

function clearResult() {
  resultRoot.replaceChildren();
  resultStatus.textContent = 'Calculation failed';
  resultStatus.className = 'status-pill status-muted';
  latestModel = null;
  latestReport = null;
  latestCashRunwayPlan = null;
  latestCashRunway = null;
  latestCashScenarioAssumptions = null;
  latestCashScenarioComparison = null;
  latestCashTimingSchedule = null;
  latestActualsComparison = null;
  latestActualsSourcePayload = null;
  latestActualsSourceRawText = null;
  latestActualsSourceSha256 = null;
  latestActualsForecastStartMonth = null;
  actualsResultRoot.replaceChildren();
  actualsStatus.textContent = 'Calculate the current plan before importing actuals.';
  actualsStatus.className = 'status-pill status-muted';
  actualsError.textContent = '';
  actualsError.hidden = true;
  downloadVarianceReportButton.disabled = true;
  downloadReportButton.disabled = true;
}

function showError(message) {
  errorBox.textContent = message;
  errorBox.hidden = false;
}

function hideError() {
  errorBox.textContent = '';
  errorBox.hidden = true;
}

function getFieldValue(id) {
  return document.getElementById(id).value;
}

function parseWholeUnits(raw) {
  const rawText = String(raw);
  if (rawText.length > 64) {
    return { ok: false, error: { message: 'The monthly sales quantity text is too long to analyze safely.' } };
  }
  const text = rawText.trim();
  if (!/^\d+$/.test(text)) {
    return { ok: false, error: { message: 'Enter a whole, non-negative number of sales per month.' } };
  }
  const amount = Number(text);
  if (!Number.isSafeInteger(amount)) {
    return { ok: false, error: { message: 'The monthly sales quantity is outside the supported exact integer range.' } };
  }
  return { ok: true, value: amount };
}

function describeCurrency() {
  const code = getFieldValue('currency-code').trim().toUpperCase();
  const exponentText = getFieldValue('minor-unit-exponent');
  if (!/^\d+$/.test(exponentText)) {
    throw new Error('Decimal places must be a whole number from 0 to 6.');
  }
  const minorUnitExponent = Number(exponentText);
  if (!Number.isInteger(minorUnitExponent) || minorUnitExponent < 0 || minorUnitExponent > 6) {
    throw new Error('Decimal places must be a whole number from 0 to 6.');
  }
  if (!/^[A-Z]{3}$/.test(code)) {
    throw new Error('Enter a three-letter currency code such as ZAR. This prototype checks format only, not currency support.');
  }
  return { code, minorUnitExponent };
}

function readModel() {
  const currency = describeCurrency();
  const model = { currency };
  for (const [id, key, label] of moneyFields) {
    const parsed = parseMajorToMinorUnits(getFieldValue(id), currency.minorUnitExponent);
    if (!parsed.ok) throw new Error(label + ': ' + parsed.error.message);
    model[key] = parsed.amountMinor;
  }

  const units = parseWholeUnits(getFieldValue('planned-units'));
  if (!units.ok) throw new Error(units.error.message);
  model.plannedUnitsPerMonth = units.value;

  const level = getFieldValue('cost-coverage');
  const omittedCategories = level === 'partial'
    ? getFieldValue('omitted-costs')
        .split(',')
        .map(value => value.trim())
        .filter(Boolean)
    : [];
  model.costCoverage = { level, omittedCategories };
  return model;
}

function minorToMajorInput(amountMinor, minorUnitExponent) {
  const amount = BigInt(amountMinor);
  const scale = 10n ** BigInt(minorUnitExponent);
  const whole = amount / scale;
  const remainder = amount % scale;
  return minorUnitExponent === 0
    ? whole.toString()
    : whole.toString() + '.' + remainder.toString().padStart(minorUnitExponent, '0');
}

function ceilRatioUnits(value, numerator, denominator) {
  const n = BigInt(value) * BigInt(numerator);
  const d = BigInt(denominator);
  return Number((n + d - 1n) / d);
}

function safeDefaultCashAmounts(model, units, month) {
  const salesReceipts = BigInt(model.sellingPricePerUnitMinor) * BigInt(units);
  const operatingOutflows = BigInt(model.variableCostPerUnitMinor) * BigInt(units)
    + BigInt(model.fixedMonthlyCostsMinor);
  if (salesReceipts > BigInt(Number.MAX_SAFE_INTEGER) || operatingOutflows > BigInt(Number.MAX_SAFE_INTEGER)) {
    throw new Error('The generated Month ' + month + ' cash default exceeds the supported exact integer range. Enter a smaller sales assumption or edit the cash values explicitly.');
  }
  return {
    salesCashReceiptsMinor: Number(salesReceipts),
    operatingCashOutflowsMinor: Number(operatingOutflows)
  };
}

function defaultCashRunwayPlan(model) {
  const targetUnits = model.plannedUnitsPerMonth;
  return Array.from({ length: CASH_RUNWAY_MONTHS }, (_, index) => {
    const month = index + 1;
    const salesUnits = index === 0
      ? ceilRatioUnits(targetUnits, 1, 2)
      : index === 1
        ? ceilRatioUnits(targetUnits, 3, 4)
        : targetUnits;
    return {
      salesUnits,
      ...safeDefaultCashAmounts(model, salesUnits, month),
      otherCashInflowsMinor: 0,
      otherCashOutflowsMinor: 0,
      investingCashInflowsMinor: 0,
      investingCashOutflowsMinor: 0,
      financingCashInflowsMinor: 0,
      financingCashOutflowsMinor: 0
    };
  });
}

function renderCashRunwayInputs(plan, currency) {
  cashRunwayPeriodsRoot.replaceChildren();
  plan.forEach((period, index) => {
    const month = index + 1;
    const row = element('tr');
    const heading = element('th', '', 'Month ' + month);
    heading.setAttribute('scope', 'row');
    row.append(heading);

    const salesInput = document.createElement('input');
    salesInput.id = 'cash-month-' + month + '-sales';
    salesInput.name = salesInput.id;
    salesInput.type = 'text';
    salesInput.inputMode = 'numeric';
    salesInput.autocomplete = 'off';
    salesInput.value = String(period.salesUnits);
    salesInput.setAttribute('aria-label', 'Month ' + month + ' sales units activity assumption');
    const salesCell = element('td');
    salesCell.append(salesInput);
    row.append(salesCell);

    for (const [suffix, value, label] of [
      ['sales-receipts', period.salesCashReceiptsMinor, 'sales cash receipts expected this month'],
      ['operating-outflows', period.operatingCashOutflowsMinor, 'operating cash payments expected this month'],
      ['inflows', period.otherCashInflowsMinor, 'other/unclassified cash inflows'],
      ['outflows', period.otherCashOutflowsMinor, 'other/unclassified cash outflows'],
      ['investing-inflows', period.investingCashInflowsMinor ?? 0, 'investing cash inflows (for example, asset sales)'],
      ['investing-outflows', period.investingCashOutflowsMinor ?? 0, 'investing cash outflows (for example, equipment purchases)'],
      ['financing-inflows', period.financingCashInflowsMinor ?? 0, 'financing cash inflows (for example, loan proceeds or owner capital)'],
      ['financing-outflows', period.financingCashOutflowsMinor ?? 0, 'financing cash outflows (for example, principal repayments or owner distributions)']
    ]) {
      const moneyInput = document.createElement('input');
      moneyInput.id = 'cash-month-' + month + '-' + suffix;
      moneyInput.name = moneyInput.id;
      moneyInput.type = 'text';
      moneyInput.inputMode = 'decimal';
      moneyInput.autocomplete = 'off';
      moneyInput.value = minorToMajorInput(value, currency.minorUnitExponent);
      moneyInput.setAttribute('aria-label', 'Month ' + month + ' ' + label);
      const cell = element('td');
      cell.append(moneyInput);
      row.append(cell);
    }
    cashRunwayPeriodsRoot.append(row);
  });
}

function readCashRunwayPlan(currency) {
  return Array.from({ length: CASH_RUNWAY_MONTHS }, (_, index) => {
    const month = index + 1;
    const parsedUnits = parseWholeUnits(getFieldValue('cash-month-' + month + '-sales'));
    if (!parsedUnits.ok) {
      throw new Error('Month ' + month + ' sales units: ' + parsedUnits.error.message);
    }
    const moneyFieldsForMonth = [
      ['sales-receipts', 'salesCashReceiptsMinor', 'sales cash receipts'],
      ['operating-outflows', 'operatingCashOutflowsMinor', 'operating cash outflows'],
      ['inflows', 'otherCashInflowsMinor', 'other/unclassified cash inflows'],
      ['outflows', 'otherCashOutflowsMinor', 'other/unclassified cash outflows'],
      ['investing-inflows', 'investingCashInflowsMinor', 'investing cash inflows'],
      ['investing-outflows', 'investingCashOutflowsMinor', 'investing cash outflows'],
      ['financing-inflows', 'financingCashInflowsMinor', 'financing cash inflows'],
      ['financing-outflows', 'financingCashOutflowsMinor', 'financing cash outflows']
    ];
    const period = { salesUnits: parsedUnits.value };
    for (const [suffix, key, label] of moneyFieldsForMonth) {
      const parsed = parseMajorToMinorUnits(
        getFieldValue('cash-month-' + month + '-' + suffix),
        currency.minorUnitExponent
      );
      if (!parsed.ok) throw new Error('Month ' + month + ' ' + label + ': ' + parsed.error.message);
      period[key] = parsed.amountMinor;
    }
    return period;
  });
}

function validateImportedCashRunwayPlan(plan, model, schemaVersion) {
  if (!Array.isArray(plan) || plan.length !== CASH_RUNWAY_MONTHS) {
    throw new Error('Cash runway data must contain exactly 12 monthly entries.');
  }
  const normalized = plan.map((period, index) => {
    if (
      schemaVersion >= 2 &&
      period !== null &&
      typeof period === 'object' &&
      !Array.isArray(period) &&
      (period.salesCashReceiptsMinor === undefined || period.operatingCashOutflowsMinor === undefined)
    ) {
      throw new Error('Version ' + schemaVersion + ' cash runway Month ' + (index + 1) + ' must include sales cash receipts and operating cash outflows.');
    }
    const newCategoryFields = [
      'investingCashInflowsMinor',
      'investingCashOutflowsMinor',
      'financingCashInflowsMinor',
      'financingCashOutflowsMinor'
    ];
    if (
      schemaVersion >= 3 &&
      period !== null &&
      typeof period === 'object' &&
      !Array.isArray(period) &&
      newCategoryFields.some(field => period[field] === undefined)
    ) {
      throw new Error('Version ' + schemaVersion + ' cash runway Month ' + (index + 1) + ' must include all investing and financing cash fields.');
    }
    if (
      period === null ||
      typeof period !== 'object' ||
      Array.isArray(period) ||
      !Number.isSafeInteger(period.salesUnits) ||
      period.salesUnits < 0 ||
      !Number.isSafeInteger(period.otherCashInflowsMinor) ||
      period.otherCashInflowsMinor < 0 ||
      !Number.isSafeInteger(period.otherCashOutflowsMinor) ||
      period.otherCashOutflowsMinor < 0
    ) {
      throw new Error('Cash runway Month ' + (index + 1) + ' contains an invalid amount or sales quantity.');
    }
    const defaults = (
      period.salesCashReceiptsMinor === undefined ||
      period.operatingCashOutflowsMinor === undefined
    ) ? safeDefaultCashAmounts(model, period.salesUnits, index + 1) : null;
    const salesCashReceiptsMinor = period.salesCashReceiptsMinor === undefined
      ? defaults.salesCashReceiptsMinor
      : period.salesCashReceiptsMinor;
    const operatingCashOutflowsMinor = period.operatingCashOutflowsMinor === undefined
      ? defaults.operatingCashOutflowsMinor
      : period.operatingCashOutflowsMinor;
    if (!Number.isSafeInteger(salesCashReceiptsMinor) || salesCashReceiptsMinor < 0) {
      throw new Error('Cash runway Month ' + (index + 1) + ' has invalid sales cash receipts.');
    }
    if (!Number.isSafeInteger(operatingCashOutflowsMinor) || operatingCashOutflowsMinor < 0) {
      throw new Error('Cash runway Month ' + (index + 1) + ' has invalid operating cash outflows.');
    }
    const categories = {};
    for (const field of newCategoryFields) {
      const value = period[field] === undefined ? 0 : period[field];
      if (!Number.isSafeInteger(value) || value < 0) {
        throw new Error('Cash runway Month ' + (index + 1) + ' has an invalid ' + field + '.');
      }
      categories[field] = value;
    }
    return {
      salesUnits: period.salesUnits,
      salesCashReceiptsMinor,
      operatingCashOutflowsMinor,
      otherCashInflowsMinor: period.otherCashInflowsMinor,
      otherCashOutflowsMinor: period.otherCashOutflowsMinor,
      ...categories
    };
  });
  const result = projectCashRunway({ model, monthlyPlan: normalized });
  if (!result.ok) throw new Error('Cash runway validation failed: ' + result.error.message);
  return normalized;
}

function applyModelToForm(model) {
  document.getElementById('currency-code').value = model.currency.code;
  document.getElementById('minor-unit-exponent').value = String(model.currency.minorUnitExponent);
  for (const [id, key] of moneyFields) {
    document.getElementById(id).value = minorToMajorInput(model[key], model.currency.minorUnitExponent);
  }
  document.getElementById('planned-units').value = String(model.plannedUnitsPerMonth);
  coverageSelect.value = model.costCoverage.level;
  omittedField.value = model.costCoverage.omittedCategories.join(', ');
  syncCoverageInput();
}

function downloadJson(filename, payload) {
  const blob = new Blob([JSON.stringify(payload, null, 2) + '\n'], { type: 'application/json;charset=utf-8' });
  const objectUrl = URL.createObjectURL(blob);
  const link = element('a');
  link.href = objectUrl;
  link.download = filename;
  link.hidden = true;
  document.body.append(link);
  link.click();
  link.remove();
  globalThis.setTimeout(() => URL.revokeObjectURL(objectUrl), 1000);
}

function validatedScenarioPayload(payload) {
  if (
    payload === null ||
    typeof payload !== 'object' ||
    ![1, 2, 3, 4, 5, 6].includes(payload.schemaVersion) ||
    payload.kind !== 'business-launchpad-scenario' ||
    payload.input === null ||
    typeof payload.input !== 'object' ||
    Array.isArray(payload.input)
  ) {
    throw new Error('This file is not a supported Business Launchpad scenario JSON file (schemaVersion 1 through 6 required).');
  }

  const result = analyzeBusinessModel(payload.input);
  if (!result.ok) {
    throw new Error('Scenario validation failed: ' + result.error.message);
  }
  if (payload.schemaVersion >= 2 && payload.cashRunwayPlan === undefined) {
    throw new Error('Version ' + payload.schemaVersion + ' scenarios must include the 12-month cash plan.');
  }
  if (payload.schemaVersion >= 4 && payload.cashScenarioAssumptions === undefined) {
    throw new Error('Version ' + payload.schemaVersion + ' scenarios must include explicit cash stress assumptions.');
  }
  if (payload.schemaVersion >= 5 && payload.cashTimingSchedule === undefined) {
    throw new Error('Version ' + payload.schemaVersion + ' scenarios must include explicit collection and supplier-payment schedules.');
  }
  if (payload.schemaVersion >= 6 &&
      (!payload.cashTimingSchedule || !Array.isArray(payload.cashTimingSchedule.variableCostCategories))) {
    throw new Error('Version 6 scenarios must include three explicit variable-cost categories with independent payment schedules.');
  }
  const plan = payload.cashRunwayPlan === undefined
    ? defaultCashRunwayPlan(payload.input)
    : validateImportedCashRunwayPlan(payload.cashRunwayPlan, payload.input, payload.schemaVersion);
  const assumptions = payload.schemaVersion >= 4
    ? payload.cashScenarioAssumptions
    : DEFAULT_CASH_SCENARIO_ASSUMPTIONS;
  const timingSchedule = payload.schemaVersion >= 5 ? payload.cashTimingSchedule : DEFAULT_CASH_TIMING_SCHEDULE;
  const comparison = compareCashRunwayScenarios({ model: payload.input, monthlyPlan: plan, assumptions, timingSchedule });
  if (!comparison.ok) {
    throw new Error('Cash scenario assumptions failed validation: ' + comparison.error.message);
  }
  return {
    input: payload.input,
    cashRunwayPlan: plan,
    cashScenarioAssumptions: comparison.report.assumptions,
    cashTimingSchedule: comparison.report.timingSchedule
  };
}

function addMetric(list, label, value, extraClass = '') {
  const item = element('div', 'metric');
  item.append(element('dt', '', label));
  item.append(element('dd', extraClass, value));
  list.append(item);
}

function formatCashPeriod(period) {
  if (period === 0) return 'After setup, before Month 1';
  return 'Month ' + period;
}

function readCashScenarioAssumptions() {
  const readBoundedInteger = (id, label, min, max) => {
    const parsed = parseWholeUnits(getFieldValue(id));
    if (!parsed.ok || parsed.value < min || parsed.value > max) {
      throw new Error(label + ' must be a whole number from ' + min + ' through ' + max + '.');
    }
    return parsed.value;
  };
  return {
    downside: {
      salesReceiptReductionPercent: readBoundedInteger(
        'downside-receipt-reduction-percent', 'Downside sales receipt reduction', 0, 100
      ),
      salesReceiptDelayMonths: readBoundedInteger(
        'downside-receipt-delay-months', 'Downside receipt delay', 0, 24
      ),
      operatingCashOutflowIncreasePercent: readBoundedInteger(
        'downside-operating-cost-increase-percent', 'Downside operating payment increase', 0, 500
      )
    },
    upside: {
      salesReceiptIncreasePercent: readBoundedInteger(
        'upside-receipt-increase-percent', 'Upside sales receipt increase', 0, 500
      ),
      salesReceiptAdvanceMonths: readBoundedInteger(
        'upside-receipt-advance-months', 'Upside receipt advance', 0, 24
      ),
      operatingCashOutflowReductionPercent: readBoundedInteger(
        'upside-operating-cost-reduction-percent', 'Upside operating payment reduction', 0, 100
      )
    }
  };
}

function readCashTimingSchedule() {
  const readPercent = (id, label) => {
    const parsed = parseWholeUnits(getFieldValue(id));
    if (!parsed.ok || parsed.value < 0 || parsed.value > 100) {
      throw new Error(label + ' must be a whole percentage from 0 through 100.');
    }
    return parsed.value;
  };
  const variableCostCategories = [1, 2, 3].map(index => ({
    id: 'category-' + index,
    label: getFieldValue('timing-cost-category-' + index + '-label').trim(),
    costSharePercent: readPercent('timing-cost-category-' + index + '-share', 'Variable-cost category ' + index + ' share'),
    paymentPercentages: [
      readPercent('timing-cost-category-' + index + '-month-0', 'Category ' + index + ' payment this month'),
      readPercent('timing-cost-category-' + index + '-month-1', 'Category ' + index + ' payment next month'),
      readPercent('timing-cost-category-' + index + '-month-2', 'Category ' + index + ' payment two months later')
    ]
  }));
  return {
    salesReceiptsPercent: [
      readPercent('timing-sales-receipts-month-0', 'Collections in sale month'),
      readPercent('timing-sales-receipts-month-1', 'Collections next month'),
      readPercent('timing-sales-receipts-month-2', 'Collections two months later')
    ],
    variableCostCategories
  };
}

function applyCashTimingSchedule(schedule) {
  const fields = [
    ['timing-sales-receipts-month-0', schedule.salesReceiptsPercent[0]],
    ['timing-sales-receipts-month-1', schedule.salesReceiptsPercent[1]],
    ['timing-sales-receipts-month-2', schedule.salesReceiptsPercent[2]]
  ];
  for (let index = 0; index < 3; index += 1) {
    const category = schedule.variableCostCategories[index];
    const slot = index + 1;
    fields.push(
      ['timing-cost-category-' + slot + '-label', category.label],
      ['timing-cost-category-' + slot + '-share', category.costSharePercent],
      ['timing-cost-category-' + slot + '-month-0', category.paymentPercentages[0]],
      ['timing-cost-category-' + slot + '-month-1', category.paymentPercentages[1]],
      ['timing-cost-category-' + slot + '-month-2', category.paymentPercentages[2]]
    );
  }
  for (const [id, value] of fields) document.getElementById(id).value = String(value);
}

function applyCashScenarioAssumptions(assumptions) {
  const fields = [
    ['downside-receipt-reduction-percent', assumptions.downside.salesReceiptReductionPercent],
    ['downside-receipt-delay-months', assumptions.downside.salesReceiptDelayMonths],
    ['downside-operating-cost-increase-percent', assumptions.downside.operatingCashOutflowIncreasePercent],
    ['upside-receipt-increase-percent', assumptions.upside.salesReceiptIncreasePercent],
    ['upside-receipt-advance-months', assumptions.upside.salesReceiptAdvanceMonths],
    ['upside-operating-cost-reduction-percent', assumptions.upside.operatingCashOutflowReductionPercent]
  ];
  for (const [id, value] of fields) document.getElementById(id).value = String(value);
}

function renderCashScenarioComparison(comparison, currency) {
  const section = element('section', 'cash-runway-result cash-scenario-comparison');
  section.append(element('h3', 'section-subheading', 'Downside / base / upside comparison'));
  const downside = comparison.assumptions.downside;
  const upside = comparison.assumptions.upside;
  const timing = comparison.timingSchedule;
  const categoryTiming = timing.variableCostCategories.map(category =>
    category.label + ' (' + category.costSharePercent + '% of per-sale variable cost): ' +
    category.paymentPercentages.join(' / ') + '% across originating month, +1 month and +2 months'
  ).join('; ');
  section.append(element(
    'p',
    'detail',
    'The base is your entered plan. Downside stress: sales receipts reduced ' + downside.salesReceiptReductionPercent + '%, delayed ' + downside.salesReceiptDelayMonths + ' month(s), and operating payments increased ' + downside.operatingCashOutflowIncreasePercent + '%. Upside stress: sales receipts increased ' + upside.salesReceiptIncreasePercent + '%, later receipts advanced ' + upside.salesReceiptAdvanceMonths + ' month(s), and operating payments reduced ' + upside.operatingCashOutflowReductionPercent + '%. Terms-based timing schedules customer collections ' + timing.salesReceiptsPercent.join(' / ') + '% across sale month, +1 month and +2 months. Variable cost per sale is allocated by cost share, with independent payment curves: ' + categoryTiming + '. Fixed monthly costs remain due in their originating month. These owner-editable values are illustrative, not benchmarks or probabilities. A negative or below-reserve result is an early-warning signal to review, not an automatic business decision.'
  ));

  const scroll = element('div', 'cash-plan-scroll');
  scroll.setAttribute('role', 'region');
  scroll.setAttribute('aria-label', 'Cash scenario comparison');
  scroll.setAttribute('tabindex', '0');
  const table = element('table', 'cash-scenario-table');
  const head = element('thead');
  const headerRow = element('tr');
  for (const label of [
    'Case',
    'Lowest cash',
    'Period of lowest cash',
    'Ending cash',
    'First reserve breach',
    'First negative balance',
    'Extra cash to maintain reserve',
    'Receipts delayed beyond horizon',
    'Variable-cost payments after horizon'
  ]) {
    const th = element('th', '', label);
    th.setAttribute('scope', 'col');
    headerRow.append(th);
  }
  head.append(headerRow);
  table.append(head);

  const body = element('tbody');
  for (const scenario of comparison.scenarios) {
    const summary = scenario.summary;
    const row = element('tr');
    const heading = element('th', '', scenario.label);
    heading.setAttribute('scope', 'row');
    row.append(heading);
    row.append(element('td', summary.minimumProjectedCashMinor < 0 ? 'negative' : '',
      formatMinorUnits(summary.minimumProjectedCashMinor, currency)));
    row.append(element('td', '', formatCashPeriod(summary.minimumCashPeriod)));
    row.append(element('td', summary.endingCashMinor < 0 ? 'negative' : '',
      formatMinorUnits(summary.endingCashMinor, currency)));
    row.append(element('td', summary.firstBelowReservePeriod === null ? '' : 'negative',
      summary.firstBelowReservePeriod === null ? 'None' : formatCashPeriod(summary.firstBelowReservePeriod)));
    row.append(element('td', summary.firstNegativeCashPeriod === null ? '' : 'negative',
      summary.firstNegativeCashPeriod === null ? 'None' : formatCashPeriod(summary.firstNegativeCashPeriod)));
    row.append(element('td', summary.additionalCashNeededToMaintainReserveMinor > 0 ? 'negative' : '',
      formatMinorUnits(summary.additionalCashNeededToMaintainReserveMinor, currency)));
    row.append(element('td', scenario.deferredSalesCashReceiptsAfterHorizonMinor > 0 ? 'negative' : '',
      formatMinorUnits(scenario.deferredSalesCashReceiptsAfterHorizonMinor, currency)));
    row.append(element('td', scenario.deferredVariableSupplierPaymentsAfterHorizonMinor > 0 ? 'negative' : '',
      formatMinorUnits(scenario.deferredVariableSupplierPaymentsAfterHorizonMinor, currency)));
    body.append(row);
  }
  table.append(body);
  scroll.append(table);
  section.append(scroll);
  section.append(element(
    'p',
    'form-footnote',
    'The downside case reduces receipts before delaying them. Any receipts pushed past Month 12 are reported separately and are not counted as collected during the projection. The upside case moves later receipts earlier only within the selected horizon; Month 1 receipts stay in Month 1. Stress cases leave the entered sales activity, unclassified movements, investing movements and financing movements unchanged. Nothing here is a probability-weighted forecast.'
  ));
  return section;
}

function renderCashRunwayReport(runway) {
  const section = element('section', 'cash-runway-result');
  section.append(element('h3', 'section-subheading', '12-month cash runway scenario'));
  section.append(element(
    'p',
    'detail',
    'An owner-supplied cash scenario, not a sales forecast. Sales activity, collections, operating payments, investing movements and financing movements are separate. Legacy generic “other” cash amounts stay unclassified and still affect closing cash, but are not included in any category subtotal. Setup costs are paid before Month 1; the working-capital buffer is a reserve target. These categories are planning aids, not an accounting classification opinion.'
  ));

  const summary = element('dl', 'metric-list');
  addMetric(summary, 'Cash after setup', formatMinorUnits(runway.startup.cashAfterSetupBeforeMonthOneMinor, runway.currency),
    runway.startup.cashAfterSetupBeforeMonthOneMinor < 0 ? 'negative' : '');
  addMetric(summary, 'Lowest projected cash', formatMinorUnits(runway.summary.minimumProjectedCashMinor, runway.currency),
    runway.summary.minimumProjectedCashMinor < 0 ? 'negative' : '');
  addMetric(summary, 'Ending cash', formatMinorUnits(runway.summary.endingCashMinor, runway.currency),
    runway.summary.endingCashMinor < 0 ? 'negative' : '');
  addMetric(summary, 'Reserve target', formatMinorUnits(runway.startup.minimumCashReserveTargetMinor, runway.currency));
  addMetric(summary, 'First reserve breach',
    runway.summary.firstBelowReservePeriod === null
      ? 'None during this scenario'
      : formatCashPeriod(runway.summary.firstBelowReservePeriod),
    runway.summary.firstBelowReservePeriod === null ? '' : 'negative');
  addMetric(summary, 'First negative cash balance',
    runway.summary.firstNegativeCashPeriod === null
      ? 'None during this scenario'
      : formatCashPeriod(runway.summary.firstNegativeCashPeriod),
    runway.summary.firstNegativeCashPeriod === null ? '' : 'negative');
  addMetric(summary, 'Extra cash to maintain reserve',
    formatMinorUnits(runway.summary.additionalCashNeededToMaintainReserveMinor, runway.currency),
    runway.summary.additionalCashNeededToMaintainReserveMinor > 0 ? 'negative' : '');
  section.append(summary);

  const scroll = element('div', 'cash-plan-scroll');
  scroll.setAttribute('role', 'region');
  scroll.setAttribute('aria-label', 'Projected monthly cash balances');
  scroll.setAttribute('tabindex', '0');
  const table = element('table', 'cash-projection-table');
  const thead = element('thead');
  const headerRow = element('tr');
  for (const label of [
    'Period',
    'Sales units (activity)',
    'Sales cash receipts',
    'Operating cash outflows',
    'Other/unclassified inflows',
    'Other/unclassified outflows',
    'Net operating cash flow',
    'Net unclassified cash flow',
    'Net investing cash flow',
    'Net financing cash flow',
    'Net cash change',
    'Closing cash',
    'Reserve status'
  ]) {
    const heading = element('th', '', label);
    heading.setAttribute('scope', 'col');
    headerRow.append(heading);
  }
  thead.append(headerRow);
  table.append(thead);

  const body = element('tbody');
  runway.periods.forEach(period => {
    const row = element('tr');
    const periodHeading = element('th', '', 'Month ' + period.period);
    periodHeading.setAttribute('scope', 'row');
    row.append(periodHeading);
    row.append(element('td', '', period.salesUnitsScenario.toLocaleString()));
    row.append(element('td', '', formatMinorUnits(period.salesCashReceiptsMinor, runway.currency)));
    row.append(element('td', '', formatMinorUnits(period.operatingCashOutflowsMinor, runway.currency)));
    row.append(element('td', '', formatMinorUnits(period.otherCashInflowsMinor, runway.currency)));
    row.append(element('td', '', formatMinorUnits(period.otherCashOutflowsMinor, runway.currency)));
    row.append(element('td', period.netOperatingCashFlowMinor < 0 ? 'negative' : '', formatMinorUnits(period.netOperatingCashFlowMinor, runway.currency)));
    row.append(element('td', period.netUnclassifiedCashFlowMinor < 0 ? 'negative' : '', formatMinorUnits(period.netUnclassifiedCashFlowMinor, runway.currency)));
    row.append(element('td', period.netInvestingCashFlowMinor < 0 ? 'negative' : '', formatMinorUnits(period.netInvestingCashFlowMinor, runway.currency)));
    row.append(element('td', period.netFinancingCashFlowMinor < 0 ? 'negative' : '', formatMinorUnits(period.netFinancingCashFlowMinor, runway.currency)));
    row.append(element('td', period.netCashChangeMinor < 0 ? 'negative' : '',
      formatMinorUnits(period.netCashChangeMinor, runway.currency)));
    row.append(element('td', period.closingCashMinor < 0 ? 'negative' : '',
      formatMinorUnits(period.closingCashMinor, runway.currency)));
    row.append(element('td', period.belowReserve ? 'negative' : '',
      period.belowReserve ? 'Below reserve' : 'At or above reserve'));
    body.append(row);
  });
  table.append(body);
  scroll.append(table);
  section.append(scroll);

  if (runway.costCoverage.level === 'partial') {
    section.append(element(
      'p',
      'coverage-line',
      'PARTIAL COST COVERAGE ALSO APPLIES TO THIS CASH SCENARIO. Include known operating costs in operating cash outflows when appropriate; keep unclassified payments in the other/unclassified field until they can be categorized. Model capital purchases and loan principal/capital events in investing/financing fields where appropriate. Classification depends on transaction and reporting rules; unknown costs remain a limitation.'
    ));
  }

  section.append(element('h4', 'section-subheading', 'Cash scenario limitations'));
  const limitations = element('ul', 'limitations');
  for (const item of runway.scopeLimitations) limitations.append(element('li', '', item));
  section.append(limitations);
  return section;
}


function renderActualsVariance(report) {
  actualsResultRoot.replaceChildren();
  const overview = element('p', 'detail',
    'Source: ' + report.source.label + ' (' + report.source.type + '), covered ' +
    report.source.coverageStartDate + ' to ' + report.source.coverageEndDate + '. ' +
    report.source.transactionCount + ' transaction rows; SHA-256 ' + report.source.sha256 +
    '. Opening-to-closing cash arithmetic reconciles. The fingerprint is not independent source authentication.'
  );
  actualsResultRoot.append(overview);

  const metrics = element('dl', 'metric-list');
  addMetric(metrics, 'Imported records', String(report.source.transactionCount));
  addMetric(metrics, 'Range opening cash', formatMinorUnits(report.source.openingCashBalanceMinor, report.currency));
  addMetric(metrics, 'Range ending cash', formatMinorUnits(report.source.closingCashBalanceMinor, report.currency));
  addMetric(metrics, 'Transaction net movement', formatMinorUnits(report.reconciliation.transactionNetMovementMinor, report.currency));
  actualsResultRoot.append(metrics);

  actualsResultRoot.append(element('h4', 'section-subheading', 'Monthly actual versus plan'));
  const scroll = element('div', 'cash-plan-scroll');
  scroll.setAttribute('role', 'region');
  scroll.setAttribute('aria-label', 'Monthly actual versus plan variance');
  scroll.setAttribute('tabindex', '0');
  const table = element('table', 'cash-scenario-table actuals-variance-table');
  const head = element('thead');
  const header = element('tr');
  for (const label of ['Month', 'Cash-flow bucket', 'Plan', 'Actual', 'Actual − plan']) {
    const cell = element('th', '', label);
    cell.setAttribute('scope', 'col');
    header.append(cell);
  }
  head.append(header);
  table.append(head);
  const body = element('tbody');
  for (const period of report.periods) {
    const summaryRows = [
      { label: 'Net cash movement', planned: period.plannedNetCashMovementMinor, actual: period.actualNetCashMovementMinor, variance: period.netCashMovementVarianceMinor },
      { label: 'Closing cash balance (derived)', planned: period.plannedClosingCashMinor, actual: period.actualClosingCashMinor, variance: period.closingCashVarianceMinor }
    ];
    for (const summary of summaryRows) {
      const row = element('tr');
      const month = element('th', '', period.month);
      month.setAttribute('scope', 'row');
      row.append(month);
      row.append(element('td', '', summary.label));
      row.append(element('td', '', formatMinorUnits(summary.planned, report.currency)));
      row.append(element('td', '', formatMinorUnits(summary.actual, report.currency)));
      row.append(element('td', summary.variance < 0 ? 'negative' : '', formatMinorUnits(summary.variance, report.currency)));
      body.append(row);
    }
    for (const category of period.categories) {
      const row = element('tr');
      row.append(element('td', '', period.month));
      row.append(element('td', '', category.label));
      row.append(element('td', '', formatMinorUnits(category.plannedMinor, report.currency)));
      row.append(element('td', '', formatMinorUnits(category.actualMinor, report.currency)));
      const unfavorable = category.direction === 'outflow'
        ? category.actualMinusPlanMinor > 0
        : category.actualMinusPlanMinor < 0;
      row.append(element('td', unfavorable ? 'negative' : '', formatMinorUnits(category.actualMinusPlanMinor, report.currency)));
      body.append(row);
    }
  }
  table.append(body);
  scroll.append(table);
  actualsResultRoot.append(scroll);
  actualsResultRoot.append(element('p', 'form-footnote',
    'Variances are actual minus plan. Positive outflow variance means spending exceeded plan. Closing balances in intermediate months are derived from imported rows; the source separately supplies and reconciles the final balance. Actuals were imported from a user-declared source and are not independently verified.'
  ));
}

async function sha256Hex(arrayBuffer) {
  if (!globalThis.crypto?.subtle || typeof globalThis.crypto.subtle.digest !== 'function') {
    throw new Error('This browser does not expose Web Crypto SHA-256; the source cannot be fingerprinted here.');
  }
  const digest = await globalThis.crypto.subtle.digest('SHA-256', arrayBuffer);
  return Array.from(new Uint8Array(digest), byte => byte.toString(16).padStart(2, '0')).join('');
}

function renderReport(report, cashRunwayReport, cashScenarioComparison) {
  resultRoot.replaceChildren();

  const headline = element('div', 'result-headline');
  headline.append(element('span', 'label', 'Monthly scenario result before excluded items'));
  const operatingResult = report.monthlyScenario.operatingResultBeforeExcludedItemsMinor;
  const resultValue = element('strong', 'value' + (operatingResult < 0 ? ' negative' : ''), formatMinorUnits(operatingResult, report.currency));
  headline.append(resultValue);
  headline.append(element('p', 'detail', 'Illustrative arithmetic only; this is not net profit.'));
  resultRoot.append(headline);

  const metrics = element('dl', 'metric-list');
  addMetric(metrics, 'Contribution per sale', formatMinorUnits(report.unitEconomics.contributionPerUnitMinor, report.currency), report.unitEconomics.contributionPerUnitMinor < 0 ? 'negative' : '');
  const breakEvenText = report.breakEven.status === 'reachable'
    ? report.breakEven.unitsPerMonth.toLocaleString() + ' units/month'
    : 'Unreachable at this price/cost';
  addMetric(metrics, 'Break-even sales', breakEvenText, report.breakEven.status !== 'reachable' ? 'negative' : '');
  addMetric(metrics, 'Planned monthly revenue', formatMinorUnits(report.monthlyScenario.revenueMinor, report.currency));
  addMetric(metrics, 'Planned variable costs', formatMinorUnits(report.monthlyScenario.variableCostsMinor, report.currency));
  addMetric(metrics, 'Planned contribution', formatMinorUnits(report.monthlyScenario.contributionMinor, report.currency));
  addMetric(metrics, 'Fixed monthly costs', formatMinorUnits(report.monthlyScenario.fixedCostsMinor, report.currency));
  addMetric(metrics, 'Startup cash requirement', formatMinorUnits(report.startup.requiredCashMinor, report.currency));
  addMetric(metrics, 'Cash available', formatMinorUnits(report.startup.availableCashMinor, report.currency));
  addMetric(metrics, 'Startup funding gap', formatMinorUnits(report.startup.fundingGapMinor, report.currency), report.startup.fundingGapMinor > 0 ? 'negative' : '');
  resultRoot.append(metrics);
  resultRoot.append(element(
    'p',
    'form-footnote',
    'Funding-gap scope: this compares setup costs plus your entered working-capital buffer with available startup cash. It does not estimate extra runway for slow sales or operating losses.'
  ));

  const coverage = element('p', 'coverage-line');
  const omissions = report.costCoverage.omittedCategories.length
    ? ' Omitted/uncertain: ' + report.costCoverage.omittedCategories.join(', ') + '.'
    : '';
  coverage.textContent = report.costCoverage.level === 'partial'
    ? 'PARTIAL COST COVERAGE. Results exclude the categories listed below and may materially overstate viability.' + omissions
    : 'COSTS DECLARED COMPLETE FOR THE USER-DEFINED SCOPE. This is a user assertion, not independent verification.';
  resultRoot.append(coverage);

  const subheading = element('h3', 'section-subheading', 'What this result does not know');
  resultRoot.append(subheading);
  const list = element('ul', 'limitations');
  for (const limitation of report.scopeLimitations) {
    list.append(element('li', '', limitation));
  }
  resultRoot.append(list);
  resultRoot.append(renderCashScenarioComparison(cashScenarioComparison, report.currency));
  resultRoot.append(renderCashRunwayReport(cashRunwayReport));

  resultStatus.textContent = 'Deterministic calculation';
  resultStatus.className = 'status-pill';
}

form.addEventListener('input', () => {
  clearResult();
  resultStatus.textContent = 'Inputs changed — recalculate';
  resultStatus.className = 'status-pill status-muted';
  hideError();
});

form.addEventListener('submit', event => {
  event.preventDefault();
  hideError();
  clearResult();
  try {
    const model = readModel();
    const result = analyzeBusinessModel(model);
    if (!result.ok) {
      showError(result.error.field ? result.error.message + ' (field: ' + result.error.field + ')' : result.error.message);
      return;
    }
    const cashRunwayPlan = readCashRunwayPlan(model.currency);
    const cashScenarioAssumptions = readCashScenarioAssumptions();
    const cashTimingSchedule = readCashTimingSchedule();
    const cashRunwayResult = projectCashRunway({ model, monthlyPlan: cashRunwayPlan });
    if (!cashRunwayResult.ok) {
      showError(cashRunwayResult.error.field
        ? cashRunwayResult.error.message + ' (field: ' + cashRunwayResult.error.field + ')'
        : cashRunwayResult.error.message);
      return;
    }
    const comparison = compareCashRunwayScenarios({ model, monthlyPlan: cashRunwayPlan, assumptions: cashScenarioAssumptions, timingSchedule: cashTimingSchedule });
    if (!comparison.ok) {
      showError(comparison.error.field
        ? comparison.error.message + ' (field: ' + comparison.error.field + ')'
        : comparison.error.message);
      return;
    }
    latestModel = model;
    latestReport = result.report;
    latestCashRunwayPlan = cashRunwayPlan;
    latestCashRunway = cashRunwayResult.report;
    latestCashScenarioAssumptions = comparison.report.assumptions;
    latestCashScenarioComparison = comparison.report;
    latestCashTimingSchedule = comparison.report.timingSchedule;
    renderReport(result.report, cashRunwayResult.report, comparison.report);
    downloadReportButton.disabled = false;
  } catch (error) {
    showError(error instanceof Error ? error.message : 'The scenario could not be read.');
  }
});

downloadScenarioButton.addEventListener('click', () => {
  hideError();
  try {
    const model = readModel();
    const result = analyzeBusinessModel(model);
    if (!result.ok) throw new Error(result.error.message);
    const cashRunwayPlan = readCashRunwayPlan(model.currency);
    const cashScenarioAssumptions = readCashScenarioAssumptions();
    const cashTimingSchedule = readCashTimingSchedule();
    const cashRunwayResult = projectCashRunway({ model, monthlyPlan: cashRunwayPlan });
    if (!cashRunwayResult.ok) throw new Error(cashRunwayResult.error.message);
    const comparison = compareCashRunwayScenarios({ model, monthlyPlan: cashRunwayPlan, assumptions: cashScenarioAssumptions, timingSchedule: cashTimingSchedule });
    if (!comparison.ok) throw new Error(comparison.error.message);
    downloadJson('business-launchpad-scenario-v6.json', {
      schemaVersion: 6,
      kind: 'business-launchpad-scenario',
      input: model,
      cashRunwayPlan,
      cashScenarioAssumptions: comparison.report.assumptions,
      cashTimingSchedule: comparison.report.timingSchedule
    });
  } catch (error) {
    showError(error instanceof Error ? error.message : 'Scenario could not be exported.');
  }
});



actualsForecastStartMonth.addEventListener('input', () => {
  if (!latestActualsComparison) return;
  latestActualsComparison = null;
  latestActualsSourcePayload = null;
  latestActualsSourceRawText = null;
  latestActualsSourceSha256 = null;
  latestActualsForecastStartMonth = null;
  actualsResultRoot.replaceChildren();
  actualsError.textContent = '';
  actualsError.hidden = true;
  actualsStatus.textContent = 'Forecast month changed — import actuals again.';
  actualsStatus.className = 'status-pill status-muted';
  downloadVarianceReportButton.disabled = true;
});

downloadActualsTemplateButton.addEventListener('click', () => {
  try {
    const model = readModel();
    downloadJson('business-launchpad-actuals-source-template-v1.json', {
      schemaVersion: 1,
      kind: 'business-launchpad-actuals-source',
      currency: model.currency,
      source: {
        type: 'bank_statement_export',
        label: 'REPLACE with a clear source export label',
        evidenceReference: 'REPLACE with source statement/document reference; do not include bank credentials',
        coverageStartDate: '',
        coverageEndDate: '',
        completeMonthlyCoverage: false,
        openingCashBalanceMinor: 0,
        closingCashBalanceMinor: 0
      },
      transactions: [{
        transactionId: 'REPLACE with unique source transaction ID',
        date: 'YYYY-MM-DD',
        currencyCode: model.currency.code,
        category: 'sales_receipt',
        amountMinor: 1,
        sourceReference: 'REPLACE with source line or receipt reference'
      }]
    });
  } catch (error) {
    showError(error instanceof Error ? error.message : 'Actuals template could not be created.');
  }
});

loadActualsInput.addEventListener('change', async event => {
  const file = event.target.files?.[0];
  if (!file) return;
  latestActualsComparison = null;
  latestActualsSourcePayload = null;
  latestActualsSourceRawText = null;
  latestActualsSourceSha256 = null;
  latestActualsForecastStartMonth = null;
  actualsResultRoot.replaceChildren();
  downloadVarianceReportButton.disabled = true;
  actualsError.textContent = '';
  actualsError.hidden = true;
  try {
    if (!latestModel || !latestCashRunwayPlan || !latestCashRunway) {
      throw new Error('Calculate the current cash plan before importing actuals.');
    }
    if (Number.isFinite(file.size) && file.size > 5_000_000) {
      throw new Error('Actuals source files larger than 5 MB are not accepted.');
    }
    if (typeof file.arrayBuffer !== 'function') {
      throw new Error('The selected file cannot expose its original bytes for source hashing.');
    }
    const bytes = await file.arrayBuffer();
    if (bytes.byteLength > 5_000_000) throw new Error('Actuals source files larger than 5 MB are not accepted.');
    const byteView = new Uint8Array(bytes);
    if (byteView.length >= 3 && byteView[0] === 0xef && byteView[1] === 0xbb && byteView[2] === 0xbf) {
      throw new Error('UTF-8 BOM is not supported; save actuals as plain UTF-8 JSON.');
    }
    const rawText = new TextDecoder('utf-8', { fatal: true }).decode(bytes);
    const payload = JSON.parse(rawText);
    const forecastStartMonth = actualsForecastStartMonth.value;
    const sourceSha256 = await sha256Hex(bytes);
    const comparison = compareCashPlanToActuals({
      model: latestModel,
      monthlyPlan: latestCashRunwayPlan,
      forecastStartMonth,
      actualsSource: payload,
      sourceSha256
    });
    if (!comparison.ok) {
      throw new Error(comparison.error.field
        ? comparison.error.message + ' (field: ' + comparison.error.field + ')'
        : comparison.error.message);
    }
    latestActualsComparison = comparison.report;
    latestActualsSourcePayload = payload;
    latestActualsSourceRawText = rawText;
    latestActualsSourceSha256 = sourceSha256;
    latestActualsForecastStartMonth = forecastStartMonth;
    renderActualsVariance(comparison.report);
    actualsStatus.textContent = 'Actuals compared — source balances reconcile';
    actualsStatus.className = 'status-pill';
    downloadVarianceReportButton.disabled = false;
  } catch (error) {
    actualsError.textContent = error instanceof Error ? error.message : 'Actuals could not be compared.';
    actualsError.hidden = false;
    actualsStatus.textContent = 'Actuals comparison failed';
    actualsStatus.className = 'status-pill status-muted';
  } finally {
    loadActualsInput.value = '';
  }
});

downloadVarianceReportButton.addEventListener('click', () => {
  if (!latestModel || !latestCashRunwayPlan || !latestCashRunway || !latestActualsComparison ||
      latestActualsSourceRawText === null || !latestActualsSourceSha256 || !latestActualsForecastStartMonth ||
      actualsForecastStartMonth.value !== latestActualsForecastStartMonth || downloadVarianceReportButton.disabled) {
    actualsError.textContent = 'Import and reconcile actuals against the current calculated plan before downloading the variance bundle.';
    actualsError.hidden = false;
    return;
  }
  try {
    downloadJson('business-launchpad-actuals-variance-v1.json', {
      schemaVersion: 1,
      kind: 'business-launchpad-actuals-variance-report-bundle',
      forecastStartMonth: latestActualsForecastStartMonth,
      sourceArtifactSha256: latestActualsSourceSha256,
      sourceArtifactRawText: latestActualsSourceRawText,
      scenarioInput: latestModel,
      cashRunwayPlan: latestCashRunwayPlan,
      cashRunwayProjection: latestCashRunway,
      comparison: latestActualsComparison
    });
  } catch (error) {
    actualsError.textContent = error instanceof Error ? error.message : 'Variance evidence bundle could not be exported.';
    actualsError.hidden = false;
  }
});

downloadReportButton.addEventListener('click', () => {
  if (!latestModel || !latestReport || !latestCashRunwayPlan || !latestCashRunway || !latestCashScenarioAssumptions || !latestCashScenarioComparison || !latestCashTimingSchedule || downloadReportButton.disabled) {
    showError('Calculate the current inputs before downloading the report.');
    return;
  }
  hideError();
  try {
    downloadJson('business-launchpad-report-v6.json', {
      schemaVersion: 6,
      kind: 'business-launchpad-report',
      input: latestModel,
      report: latestReport,
      cashRunwayPlan: latestCashRunwayPlan,
      cashRunwayProjection: latestCashRunway,
      cashScenarioAssumptions: latestCashScenarioAssumptions,
      cashTimingSchedule: latestCashTimingSchedule,
      cashScenarioComparison: latestCashScenarioComparison
    });
  } catch (error) {
    showError(error instanceof Error ? error.message : 'Report could not be exported.');
  }
});

loadScenarioInput.addEventListener('change', async event => {
  const file = event.target.files?.[0];
  if (!file) return;
  hideError();
  try {
    if (Number.isFinite(file.size) && file.size > 1_000_000) {
      throw new Error('Scenario files larger than 1 MB are not accepted by this prototype.');
    }
    const payload = JSON.parse(await file.text());
    const scenario = validatedScenarioPayload(payload);
    applyModelToForm(scenario.input);
    renderCashRunwayInputs(scenario.cashRunwayPlan, scenario.input.currency);
    applyCashScenarioAssumptions(scenario.cashScenarioAssumptions);
    applyCashTimingSchedule(scenario.cashTimingSchedule);
    clearResult();
    resultStatus.textContent = 'Scenario loaded — calculate to review';
    resultStatus.className = 'status-pill status-muted';
  } catch (error) {
    clearResult();
    showError(error instanceof Error ? error.message : 'Scenario file could not be loaded.');
  } finally {
    loadScenarioInput.value = '';
  }
});

function syncCoverageInput() {
  const partial = coverageSelect.value === 'partial';
  omittedField.disabled = !partial;
  omittedField.setAttribute('aria-disabled', String(!partial));
}

coverageSelect.addEventListener('change', syncCoverageInput);

resetCashRunwayButton.addEventListener('click', () => {
  hideError();
  try {
    const units = parseWholeUnits(getFieldValue('planned-units'));
    if (!units.ok) throw new Error(units.error.message);
    renderCashRunwayInputs(defaultCashRunwayPlan(readModel()), describeCurrency());
    clearResult();
    resultStatus.textContent = 'Cash plan reset — recalculate';
    resultStatus.className = 'status-pill status-muted';
  } catch (error) {
    showError(error instanceof Error ? error.message : 'Cash plan could not be reset.');
  }
});

try { const initialModel = readModel(); renderCashRunwayInputs(defaultCashRunwayPlan(initialModel), initialModel.currency); applyCashScenarioAssumptions(DEFAULT_CASH_SCENARIO_ASSUMPTIONS); applyCashTimingSchedule(DEFAULT_CASH_TIMING_SCHEDULE); } catch { /* show validation only when the owner requests a calculation */ }
syncCoverageInput();
