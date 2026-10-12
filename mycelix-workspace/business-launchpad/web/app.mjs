// Copyright (C) 2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

import { analyzeBusinessModel, formatMinorUnits, parseMajorToMinorUnits, projectCashRunway } from '../core/src/index.js';

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
const CASH_RUNWAY_MONTHS = 12;

let latestModel = null;
let latestReport = null;
let latestCashRunwayPlan = null;
let latestCashRunway = null;

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
      otherCashOutflowsMinor: 0
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
      ['inflows', period.otherCashInflowsMinor, 'other cash inflows'],
      ['outflows', period.otherCashOutflowsMinor, 'other cash outflows']
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
      ['inflows', 'otherCashInflowsMinor', 'other cash inflows'],
      ['outflows', 'otherCashOutflowsMinor', 'other cash outflows']
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
      schemaVersion === 2 &&
      period !== null &&
      typeof period === 'object' &&
      !Array.isArray(period) &&
      (period.salesCashReceiptsMinor === undefined || period.operatingCashOutflowsMinor === undefined)
    ) {
      throw new Error('Version 2 cash runway Month ' + (index + 1) + ' must include sales cash receipts and operating cash outflows.');
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
    return {
      salesUnits: period.salesUnits,
      salesCashReceiptsMinor,
      operatingCashOutflowsMinor,
      otherCashInflowsMinor: period.otherCashInflowsMinor,
      otherCashOutflowsMinor: period.otherCashOutflowsMinor
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
    ![1, 2].includes(payload.schemaVersion) ||
    payload.kind !== 'business-launchpad-scenario' ||
    payload.input === null ||
    typeof payload.input !== 'object' ||
    Array.isArray(payload.input)
  ) {
    throw new Error('This file is not a supported Business Launchpad scenario JSON file (schemaVersion 1 or 2 required).');
  }

  const result = analyzeBusinessModel(payload.input);
  if (!result.ok) {
    throw new Error('Scenario validation failed: ' + result.error.message);
  }
  if (payload.schemaVersion === 2 && payload.cashRunwayPlan === undefined) {
    throw new Error('Version 2 scenarios must include the 12-month cash plan.');
  }
  const plan = payload.cashRunwayPlan === undefined
    ? defaultCashRunwayPlan(payload.input)
    : validateImportedCashRunwayPlan(payload.cashRunwayPlan, payload.input, payload.schemaVersion);
  return { input: payload.input, cashRunwayPlan: plan };
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

function renderCashRunwayReport(runway) {
  const section = element('section', 'cash-runway-result');
  section.append(element('h3', 'section-subheading', '12-month cash runway scenario'));
  section.append(element(
    'p',
    'detail',
    'An owner-supplied cash scenario, not a sales forecast. Sales units are activity assumptions; sales cash receipts and operating payments are independently editable by month. Setup costs are paid before Month 1 and the working-capital buffer is a minimum reserve target.'
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
    'Other inflows',
    'Other outflows',
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
      'PARTIAL COST COVERAGE ALSO APPLIES TO THIS CASH SCENARIO. Include known taxes, owner draw, debt service, taxes, and other excluded payments in the monthly other-outflow fields; unknown costs remain a limitation.'
    ));
  }

  section.append(element('h4', 'section-subheading', 'Cash scenario limitations'));
  const limitations = element('ul', 'limitations');
  for (const item of runway.scopeLimitations) limitations.append(element('li', '', item));
  section.append(limitations);
  return section;
}

function renderReport(report, cashRunwayReport) {
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
    const cashRunwayResult = projectCashRunway({ model, monthlyPlan: cashRunwayPlan });
    if (!cashRunwayResult.ok) {
      showError(cashRunwayResult.error.field
        ? cashRunwayResult.error.message + ' (field: ' + cashRunwayResult.error.field + ')'
        : cashRunwayResult.error.message);
      return;
    }
    latestModel = model;
    latestReport = result.report;
    latestCashRunwayPlan = cashRunwayPlan;
    latestCashRunway = cashRunwayResult.report;
    renderReport(result.report, cashRunwayResult.report);
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
    const cashRunwayResult = projectCashRunway({ model, monthlyPlan: cashRunwayPlan });
    if (!cashRunwayResult.ok) throw new Error(cashRunwayResult.error.message);
    downloadJson('business-launchpad-scenario-v2.json', {
      schemaVersion: 2,
      kind: 'business-launchpad-scenario',
      input: model,
      cashRunwayPlan
    });
  } catch (error) {
    showError(error instanceof Error ? error.message : 'Scenario could not be exported.');
  }
});

downloadReportButton.addEventListener('click', () => {
  if (!latestModel || !latestReport || !latestCashRunwayPlan || !latestCashRunway || downloadReportButton.disabled) {
    showError('Calculate the current inputs before downloading the report.');
    return;
  }
  hideError();
  try {
    downloadJson('business-launchpad-report-v2.json', {
      schemaVersion: 2,
      kind: 'business-launchpad-report',
      input: latestModel,
      report: latestReport,
      cashRunwayPlan: latestCashRunwayPlan,
      cashRunwayProjection: latestCashRunway
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

try { const initialModel = readModel(); renderCashRunwayInputs(defaultCashRunwayPlan(initialModel), initialModel.currency); } catch { /* show validation only when the owner requests a calculation */ }
syncCoverageInput();
