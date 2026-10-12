import { analyzeBusinessModel, formatMinorUnits, parseMajorToMinorUnits } from '../core/src/index.js';

const form = document.querySelector('#model-form');
const errorBox = document.querySelector('#input-error');
const resultRoot = document.querySelector('#result');
const resultStatus = document.querySelector('#result-status');
const coverageSelect = document.querySelector('#cost-coverage');
const omittedField = document.querySelector('#omitted-costs');

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
  const text = String(raw).trim();
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
  const omittedCategories = getFieldValue('omitted-costs')
    .split(',')
    .map(value => value.trim())
    .filter(Boolean);
  model.costCoverage = { level, omittedCategories };
  return model;
}

function addMetric(list, label, value, extraClass = '') {
  const item = element('div', 'metric');
  item.append(element('dt', '', label));
  item.append(element('dd', extraClass, value));
  list.append(item);
}

function renderReport(report) {
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

  resultStatus.textContent = 'Deterministic calculation';
  resultStatus.className = 'status-pill';
}

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
    renderReport(result.report);
  } catch (error) {
    showError(error instanceof Error ? error.message : 'The scenario could not be read.');
  }
});

coverageSelect.addEventListener('change', () => {
  const partial = coverageSelect.value === 'partial';
  omittedField.disabled = !partial;
  omittedField.setAttribute('aria-disabled', String(!partial));
});

omittedField.disabled = coverageSelect.value !== 'partial';
