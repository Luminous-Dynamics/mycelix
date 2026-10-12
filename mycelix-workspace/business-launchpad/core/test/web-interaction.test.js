// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//
// This is a dependency-free interaction-contract simulation, not a real browser
// or accessibility test. A browser engine must still validate layout and APIs.

import test from 'node:test';
import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import { fileURLToPath } from 'node:url';
import { dirname, resolve } from 'node:path';

class MockElement {
  constructor(tag = 'div', id = '') {
    this.tagName = tag.toUpperCase();
    this.id = id;
    this.value = '';
    this.disabled = false;
    this.hidden = false;
    this.className = '';
    this.attributes = {};
    this.children = [];
    this.listeners = new Map();
    this._text = '';
    this.files = [];
    this.clicked = false;
    this.removed = false;
  }
  set textContent(value) {
    this._text = value == null ? '' : String(value);
    this.children = [];
  }
  get textContent() {
    return this._text + this.children.map(child => child.textContent).join('');
  }
  append(child) { this.children.push(child); }
  replaceChildren(...children) { this._text = ''; this.children = children; }
  click() { this.clicked = true; }
  remove() { this.removed = true; }
  setAttribute(name, value) { this.attributes[name] = String(value); }
  addEventListener(type, listener) {
    const entries = this.listeners.get(type) ?? [];
    entries.push(listener);
    this.listeners.set(type, entries);
  }
  dispatchEvent(event) {
    event.target = this;
    event.preventDefault ??= () => { event.defaultPrevented = true; };
    for (const listener of this.listeners.get(event.type) ?? []) listener(event);
    return !event.defaultPrevented;
  }
}

class MockDocument {
  constructor() {
    this.nodes = new Map();
    const ids = [
      'model-form', 'input-error', 'result', 'result-status', 'currency-code',
      'minor-unit-exponent', 'selling-price', 'variable-cost', 'planned-units',
      'fixed-monthly', 'setup-costs', 'working-capital', 'available-cash',
      'cost-coverage', 'omitted-costs', 'download-scenario', 'load-scenario-file', 'download-report'
    ];
    for (const id of ids) this.nodes.set(id, new MockElement(id === 'model-form' ? 'form' : 'div', id));
    this.body = new MockElement('body');
    this.createdElements = [];
    this.nodes.get('download-report').disabled = true;
    this.nodes.get('currency-code').value = 'ZAR';
    this.nodes.get('minor-unit-exponent').value = '2';
    this.nodes.get('selling-price').value = '180.00';
    this.nodes.get('variable-cost').value = '60.00';
    this.nodes.get('planned-units').value = '50';
    this.nodes.get('fixed-monthly').value = '900.00';
    this.nodes.get('setup-costs').value = '2500.00';
    this.nodes.get('working-capital').value = '3000.00';
    this.nodes.get('available-cash').value = '6000.00';
    this.nodes.get('cost-coverage').value = 'partial';
    this.nodes.get('omitted-costs').value = 'owner labor, tax, insurance';
    this.nodes.get('result-status').textContent = 'Example not yet calculated';
  }
  querySelector(selector) {
    if (!selector.startsWith('#')) throw new Error('Mock DOM only implements ID selectors.');
    return this.nodes.get(selector.slice(1)) ?? null;
  }
  getElementById(id) { return this.nodes.get(id) ?? null; }
  createElement(tag) {
    const node = new MockElement(tag);
    this.createdElements.push(node);
    return node;
  }
  reset() {
    const values = {
      'currency-code': 'ZAR',
      'minor-unit-exponent': '2',
      'selling-price': '180.00',
      'variable-cost': '60.00',
      'planned-units': '50',
      'fixed-monthly': '900.00',
      'setup-costs': '2500.00',
      'working-capital': '3000.00',
      'available-cash': '6000.00',
      'cost-coverage': 'partial',
      'omitted-costs': 'owner labor, tax, insurance'
    };
    for (const [id, value] of Object.entries(values)) this.nodes.get(id).value = value;
    this.nodes.get('input-error').hidden = true;
    this.nodes.get('input-error').textContent = '';
    this.nodes.get('result').replaceChildren();
    this.nodes.get('result-status').textContent = 'Example not yet calculated';
    this.nodes.get('omitted-costs').disabled = false;
    this.nodes.get('load-scenario-file').value = '';
    this.nodes.get('load-scenario-file').files = [];
    this.nodes.get('download-report').disabled = true;
  }
  submit() {
    this.nodes.get('model-form').dispatchEvent({
      type: 'submit',
      defaultPrevented: false,
      preventDefault() { this.defaultPrevented = true; }
    });
  }
  change(id) { this.nodes.get(id).dispatchEvent({ type: 'change' }); }
}

const documentMock = new MockDocument();
globalThis.document = documentMock;
let capturedBlob = null;
const revokedUrls = [];
const createdUrls = [];
globalThis.URL = {
  createObjectURL(blob) {
    capturedBlob = blob;
    createdUrls.push('blob:business-launchpad-test');
    return 'blob:business-launchpad-test';
  },
  revokeObjectURL(url) {
    revokedUrls.push(url);
  }
};
await import('../../web/app.mjs');

test('default business scenario renders deterministic totals and cost warnings', () => {
  documentMock.reset();
  documentMock.submit();
  const result = documentMock.getElementById('result');
  assert.equal(documentMock.getElementById('result-status').textContent, 'Deterministic calculation');
  assert.match(result.textContent, /ZAR 5100\.00/);
  assert.match(result.textContent, /8 units\/month/);
  assert.match(result.textContent, /PARTIAL COST COVERAGE/);
  assert.match(result.textContent, /owner labor, tax, insurance/);
  assert.equal(documentMock.getElementById('input-error').hidden, true);
});

test('rejects amounts with excess decimal precision instead of rounding them', () => {
  documentMock.reset();
  documentMock.getElementById('selling-price').value = '180.001';
  documentMock.submit();
  assert.equal(documentMock.getElementById('input-error').hidden, false);
  assert.match(documentMock.getElementById('input-error').textContent, /more fractional digits/);
  assert.equal(documentMock.getElementById('result-status').textContent, 'Calculation failed');
});

test('complete-cost declaration does not carry disabled omitted categories into the model', () => {
  documentMock.reset();
  documentMock.getElementById('cost-coverage').value = 'complete_for_declared_scope';
  documentMock.change('cost-coverage');
  documentMock.submit();
  assert.equal(documentMock.getElementById('omitted-costs').disabled, true);
  assert.equal(documentMock.getElementById('result-status').textContent, 'Deterministic calculation');
  assert.match(documentMock.getElementById('result').textContent, /COSTS DECLARED COMPLETE FOR THE USER-DEFINED SCOPE/);
  assert.doesNotMatch(documentMock.getElementById('result').textContent, /Omitted\/uncertain/);
});

test('negative unit contribution is not displayed as a reachable break-even', () => {
  documentMock.reset();
  documentMock.getElementById('variable-cost').value = '200.00';
  documentMock.submit();
  const output = documentMock.getElementById('result').textContent;
  assert.match(output, /Unreachable at this price\/cost/);
  assert.match(output, /ZAR -1900\.00/);
});

test('malformed currency code and non-integer unit volume produce explicit errors', () => {
  documentMock.reset();
  documentMock.getElementById('currency-code').value = 'Z$';
  documentMock.submit();
  assert.match(documentMock.getElementById('input-error').textContent, /three-letter currency code/);

  documentMock.reset();
  documentMock.getElementById('planned-units').value = '1.5';
  documentMock.submit();
  assert.match(documentMock.getElementById('input-error').textContent, /whole, non-negative number/);
});

test('assumptions export produces versioned portable JSON without uploading inputs', async () => {
  documentMock.reset();
  documentMock.getElementById('download-scenario').dispatchEvent({ type: 'click' });
  assert.ok(capturedBlob instanceof Blob);
  assert.equal(capturedBlob.type, 'application/json;charset=utf-8');

  const payload = JSON.parse(await capturedBlob.text());
  assert.equal(payload.schemaVersion, 1);
  assert.equal(payload.kind, 'business-launchpad-scenario');
  assert.equal(payload.input.currency.code, 'ZAR');
  assert.equal(payload.input.sellingPricePerUnitMinor, 18_000);
  assert.deepEqual(payload.input.costCoverage.omittedCategories, ['owner labor', 'tax', 'insurance']);

  const anchor = documentMock.createdElements.at(-1);
  assert.equal(anchor.tagName, 'A');
  assert.equal(anchor.download, 'business-launchpad-scenario-v1.json');
  assert.equal(anchor.clicked, true);
  assert.equal(anchor.removed, true);
});

test('imported scenario is validated, restores fields, and requires recalculation', async () => {
  documentMock.reset();
  documentMock.getElementById('download-scenario').dispatchEvent({ type: 'click' });
  const payload = JSON.parse(await capturedBlob.text());
  payload.input.sellingPricePerUnitMinor = 20_000;
  payload.input.plannedUnitsPerMonth = 40;

  const fileInput = documentMock.getElementById('load-scenario-file');
  fileInput.files = [{ text: async () => JSON.stringify(payload) }];
  fileInput.value = 'modified-scenario.json';
  await fileInput.listeners.get('change')[0]({ target: fileInput });

  assert.equal(fileInput.value, '');
  assert.equal(documentMock.getElementById('selling-price').value, '200.00');
  assert.equal(documentMock.getElementById('planned-units').value, '40');
  assert.equal(documentMock.getElementById('result-status').textContent, 'Scenario loaded — calculate to review');
  assert.equal(documentMock.getElementById('download-report').disabled, true);

  documentMock.submit();
  assert.equal(documentMock.getElementById('result-status').textContent, 'Deterministic calculation');
  assert.match(documentMock.getElementById('result').textContent, /ZAR 4700\.00/);
  assert.equal(documentMock.getElementById('download-report').disabled, false);
});

test('report export is disabled until a valid calculation exists and includes both inputs and report', async () => {
  documentMock.reset();
  assert.equal(documentMock.getElementById('download-report').disabled, true);

  documentMock.submit();
  assert.equal(documentMock.getElementById('download-report').disabled, false);
  documentMock.getElementById('download-report').dispatchEvent({ type: 'click' });
  const payload = JSON.parse(await capturedBlob.text());

  assert.equal(payload.schemaVersion, 1);
  assert.equal(payload.kind, 'business-launchpad-report');
  assert.equal(payload.input.currency.code, 'ZAR');
  assert.equal(payload.report.breakEven.unitsPerMonth, 8);
  assert.equal(payload.report.startup.fundingGapMinor, 0);
  assert.equal(documentMock.createdElements.at(-1).download, 'business-launchpad-report-v1.json');
  assert.ok(createdUrls.includes('blob:business-launchpad-test'));
});

test('malformed or unsupported import files fail visibly without changing inputs', async () => {
  documentMock.reset();
  const originalPrice = documentMock.getElementById('selling-price').value;
  const fileInput = documentMock.getElementById('load-scenario-file');
  fileInput.files = [{ text: async () => JSON.stringify({ schemaVersion: 2, kind: 'business-launchpad-scenario', input: {} }) }];
  await fileInput.listeners.get('change')[0]({ target: fileInput });

  assert.equal(documentMock.getElementById('selling-price').value, originalPrice);
  assert.equal(documentMock.getElementById('input-error').hidden, false);
  assert.match(documentMock.getElementById('input-error').textContent, /schemaVersion 1 required/);
  assert.equal(fileInput.value, '');
});

test('editing inputs invalidates a previously calculated report', () => {
  documentMock.reset();
  documentMock.submit();
  assert.equal(documentMock.getElementById('download-report').disabled, false);

  documentMock.getElementById('selling-price').value = '190.00';
  documentMock.getElementById('model-form').dispatchEvent({ type: 'input' });
  assert.equal(documentMock.getElementById('download-report').disabled, true);
  assert.equal(documentMock.getElementById('result-status').textContent, 'Inputs changed — recalculate');
  assert.equal(documentMock.getElementById('result').textContent, '');
});

test('static client does not contain application-level network or browser persistence calls', async () => {
  const here = dirname(fileURLToPath(import.meta.url));
  const source = await readFile(resolve(here, '../../web/app.mjs'), 'utf8');
  assert.doesNotMatch(source, /\bfetch\s*\(/);
  assert.doesNotMatch(source, /XMLHttpRequest|localStorage|sessionStorage|indexedDB|serviceWorker/);
  assert.match(source, /URL\.createObjectURL/);
  assert.match(source, /file\.text\(\)/);
  assert.doesNotMatch(source, /\.innerHTML\s*=/);
});
