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
      'cost-coverage', 'omitted-costs'
    ];
    for (const id of ids) this.nodes.set(id, new MockElement(id === 'model-form' ? 'form' : 'div', id));
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
  createElement(tag) { return new MockElement(tag); }
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
await import('../web/app.mjs');

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

test('static client does not contain application-level network or browser persistence calls', async () => {
  const here = dirname(fileURLToPath(import.meta.url));
  const source = await readFile(resolve(here, '../web/app.mjs'), 'utf8');
  assert.doesNotMatch(source, /\bfetch\s*\(/);
  assert.doesNotMatch(source, /XMLHttpRequest|localStorage|sessionStorage|indexedDB|serviceWorker/);
  assert.doesNotMatch(source, /\.innerHTML\s*=/);
});
