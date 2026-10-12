// Copyright (C) 2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//
// Real Chromium journey for the static Business Launchpad prototype.
// This is browser-engine smoke evidence, not an accessibility certification,
// legal review, market-fit study, or production-readiness claim.

import assert from 'node:assert/strict';
import { createServer } from 'node:http';
import { mkdir, readFile, writeFile } from 'node:fs/promises';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import { chromium } from 'playwright';

const here = dirname(fileURLToPath(import.meta.url));
const appRoot = resolve(here, '../..');
const evidenceDir = resolve(here, 'browser-evidence');
await mkdir(evidenceDir, { recursive: true });

const routes = new Map([
  ['/web/index.html', { path: resolve(appRoot, 'web/index.html'), type: 'text/html; charset=utf-8' }],
  ['/web/styles.css', { path: resolve(appRoot, 'web/styles.css'), type: 'text/css; charset=utf-8' }],
  ['/web/app.mjs', { path: resolve(appRoot, 'web/app.mjs'), type: 'text/javascript; charset=utf-8' }],
  ['/core/src/index.js', { path: resolve(appRoot, 'core/src/index.js'), type: 'text/javascript; charset=utf-8' }]
]);

const server = createServer(async (request, response) => {
  const pathname = new URL(request.url || '/', 'http://127.0.0.1').pathname;
  if (pathname === '/favicon.ico') {
    response.writeHead(204);
    response.end();
    return;
  }

  const route = routes.get(pathname);
  if (!route) {
    response.writeHead(404, { 'content-type': 'text/plain; charset=utf-8' });
    response.end('Not found');
    return;
  }

  try {
    const bytes = await readFile(route.path);
    response.writeHead(200, {
      'content-type': route.type,
      'cache-control': 'no-store',
      'x-content-type-options': 'nosniff'
    });
    response.end(bytes);
  } catch {
    response.writeHead(500, { 'content-type': 'text/plain; charset=utf-8' });
    response.end('Unable to load the test fixture');
  }
});

await new Promise((resolveListen, rejectListen) => {
  server.once('error', rejectListen);
  server.listen(0, '127.0.0.1', resolveListen);
});

const address = server.address();
if (!address || typeof address === 'string') {
  throw new Error('The local browser-test server did not provide a TCP address.');
}
const origin = 'http://127.0.0.1:' + address.port;

async function tabUntil(page, targetId, limit = 40) {
  for (let index = 0; index < limit; index += 1) {
    await page.keyboard.press('Tab');
    const activeId = await page.evaluate(() => document.activeElement?.id || '');
    if (activeId === targetId) return;
  }
  throw new Error('Keyboard traversal did not reach #' + targetId + ' within ' + limit + ' Tab presses.');
}

async function readDownloadJson(page, selector, expectedFilename) {
  const pendingDownload = page.waitForEvent('download');
  await page.locator(selector).click();
  const download = await pendingDownload;
  assert.equal(download.suggestedFilename(), expectedFilename, 'download filename should be stable');
  const path = await download.path();
  assert.ok(path, 'the browser should expose the completed download path');
  return JSON.parse(await readFile(path, 'utf8'));
}

let browser;
let page;
const pageErrors = [];
const completedChecks = [];

try {
  browser = await chromium.launch({ headless: true });
  page = await browser.newPage({
    viewport: { width: 1280, height: 1000 },
    acceptDownloads: true
  });
  page.on('pageerror', error => pageErrors.push('pageerror: ' + error.message));
  page.on('console', message => {
    if (message.type() === 'error') pageErrors.push('console: ' + message.text());
  });

  await page.goto(origin + '/web/index.html', { waitUntil: 'networkidle' });
  assert.equal(await page.title(), 'Business Launchpad — Viability Worksheet');
  completedChecks.push('served static application and loaded shared calculator module');

  // Keyboard-only validation and calculation: no mouse or locator click is
  // used in this part of the journey.
  await tabUntil(page, 'selling-price');
  await page.keyboard.press('Control+A');
  await page.keyboard.type('180.001');
  for (let index = 0; index < 9; index += 1) await page.keyboard.press('Tab');
  assert.equal(
    await page.evaluate(() => document.activeElement?.classList.contains('primary-button') || false),
    true,
    'keyboard traversal should reach the Calculate button'
  );
  await page.keyboard.press('Enter');
  const errorBox = page.locator('#input-error');
  await errorBox.waitFor({ state: 'visible' });
  assert.match(await errorBox.innerText(), /more fractional digits/);
  await page.screenshot({ path: resolve(evidenceDir, '01-keyboard-validation-error.png'), fullPage: true });
  completedChecks.push('keyboard-only validation reports excessive decimal precision');

  for (let index = 0; index < 9; index += 1) await page.keyboard.press('Shift+Tab');
  assert.equal(
    await page.evaluate(() => document.activeElement?.id || ''),
    'selling-price',
    'reverse keyboard traversal should return to the amount field'
  );
  await page.keyboard.press('Control+A');
  await page.keyboard.type('180.00');
  for (let index = 0; index < 9; index += 1) await page.keyboard.press('Tab');
  await page.keyboard.press('Enter');
  await page.locator('#result').getByText('ZAR 5100.00', { exact: true }).waitFor({ state: 'visible' });
  assert.match(await page.locator('#result').innerText(), /8 units\/month/);
  assert.match(await page.locator('#result').innerText(), /PARTIAL COST COVERAGE/);
  assert.match(await page.locator('#result').innerText(), /does not estimate extra runway for slow sales or operating losses/);
  assert.match(await page.locator('#result').innerText(), /single-offer model/);
  await page.screenshot({ path: resolve(evidenceDir, '02-keyboard-calculation-result.png'), fullPage: true });
  completedChecks.push('keyboard-only calculation shows result, break-even and partial-cost warning');

  // The invisible file input is focusable; focus-within must expose a visible
  // outline on its associated visible label.
  await page.keyboard.press('Tab');
  await page.keyboard.press('Tab');
  assert.equal(
    await page.evaluate(() => document.activeElement?.id || ''),
    'load-scenario-file',
    'keyboard traversal should reach the scenario file input'
  );
  const focusStyle = await page.locator('label[for="load-scenario-file"]').evaluate(label => {
    const style = getComputedStyle(label);
    return { outlineStyle: style.outlineStyle, outlineWidth: style.outlineWidth, outlineColor: style.outlineColor };
  });
  assert.equal(focusStyle.outlineStyle, 'solid');
  assert.equal(focusStyle.outlineWidth, '3px');
  assert.equal(focusStyle.outlineColor, 'rgb(23, 108, 82)');
  completedChecks.push('keyboard focus on scenario picker produces the expected visible outline');

  const exportedScenario = await readDownloadJson(
    page,
    '#download-scenario',
    'business-launchpad-scenario-v1.json'
  );
  assert.equal(exportedScenario.schemaVersion, 1);
  assert.equal(exportedScenario.kind, 'business-launchpad-scenario');
  assert.equal(exportedScenario.input.currency.code, 'ZAR');
  completedChecks.push('assumptions export is versioned JSON with the expected schema');

  exportedScenario.input.sellingPricePerUnitMinor = 20_000;
  exportedScenario.input.plannedUnitsPerMonth = 40;
  await page.locator('#load-scenario-file').setInputFiles({
    name: 'business-launchpad-scenario-v1.json',
    mimeType: 'application/json',
    buffer: Buffer.from(JSON.stringify(exportedScenario))
  });
  await page.waitForFunction(() =>
    document.querySelector('#result-status')?.textContent.includes('Scenario loaded — calculate to review')
  );
  assert.equal(await page.locator('#selling-price').inputValue(), '200.00');
  assert.equal(await page.locator('#planned-units').inputValue(), '40');
  assert.equal(await page.locator('#download-report').isDisabled(), true);
  await page.getByRole('button', { name: /Calculate scenario/ }).click();
  await page.locator('#result').getByText('ZAR 4700.00', { exact: true }).waitFor({ state: 'visible' });
  assert.match(await page.locator('#result').innerText(), /7 units\/month/);
  completedChecks.push('scenario import validates and populates fields but requires explicit recalculation');

  const report = await readDownloadJson(
    page,
    '#download-report',
    'business-launchpad-report-v1.json'
  );
  assert.equal(report.schemaVersion, 1);
  assert.equal(report.kind, 'business-launchpad-report');
  assert.equal(report.input.sellingPricePerUnitMinor, 20_000);
  assert.equal(report.report.monthlyScenario.operatingResultBeforeExcludedItemsMinor, 470_000);
  completedChecks.push('report export contains the evaluated inputs and matching integer-minor-unit result');

  await page.locator('#selling-price').fill('201.00');
  assert.equal(
    await page.locator('#download-report').isDisabled(),
    true,
    'editing an input must invalidate the previously calculated report'
  );
  completedChecks.push('editing a field disables a stale report download');

  const priceBeforeInvalidImport = await page.locator('#selling-price').inputValue();
  await page.locator('#load-scenario-file').setInputFiles({
    name: 'unsupported.json',
    mimeType: 'application/json',
    buffer: Buffer.from(JSON.stringify({ schemaVersion: 2, kind: 'business-launchpad-scenario', input: {} }))
  });
  await errorBox.waitFor({ state: 'visible' });
  assert.match(await errorBox.innerText(), /schemaVersion 1 required/);
  assert.equal(await page.locator('#selling-price').inputValue(), priceBeforeInvalidImport);
  completedChecks.push('unsupported scenario version fails visibly without overwriting entered values');

  await page.locator('#load-scenario-file').setInputFiles({
    name: 'oversized.json',
    mimeType: 'application/json',
    buffer: Buffer.alloc(1_000_001, 0x20)
  });
  await errorBox.waitFor({ state: 'visible' });
  assert.match(await errorBox.innerText(), /larger than 1 MB/);
  completedChecks.push('oversized scenario import is rejected before JSON parsing');

  await page.setViewportSize({ width: 390, height: 844 });
  await page.goto(origin + '/web/index.html', { waitUntil: 'networkidle' });
  await page.getByRole('button', { name: /Calculate scenario/ }).click();
  await page.locator('#result').getByText('ZAR 5100.00', { exact: true }).waitFor({ state: 'visible' });
  const mobileLayout = await page.evaluate(() => ({
    viewportWidth: window.innerWidth,
    documentScrollWidth: document.documentElement.scrollWidth
  }));
  assert.ok(
    mobileLayout.documentScrollWidth <= mobileLayout.viewportWidth,
    'mobile layout should not require horizontal scrolling: ' + JSON.stringify(mobileLayout)
  );
  await page.screenshot({ path: resolve(evidenceDir, '03-mobile-calculation.png'), fullPage: true });
  completedChecks.push('390px viewport renders a calculation without horizontal page overflow');

  assert.deepEqual(pageErrors, [], 'browser runtime should not emit uncaught page or console errors');

  const record = {
    status: 'passed',
    qualifiedSha: process.env.QUALIFIED_SHA || 'not-provided',
    browser: 'Chromium',
    browserVersion: browser.version(),
    checks: completedChecks,
    pageErrors,
    mobileLayout,
    screenshots: [
      '01-keyboard-validation-error.png',
      '02-keyboard-calculation-result.png',
      '03-mobile-calculation.png'
    ],
    limits: [
      'One Chromium version only; cross-browser compatibility is not established.',
      'Keyboard interaction checks do not establish a full screen-reader or assistive-technology audit.',
      'No legal compliance, tax accuracy, demand forecast, market fit or production readiness is established.',
      'No external service, account, payment provider or real-money operation is invoked.'
    ]
  };
  await writeFile(resolve(evidenceDir, 'browser-journey-result.json'), JSON.stringify(record, null, 2) + '\n');
  console.log(JSON.stringify(record, null, 2));
} catch (error) {
  if (page) {
    await page.screenshot({ path: resolve(evidenceDir, 'failure.png'), fullPage: true }).catch(() => {});
  }
  throw error;
} finally {
  if (browser) await browser.close();
  await new Promise(resolveClose => {
    server.close(resolveClose);
    server.closeAllConnections?.();
  });
}
