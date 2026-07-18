#!/usr/bin/env node
// Ad-hoc diagnostic: capture console + network for Alice only.
import { chromium } from 'playwright';
import { issueAppAuthTokenB64 } from './issue-token.mjs';

const PULSE_UI_URL = process.env.PULSE_UI_URL || 'http://127.0.0.1:8409/';
const ALICE_CONDUCTOR_URL = process.env.ALICE_CONDUCTOR_URL || 'ws://127.0.0.1:4445';
const ALICE_ADMIN_URL = process.env.ALICE_ADMIN_URL || 'ws://127.0.0.1:4444';
const INSTALLED_APP_ID = process.env.INSTALLED_APP_ID || 'mycelix_mail';
const CHROMIUM_PATH = process.env.PLAYWRIGHT_CHROMIUM_EXECUTABLE_PATH;

async function main() {
  const browser = await chromium.launch({ headless: true, executablePath: CHROMIUM_PATH });
  const context = await browser.newContext();
  await context.addInitScript((url) => {
    window.__HC_CONDUCTOR_URL = url;
  }, ALICE_CONDUCTOR_URL);

  const tokenB64 = await issueAppAuthTokenB64(ALICE_ADMIN_URL, INSTALLED_APP_ID);
  console.log('[diag] token issued, length b64:', tokenB64.length);
  await context.route('**/api/token', (route) => {
    console.log('[diag] /api/token intercepted ->', route.request().url());
    route.fulfill({
      status: 200,
      contentType: 'application/json',
      body: JSON.stringify({ token: tokenB64 }),
    });
  });

  const page = await context.newPage();
  page.on('console', (msg) => console.log(`[console:${msg.type()}]`, msg.text()));
  page.on('pageerror', (err) => console.log('[pageerror]', err.message));
  page.on('requestfailed', (req) => console.log('[requestfailed]', req.url(), req.failure()?.errorText));
  page.on('response', (resp) => {
    if (resp.url().includes('/api/token')) {
      console.log('[response] /api/token status', resp.status());
    }
  });

  await page.goto(PULSE_UI_URL);
  await page.waitForTimeout(8000);
  const status = await page.locator('.status-label').first().innerText().catch(() => 'N/A');
  console.log('[diag] final status label:', status);

  await browser.close();
}

main().catch((e) => {
  console.error('[diag] fatal', e);
  process.exit(1);
});
