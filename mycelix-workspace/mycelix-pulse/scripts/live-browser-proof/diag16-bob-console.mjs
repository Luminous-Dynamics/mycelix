#!/usr/bin/env node
// Load Bob's page fresh against the still-running conductor from a failed
// live-proof.mjs attempt, capturing full browser console output — the main
// script never forwards console logs, so this is the only way to see the
// real [Mail] load_inbox / V2 errors.
import { chromium } from 'playwright';
import { issueAppAuthTokenB64 } from './issue-token.mjs';
import { issueSigningCredentials } from './issue-signing-credentials.mjs';
import { discoverCellId } from './discover-cell-id.mjs';

const PULSE_UI_URL = process.env.PULSE_UI_URL || 'http://127.0.0.1:8409/';
const BOB_CONDUCTOR_URL = process.env.BOB_CONDUCTOR_URL || 'ws://127.0.0.1:4447';
const BOB_ADMIN_URL = process.env.BOB_ADMIN_URL || 'ws://127.0.0.1:4446';
const INSTALLED_APP_ID = process.env.INSTALLED_APP_ID || 'mycelix_mail';
const CHROMIUM_PATH = process.env.PLAYWRIGHT_CHROMIUM_EXECUTABLE_PATH;

async function main() {
  const browser = await chromium.launch({ headless: true, executablePath: CHROMIUM_PATH });
  const context = await browser.newContext();
  await context.addInitScript((url) => { window.__HC_CONDUCTOR_URL = url; }, BOB_CONDUCTOR_URL);

  const tokenB64 = await issueAppAuthTokenB64(BOB_ADMIN_URL, INSTALLED_APP_ID);
  await context.route('**/api/token', (route) =>
    route.fulfill({ status: 200, contentType: 'application/json', body: JSON.stringify({ token: tokenB64 }) }));
  const { dnaHash, agentPubKey } = await discoverCellId(BOB_CONDUCTOR_URL, BOB_ADMIN_URL, INSTALLED_APP_ID);
  const creds = await issueSigningCredentials(BOB_ADMIN_URL, dnaHash, agentPubKey);
  await context.route('**/api/signing-credentials', (route) =>
    route.fulfill({
      status: 200, contentType: 'application/json',
      body: JSON.stringify({
        signingAgentKey: creds.signingAgentKeyB64,
        privateKey: creds.privateKeyB64,
        capSecret: creds.capSecretB64,
      }),
    }));

  const page = await context.newPage();
  page.on('console', (msg) => console.log(`[console:${msg.type()}]`, msg.text()));
  page.on('pageerror', (err) => console.log('[pageerror]', err.message));

  await page.goto(PULSE_UI_URL);
  await page.waitForFunction(() => document.querySelector('.status-label')?.textContent !== 'Connecting', { timeout: 30000 });
  await page.waitForTimeout(8000);
  console.log('[diag] done waiting');
  await browser.close();
}
main().catch((e) => { console.error('[diag] fatal', e); process.exit(1); });
