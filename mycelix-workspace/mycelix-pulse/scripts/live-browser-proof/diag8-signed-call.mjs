#!/usr/bin/env node
// Ad-hoc diagnostic: drive Alice through connect + signing-credentials
// install + a real zome call, capturing full console output.
import { chromium } from 'playwright';
import { issueAppAuthTokenB64 } from './issue-token.mjs';
import { issueSigningCredentials } from './issue-signing-credentials.mjs';
import { discoverCellId } from './discover-cell-id.mjs';

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
  await context.route('**/api/token', (route) =>
    route.fulfill({
      status: 200,
      contentType: 'application/json',
      body: JSON.stringify({ token: tokenB64 }),
    }),
  );

  console.log('[diag] discovering cell_id...');
  const { dnaHash, agentPubKey } = await discoverCellId(
    ALICE_CONDUCTOR_URL,
    ALICE_ADMIN_URL,
    INSTALLED_APP_ID,
  );
  console.log('[diag] granting signing credentials...');
  const creds = await issueSigningCredentials(ALICE_ADMIN_URL, dnaHash, agentPubKey);
  console.log('[diag] credentials granted:', {
    signingAgentKeyB64: creds.signingAgentKeyB64,
  });

  await context.route('**/api/signing-credentials', (route) =>
    route.fulfill({
      status: 200,
      contentType: 'application/json',
      body: JSON.stringify({
        signingAgentKey: creds.signingAgentKeyB64,
        privateKey: creds.privateKeyB64,
        capSecret: creds.capSecretB64,
      }),
    }),
  );

  const page = await context.newPage();
  page.on('console', (msg) => console.log(`[console:${msg.type()}]`, msg.text()));
  page.on('pageerror', (err) => console.log('[pageerror]', err.message));

  await page.goto(PULSE_UI_URL);
  // Wait long enough for connect + signing-credentials install + the app's
  // own post-connect zome calls (get_folders, get_inbox, etc.) to settle.
  await page.waitForTimeout(15000);

  const status = await page.locator('.status-label').first().innerText().catch(() => 'N/A');
  console.log('[diag] final status label:', status);

  await browser.close();
}

main().catch((e) => {
  console.error('[diag] fatal', e);
  process.exit(1);
});
