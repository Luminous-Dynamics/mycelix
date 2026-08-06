#!/usr/bin/env node
// Diagnostic for the Bob-onboarding "Saving profile to DHT..." stall
// documented in docs/ALPHA_EVIDENCE.md (2026-07-19 entry). Instruments the
// actual app-websocket zome-call round trip for both Alice and Bob via
// Playwright's page.on('websocket') — no application code changes needed —
// so we can see whether Bob's set_profile zome call is ever sent, and
// whether any response (success or error) ever comes back.
//
// Usage: same prerequisites as live-proof.mjs (fresh happ/UI build, both
// conductors launched, static server running on :8409).

import { chromium } from 'playwright';
import { decode } from '@msgpack/msgpack';
import { issueAppAuthTokenB64 } from './issue-token.mjs';
import { issueSigningCredentials } from './issue-signing-credentials.mjs';
import { discoverCellId } from './discover-cell-id.mjs';

const PULSE_UI_URL = process.env.PULSE_UI_URL || 'http://127.0.0.1:8409/';
const ALICE_CONDUCTOR_URL = process.env.ALICE_CONDUCTOR_URL || 'ws://127.0.0.1:4445';
const BOB_CONDUCTOR_URL = process.env.BOB_CONDUCTOR_URL || 'ws://127.0.0.1:4447';
const ALICE_ADMIN_URL = process.env.ALICE_ADMIN_URL || 'ws://127.0.0.1:4444';
const BOB_ADMIN_URL = process.env.BOB_ADMIN_URL || 'ws://127.0.0.1:4446';
const INSTALLED_APP_ID = process.env.INSTALLED_APP_ID || 'mycelix_mail';
const CHROMIUM_PATH = process.env.PLAYWRIGHT_CHROMIUM_EXECUTABLE_PATH;

function log(who, msg) {
  console.log(`[${who}] ${msg}`);
}

// Best-effort msgpack decode of a WS frame payload for logging. Falls back
// to a byte-length summary if it isn't decodable (binary ciphertext, etc.).
function describeFrame(payload) {
  try {
    const bytes = typeof payload === 'string' ? Buffer.from(payload, 'base64') : payload;
    const decoded = decode(bytes);
    return JSON.stringify(decoded, (_key, value) => {
      if (value instanceof Uint8Array || Buffer.isBuffer(value)) {
        return `<${value.length} bytes>`;
      }
      // Buffer.prototype.toJSON already ran by the time the replacer sees a
      // Buffer, turning it into {type:'Buffer', data:[...]} — catch that
      // shape too, or the raw byte array prints in full.
      if (
        value &&
        typeof value === 'object' &&
        value.type === 'Buffer' &&
        Array.isArray(value.data)
      ) {
        return `<${value.data.length} bytes>`;
      }
      return value;
    }).slice(0, 800);
  } catch {
    const len = typeof payload === 'string' ? payload.length : payload?.length ?? 0;
    return `<undecodable, ${len} bytes>`;
  }
}

function instrumentAppWebSocket(page, who, conductorUrl) {
  page.on('websocket', (ws) => {
    const url = ws.url();
    if (!url.includes(conductorUrl.replace('ws://', ''))) {
      log(who, `(ignoring unrelated websocket: ${url})`);
      return;
    }
    log(who, `app websocket opened: ${url}`);
    ws.on('framesent', (frame) => {
      log(who, `WS -> conductor: ${describeFrame(frame.payload)}`);
    });
    ws.on('framereceived', (frame) => {
      log(who, `WS <- conductor: ${describeFrame(frame.payload)}`);
    });
    ws.on('close', () => log(who, 'app websocket closed'));
    ws.on('socketerror', (error) => log(who, `app websocket error: ${error}`));
  });
}

async function interceptAuthToken(context, adminUrl) {
  const tokenB64 = await issueAppAuthTokenB64(adminUrl, INSTALLED_APP_ID);
  await context.route('**/api/token', (route) =>
    route.fulfill({
      status: 200,
      contentType: 'application/json',
      body: JSON.stringify({ token: tokenB64 }),
    }),
  );
}

async function interceptSigningCredentials(context, adminUrl, appUrl) {
  const { dnaHash, agentPubKey } = await discoverCellId(appUrl, adminUrl, INSTALLED_APP_ID);
  const creds = await issueSigningCredentials(adminUrl, dnaHash, agentPubKey);
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
}

async function waitForLiveStatus(page, who, timeoutMs = 30000) {
  await page.waitForFunction(
    () => {
      const el = document.querySelector('.status-label');
      return el && el.textContent !== 'Connecting';
    },
    { timeout: timeoutMs },
  );
  const status = await page.locator('.status-label').first().innerText();
  log(who, `runtime status observed: ${status}`);
  if (status !== 'Live') {
    throw new Error(`${who}: expected "Live" status, observed "${status}"`);
  }
}

// #setup-name's visibility is tied directly to SetupStep::NameEntry
// (profile_setup.rs: <form style=move || if is_name_entry() {""} else
// {"display:none"} ...>). Clicking submit sets step to KeyGen SYNCHRONOUSLY,
// hiding #setup-name immediately regardless of whether the async
// set_profile call ultimately succeeds or fails — so waitForSelector(...,
// {state:'hidden'}) is a false-positive completion signal on the failure
// path too (it resets step back to NameEntry, re-showing the form, but only
// *after* a brief window where it was hidden). Poll explicitly instead:
// watch for either the error-message div (inline style color #f87171)
// getting real text, or #setup-name staying hidden through a settle window
// with no error appearing.
async function attemptOnboarding(page, who, displayName, timeoutMs) {
  log(who, 'waiting for onboarding modal');
  await page.waitForSelector('#setup-name', { timeout: 30000 });
  await page.fill('#setup-name', displayName);
  log(who, 'submitting profile (this triggers set_profile zome call)');
  await page.click('input[type=submit]');

  const pollIntervalMs = 250;
  const settleWindowMs = 2000;
  let lastHiddenAt = null;
  const deadline = Date.now() + timeoutMs;

  while (Date.now() < deadline) {
    const errorText = await page
      .locator('div[style*="f87171"]')
      .first()
      .innerText()
      .catch(() => '');
    if (errorText && errorText.trim()) {
      log(who, `onboarding FAILED — error message shown: "${errorText.trim()}"`);
      return false;
    }
    const setupNameVisible = await page.locator('#setup-name').isVisible().catch(() => true);
    if (!setupNameVisible) {
      if (lastHiddenAt === null) lastHiddenAt = Date.now();
      if (Date.now() - lastHiddenAt >= settleWindowMs) {
        log(who, 'onboarding complete — #setup-name stayed hidden through the settle window, no error shown');
        return true;
      }
    } else {
      lastHiddenAt = null; // bounced back to visible — genuinely still in NameEntry
    }
    await page.waitForTimeout(pollIntervalMs);
  }

  log(who, `onboarding did NOT reach a definitive outcome within ${timeoutMs}ms`);
  const finalErrorText = await page.locator('div[style*="f87171"]').first().innerText().catch(() => '(none)');
  const finalSetupVisible = await page.locator('#setup-name').isVisible().catch(() => 'unknown');
  log(who, `final state: setup-name visible=${finalSetupVisible}, error text="${finalErrorText}"`);
  return false;
}

async function main() {
  const browser = await chromium.launch({ headless: true, executablePath: CHROMIUM_PATH });
  const alice = await browser.newContext();
  const bob = await browser.newContext();

  await alice.addInitScript((url) => { window.__HC_CONDUCTOR_URL = url; }, ALICE_CONDUCTOR_URL);
  await bob.addInitScript((url) => { window.__HC_CONDUCTOR_URL = url; }, BOB_CONDUCTOR_URL);

  await interceptAuthToken(alice, ALICE_ADMIN_URL);
  await interceptAuthToken(bob, BOB_ADMIN_URL);
  await interceptSigningCredentials(alice, ALICE_ADMIN_URL, ALICE_CONDUCTOR_URL);
  await interceptSigningCredentials(bob, BOB_ADMIN_URL, BOB_CONDUCTOR_URL);

  const alicePage = await alice.newPage();
  const bobPage = await bob.newPage();

  instrumentAppWebSocket(alicePage, 'alice-ws', ALICE_CONDUCTOR_URL);
  instrumentAppWebSocket(bobPage, 'bob-ws', BOB_CONDUCTOR_URL);
  alicePage.on('console', (msg) => log('alice-console', msg.text()));
  bobPage.on('console', (msg) => log('bob-console', msg.text()));
  alicePage.on('pageerror', (err) => log('alice-pageerror', String(err)));
  bobPage.on('pageerror', (err) => log('bob-pageerror', String(err)));

  try {
    log('alice', `navigating to ${PULSE_UI_URL}`);
    await alicePage.goto(PULSE_UI_URL);
    log('bob', `navigating to ${PULSE_UI_URL}`);
    await bobPage.goto(PULSE_UI_URL);

    await waitForLiveStatus(alicePage, 'alice');
    await waitForLiveStatus(bobPage, 'bob');

    const aliceOk = await attemptOnboarding(alicePage, 'alice', 'Alice (diag)', 60000);
    log('alice', `onboarding result: ${aliceOk}`);

    const bobOk = await attemptOnboarding(bobPage, 'bob', 'Bob (diag)', 60000);
    log('bob', `onboarding result: ${bobOk}`);

    console.log('\n=== DIAGNOSTIC COMPLETE ===');
  } finally {
    await alice.close();
    await bob.close();
    await browser.close();
  }
}

main().catch((error) => {
  console.error('\n=== DIAGNOSTIC FAILED ===');
  console.error(error);
  process.exitCode = 1;
});
