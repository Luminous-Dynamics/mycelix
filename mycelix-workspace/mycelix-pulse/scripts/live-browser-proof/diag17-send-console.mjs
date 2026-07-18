#!/usr/bin/env node
// Reproduce a real compose->send on the still-live Alice conductor (already
// onboarded from a prior failed live-proof.mjs run) with full console
// forwarding, to see the ACTUAL success/failure signal from compose.rs —
// live-proof.mjs's "button leaves Sending..." wait fires on both Ok and Err
// arms of the send_email_v2 match, so it can't distinguish them.
import { chromium } from 'playwright';
import { issueAppAuthTokenB64 } from './issue-token.mjs';
import { issueSigningCredentials } from './issue-signing-credentials.mjs';
import { discoverCellId } from './discover-cell-id.mjs';

const PULSE_UI_URL = process.env.PULSE_UI_URL || 'http://127.0.0.1:8409/';
const ALICE_CONDUCTOR_URL = process.env.ALICE_CONDUCTOR_URL || 'ws://127.0.0.1:4445';
const ALICE_ADMIN_URL = process.env.ALICE_ADMIN_URL || 'ws://127.0.0.1:4444';
const BOB_CONDUCTOR_URL = process.env.BOB_CONDUCTOR_URL || 'ws://127.0.0.1:4447';
const BOB_ADMIN_URL = process.env.BOB_ADMIN_URL || 'ws://127.0.0.1:4446';
const INSTALLED_APP_ID = process.env.INSTALLED_APP_ID || 'mycelix_mail';
const CHROMIUM_PATH = process.env.PLAYWRIGHT_CHROMIUM_EXECUTABLE_PATH;

function agentPubKeyToHolochainKeyString(bytes) {
  return 'u' + Buffer.from(bytes).toString('base64url');
}

async function clickByDom(page, selector) {
  await page.waitForSelector(selector, { timeout: 15000 });
  await page.evaluate((sel) => document.querySelector(sel)?.click(), selector);
}
async function clickByDomWithText(page, selector, text) {
  await page.waitForFunction(
    ({ sel, txt }) => Array.from(document.querySelectorAll(sel)).some((el) => el.textContent.includes(txt)),
    { sel: selector, txt: text }, { timeout: 15000 },
  );
  await page.evaluate(({ sel, txt }) => {
    const el = Array.from(document.querySelectorAll(sel)).find((e) => e.textContent.includes(txt));
    el?.click();
  }, { sel: selector, txt: text });
}
async function focusByDom(page, selector) {
  await page.waitForSelector(selector, { timeout: 15000 });
  await page.evaluate((sel) => document.querySelector(sel)?.focus(), selector);
}
async function clickNavLink(page, text) {
  await clickByDomWithText(page, '.navbar .nav-links a', text);
}

async function main() {
  // resolve bob's real key first
  const bobTokenB64 = await issueAppAuthTokenB64(BOB_ADMIN_URL, INSTALLED_APP_ID);
  const { agentPubKey: bobKeyBytes } = await discoverCellId(BOB_CONDUCTOR_URL, BOB_ADMIN_URL, INSTALLED_APP_ID);
  const bobKey = agentPubKeyToHolochainKeyString(bobKeyBytes);
  console.log('[diag] bob key:', bobKey);

  const browser = await chromium.launch({ headless: true, executablePath: CHROMIUM_PATH });
  const context = await browser.newContext();
  await context.addInitScript((url) => { window.__HC_CONDUCTOR_URL = url; }, ALICE_CONDUCTOR_URL);

  const tokenB64 = await issueAppAuthTokenB64(ALICE_ADMIN_URL, INSTALLED_APP_ID);
  await context.route('**/api/token', (route) =>
    route.fulfill({ status: 200, contentType: 'application/json', body: JSON.stringify({ token: tokenB64 }) }));
  const { dnaHash, agentPubKey } = await discoverCellId(ALICE_CONDUCTOR_URL, ALICE_ADMIN_URL, INSTALLED_APP_ID);
  const creds = await issueSigningCredentials(ALICE_ADMIN_URL, dnaHash, agentPubKey);
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
  await page.waitForTimeout(2000);

  await clickNavLink(page, 'Compose');
  await page.fill('#to', bobKey);
  await page.fill('#subject', 'diag17 console test');
  await focusByDom(page, '.rich-editor .editor-content');
  await page.keyboard.type('diag17 body');
  console.log('[diag] clicking send...');
  await clickByDomWithText(page, '.compose-actions .btn-primary', 'Send');
  await page.waitForSelector('.compose-actions .btn-primary:has-text("Sending...")', { state: 'detached', timeout: 60000 });
  console.log('[diag] send button settled — now waiting past the 5s undo window...');
  await page.waitForTimeout(12000);

  await browser.close();
}
main().catch((e) => { console.error('[diag] fatal', e); process.exit(1); });
