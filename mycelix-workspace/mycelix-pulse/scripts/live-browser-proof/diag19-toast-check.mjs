#!/usr/bin/env node
// Full onboard->compose->send flow (own fresh implementation, not reusing
// live-proof.mjs's helpers) with BOTH console forwarding AND direct DOM
// inspection of .toast-message after send — every error branch in
// compose.rs's on_send EXCEPT the final send_email_v2 match only shows a
// toast, with zero console output, so console alone can't distinguish
// "hung" from "failed silently at an earlier step".
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

function agentPubKeyToHolochainKeyString(bytes) { return 'u' + Buffer.from(bytes).toString('base64url'); }
async function clickByDomWithText(page, selector, text) {
  await page.waitForFunction(
    ({ sel, txt }) => Array.from(document.querySelectorAll(sel)).some((el) => el.textContent.includes(txt)),
    { sel: selector, txt: text }, { timeout: 15000 });
  await page.evaluate(({ sel, txt }) => {
    const el = Array.from(document.querySelectorAll(sel)).find((e) => e.textContent.includes(txt));
    el?.click();
  }, { sel: selector, txt: text });
}
async function focusByDom(page, selector) {
  await page.waitForSelector(selector, { timeout: 15000 });
  await page.evaluate((sel) => document.querySelector(sel)?.focus(), selector);
}
async function clickNavLink(page, text) { await clickByDomWithText(page, '.navbar .nav-links a', text); }

async function setupContext(browser, conductorUrl, adminUrl) {
  const context = await browser.newContext();
  await context.addInitScript((url) => { window.__HC_CONDUCTOR_URL = url; }, conductorUrl);
  const tokenB64 = await issueAppAuthTokenB64(adminUrl, INSTALLED_APP_ID);
  await context.route('**/api/token', (route) =>
    route.fulfill({ status: 200, contentType: 'application/json', body: JSON.stringify({ token: tokenB64 }) }));
  const { dnaHash, agentPubKey } = await discoverCellId(conductorUrl, adminUrl, INSTALLED_APP_ID);
  const creds = await issueSigningCredentials(adminUrl, dnaHash, agentPubKey);
  await context.route('**/api/signing-credentials', (route) =>
    route.fulfill({
      status: 200, contentType: 'application/json',
      body: JSON.stringify({
        signingAgentKey: creds.signingAgentKeyB64, privateKey: creds.privateKeyB64, capSecret: creds.capSecretB64,
      }),
    }));
  return { context, agentPubKey };
}

async function onboard(page, who, name) {
  await page.goto(PULSE_UI_URL);
  await page.waitForFunction(() => document.querySelector('.status-label')?.textContent !== 'Connecting', { timeout: 30000 });
  await page.waitForTimeout(1500);
  // If already onboarded (HasProfile), the modal never appears — that's fine.
  const hasModal = await page.locator('#setup-name').count();
  if (hasModal && await page.locator('#setup-name').isVisible().catch(() => false)) {
    await page.fill('#setup-name', name);
    await page.click('input[type=submit]');
    await page.waitForSelector('#setup-name', { state: 'hidden', timeout: 60000 });
    await page.waitForTimeout(1000);
    const dismissNotifPrompt = page.locator('button:has-text("Not now")');
    if (await dismissNotifPrompt.isVisible({ timeout: 5000 }).catch(() => false)) {
      await clickByDomWithText(page, 'button', 'Not now');
    }
    console.log(`[${who}] onboarded fresh`);
  } else {
    console.log(`[${who}] already had a profile, skipped onboarding`);
  }
}

async function main() {
  const browser = await chromium.launch({ headless: true, executablePath: CHROMIUM_PATH });

  const { context: bobCtx, agentPubKey: bobKeyBytes } = await setupContext(browser, BOB_CONDUCTOR_URL, BOB_ADMIN_URL);
  const bobPage = await bobCtx.newPage();
  await onboard(bobPage, 'bob', 'Bob Diag19');
  const bobKey = agentPubKeyToHolochainKeyString(bobKeyBytes);
  console.log('[diag] bob key:', bobKey);

  const { context: aliceCtx } = await setupContext(browser, ALICE_CONDUCTOR_URL, ALICE_ADMIN_URL);
  const alicePage = await aliceCtx.newPage();
  alicePage.on('console', (msg) => {
    const t = msg.text();
    if (msg.type() === 'error' || /\[Mail\]|panicked/i.test(t)) console.log(`[alice:console:${msg.type()}]`, t);
  });
  alicePage.on('pageerror', (err) => console.log('[alice:pageerror]', err.message));
  await onboard(alicePage, 'alice', 'Alice Diag19');

  console.log('[diag] waiting 20s for bob key bundle to propagate before sending...');
  await alicePage.waitForTimeout(20000);

  await alicePage.bringToFront();
  await clickNavLink(alicePage, 'Compose');
  await alicePage.fill('#to', bobKey);
  await alicePage.fill('#subject', 'diag19 toast test');
  await focusByDom(alicePage, '.rich-editor .editor-content');
  await alicePage.keyboard.type('diag19 body');
  console.log('[diag] clicking send...');
  await clickByDomWithText(alicePage, '.compose-actions .btn-primary', 'Send');
  await alicePage.waitForSelector('.compose-actions .btn-primary:has-text("Sending...")', { state: 'detached', timeout: 60000 });
  console.log('[diag] button settled, watching toasts for 12s...');

  for (let i = 0; i < 6; i++) {
    await alicePage.waitForTimeout(2000);
    const toasts = await alicePage.evaluate(() =>
      Array.from(document.querySelectorAll('.toast-message')).map((el) => el.textContent));
    console.log(`[diag] t+${(i + 1) * 2}s toasts:`, JSON.stringify(toasts));
  }

  await browser.close();
}
main().catch((e) => { console.error('[diag] fatal', e); process.exit(1); });
