#!/usr/bin/env node
// Live-browser Live-mode proof for the Pulse working alpha.
//
// Drives two independent Playwright browser contexts (Alice, Bob) against
// two independent throwaway Holochain conductors (see launch-conductors.sh),
// each context pointed at its own conductor via an injected
// window.__HC_CONDUCTOR_URL — the same override mechanism the app already
// uses for production vs. local dev, so this requires zero application code
// changes. Proves: real Live-mode connection (not Demo/Unavailable), real
// onboarding (device-local hybrid-PQC key generation + publish through the
// actual WASM crypto + IndexedDB path, never exercised outside a browser
// before), and a real compose -> send -> read round trip between two
// distinct agents.
//
// Prerequisites:
//   1. just build-happ   (fresh restricted alpha hApp)
//   2. just build-ui     (fresh apps/leptos/dist/)
//   3. ./launch-conductors.sh   (starts Alice on :4445, Bob on :4447)
//   4. python3 -m http.server 8409 --directory ../../apps/leptos/dist
//   5. npm install playwright   (in this directory or nearby scratch dir)
//
// Real selectors below were read directly out of the Leptos source
// (profile_setup.rs, compose.rs, settings.rs, rich_editor.rs) — not
// guessed — but this script has not yet been run against a live app, so
// treat it as a strong first draft, not confirmed-working automation.
// Update PULSE_UI_URL / conductor ports to match your environment.

import { chromium } from 'playwright';
import { issueAppAuthTokenB64 } from './issue-token.mjs';
import { issueSigningCredentials } from './issue-signing-credentials.mjs';
import { discoverCellId } from './discover-cell-id.mjs';

const PULSE_UI_URL = process.env.PULSE_UI_URL || 'http://127.0.0.1:8409/';
const ALICE_CONDUCTOR_URL = process.env.ALICE_CONDUCTOR_URL || 'ws://127.0.0.1:4445';
const BOB_CONDUCTOR_URL = process.env.BOB_CONDUCTOR_URL || 'ws://127.0.0.1:4447';
const ALICE_ADMIN_URL = process.env.ALICE_ADMIN_URL || 'ws://127.0.0.1:4444';
const BOB_ADMIN_URL = process.env.BOB_ADMIN_URL || 'ws://127.0.0.1:4446';
const INSTALLED_APP_ID = process.env.INSTALLED_APP_ID || 'mycelix_mail';
const CHROMIUM_PATH = process.env.PLAYWRIGHT_CHROMIUM_EXECUTABLE_PATH; // falls back to Playwright's own if unset

// The app fetches its app-authentication token from /api/token
// (holochain.rs::fetch_auth_token) — the static dist/ server has no such
// endpoint, so intercept it per-context and serve a real token issued
// straight from that context's own admin API. Without this the app can't
// authenticate its app WebSocket and reports "Unavailable".
async function interceptAuthToken(context, adminUrl) {
  const tokenB64 = await issueAppAuthTokenB64(adminUrl, INSTALLED_APP_ID);
  // holochain.rs::fetch_auth_token expects JSON `{"token": "<base64>"}`,
  // then base64-decodes the `token` field itself — plain text 400s the parse.
  await context.route('**/api/token', (route) =>
    route.fulfill({
      status: 200,
      contentType: 'application/json',
      body: JSON.stringify({ token: tokenB64 }),
    }),
  );
}

// The app fetches zome-call signing credentials from
// /api/signing-credentials (holochain.rs::fetch_signing_credentials) —
// same broker role as interceptAuthToken above, playing the part a real
// production backend (matching the existing /api/token pattern) would.
// Discovers this context's own cell_id independently rather than requiring
// the browser to report it, so no application code needs to plumb it
// through an extra round trip.
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

function log(who, msg) {
  console.log(`[${who}] ${msg}`);
}

async function waitForLiveStatus(page, who, timeoutMs = 30000) {
  // nav.rs renders `.status-label` text as one of "Live" / "Connecting" /
  // "Demo" / "Unavailable" / "Offline", with a matching `.status-dot <word>`
  // class — status-label text is the more legible signal, prefer it.
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
    throw new Error(
      `${who}: expected "Live" status, observed "${status}" — this is the exact ` +
        `failure mode ALPHA_EVIDENCE.md requires: never silently fall into Demo.`,
    );
  }
}

async function completeOnboarding(page, who, displayName) {
  log(who, 'waiting for onboarding modal (profile_setup.rs SetupStep::NameEntry)');
  await page.waitForSelector('#setup-name', { timeout: 30000 });
  await page.fill('#setup-name', displayName);
  log(who, 'submitting profile — this triggers device-local hybrid-PQC key ' +
    'generation (device_keystore.rs) and publish_hybrid_key_bundle_v2');
  await page.click('input[type=submit]');
  // SetupStep::KeyGen -> Complete -> the modal auto-dismisses after ~2s
  // (see profile_setup.rs: sleep(2000) then step.set(HasProfile)). The
  // modal is never removed from the DOM — profile_setup.rs toggles
  // `display:none` on step changes rather than conditionally rendering —
  // so 'detached' here never fires; a prior version of this script used
  // 'detached' and always timed out once real (signed) zome calls made it
  // this far, found live via the Pulse browser proof.
  await page.waitForSelector('#setup-name', { state: 'hidden', timeout: 60000 });
  log(who, 'onboarding complete, modal dismissed');

  // A "Get notified of new mail?" prompt appears after onboarding and
  // overlays the nav bar, intercepting subsequent clicks — found live via
  // the Pulse browser proof once onboarding itself started succeeding.
  // Dismiss it the same way a real user declining notifications would.
  const dismissNotifPrompt = page.locator('button:has-text("Not now")');
  if (await dismissNotifPrompt.isVisible({ timeout: 5000 }).catch(() => false)) {
    await dismissNotifPrompt.click();
    log(who, 'dismissed the notification-permission prompt');
  }
}

async function readOwnAgentKey(page, who) {
  // The app uses real leptos_router path-based client-side navigation
  // (nav.rs: <A href="/settings">), NOT hash routing — a prior
  // page.goto('#/settings') never actually navigated at all (the fragment
  // is invisible to a path-based router, and the plain python static
  // server has no SPA fallback for a real page.goto('/settings') either),
  // silently leaving every post-onboarding step running against whatever
  // page onboarding happened to end on. Found live via the Pulse browser
  // proof once onboarding itself started succeeding for the first time.
  // Click the real nav link instead, exactly like a user would.
  await page.click('nav a:has-text("Settings"), a:has-text("Settings")');
  // Settings page displays "Agent Public Key" as a <p> sibling of its
  // <label> (settings.rs ~line 947) — no stable id, so match by text.
  const key = await page
    .locator('label:has-text("Agent Public Key") + p')
    .first()
    .innerText({ timeout: 30000 });
  log(who, `own agent public key: ${key.slice(0, 24)}...`);
  return key.trim();
}

async function composeAndSend(page, who, recipientKeyOrDsid, subject, body) {
  await page.click('nav a:has-text("Compose"), a:has-text("Compose")');
  await page.fill('#to', recipientKeyOrDsid);
  await page.fill('#subject', subject);
  // Body is a contenteditable div, not a <textarea> — rich_editor.rs.
  await page.click('.rich-editor .editor-content');
  await page.keyboard.type(body);
  log(who, `sending "${subject}" to ${recipientKeyOrDsid.slice(0, 16)}...`);
  await page.click('.compose-actions .btn-primary:has-text("Send")');
  // Send is async (V2 hybrid seal + zome call); wait for the button to
  // leave its "Sending..." state as the simplest completion signal.
  await page.waitForSelector('.compose-actions .btn-primary:has-text("Sending...")', {
    state: 'detached',
    timeout: 60000,
  });
  log(who, 'send completed (button left Sending... state)');
}

async function readInboxAndAssert(page, who, expectedSubject) {
  // Inbox is the root route ("/"), not "/inbox" — nav.rs: <A href="/">.
  await page.click('nav a:has-text("Inbox"), a:has-text("Inbox")');
  await page.waitForFunction(
    (subject) => document.body.innerText.includes(subject),
    expectedSubject,
    { timeout: 60000 },
  );
  log(who, `inbox shows expected subject "${expectedSubject}" — decrypted, ` +
    'ML-DSA-verified V2 content is genuinely rendering in a real browser');
}

async function main() {
  const browser = await chromium.launch({
    headless: true,
    executablePath: CHROMIUM_PATH,
  });

  const alice = await browser.newContext();
  const bob = await browser.newContext();

  // This is the whole trick for two identities against one built UI: inject
  // the override the app already reads (holochain.rs reads
  // window.__HC_CONDUCTOR_URL) before any page script runs.
  await alice.addInitScript((url) => {
    window.__HC_CONDUCTOR_URL = url;
  }, ALICE_CONDUCTOR_URL);
  await bob.addInitScript((url) => {
    window.__HC_CONDUCTOR_URL = url;
  }, BOB_CONDUCTOR_URL);

  await interceptAuthToken(alice, ALICE_ADMIN_URL);
  await interceptAuthToken(bob, BOB_ADMIN_URL);
  await interceptSigningCredentials(alice, ALICE_ADMIN_URL, ALICE_CONDUCTOR_URL);
  await interceptSigningCredentials(bob, BOB_ADMIN_URL, BOB_CONDUCTOR_URL);

  const alicePage = await alice.newPage();
  const bobPage = await bob.newPage();

  try {
    log('alice', `navigating to ${PULSE_UI_URL} (conductor ${ALICE_CONDUCTOR_URL})`);
    await alicePage.goto(PULSE_UI_URL);
    log('bob', `navigating to ${PULSE_UI_URL} (conductor ${BOB_CONDUCTOR_URL})`);
    await bobPage.goto(PULSE_UI_URL);

    await waitForLiveStatus(alicePage, 'alice');
    await waitForLiveStatus(bobPage, 'bob');

    await completeOnboarding(alicePage, 'alice', 'Alice (live-proof)');
    await completeOnboarding(bobPage, 'bob', 'Bob (live-proof)');

    const bobKey = await readOwnAgentKey(bobPage, 'bob');

    const subject = `Live-proof ${new Date().toISOString()}`;
    await composeAndSend(alicePage, 'alice', bobKey, subject, 'This round-trip proves Live mode end-to-end.');

    await readInboxAndAssert(bobPage, 'bob', subject);

    console.log('\n=== LIVE-BROWSER PROOF PASSED ===');
  } finally {
    await alice.close();
    await bob.close();
    await browser.close();
  }
}

main().catch((error) => {
  console.error('\n=== LIVE-BROWSER PROOF FAILED ===');
  console.error(error);
  process.exitCode = 1;
});
