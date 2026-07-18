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
// through an extra round trip. Returns the raw agentPubKey bytes too —
// composeAndSend needs the recipient's address, and resolving it this way
// (already available from the broker's own app_info lookup) is far more
// reliable than reading it back out of the Settings page UI, which has an
// unrelated, still-open bug: settings.rs's `loading` flag only flips to
// false after BOTH get_my_profile AND a get_did_document call to a
// separate "identity" role/cell that this restricted alpha DNA doesn't
// have installed — found live via the Pulse browser proof, not yet fixed.
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
  return agentPubKeyToHolochainKeyString(agentPubKey);
}

// Matches connected_agent_pub_key_b64() in browser.rs: `u` + unpadded
// base64url of the raw 39-byte AgentPubKey — the format compose.rs's "To"
// field parses.
function agentPubKeyToHolochainKeyString(agentPubKeyBytes) {
  return 'u' + Buffer.from(agentPubKeyBytes).toString('base64url');
}

function log(who, msg) {
  console.log(`[${who}] ${msg}`);
}

// Clicks the first .navbar .nav-links <a> whose text matches, via the DOM
// API directly (element.click()) rather than a coordinate-based Playwright
// click. Found live via the Pulse browser proof, reproduced 6/6 times: even
// with `force: true`, a Playwright click still dispatches at the target
// element's on-screen COORDINATES, and Chrome resolves that to whatever is
// actually topmost at that pixel — some other overlay (a toast, the
// "Today's Digest" panel, or a leftover notification-prompt remnant) was
// silently swallowing the event, so the link's own click handler never
// fired and no navigation ever happened, with no error surfaced anywhere.
// `element.click()` from inside the page invokes the handler directly,
// sidestepping stacking/coordinates entirely — confirmed to actually
// navigate where the coordinate-based version silently didn't.
async function clickNavLink(page, text) {
  const clicked = await page.evaluate((linkText) => {
    const links = Array.from(document.querySelectorAll('.navbar .nav-links a'));
    const link = links.find((a) => a.textContent.trim() === linkText);
    if (!link) return false;
    link.click();
    return true;
  }, text);
  if (!clicked) {
    throw new Error(`clickNavLink: no ".navbar .nav-links a" with text "${text}" found`);
  }
}

// Same rationale as clickNavLink: a leftover element from an earlier step
// (the onboarding modal's own inputs, e.g. #setup-bio, never leaves the DOM
// — profile_setup.rs only toggles display:none — and can still occupy
// enough layout/stacking to swallow a coordinate-based click aimed at an
// unrelated element elsewhere on the page) can silently eat a real click
// even when Playwright reports the target itself as visible/stable.
// Selecting and invoking .click()/.focus() directly from inside the page
// sidesteps this class of bug entirely.
async function clickByDom(page, selector) {
  const clicked = await page.evaluate((sel) => {
    const el = document.querySelector(sel);
    if (!el) return false;
    el.click();
    return true;
  }, selector);
  if (!clicked) {
    throw new Error(`clickByDom: no element matching "${selector}"`);
  }
}

async function clickByDomWithText(page, selector, text) {
  const clicked = await page.evaluate(
    ({ sel, needle }) => {
      const els = Array.from(document.querySelectorAll(sel));
      const el = els.find((e) => e.textContent.includes(needle));
      if (!el) return false;
      el.click();
      return true;
    },
    { sel: selector, needle: text },
  );
  if (!clicked) {
    throw new Error(`clickByDomWithText: no element matching "${selector}" containing "${text}"`);
  }
}

async function focusByDom(page, selector) {
  const focused = await page.evaluate((sel) => {
    const el = document.querySelector(sel);
    if (!el) return false;
    el.focus();
    return true;
  }, selector);
  if (!focused) {
    throw new Error(`focusByDom: no element matching "${selector}"`);
  }
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

  // A "Get notified of new mail?" prompt (notif_prompt.rs) appears after
  // onboarding — found live via the Pulse browser proof once onboarding
  // itself started succeeding. Dismissed via a direct DOM click (see
  // clickByDom's doc comment) — coordinate-based clicks here were
  // originally fixed with `force: true`, which helped intermittently, but
  // the fully robust fix is the same one needed everywhere else in this
  // script: the leftover, never-detached onboarding modal DOM can still
  // swallow coordinate-based clicks aimed elsewhere on the page.
  const dismissNotifPrompt = page.locator('button:has-text("Not now")');
  if (await dismissNotifPrompt.isVisible({ timeout: 5000 }).catch(() => false)) {
    await clickByDomWithText(page, 'button', 'Not now');
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
  await clickNavLink(page, 'Settings');
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
  // Scoped to `.navbar .nav-links` specifically, not a bare `nav a` —
  // bottom_nav.rs renders a SECOND, separately-labeled "Compose" link
  // (`<nav class="bottom-nav"><a class="bottom-nav-item">...<span
  // class="bottom-nav-label">Compose</span>`) for the mobile layout, which
  // an unscoped selector can also match and click even when it's the wrong
  // (CSS-hidden at this viewport) one — found live via the Pulse browser
  // proof: the click reported success with no error, but #to never
  // appeared because navigation never actually happened.
  await clickNavLink(page, 'Compose');
  await page.fill('#to', recipientKeyOrDsid);
  await page.fill('#subject', subject);
  // Body is a contenteditable div, not a <textarea> — rich_editor.rs.
  // Focused directly (see clickByDom's doc comment for why coordinate-based
  // interaction is unreliable here) — .focus() is enough for
  // page.keyboard.type() to work, since keyboard events target whatever
  // element currently has focus, not a screen coordinate.
  await focusByDom(page, '.rich-editor .editor-content');
  await page.keyboard.type(body);
  // Keep this page frontmost through the whole send — compose.rs's on_send
  // clears the form and flips `sending` back to false SYNCHRONOUSLY, then
  // does the real work (5s undo delay, then resolve_hybrid_send_context_v2
  // + encrypt + send_email_v2) inside a `spawn_local` task that only
  // resumes once its `gloo_timers::future::sleep` (a setTimeout under the
  // hood) actually fires. Found live: if this page isn't the frontmost one
  // when the caller moves on to interact with the OTHER agent's page,
  // Chromium's background-tab timer throttling can stall that setTimeout
  // indefinitely.
  await page.bringToFront();

  // Retry the whole compose->send cycle if the recipient's hybrid-PQC V2
  // key bundle hasn't propagated to this agent's DHT view yet.
  // resolve_hybrid_send_context_v2 (called inside on_send's delayed block,
  // after the 5s undo window) can legitimately return None for a few
  // seconds right after the recipient's own onboarding publishes their key
  // bundle — a real DHT-propagation race, not a bug in send_email_v2 or in
  // the P2P transport. compose.rs only surfaces this as a toast with NO
  // console output at all, which is why earlier debugging here treated it
  // as a mysterious total-silence hang — found live via
  // scripts/live-browser-proof/diag19-toast-check.mjs reading
  // `.toast-message` directly.
  const maxSendAttempts = 6;
  for (let attempt = 1; attempt <= maxSendAttempts; attempt++) {
    log(who, `sending "${subject}" to ${recipientKeyOrDsid.slice(0, 16)}... (attempt ${attempt}/${maxSendAttempts})`);
    await clickByDomWithText(page, '.compose-actions .btn-primary', 'Send');
    // "Sending..." clears almost immediately (see comment above) — it does
    // NOT mean the real send finished, just that the synchronous part of
    // the click handler ran. Wait for it anyway (cheap), then hold this
    // page in the foreground past the 5s undo window plus margin for the
    // actual zome-call round trip before checking the outcome.
    await page.waitForSelector('.compose-actions .btn-primary:has-text("Sending...")', {
      state: 'detached',
      timeout: 60000,
    });
    // Toasts auto-dismiss after a few seconds — a single snapshot at a
    // fixed delay can land in the gap between the error toast appearing
    // and fading, missing it entirely and wrongly treating the attempt as
    // successful. Poll the whole window instead.
    let keyBundleMissing = false;
    let allToastsSeen = [];
    for (let i = 0; i < 9; i++) {
      await page.waitForTimeout(1000);
      const toasts = await page.evaluate(() =>
        Array.from(document.querySelectorAll('.toast-message')).map((el) => el.textContent));
      for (const t of toasts) {
        if (!allToastsSeen.includes(t)) allToastsSeen.push(t);
      }
      if (toasts.some((t) => t.includes('no active hybrid-PQC V2 key bundle'))) {
        keyBundleMissing = true;
        break;
      }
    }
    if (!keyBundleMissing) {
      log(who, `send completed (toasts seen: ${JSON.stringify(allToastsSeen)})`);
      return;
    }
    log(who, `recipient key bundle not yet visible (attempt ${attempt}/${maxSendAttempts}), retrying...`);
    // Re-open Compose (the failed attempt already cleared/reset the form)
    // and wait a bit longer for gossip before trying again.
    await page.waitForTimeout(5000);
    await clickNavLink(page, 'Compose');
    await page.fill('#to', recipientKeyOrDsid);
    await page.fill('#subject', subject);
    await focusByDom(page, '.rich-editor .editor-content');
    await page.keyboard.type(body);
  }
  throw new Error(`${who}: recipient key bundle never became visible after ${maxSendAttempts} send attempts`);
}

async function readInboxAndAssert(page, who, expectedSubject) {
  // mail_context.rs fetches get_inbox exactly once, at startup — there is
  // no polling/interval and no re-fetch on navigation, so simply
  // navigating to the Inbox route can never show a message that arrived
  // after the page first loaded. Found live via the Pulse browser proof:
  // the send genuinely succeeded (real signed zome call), but clicking the
  // Inbox nav link alone left the page on its original, stale in-memory
  // inbox state forever. A real user would refresh to see new mail; do the
  // same here with a real page reload, which re-runs mail_context's
  // startup fetch from scratch.
  //
  // A single reload wasn't enough on its own, even with a 60s wait after
  // it — Alice and Bob are on two fully independent conductors here (not
  // sharing one in-process test harness the way SweetTest does), so the
  // message has to be gossiped over a real network via the external
  // dev-test bootstrap service before Bob's DHT even has it to serve.
  // Retry the reload itself every 20s instead of doing one long wait after
  // a single fetch — each reload is a fresh chance to catch the message
  // once gossip has actually landed, rather than betting on a single
  // fetch's timing.
  const deadline = Date.now() + 4 * 60 * 1000;
  let lastError;
  while (Date.now() < deadline) {
    await page.reload();
    await page.waitForFunction(() => document.querySelector('.status-label')?.textContent !== 'Connecting', {
      timeout: 30000,
    });
    await clickNavLink(page, 'Inbox');
    try {
      await page.waitForFunction(
        (subject) => document.body.innerText.includes(subject),
        expectedSubject,
        { timeout: 20000 },
      );
      log(who, `inbox shows expected subject "${expectedSubject}" — decrypted, ` +
        'ML-DSA-verified V2 content is genuinely rendering in a real browser');
      return;
    } catch (error) {
      lastError = error;
      log(who, 'message not visible yet, reloading and retrying...');
    }
  }
  throw lastError;
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
  const bobKey = await interceptSigningCredentials(bob, BOB_ADMIN_URL, BOB_CONDUCTOR_URL);

  const alicePage = await alice.newPage();
  const bobPage = await bob.newPage();

  // Forward browser console/page errors — without this, a WASM panic
  // (e.g. a Leptos `expect_context` failure inside a delayed spawn_local)
  // kills a send/read silently: the surrounding Promise never rejects
  // Node-side, so nothing here would otherwise show it happened.
  for (const [who, page] of [['alice', alicePage], ['bob', bobPage]]) {
    page.on('console', (msg) => {
      const text = msg.text();
      if (msg.type() === 'error' || /\[Mail\]|panicked/i.test(text)) {
        log(who, `console:${msg.type()} ${text}`);
      }
    });
    page.on('pageerror', (err) => log(who, `pageerror: ${err.message}`));
  }

  try {
    log('alice', `navigating to ${PULSE_UI_URL} (conductor ${ALICE_CONDUCTOR_URL})`);
    await alicePage.goto(PULSE_UI_URL);
    log('bob', `navigating to ${PULSE_UI_URL} (conductor ${BOB_CONDUCTOR_URL})`);
    await bobPage.goto(PULSE_UI_URL);

    await waitForLiveStatus(alicePage, 'alice');
    await waitForLiveStatus(bobPage, 'bob');

    await completeOnboarding(alicePage, 'alice', 'Alice (live-proof)');
    await completeOnboarding(bobPage, 'bob', 'Bob (live-proof)');

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
