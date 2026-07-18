#!/usr/bin/env node
// Direct native check of get_inbox_v2 (bob) and get_sent (v1, for cross-
// check) — bypasses the browser and mail_context.rs's rendering path
// entirely, to see raw DHT state independent of any UI bug.
import { issueAppAuthTokenB64 } from './issue-token.mjs';
import { issueSigningCredentials } from './issue-signing-credentials.mjs';
import { discoverCellId } from './discover-cell-id.mjs';
import { callZome } from './call-zome.mjs';

const ALICE_CONDUCTOR_URL = process.env.ALICE_CONDUCTOR_URL || 'ws://127.0.0.1:4445';
const ALICE_ADMIN_URL = process.env.ALICE_ADMIN_URL || 'ws://127.0.0.1:4444';
const BOB_CONDUCTOR_URL = process.env.BOB_CONDUCTOR_URL || 'ws://127.0.0.1:4447';
const BOB_ADMIN_URL = process.env.BOB_ADMIN_URL || 'ws://127.0.0.1:4446';
const INSTALLED_APP_ID = process.env.INSTALLED_APP_ID || 'mycelix_mail';

async function callFn(who, appUrl, adminUrl, fnName, payloadValue) {
  const tokenB64 = await issueAppAuthTokenB64(adminUrl, INSTALLED_APP_ID);
  const tokenBytes = Buffer.from(tokenB64, 'base64');
  const { dnaHash, agentPubKey } = await discoverCellId(appUrl, adminUrl, INSTALLED_APP_ID);
  const creds = await issueSigningCredentials(adminUrl, dnaHash, agentPubKey);
  try {
    const result = await callZome({
      appUrl, tokenBytes, cellId: [dnaHash, agentPubKey],
      signingAgentKey: Buffer.from(creds.signingAgentKeyB64, 'base64'),
      privateKey: Buffer.from(creds.privateKeyB64, 'base64'),
      capSecret: Buffer.from(creds.capSecretB64, 'base64'),
      zomeName: 'mail_messages', fnName, payloadValue,
    });
    console.log(`[${who}] ${fnName} =>`, JSON.stringify(result));
  } catch (e) {
    console.log(`[${who}] ${fnName} FAILED:`, e.message);
  }
}

async function main() {
  await callFn('alice', ALICE_CONDUCTOR_URL, ALICE_ADMIN_URL, 'get_sent', { limit: 50 });
  await callFn('bob', BOB_CONDUCTOR_URL, BOB_ADMIN_URL, 'get_inbox_v2', undefined);
  await callFn('bob', BOB_CONDUCTOR_URL, BOB_ADMIN_URL, 'get_inbox', { limit: 50 });
}
main().catch((e) => { console.error('[diag] fatal', e); process.exit(1); });
