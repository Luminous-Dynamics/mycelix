#!/usr/bin/env node
// Ad-hoc diagnostic: query Bob's get_inbox and Alice's get_outbox directly
// via a native zome call (no browser), to see whether the message from the
// last live-proof run actually reached the DHT.
import { issueAppAuthTokenB64 } from './issue-token.mjs';
import { issueSigningCredentials } from './issue-signing-credentials.mjs';
import { discoverCellId } from './discover-cell-id.mjs';
import { callZome } from './call-zome.mjs';

const ALICE_CONDUCTOR_URL = process.env.ALICE_CONDUCTOR_URL || 'ws://127.0.0.1:4445';
const BOB_CONDUCTOR_URL = process.env.BOB_CONDUCTOR_URL || 'ws://127.0.0.1:4447';
const ALICE_ADMIN_URL = process.env.ALICE_ADMIN_URL || 'ws://127.0.0.1:4444';
const BOB_ADMIN_URL = process.env.BOB_ADMIN_URL || 'ws://127.0.0.1:4446';
const INSTALLED_APP_ID = process.env.INSTALLED_APP_ID || 'mycelix_mail';

async function queryInbox(who, appUrl, adminUrl, fnName, payloadValue) {
  const tokenB64 = await issueAppAuthTokenB64(adminUrl, INSTALLED_APP_ID);
  const tokenBytes = Buffer.from(tokenB64, 'base64');
  const { dnaHash, agentPubKey } = await discoverCellId(appUrl, adminUrl, INSTALLED_APP_ID);
  const creds = await issueSigningCredentials(adminUrl, dnaHash, agentPubKey);

  const result = await callZome({
    appUrl,
    tokenBytes,
    cellId: [dnaHash, agentPubKey],
    signingAgentKey: Buffer.from(creds.signingAgentKeyB64, 'base64'),
    privateKey: Buffer.from(creds.privateKeyB64, 'base64'),
    capSecret: Buffer.from(creds.capSecretB64, 'base64'),
    zomeName: 'mail_messages',
    fnName,
    payloadValue,
  });
  console.log(`[${who}] ${fnName} result:`, JSON.stringify(result, null, 2));
  console.log(`[${who}] agent_pub_key (u-form):`, 'u' + Buffer.from(agentPubKey).toString('base64url'));
}

async function main() {
  // get_inbox takes EmailQuery{folder,is_read,is_starred,is_archived,since,
  // limit,offset: all Option<T>} — bare Option<T> fields default to None
  // when their key is missing (a serde-derive special case for Option<T>
  // specifically), so {"limit": 50} alone is valid, matching what
  // mail_context.rs actually sends.
  await queryInbox('bob', BOB_CONDUCTOR_URL, BOB_ADMIN_URL, 'get_inbox', { limit: 50 });
}

main().catch((e) => {
  console.error('[diag] fatal', e);
  process.exit(1);
});
