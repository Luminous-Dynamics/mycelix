#!/usr/bin/env node
// Ad-hoc diagnostic: fetch a real cell_id from app_info, then attempt to
// grant a fresh browser signing key for it via issue-signing-credentials.mjs.
import { WebSocket } from 'ws';
import { encode, decode } from '@msgpack/msgpack';
import { issueAppAuthTokenB64 } from './issue-token.mjs';
import { issueSigningCredentials } from './issue-signing-credentials.mjs';

const APP_URL = process.env.ALICE_CONDUCTOR_URL || 'ws://127.0.0.1:4445';
const ADMIN_URL = process.env.ALICE_ADMIN_URL || 'ws://127.0.0.1:4444';
const APP_ID = process.env.INSTALLED_APP_ID || 'mycelix_mail';

async function main() {
  const tokenB64 = await issueAppAuthTokenB64(ADMIN_URL, APP_ID);
  const token = Buffer.from(tokenB64, 'base64');

  const ws = new WebSocket(APP_URL, { headers: { Origin: 'http://localhost' } });
  await new Promise((resolve, reject) => {
    ws.once('open', resolve);
    ws.once('error', reject);
  });
  ws.send(encode({ type: 'authenticate', data: encode({ token: Array.from(token) }) }));
  await new Promise((r) => setTimeout(r, 300));

  const requestData = encode({ type: 'app_info' });
  const envelope = encode({ id: 1, type: 'request', data: requestData });
  const responsePromise = new Promise((resolve, reject) => {
    ws.once('message', (data) => {
      try {
        resolve(decode(data));
      } catch (e) {
        reject(e);
      }
    });
    ws.once('error', reject);
  });
  ws.send(envelope);
  const outer = await responsePromise;
  const inner = decode(outer.data);
  const cellInfo = inner.value.cell_info;
  const mainCells = cellInfo.main;
  const provisioned = mainCells.find((c) => c.type === 'provisioned');
  const [dnaHash, agentPubKey] = provisioned.value.cell_id;
  console.log('dna_hash bytes:', dnaHash.length, 'agent_pub_key bytes:', agentPubKey.length);

  const creds = await issueSigningCredentials(ADMIN_URL, dnaHash, agentPubKey);
  console.log('GRANTED:', creds);

  ws.close();
}

main().catch((e) => {
  console.error('fatal:', e);
  process.exit(1);
});
