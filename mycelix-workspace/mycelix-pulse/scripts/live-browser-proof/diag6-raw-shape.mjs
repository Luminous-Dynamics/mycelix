#!/usr/bin/env node
// Ad-hoc diagnostic: connect directly to the app websocket (bypassing our
// own Rust client entirely) and print the RAW decoded shape of a real
// app_info response, to determine whether the conductor encodes the
// adjacently-tagged AppResponse envelope (and nested structs) as msgpack
// maps or arrays.
import { WebSocket } from 'ws';
import { encode, decode } from '@msgpack/msgpack';
import { issueAppAuthTokenB64 } from './issue-token.mjs';

const APP_URL = process.env.ALICE_CONDUCTOR_URL || 'ws://127.0.0.1:4445';
const ADMIN_URL = process.env.ALICE_ADMIN_URL || 'ws://127.0.0.1:4444';
const APP_ID = process.env.INSTALLED_APP_ID || 'mycelix_mail';

function describe(v, depth = 0) {
  const indent = '  '.repeat(depth);
  if (Array.isArray(v)) {
    console.log(`${indent}ARRAY[${v.length}]`);
    v.forEach((el) => describe(el, depth + 1));
  } else if (v instanceof Uint8Array) {
    console.log(`${indent}BYTES[${v.length}]`);
  } else if (v && typeof v === 'object') {
    console.log(`${indent}MAP{${Object.keys(v).join(',')}}`);
    for (const [k, val] of Object.entries(v)) {
      console.log(`${indent}  key="${k}":`);
      describe(val, depth + 2);
    }
  } else {
    console.log(`${indent}${typeof v}: ${String(v).slice(0, 60)}`);
  }
}

async function main() {
  const tokenB64 = await issueAppAuthTokenB64(ADMIN_URL, APP_ID);
  const token = Buffer.from(tokenB64, 'base64');

  const ws = new WebSocket(APP_URL, { headers: { Origin: 'http://localhost' } });
  await new Promise((resolve, reject) => {
    ws.once('open', resolve);
    ws.once('error', reject);
  });

  // Authenticate (fire-and-forget, no response expected)
  ws.send(encode({ type: 'authenticate', data: encode({ token: Array.from(token) }) }));
  await new Promise((r) => setTimeout(r, 300));

  // Request app_info: {"type": "app_info"} — matches our fixed unit variant.
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
  console.log('=== OUTER WireResponse envelope ===');
  describe(outer);

  console.log('\n=== INNER AppResponse (outer.data decoded) ===');
  const inner = decode(outer.data);
  describe(inner);

  ws.close();
}

main().catch((e) => {
  console.error('fatal:', e);
  process.exit(1);
});
