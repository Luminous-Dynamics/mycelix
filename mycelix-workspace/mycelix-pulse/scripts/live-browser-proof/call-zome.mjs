#!/usr/bin/env node
// Makes a real signed zome call directly over the app websocket, without a
// browser — for diagnosing whether a given zome function's result is
// correct at the conductor level, independent of any UI code.
import { WebSocket } from 'ws';
import { encode, decode } from '@msgpack/msgpack';
import { sha512 } from '@noble/hashes/sha2.js';

async function connectAppWs(appUrl, tokenBytes) {
  const ws = new WebSocket(appUrl, { headers: { Origin: 'http://localhost' } });
  await new Promise((resolve, reject) => {
    ws.once('open', resolve);
    ws.once('error', reject);
  });
  ws.send(encode({ type: 'authenticate', data: encode({ token: Array.from(tokenBytes) }) }));
  await new Promise((r) => setTimeout(r, 300));
  return ws;
}

function sendRequest(ws, id, payload) {
  const inner = encode(payload);
  const envelope = encode({ id, type: 'request', data: inner });
  ws.send(envelope);
}

async function waitForResponse(ws) {
  return new Promise((resolve, reject) => {
    ws.once('message', (data) => {
      try {
        resolve(decode(data));
      } catch (e) {
        reject(e);
      }
    });
    ws.once('error', reject);
  });
}

/**
 * @param appUrl app websocket URL
 * @param tokenBytes app-auth token bytes
 * @param cellId [dnaHashBytes, agentPubKeyBytes]
 * @param signingAgentKey Uint8Array(39) — the granted key's wire-format AgentPubKey
 * @param privateKey Uint8Array(32) — the granted key's Ed25519 private key
 * @param capSecret Uint8Array(64)
 * @param zomeName, fnName, payloadValue — the zome call itself (payload is JS-side, will be msgpack-encoded)
 */
export async function callZome({
  appUrl,
  tokenBytes,
  cellId,
  signingAgentKey,
  privateKey,
  capSecret,
  zomeName,
  fnName,
  payloadValue,
}) {
  const ws = await connectAppWs(appUrl, tokenBytes);

  const payload = encode(payloadValue);
  const nonce = new Uint8Array(32);
  crypto.getRandomValues(nonce);
  const expiresAt = (Date.now() + 60_000) * 1000; // microseconds

  const zomeCallParams = {
    provenance: signingAgentKey,
    cell_id: cellId,
    zome_name: zomeName,
    fn_name: fnName,
    cap_secret: capSecret,
    payload,
    nonce,
    expires_at: expiresAt,
  };
  const bytes = encode(zomeCallParams);
  const hash = sha512(bytes);

  const ed = await import('@noble/ed25519');
  ed.hashes.sha512 = sha512;
  const signature = ed.sign(hash, privateKey);

  const callZomeRequest = { type: 'call_zome', value: { bytes, signature } };
  const requestData = encode(callZomeRequest);
  const envelope = encode({ id: 1, type: 'request', data: requestData });

  const responsePromise = waitForResponse(ws);
  ws.send(envelope);
  const outer = await responsePromise;
  ws.close();

  if (outer.error) {
    throw new Error(`WireResponse error: ${outer.error}`);
  }
  const inner = decode(outer.data);
  if (inner.type === 'error') {
    throw new Error(`AppResponse error: ${JSON.stringify(inner.value)}`);
  }
  if (inner.type !== 'zome_called') {
    throw new Error(`Unexpected AppResponse type: ${JSON.stringify(inner)}`);
  }
  return decode(inner.value);
}
