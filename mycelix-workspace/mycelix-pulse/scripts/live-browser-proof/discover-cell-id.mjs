#!/usr/bin/env node
// Discovers a cell's (dna_hash, agent_pub_key) via a real app_info call —
// the same lookup the browser itself does, but performed independently by
// the trusted broker so it doesn't need the browser to report its own
// cell_id back over an extra round trip.
import { WebSocket } from 'ws';
import { encode, decode } from '@msgpack/msgpack';
import { issueAppAuthTokenB64 } from './issue-token.mjs';

/**
 * @param appUrl e.g. "ws://127.0.0.1:4445"
 * @param adminUrl e.g. "ws://127.0.0.1:4444"
 * @param installedAppId e.g. "mycelix_mail"
 * @param role role name to resolve, defaults to the alpha hApp's single role
 * @returns {dnaHash: Uint8Array(39), agentPubKey: Uint8Array(39)}
 */
export async function discoverCellId(appUrl, adminUrl, installedAppId, role = 'main') {
  const tokenB64 = await issueAppAuthTokenB64(adminUrl, installedAppId);
  const token = Buffer.from(tokenB64, 'base64');

  const ws = new WebSocket(appUrl, { headers: { Origin: 'http://localhost' } });
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
  ws.close();

  const inner = decode(outer.data);
  if (inner.type !== 'app_info' || !inner.value) {
    throw new Error(`app_info failed: ${JSON.stringify(inner)}`);
  }
  const cells = inner.value.cell_info[role];
  if (!cells) {
    throw new Error(`role '${role}' not found in cell_info: ${Object.keys(inner.value.cell_info)}`);
  }
  const provisioned = cells.find((c) => c.type === 'provisioned');
  if (!provisioned) {
    throw new Error(`no provisioned cell for role '${role}'`);
  }
  const [dnaHash, agentPubKey] = provisioned.value.cell_id;
  return { dnaHash, agentPubKey };
}
