#!/usr/bin/env node
// Grants a fresh, browser-held Ed25519 "zome-call signing key" for a cell,
// via the real admin API (AdminRequest::GrantZomeCallCapability), and
// returns the private key material to hand to the browser.
//
// This is the client-side half of Holochain 0.6's REQUIRED zome-call
// signing flow — the conductor's handle_external_zome_call unconditionally
// verifies a real Ed25519 signature over sha512(serialized ZomeCallParams)
// against `provenance` (see zome_call_signature_verification.rs upstream).
// A bare browser has no access to the cell's own lair-held private key, so
// it can never sign as the cell's real agent directly. Instead, following
// the pattern in holochain_client::AdminWebsocket::authorize_signing_credentials
// (Rust reference client), a NEW throwaway Ed25519 keypair is generated
// here, its public key is registered as an `Assigned` capability grant
// (secret + assignee list) on the cell's own source chain via the admin
// API, and the browser signs future zome calls with the corresponding
// private key — using this GRANTED key as `provenance`, not the cell's own
// agent key. The zome sees the CELL's real agent as the on-chain author
// regardless (capability delegation authorizes the call; it doesn't change
// authorship) — see conductor.rs's handle_external_zome_call /
// CapGrant::is_valid.
//
// In production this whole function belongs server-side (a small trusted
// broker, matching the existing /api/token pattern) — never expose the raw
// admin interface to a real browser page. Here it plays that broker role
// for the local Alice/Bob test conductors.

import { WebSocket } from 'ws';
import { encode, decode } from '@msgpack/msgpack';
import { agentPubKeyBytes } from './holo-hash-utils.mjs';

function send(ws, id, payload) {
  const inner = encode(payload);
  const envelope = encode({ id, type: 'request', data: inner });
  ws.send(envelope);
}

async function adminCall(adminUrl, payload) {
  const ws = new WebSocket(adminUrl, { headers: { Origin: 'http://localhost' } });
  await new Promise((resolve, reject) => {
    ws.once('open', resolve);
    ws.once('error', reject);
  });
  const response = await new Promise((resolve, reject) => {
    ws.once('message', (data) => {
      try {
        const envelope = decode(data);
        resolve(decode(envelope.data));
      } catch (e) {
        reject(e);
      }
    });
    ws.once('error', reject);
    send(ws, 1, payload);
  });
  ws.close();
  return response;
}

/**
 * @param adminUrl e.g. "ws://127.0.0.1:4444"
 * @param dnaHashBytes Uint8Array(39) — from app_info's cell_id[0]
 * @param cellAgentPubKeyBytes Uint8Array(39) — from app_info's cell_id[1]
 * @param functionsAllowed {zome, fn}[] or null for "all zomes, all functions"
 * @returns {signingAgentKeyB64, privateKeyB64, capSecretB64} all base64
 */
export async function issueSigningCredentials(
  adminUrl,
  dnaHashBytes,
  cellAgentPubKeyBytes,
  functionsAllowed = null,
) {
  const { getPublicKey, utils: edUtils, hashes } = await import('@noble/ed25519');
  const { sha512 } = await import('@noble/hashes/sha2.js');
  hashes.sha512 = sha512;

  const privateKey = edUtils.randomSecretKey(); // 32 bytes
  const rawPublicKey = getPublicKey(privateKey); // 32 bytes
  const signingAgentKey = agentPubKeyBytes(rawPublicKey); // 39 bytes, wire format

  const capSecret = new Uint8Array(64);
  crypto.getRandomValues(capSecret);

  const functions = functionsAllowed
    ? {
        type: 'listed',
        value: functionsAllowed.map((f) => [f.zome, f.fn]),
      }
    : { type: 'all' };

  const response = await adminCall(adminUrl, {
    type: 'grant_zome_call_capability',
    value: {
      cell_id: [dnaHashBytes, cellAgentPubKeyBytes],
      cap_grant: {
        tag: 'pulse-browser-signing-key',
        access: {
          type: 'assigned',
          value: {
            secret: capSecret,
            assignees: [signingAgentKey],
          },
        },
        functions,
      },
    },
  });

  if (response.type !== 'zome_call_capability_granted') {
    throw new Error(`GrantZomeCallCapability failed: ${JSON.stringify(response)}`);
  }

  return {
    signingAgentKeyB64: Buffer.from(signingAgentKey).toString('base64'),
    privateKeyB64: Buffer.from(privateKey).toString('base64'),
    capSecretB64: Buffer.from(capSecret).toString('base64'),
  };
}
