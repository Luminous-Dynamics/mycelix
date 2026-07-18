// Minimal reimplementation of holo_hash's wire format for a raw 32-byte
// Ed25519 public key -> a 39-byte AgentPubKey (3-byte type prefix + 32-byte
// key + 4-byte DHT "location" checksum), matching
// holo_hash::HoloHash::from_raw_32 / holo_dht_location_bytes exactly (see
// holo_hash-0.6.1/src/hash.rs + src/encode.rs). Needed because we generate
// the browser's zome-call signing keypair here (Node), not inside a real
// Holochain conductor, so nothing else computes this framing for us.
import { blake2b } from '@noble/hashes/blake2.js';

const AGENT_PREFIX = new Uint8Array([0x84, 0x20, 0x24]); // "uhCAk" — Agent hash type

function dhtLocationBytes(data32) {
  const hash = blake2b(data32, { dkLen: 16 });
  const out = new Uint8Array([hash[0], hash[1], hash[2], hash[3]]);
  for (let i = 4; i < 16; i += 4) {
    out[0] ^= hash[i];
    out[1] ^= hash[i + 1];
    out[2] ^= hash[i + 2];
    out[3] ^= hash[i + 3];
  }
  return out;
}

/** Raw 32-byte Ed25519 public key -> 39-byte wire-format AgentPubKey. */
export function agentPubKeyBytes(rawPub32) {
  if (rawPub32.length !== 32) {
    throw new Error(`expected a 32-byte public key, got ${rawPub32.length}`);
  }
  const loc = dhtLocationBytes(rawPub32);
  return new Uint8Array([...AGENT_PREFIX, ...rawPub32, ...loc]);
}
