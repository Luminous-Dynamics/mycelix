#!/usr/bin/env node
import fs from "node:fs";
import crypto from "node:crypto";

class Refusal extends Error {}

const MAGIC_AAD = Buffer.from("mycelix/protected-envelope/payload-aad/v1\0", "ascii");
const SCHEMA_V1 = 1;
const PRODUCTION_AES_GCM_PROFILE_ID = 1;
const TEST_PROFILE_MIN = 0xF000;
const TEST_PROFILE_MAX = 0xFFFF;

function u16be(n) {
  const b = Buffer.alloc(2);
  b.writeUInt16BE(n);
  return b;
}
function u64be(n) {
  const b = Buffer.alloc(8);
  b.writeBigUInt64BE(BigInt(n));
  return b;
}
function hex(s) {
  if (typeof s !== "string" || s.length % 2 !== 0) throw new Refusal("bad-hex");
  return Buffer.from(s, "hex");
}
function buildPayloadAad(v) {
  const subject = hex(v.payload_subject_commitment);
  if (subject.length !== 32 || subject.equals(Buffer.alloc(32))) throw new Refusal("payload-subject");
  if (v.aead_profile_id !== PRODUCTION_AES_GCM_PROFILE_ID) throw new Refusal("aead-profile");
  if (v.payload_version < 1 || v.key_epoch < 1) throw new Refusal("counter");
  return Buffer.concat([
    MAGIC_AAD,
    u16be(SCHEMA_V1),
    u16be(v.aead_profile_id),
    subject,
    u64be(v.payload_version),
    u64be(v.key_epoch),
  ]);
}
function validateProfile(key, nonce, tag, profileId) {
  if (profileId >= TEST_PROFILE_MIN && profileId <= TEST_PROFILE_MAX) throw new Refusal("test-profile-id");
  if (profileId !== PRODUCTION_AES_GCM_PROFILE_ID) throw new Refusal("aead-profile");
  if (key.length !== 32) throw new Refusal("key-length");
  if (nonce.length !== 12) throw new Refusal("nonce-length");
  if (tag.length !== 16) throw new Refusal("tag-length");
}
function encrypt(key, nonce, aad, plaintext) {
  const cipher = crypto.createCipheriv("aes-256-gcm", key, nonce, {authTagLength:16});
  cipher.setAAD(aad);
  const ciphertext = Buffer.concat([cipher.update(plaintext), cipher.final()]);
  const tag = cipher.getAuthTag();
  return {ciphertext, tag};
}
function decrypt(key, nonce, aad, ciphertext, tag) {
  const decipher = crypto.createDecipheriv("aes-256-gcm", key, nonce, {authTagLength:16});
  decipher.setAAD(aad);
  decipher.setAuthTag(tag);
  return Buffer.concat([decipher.update(ciphertext), decipher.final()]);
}
function expectAuthFailure(key, nonce, aad, ciphertext, tag) {
  try {
    decrypt(key, nonce, aad, ciphertext, tag);
  } catch {
    return;
  }
  throw new Error("expected-auth-failure");
}
function expectRefusal(fn) {
  try {
    fn();
  } catch (e) {
    if (e instanceof Refusal) return;
    throw e;
  }
  throw new Error("expected-profile-refusal");
}

const path = process.argv[2];
if (!path) throw new Error("usage: node sup_civ_aes_gcm_reference_v1.mjs <vectors.json>");
const manifest = JSON.parse(fs.readFileSync(path, "utf8"));

if (manifest.schema !== "sup-civ-000d1c1-aes-gcm-vectors-v1") throw new Error("schema");
if (manifest.parent_exact_head !== "1152251e1b8d09bf69cc2497164a0fc1d58da320") throw new Error("parent");
if (manifest.profile.profile_id !== 1) throw new Error("profile-id");
if (manifest.profile.algorithm !== "AES-256-GCM") throw new Error("algorithm");
if (manifest.profile.key_bytes !== 32 || manifest.profile.nonce_bytes !== 12 || manifest.profile.tag_bytes !== 16) throw new Error("sizes");
if (manifest.profile.aad_domain !== "PayloadAadV1") throw new Error("aad-domain");
if (manifest.profile.fresh_dek_per_ciphertext_required !== true) throw new Error("fresh-dek-law");

const byId = new Map();
for (const v of manifest.positive_vectors) {
  if (byId.has(v.id)) throw new Error("duplicate-vector-id");
  byId.set(v.id, v);
  const key = hex(v.key_hex);
  const nonce = hex(v.nonce_hex);
  const aad = hex(v.aad_hex);
  const plaintext = hex(v.plaintext_hex);
  const expectedCiphertext = hex(v.ciphertext_hex);
  const expectedTag = hex(v.tag_hex);

  if (v.source === "mycelix-custom") {
    const rebuilt = buildPayloadAad(v);
    if (!rebuilt.equals(aad)) throw new Error(`aad-mismatch:${v.id}`);
    validateProfile(key, nonce, expectedTag, v.aead_profile_id);
  } else if (v.source === "NIST-SP-800-38D-known-answer") {
    if (key.length !== 32 || nonce.length !== 12 || expectedTag.length !== 16) throw new Error("nist-size");
  } else {
    throw new Error(`unknown-source:${v.source}`);
  }

  const got = encrypt(key, nonce, aad, plaintext);
  if (!got.ciphertext.equals(expectedCiphertext)) throw new Error(`ciphertext:${v.id}`);
  if (!got.tag.equals(expectedTag)) throw new Error(`tag:${v.id}`);
  const recovered = decrypt(key, nonce, aad, expectedCiphertext, expectedTag);
  if (!recovered.equals(plaintext)) throw new Error(`decrypt:${v.id}`);
}

const custom = manifest.positive_vectors.filter(v => v.source === "mycelix-custom");
const keySet = new Set(custom.map(v => v.key_hex));
const nonceSet = new Set(custom.map(v => v.nonce_hex));
if (keySet.size !== custom.length) throw new Error("fixture-dek-reuse");
if (nonceSet.size !== custom.length) throw new Error("fixture-nonce-reuse");

for (const n of manifest.negative_cases) {
  const v = byId.get(n.base);
  if (!v) throw new Error(`missing-base:${n.id}`);
  let key = hex(v.key_hex);
  let nonce = hex(v.nonce_hex);
  let aad = hex(v.aad_hex);
  let ciphertext = hex(v.ciphertext_hex);
  let tag = hex(v.tag_hex);
  let profileId = v.aead_profile_id;

  switch (n.mutation) {
    case "wrong_key": key = Buffer.from(key); key[0] ^= 1; break;
    case "wrong_nonce": nonce = Buffer.from(nonce); nonce[0] ^= 1; break;
    case "aad_bit_flip": aad = Buffer.from(aad); aad[aad.length - 1] ^= 1; break;
    case "aad_empty": aad = Buffer.alloc(0); break;
    case "ciphertext_bit_flip": ciphertext = Buffer.from(ciphertext); ciphertext[0] ^= 1; break;
    case "ciphertext_truncated": ciphertext = ciphertext.subarray(0, ciphertext.length - 1); break;
    case "tag_bit_flip": tag = Buffer.from(tag); tag[0] ^= 1; break;
    case "tag_truncated": tag = tag.subarray(0, 15); break;
    case "key_length_31": key = key.subarray(0, 31); break;
    case "nonce_length_11": nonce = nonce.subarray(0, 11); break;
    case "test_profile_id": profileId = 0xF101; break;
    default: throw new Error(`unknown-mutation:${n.mutation}`);
  }

  if (n.expected === "profile-refusal") {
    expectRefusal(() => validateProfile(key, nonce, tag, profileId));
  } else if (n.expected === "auth-failure") {
    validateProfile(key, nonce, tag, profileId);
    expectAuthFailure(key, nonce, aad, ciphertext, tag);
  } else {
    throw new Error(`unknown-negative-expectation:${n.expected}`);
  }
}

console.log(JSON.stringify({
  schema: "sup-civ-000d1c1-aes-gcm-receipt-v1",
  positive_count: manifest.positive_vectors.length,
  negative_count: manifest.negative_cases.length,
  profile_id: manifest.profile.profile_id,
  algorithm: manifest.profile.algorithm,
  result: "PASS_REFERENCE_VECTOR_EXECUTION"
}));
