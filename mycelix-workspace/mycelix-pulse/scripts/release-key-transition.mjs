#!/usr/bin/env node
// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

import {
  createHash, createPrivateKey, createPublicKey, generateKeyPairSync, sign, verify,
} from 'node:crypto';
import { mkdtempSync, readFileSync, writeFileSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { join, resolve } from 'node:path';

const HEX_256 = /^[0-9a-f]{64}$/u;
const MODES = new Set(['genesis', 'retained', 'rotated']);

function argsOf(values) {
  const result = {};
  for (let index = 2; index < values.length; index += 1) {
    const value = values[index];
    if (value === '--self-test' || value === '--verify') result[value.slice(2)] = true;
    else if (value.startsWith('--')) result[value.slice(2)] = values[++index];
    else throw new Error(`unexpected argument: ${value}`);
  }
  return result;
}

function canonical(value) {
  if (Array.isArray(value)) return `[${value.map(canonical).join(',')}]`;
  if (value && typeof value === 'object') {
    return `{${Object.keys(value).sort().map((key) => `${JSON.stringify(key)}:${canonical(value[key])}`).join(',')}}`;
  }
  return JSON.stringify(value);
}

function canonicalBase64(value, field) {
  if (typeof value !== 'string') throw new Error(`${field} is not base64`);
  const bytes = Buffer.from(value, 'base64');
  if (bytes.toString('base64') !== value) throw new Error(`${field} is not canonical base64`);
  return bytes;
}

function keyInfo(privateKey) {
  const publicKey = createPublicKey(privateKey);
  const spki = publicKey.export({ type: 'spki', format: 'der' });
  return {
    privateKey,
    publicKey,
    spki,
    keyIdSha256: createHash('sha256').update(spki).digest('hex'),
  };
}

function receiptInfo(bytes) {
  if (!bytes) return null;
  const receipt = JSON.parse(bytes.toString('utf8'));
  const keyId = receipt?.release?.signingKeyIdSha256;
  if (receipt?.format !== 'mycelix-pulse-release-acceptance/v1' || !HEX_256.test(keyId || '')) {
    throw new Error('previous acceptance receipt is invalid');
  }
  return {
    receipt,
    bytes,
    sha256: createHash('sha256').update(bytes).digest('hex'),
    keyIdSha256: keyId,
  };
}

export function createTransition({ mode, version, sourceDateEpoch, currentPrivateKey, previousPrivateKey = null, previousAcceptance = null }) {
  if (!MODES.has(mode)) throw new Error('invalid release-key transition mode');
  if (typeof version !== 'string' || !version || !Number.isSafeInteger(sourceDateEpoch) || sourceDateEpoch < 0) {
    throw new Error('release-key transition identity is invalid');
  }
  const next = keyInfo(currentPrivateKey);
  const previous = receiptInfo(previousAcceptance);
  if (mode === 'genesis' && previous) throw new Error('genesis cannot include previous acceptance');
  if (mode !== 'genesis' && !previous) throw new Error('successor transition requires previous acceptance');
  if (mode === 'retained' && previous.keyIdSha256 !== next.keyIdSha256) {
    throw new Error('retained transition changed the signing key');
  }
  if (mode === 'rotated' && previous.keyIdSha256 === next.keyIdSha256) {
    throw new Error('rotated transition retained the signing key');
  }
  const old = previousPrivateKey ? keyInfo(previousPrivateKey) : null;
  if (mode === 'rotated' && (!old || old.keyIdSha256 !== previous.keyIdSha256)) {
    throw new Error('rotation requires the private key matching the previous acceptance');
  }
  const payload = {
    format: 'mycelix-pulse-release-key-transition-payload/v1',
    mode,
    currentVersion: version,
    currentSourceDateEpoch: sourceDateEpoch,
    previousAcceptanceSha256: previous?.sha256 ?? null,
    previousKeyIdSha256: previous?.keyIdSha256 ?? null,
    nextKeyIdSha256: next.keyIdSha256,
  };
  const payloadBytes = Buffer.from(canonical(payload));
  return {
    format: 'mycelix-pulse-release-key-transition/v1',
    algorithm: 'Ed25519',
    payload,
    previousAcceptanceB64: previous?.bytes.toString('base64') ?? null,
    previousPublicKeySpkiB64: old?.spki.toString('base64') ?? null,
    previousSignatureB64: old ? sign(null, payloadBytes, old.privateKey).toString('base64') : null,
    nextPublicKeySpkiB64: next.spki.toString('base64'),
    nextSignatureB64: sign(null, payloadBytes, next.privateKey).toString('base64'),
  };
}

export function verifyTransition(document, { expectedNextKeyId = null, release = null } = {}) {
  const exact = [
    'format', 'algorithm', 'payload', 'previousAcceptanceB64', 'previousPublicKeySpkiB64',
    'previousSignatureB64', 'nextPublicKeySpkiB64', 'nextSignatureB64',
  ].sort();
  if (!document || typeof document !== 'object' || JSON.stringify(Object.keys(document).sort()) !== JSON.stringify(exact)) {
    throw new Error('release-key transition fields differ');
  }
  if (document.format !== 'mycelix-pulse-release-key-transition/v1' || document.algorithm !== 'Ed25519') {
    throw new Error('unsupported release-key transition');
  }
  const payload = document.payload;
  const payloadFields = [
    'format', 'mode', 'currentVersion', 'currentSourceDateEpoch', 'previousAcceptanceSha256',
    'previousKeyIdSha256', 'nextKeyIdSha256',
  ].sort();
  if (!payload || JSON.stringify(Object.keys(payload).sort()) !== JSON.stringify(payloadFields) ||
      payload.format !== 'mycelix-pulse-release-key-transition-payload/v1' || !MODES.has(payload.mode)) {
    throw new Error('release-key transition payload differs');
  }
  if (!HEX_256.test(payload.nextKeyIdSha256 || '') ||
      (expectedNextKeyId && payload.nextKeyIdSha256 !== expectedNextKeyId)) {
    throw new Error('next release signing key is not trusted');
  }
  const nextSpki = canonicalBase64(document.nextPublicKeySpkiB64, 'nextPublicKeySpkiB64');
  const nextKeyId = createHash('sha256').update(nextSpki).digest('hex');
  if (nextKeyId !== payload.nextKeyIdSha256) throw new Error('next signing-key identifier mismatch');
  const nextSignature = canonicalBase64(document.nextSignatureB64, 'nextSignatureB64');
  if (nextSignature.length !== 64 || !verify(null, Buffer.from(canonical(payload)), createPublicKey({ key: nextSpki, type: 'spki', format: 'der' }), nextSignature)) {
    throw new Error('next signing-key authorization failed');
  }
  const previousBytes = document.previousAcceptanceB64 === null ? null : canonicalBase64(document.previousAcceptanceB64, 'previousAcceptanceB64');
  const previous = receiptInfo(previousBytes);
  if (payload.mode === 'genesis') {
    if (previous || payload.previousAcceptanceSha256 !== null || payload.previousKeyIdSha256 !== null ||
        document.previousPublicKeySpkiB64 !== null || document.previousSignatureB64 !== null) {
      throw new Error('genesis transition contains predecessor material');
    }
  } else {
    if (!previous || previous.sha256 !== payload.previousAcceptanceSha256 || previous.keyIdSha256 !== payload.previousKeyIdSha256) {
      throw new Error('previous acceptance binding mismatch');
    }
    if (payload.mode === 'retained') {
      if (payload.previousKeyIdSha256 !== payload.nextKeyIdSha256 || document.previousPublicKeySpkiB64 !== null || document.previousSignatureB64 !== null) {
        throw new Error('retained-key transition is malformed');
      }
    } else {
      if (payload.previousKeyIdSha256 === payload.nextKeyIdSha256) throw new Error('rotation did not change the key');
      const oldSpki = canonicalBase64(document.previousPublicKeySpkiB64, 'previousPublicKeySpkiB64');
      if (createHash('sha256').update(oldSpki).digest('hex') !== payload.previousKeyIdSha256) {
        throw new Error('previous signing-key identifier mismatch');
      }
      const oldSignature = canonicalBase64(document.previousSignatureB64, 'previousSignatureB64');
      if (oldSignature.length !== 64 || !verify(null, Buffer.from(canonical(payload)), createPublicKey({ key: oldSpki, type: 'spki', format: 'der' }), oldSignature)) {
        throw new Error('previous signing-key authorization failed');
      }
    }
  }
  if (release) {
    if (payload.currentVersion !== release.version || payload.currentSourceDateEpoch !== release.source_date_epoch ||
        release.key_transition?.mode !== payload.mode || release.key_transition?.evidence !== 'evidence/release-key-transition.json') {
      throw new Error('release-key transition differs from RELEASE.json');
    }
  }
  return payload;
}

function selfTest() {
  const oldKey = generateKeyPairSync('ed25519').privateKey;
  const nextKey = generateKeyPairSync('ed25519').privateKey;
  const oldId = keyInfo(oldKey).keyIdSha256;
  const receipt = Buffer.from(JSON.stringify({
    format: 'mycelix-pulse-release-acceptance/v1',
    release: { signingKeyIdSha256: oldId },
  }) + '\n');
  const genesis = createTransition({ mode: 'genesis', version: '1', sourceDateEpoch: 1, currentPrivateKey: oldKey });
  verifyTransition(genesis, { expectedNextKeyId: oldId });
  const retained = createTransition({ mode: 'retained', version: '2', sourceDateEpoch: 2, currentPrivateKey: oldKey, previousAcceptance: receipt });
  verifyTransition(retained, { expectedNextKeyId: oldId });
  const rotated = createTransition({ mode: 'rotated', version: '3', sourceDateEpoch: 3, currentPrivateKey: nextKey, previousPrivateKey: oldKey, previousAcceptance: receipt });
  verifyTransition(rotated, { expectedNextKeyId: keyInfo(nextKey).keyIdSha256 });
  rotated.payload.currentVersion = 'tampered';
  try { verifyTransition(rotated); } catch { console.log('Release key-transition self-test passed.'); return; }
  throw new Error('release key-transition self-test accepted tampering');
}

async function main() {
  const args = argsOf(process.argv);
  if (args['self-test']) { selfTest(); return; }
  if (args.verify) {
    const document = JSON.parse(readFileSync(resolve(args.document), 'utf8'));
    const release = args.release ? JSON.parse(readFileSync(resolve(args.release), 'utf8')) : null;
    verifyTransition(document, { expectedNextKeyId: args['expected-next-key-id'], release });
    console.log(`Release key transition verified (${document.payload.mode}).`);
    return;
  }
  for (const required of ['mode', 'version', 'source-date-epoch', 'current-private-key', 'output']) {
    if (!args[required]) throw new Error(`--${required} is required`);
  }
  const document = createTransition({
    mode: args.mode,
    version: args.version,
    sourceDateEpoch: Number(args['source-date-epoch']),
    currentPrivateKey: createPrivateKey(readFileSync(resolve(args['current-private-key']))),
    previousPrivateKey: args['previous-private-key'] ? createPrivateKey(readFileSync(resolve(args['previous-private-key']))) : null,
    previousAcceptance: args['previous-receipt'] ? readFileSync(resolve(args['previous-receipt'])) : null,
  });
  if (args['expected-next-key-id'] && document.payload.nextKeyIdSha256 !== args['expected-next-key-id']) {
    throw new Error('current private key does not match the trusted next-key identifier');
  }
  writeFileSync(resolve(args.output), `${JSON.stringify(document, null, 2)}\n`, { mode: 0o644 });
  console.log(`Created release key transition (${document.payload.mode}).`);
}

main().catch((error) => {
  console.error(`Release key-transition operation FAILED: ${error.message}`);
  process.exitCode = 1;
});
