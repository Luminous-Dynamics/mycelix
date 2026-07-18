#!/usr/bin/env node

import { readFile, readdir } from 'node:fs/promises';
import { fileURLToPath } from 'node:url';
import path from 'node:path';

const scriptDir = path.dirname(fileURLToPath(import.meta.url));
const root = path.resolve(scriptDir, '..');
const contractPath = path.join(root, 'backend/zome-api.json');
const gapsPath = path.join(root, 'frontend/src/lib/holochain/known-contract-gaps.json');
const holochainDir = path.join(root, 'frontend/src/lib/holochain');

const contract = JSON.parse(await readFile(contractPath, 'utf8'));
const knownGaps = JSON.parse(await readFile(gapsPath, 'utf8'));
const errors = [];

if (contract.schema_version !== 1) {
  errors.push(`Unsupported backend contract schema: ${contract.schema_version}`);
}
if (knownGaps.schema_version !== 1) {
  errors.push(`Unsupported known-gap schema: ${knownGaps.schema_version}`);
}

const dnaText = await readFile(path.join(root, contract.dna_manifest), 'utf8');
const coordinatorText = dnaText.split(/^coordinator:\s*$/m)[1] ?? '';
const bundledZomes = new Set(
  [...coordinatorText.matchAll(/^\s{4}- name:\s*([A-Za-z0-9_-]+)\s*$/gm)].map((match) => match[1])
);
const contractZomes = new Set(Object.keys(contract.zomes));

for (const zome of contractZomes) {
  if (!bundledZomes.has(zome)) {
    errors.push(`Contract zome ${zome} is not bundled by backend/dna.yaml`);
  }
}
for (const zome of bundledZomes) {
  if (!contractZomes.has(zome)) {
    errors.push(`Bundled coordinator zome ${zome} is missing from backend/zome-api.json`);
  }
}

const exported = new Map();
for (const [zome, definition] of Object.entries(contract.zomes)) {
  const source = await readFile(path.join(root, definition.source), 'utf8');
  const functions = new Set(
    [...source.matchAll(/#\[hdk_extern\]\s*pub fn\s+([A-Za-z0-9_]+)/g)].map((match) => match[1])
  );
  exported.set(zome, functions);

  const declared = new Set(definition.functions);
  for (const fn of declared) {
    if (!functions.has(fn)) {
      errors.push(`${zome}.${fn} is declared in zome-api.json but not exported by ${definition.source}`);
    }
  }
  for (const fn of functions) {
    if (!declared.has(fn)) {
      errors.push(`${zome}.${fn} is exported by Rust but missing from zome-api.json`);
    }
  }
}

const gapKey = ({ file, zome, function: fn }) => `${file}:${zome}.${fn}`;
const gapMap = new Map();
for (const gap of knownGaps.gaps) {
  const key = gapKey(gap);
  if (gapMap.has(key)) {
    errors.push(`Duplicate known contract gap: ${key}`);
  }
  if (!gap.reason?.trim()) {
    errors.push(`Known contract gap lacks a reason: ${key}`);
  }
  if (gap.replacement) {
    const separator = gap.replacement.indexOf('.');
    const zome = gap.replacement.slice(0, separator);
    const fn = gap.replacement.slice(separator + 1);
    if (separator < 1 || !exported.get(zome)?.has(fn)) {
      errors.push(`Known contract gap has invalid replacement ${gap.replacement}: ${key}`);
    }
  }
  gapMap.set(key, gap);
}

const files = (await readdir(holochainDir))
  .filter((file) => file.endsWith('.ts'))
  .filter((file) => !['client.ts', 'identity.ts'].includes(file));

const calls = [];
const callPattern = /callZome(?:<[^>]*>)?\(\s*[^,]+,\s*['"]([^'"]+)['"]\s*,\s*['"]([^'"]+)['"]/gs;
for (const file of files) {
  const source = await readFile(path.join(holochainDir, file), 'utf8');
  for (const match of source.matchAll(callPattern)) {
    calls.push({ file, zome: match[1], function: match[2] });
  }
}

let validCount = 0;
let acknowledgedGapCount = 0;
const observedGaps = new Set();

for (const call of calls) {
  if (exported.get(call.zome)?.has(call.function)) {
    validCount += 1;
    continue;
  }

  const key = gapKey(call);
  if (!gapMap.has(key)) {
    errors.push(`Unacknowledged frontend contract mismatch: ${key}`);
    continue;
  }

  acknowledgedGapCount += 1;
  observedGaps.add(key);
}

for (const key of gapMap.keys()) {
  if (!observedGaps.has(key)) {
    errors.push(`Stale known contract gap (call no longer observed): ${key}`);
  }
}

if (errors.length > 0) {
  console.error('Zome contract check failed:\n');
  for (const error of errors) console.error(`- ${error}`);
  process.exit(1);
}

console.log(`Zome contract check passed.`);
console.log(`- ${contractZomes.size} bundled coordinator zomes`);
console.log(`- ${[...exported.values()].reduce((sum, functions) => sum + functions.size, 0)} exported functions`);
console.log(`- ${validCount} frontend calls target bundled exports`);
console.log(`- ${acknowledgedGapCount} frontend calls are explicitly tracked contract gaps`);
