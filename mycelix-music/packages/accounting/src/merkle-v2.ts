import { createHash } from 'node:crypto';
import { serializeAccountingWire } from './accounting-wire.js';
import type { Digest } from './merkle.js';

export const ACCOUNTING_MERKLE_V2_PROTOCOL = 'mycelix-accounting-merkle-v2' as const;
const LEAF_DOMAIN = Buffer.from('mycelix-accounting-merkle-leaf-v2\0', 'utf8');
const NODE_DOMAIN = Buffer.from('mycelix-accounting-merkle-node-v2\0', 'utf8');
const EMPTY_DOMAIN = Buffer.from('mycelix-accounting-merkle-empty-v2\0', 'utf8');
const SHA256_HEX = /^[0-9a-f]{64}$/;

export interface MerkleCommitmentV2 {
  readonly protocolVersion: 2;
  readonly root: Digest;
  readonly count: number;
}

export type MerkleProofStepV2 = Readonly<{
  side: 'left' | 'right';
  hash: Digest;
}>;

export interface MerkleInclusionProofV2 {
  readonly protocolVersion: 2;
  readonly index: number;
  readonly count: number;
  readonly steps: readonly MerkleProofStepV2[];
}

function digestBytes(label: string, value: Digest): Buffer {
  if (!SHA256_HEX.test(value)) throw new Error(`${label} must be a lowercase SHA-256 digest`);
  return Buffer.from(value, 'hex');
}

function sha256(parts: readonly Buffer[]): Digest {
  const hash = createHash('sha256');
  for (const part of parts) hash.update(part);
  return hash.digest('hex');
}

export function hashAccountingLeafV2(value: unknown): Digest {
  return sha256([
    LEAF_DOMAIN,
    Buffer.from(serializeAccountingWire(value), 'utf8'),
  ]);
}

function hashNodeV2(left: Digest, right: Digest): Digest {
  return sha256([
    NODE_DOMAIN,
    digestBytes('Merkle v2 left child', left),
    digestBytes('Merkle v2 right child', right),
  ]);
}

function emptyRootV2(): Digest {
  return sha256([EMPTY_DOMAIN]);
}

function nextLevelV2(level: readonly Digest[]): Digest[] {
  const next: Digest[] = [];
  for (let index = 0; index < level.length; index += 2) {
    const left = level[index]!;
    const right = level[index + 1] ?? left;
    next.push(hashNodeV2(left, right));
  }
  return next;
}

function expectedProofDepth(count: number): number {
  let width = count;
  let depth = 0;
  while (width > 1) {
    width = Math.ceil(width / 2);
    depth += 1;
  }
  return depth;
}

export function buildMerkleCommitmentV2(values: readonly unknown[]): Readonly<MerkleCommitmentV2> {
  if (values.length === 0) {
    return Object.freeze({ protocolVersion: 2 as const, root: emptyRootV2(), count: 0 });
  }
  let level = values.map(hashAccountingLeafV2);
  while (level.length > 1) level = nextLevelV2(level);
  return Object.freeze({ protocolVersion: 2 as const, root: level[0]!, count: values.length });
}

export function buildMerkleInclusionProofV2(
  values: readonly unknown[],
  index: number,
): Readonly<MerkleInclusionProofV2> {
  if (!Number.isSafeInteger(index) || index < 0 || index >= values.length) {
    throw new Error('Merkle v2 inclusion index out of range');
  }

  let level = values.map(hashAccountingLeafV2);
  let cursor = index;
  const steps: MerkleProofStepV2[] = [];
  while (level.length > 1) {
    const isRight = cursor % 2 === 1;
    const siblingIndex = isRight ? cursor - 1 : Math.min(cursor + 1, level.length - 1);
    steps.push(Object.freeze({
      side: isRight ? 'left' : 'right',
      hash: level[siblingIndex]!,
    }));
    cursor = Math.floor(cursor / 2);
    level = nextLevelV2(level);
  }

  return Object.freeze({
    protocolVersion: 2 as const,
    index,
    count: values.length,
    steps: Object.freeze(steps),
  });
}

export function verifyMerkleInclusionProofV2(
  value: unknown,
  proof: MerkleInclusionProofV2,
  expectedRoot: Digest,
): boolean {
  if (proof.protocolVersion !== 2 || !SHA256_HEX.test(expectedRoot)) return false;
  if (!Number.isSafeInteger(proof.count) || proof.count <= 0) return false;
  if (!Number.isSafeInteger(proof.index) || proof.index < 0 || proof.index >= proof.count) return false;
  if (proof.steps.length !== expectedProofDepth(proof.count)) return false;

  let current = hashAccountingLeafV2(value);
  let cursor = proof.index;
  let width = proof.count;

  for (const step of proof.steps) {
    if ((step.side !== 'left' && step.side !== 'right') || !SHA256_HEX.test(step.hash)) return false;
    const expectedSide = cursor % 2 === 1 ? 'left' : 'right';
    if (step.side !== expectedSide) return false;

    if (cursor % 2 === 0 && cursor + 1 >= width && step.hash !== current) return false;

    current = step.side === 'left'
      ? hashNodeV2(step.hash, current)
      : hashNodeV2(current, step.hash);
    cursor = Math.floor(cursor / 2);
    width = Math.ceil(width / 2);
  }
  return current === expectedRoot;
}
