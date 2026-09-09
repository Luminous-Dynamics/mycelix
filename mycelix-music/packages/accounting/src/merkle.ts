import { createHash } from 'node:crypto';

const LEAF_DOMAIN = 'mycelix-accounting-merkle-leaf-v1\0';
const NODE_DOMAIN = 'mycelix-accounting-merkle-node-v1\0';
const EMPTY_DOMAIN = 'mycelix-accounting-merkle-empty-v1\0';

export type Digest = string;

export function canonicalAccountingValue(value: unknown): string {
  if (value === null) return 'n';
  if (typeof value === 'string') return `s${JSON.stringify(value)}`;
  if (typeof value === 'boolean') return value ? 't' : 'f';
  if (typeof value === 'bigint') return `i${value.toString(10)};`;
  if (typeof value === 'number') {
    if (!Number.isFinite(value)) throw new Error('canonical accounting values require finite numbers');
    return `d${Object.is(value, -0) ? '-0' : value.toString()};`;
  }
  if (Array.isArray(value)) {
    return `[${value.map(canonicalAccountingValue).join(',')}]`;
  }
  if (typeof value === 'object') {
    const record = value as Record<string, unknown>;
    const keys = Object.keys(record).sort();
    for (const key of keys) {
      if (record[key] === undefined) throw new Error('canonical accounting values forbid undefined');
    }
    return `{${keys.map(key => `${JSON.stringify(key)}:${canonicalAccountingValue(record[key])}`).join(',')}}`;
  }
  throw new Error(`unsupported canonical accounting value: ${typeof value}`);
}

function sha256(text: string): Digest {
  return createHash('sha256').update(text, 'utf8').digest('hex');
}

export function hashAccountingLeaf(value: unknown): Digest {
  return sha256(LEAF_DOMAIN + canonicalAccountingValue(value));
}

function hashNode(left: Digest, right: Digest): Digest {
  return sha256(NODE_DOMAIN + left + right);
}

export interface MerkleCommitment {
  readonly root: Digest;
  readonly count: number;
}

export type MerkleProofStep = Readonly<{
  side: 'left' | 'right';
  hash: Digest;
}>;

export interface MerkleInclusionProof {
  readonly index: number;
  readonly count: number;
  readonly steps: readonly MerkleProofStep[];
}

function nextLevel(level: readonly Digest[]): Digest[] {
  const next: Digest[] = [];
  for (let i = 0; i < level.length; i += 2) {
    const left = level[i]!;
    const right = level[i + 1] ?? left;
    next.push(hashNode(left, right));
  }
  return next;
}

export function buildMerkleCommitment(values: readonly unknown[]): MerkleCommitment {
  if (values.length === 0) return Object.freeze({ root: sha256(EMPTY_DOMAIN), count: 0 });
  let level = values.map(hashAccountingLeaf);
  while (level.length > 1) level = nextLevel(level);
  return Object.freeze({ root: level[0]!, count: values.length });
}

export function buildMerkleInclusionProof(
  values: readonly unknown[],
  index: number,
): MerkleInclusionProof {
  if (!Number.isInteger(index) || index < 0 || index >= values.length) {
    throw new Error('Merkle inclusion index out of range');
  }

  let level = values.map(hashAccountingLeaf);
  let cursor = index;
  const steps: MerkleProofStep[] = [];
  while (level.length > 1) {
    const isRight = cursor % 2 === 1;
    const siblingIndex = isRight ? cursor - 1 : Math.min(cursor + 1, level.length - 1);
    steps.push(Object.freeze({
      side: isRight ? 'left' : 'right',
      hash: level[siblingIndex]!,
    }));
    cursor = Math.floor(cursor / 2);
    level = nextLevel(level);
  }

  return Object.freeze({ index, count: values.length, steps: Object.freeze(steps) });
}

export function verifyMerkleInclusionProof(
  value: unknown,
  proof: MerkleInclusionProof,
  expectedRoot: Digest,
): boolean {
  if (proof.count <= 0 || proof.index < 0 || proof.index >= proof.count) return false;
  let current = hashAccountingLeaf(value);
  for (const step of proof.steps) {
    current = step.side === 'left'
      ? hashNode(step.hash, current)
      : hashNode(current, step.hash);
  }
  return current === expectedRoot;
}

export interface StatementCommitment {
  readonly statementId: string;
  readonly schemaVersion: 1;
  readonly obligationRoot: Digest;
  readonly adjustmentRoot: Digest;
  readonly paymentRoot: Digest;
  readonly obligationCount: number;
  readonly adjustmentCount: number;
  readonly paymentCount: number;
}

export function createStatementCommitment(input: {
  statementId: string;
  obligations: readonly unknown[];
  adjustments: readonly unknown[];
  payments: readonly unknown[];
}): StatementCommitment {
  if (!input.statementId.trim()) throw new Error('statementId must be non-empty');
  const obligations = buildMerkleCommitment(input.obligations);
  const adjustments = buildMerkleCommitment(input.adjustments);
  const payments = buildMerkleCommitment(input.payments);
  return Object.freeze({
    statementId: input.statementId,
    schemaVersion: 1,
    obligationRoot: obligations.root,
    adjustmentRoot: adjustments.root,
    paymentRoot: payments.root,
    obligationCount: obligations.count,
    adjustmentCount: adjustments.count,
    paymentCount: payments.count,
  });
}
