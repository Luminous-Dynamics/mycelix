import {
  buildMerkleCommitment,
  buildMerkleInclusionProof,
  verifyMerkleInclusionProof,
  type Digest,
  type MerkleInclusionProof,
} from './merkle.js';

export type StatementDisclosureSection = 'obligations' | 'adjustments' | 'settlements';

/**
 * A policy descriptor, not a credential. The caller must authenticate and
 * authorize this policy outside the accounting compiler before using it.
 */
export interface DisclosurePolicy {
  readonly policyId: string;
  readonly statementId: string;
  readonly audience: string;
  readonly purpose: string;
  readonly allowedSections: readonly StatementDisclosureSection[];
  readonly maxItemsPerSection: number;
  readonly notBefore: string;
  readonly expiresAt: string;
}

export interface DisclosedMerkleItem {
  readonly index: number;
  readonly value: unknown;
  readonly proof: MerkleInclusionProof;
}

export interface SelectiveDisclosureBundle {
  readonly protocolVersion: 1;
  readonly proofKind: 'merkle_inclusion_not_zero_knowledge';
  readonly statementId: string;
  readonly section: StatementDisclosureSection;
  readonly sectionRoot: Digest;
  readonly sectionCount: number;
  readonly policyId: string;
  readonly audience: string;
  readonly purpose: string;
  readonly disclosedAt: string;
  readonly items: readonly DisclosedMerkleItem[];
}

function parseTimestamp(label: string, value: string): number {
  const parsed = Date.parse(value);
  if (!Number.isFinite(parsed)) throw new Error(`${label} must be a valid timestamp`);
  return parsed;
}

export function assertDisclosurePolicy(policy: DisclosurePolicy, now: string): void {
  if (!policy.policyId.trim() || !policy.statementId.trim() || !policy.audience.trim() || !policy.purpose.trim()) {
    throw new Error('disclosure policy identity, audience and purpose must be non-empty');
  }
  if (!Number.isSafeInteger(policy.maxItemsPerSection) || policy.maxItemsPerSection <= 0) {
    throw new Error('disclosure policy maxItemsPerSection must be a positive safe integer');
  }
  if (new Set(policy.allowedSections).size !== policy.allowedSections.length) {
    throw new Error('disclosure policy sections must be unique');
  }
  const from = parseTimestamp('notBefore', policy.notBefore);
  const until = parseTimestamp('expiresAt', policy.expiresAt);
  const current = parseTimestamp('disclosedAt', now);
  if (from >= until) throw new Error('disclosure policy notBefore must precede expiresAt');
  if (current < from || current >= until) throw new Error('disclosure policy is not active');
}

function cloneCanonicalValue(value: unknown): unknown {
  if (value === null || typeof value === 'string' || typeof value === 'boolean' || typeof value === 'bigint') {
    return value;
  }
  if (typeof value === 'number') {
    if (!Number.isFinite(value)) throw new Error('disclosure values require finite numbers');
    return value;
  }
  if (Array.isArray(value)) return Object.freeze(value.map(cloneCanonicalValue));
  if (typeof value === 'object') {
    const record = value as Record<string, unknown>;
    const result: Record<string, unknown> = {};
    for (const key of Object.keys(record)) {
      if (record[key] === undefined) throw new Error('disclosure values forbid undefined');
      result[key] = cloneCanonicalValue(record[key]);
    }
    return Object.freeze(result);
  }
  throw new Error(`unsupported disclosure value: ${typeof value}`);
}

export function createSelectiveDisclosureBundle(input: {
  readonly statementId: string;
  readonly section: StatementDisclosureSection;
  readonly expectedRoot: Digest;
  readonly lines: readonly unknown[];
  readonly indexes: readonly number[];
  readonly policy: DisclosurePolicy;
  readonly disclosedAt: string;
}): Readonly<SelectiveDisclosureBundle> {
  assertDisclosurePolicy(input.policy, input.disclosedAt);
  if (input.policy.statementId !== input.statementId) throw new Error('disclosure policy statement mismatch');
  if (!input.policy.allowedSections.includes(input.section)) throw new Error('statement section is not allowed by disclosure policy');

  const uniqueIndexes = [...new Set(input.indexes)].sort((a, b) => a - b);
  if (uniqueIndexes.length !== input.indexes.length) throw new Error('disclosure indexes must be unique');
  if (uniqueIndexes.length === 0) throw new Error('selective disclosure requires at least one item');
  if (uniqueIndexes.length > input.policy.maxItemsPerSection) {
    throw new Error('selective disclosure exceeds policy item limit');
  }

  const commitment = buildMerkleCommitment(input.lines);
  if (commitment.root !== input.expectedRoot) throw new Error('disclosure source lines do not match expected statement root');

  const items = uniqueIndexes.map(index => Object.freeze({
    index,
    value: cloneCanonicalValue(input.lines[index]),
    proof: buildMerkleInclusionProof(input.lines, index),
  }));

  return Object.freeze({
    protocolVersion: 1,
    proofKind: 'merkle_inclusion_not_zero_knowledge',
    statementId: input.statementId,
    section: input.section,
    sectionRoot: commitment.root,
    sectionCount: commitment.count,
    policyId: input.policy.policyId,
    audience: input.policy.audience,
    purpose: input.policy.purpose,
    disclosedAt: input.disclosedAt,
    items: Object.freeze(items),
  });
}

export function verifySelectiveDisclosureBundle(bundle: SelectiveDisclosureBundle): boolean {
  if (bundle.protocolVersion !== 1 || bundle.proofKind !== 'merkle_inclusion_not_zero_knowledge') return false;
  if (!bundle.statementId.trim() || !bundle.sectionRoot.trim() || bundle.sectionCount <= 0) return false;
  const seen = new Set<number>();
  for (const item of bundle.items) {
    if (seen.has(item.index)) return false;
    seen.add(item.index);
    if (item.proof.count !== bundle.sectionCount || item.proof.index !== item.index) return false;
    if (!verifyMerkleInclusionProof(item.value, item.proof, bundle.sectionRoot)) return false;
  }
  return bundle.items.length > 0;
}
