import { describe, expect, it } from 'vitest';
import {
  buildMerkleCommitment,
  buildMerkleInclusionProof,
  canonicalAccountingValue,
  createStatementCommitment,
  verifyMerkleInclusionProof,
} from './merkle.js';

describe('accounting Merkle commitments', () => {
  const obligations = [
    { id: 'o1', amountMinor: 100n, beneficiary: 'alice' },
    { id: 'o2', amountMinor: 200n, beneficiary: 'alice' },
    { id: 'o3', amountMinor: 27n, beneficiary: 'alice' },
  ];

  it('is deterministic across object key insertion order', () => {
    expect(canonicalAccountingValue({ b: 2, a: 1 }))
      .toBe(canonicalAccountingValue({ a: 1, b: 2 }));
  });

  it('proves one line without revealing the whole corpus', () => {
    const commitment = buildMerkleCommitment(obligations);
    const proof = buildMerkleInclusionProof(obligations, 1);
    expect(verifyMerkleInclusionProof(obligations[1], proof, commitment.root)).toBe(true);
    expect(verifyMerkleInclusionProof({ ...obligations[1], amountMinor: 201n }, proof, commitment.root)).toBe(false);
  });

  it('rejects structurally malformed proof depth and digest shapes', () => {
    const commitment = buildMerkleCommitment(obligations);
    const proof = buildMerkleInclusionProof(obligations, 1);
    expect(verifyMerkleInclusionProof(obligations[1], {
      ...proof,
      steps: proof.steps.slice(0, -1),
    }, commitment.root)).toBe(false);
    expect(verifyMerkleInclusionProof(obligations[1], proof, 'not-a-sha256-root')).toBe(false);
    expect(verifyMerkleInclusionProof(obligations[1], {
      ...proof,
      steps: [{ ...proof.steps[0]!, hash: 'bad' }, ...proof.steps.slice(1)],
    }, commitment.root)).toBe(false);
  });

  it('rejects a proof whose sibling side contradicts the tree position', () => {
    const commitment = buildMerkleCommitment(obligations);
    const proof = buildMerkleInclusionProof(obligations, 1);
    expect(verifyMerkleInclusionProof(obligations[1], {
      ...proof,
      steps: [{ ...proof.steps[0]!, side: 'right' }, ...proof.steps.slice(1)],
    }, commitment.root)).toBe(false);
  });

  it('validates duplicate-last topology for odd-width levels', () => {
    const commitment = buildMerkleCommitment(obligations);
    const proof = buildMerkleInclusionProof(obligations, 2);
    expect(verifyMerkleInclusionProof(obligations[2], proof, commitment.root)).toBe(true);
    expect(verifyMerkleInclusionProof(obligations[2], {
      ...proof,
      steps: [{ ...proof.steps[0]!, hash: '0'.repeat(64) }, ...proof.steps.slice(1)],
    }, commitment.root)).toBe(false);
  });

  it('binds roots and exact counts into statement metadata', () => {
    const statement = createStatementCommitment({
      statementId: 'statement:s92',
      obligations,
      adjustments: [{ id: 'a1', amountMinor: -5n }],
      payments: [{ id: 'p1', amountMinor: 322n }],
    });
    expect(statement.obligationCount).toBe(3);
    expect(statement.adjustmentCount).toBe(1);
    expect(statement.paymentCount).toBe(1);
  });
});
