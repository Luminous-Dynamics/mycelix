import { describe, expect, it } from 'vitest';
import {
  buildMerkleCommitmentV2,
  buildMerkleInclusionProofV2,
  hashAccountingLeafV2,
  verifyMerkleInclusionProofV2,
} from './merkle-v2.js';
import { buildMerkleCommitment } from './merkle.js';

const golden = [
  {
    name: 'empty',
    values: [] as const,
    root: 'c3f59a327d432c5ca62e0c83780a2f7b4979d2d67cec42949f69ca0312ba5d5e',
  },
  {
    name: 'null leaf',
    values: [null] as const,
    root: '34e2e4c226ea9bc64493fc48384c60a761a90656e5d4f2c790ed9ed11ef9367e',
  },
  {
    name: 'arbitrary precision integer leaf',
    values: [123456789012345678901234567890n] as const,
    root: '9de130683cdd49dd1b8ade66ffa1aa030274eab646607732e72327ca5871f8ca',
  },
  {
    name: 'two leaves with Unicode and negative zero',
    values: [{ z: 3n, a: 'é' }, -0] as const,
    root: '327a078e7127c79e8c66099c1b3a7e5f367d07ca18524f1fb06e19cb0005cd5d',
  },
  {
    name: 'odd-width tree with UTF-8 key-order vector',
    values: [null, 1.5, { '\uE000': 'bmp', '𐀀': 'astral' }] as const,
    root: '1ae6b649185c96953b157f9226fb286a45056f03e7f682c2fa268aa958f63531',
  },
] as const;

describe('cross-language accounting Merkle v2', () => {
  it('matches fixed roots derived from accounting-wire v1 bytes', () => {
    for (const vector of golden) {
      const commitment = buildMerkleCommitmentV2(vector.values);
      expect(commitment.protocolVersion, vector.name).toBe(2);
      expect(commitment.count, vector.name).toBe(vector.values.length);
      expect(commitment.root, vector.name).toBe(vector.root);
    }
  });

  it('builds and verifies inclusion proofs for every leaf in an odd-width tree', () => {
    const values = golden[4].values;
    const root = buildMerkleCommitmentV2(values).root;
    values.forEach((value, index) => {
      const proof = buildMerkleInclusionProofV2(values, index);
      expect(proof.protocolVersion).toBe(2);
      expect(verifyMerkleInclusionProofV2(value, proof, root)).toBe(true);
    });
  });

  it('rejects leaf, sibling, side and root tampering', () => {
    const values = golden[4].values;
    const root = buildMerkleCommitmentV2(values).root;
    const proof = buildMerkleInclusionProofV2(values, 1);
    expect(verifyMerkleInclusionProofV2('tampered', proof, root)).toBe(false);

    const first = proof.steps[0]!;
    const siblingTamper = {
      ...proof,
      steps: [{ ...first, hash: '0'.repeat(64) }, ...proof.steps.slice(1)],
    };
    expect(verifyMerkleInclusionProofV2(values[1], siblingTamper, root)).toBe(false);

    const sideTamper = {
      ...proof,
      steps: [{ ...first, side: first.side === 'left' ? 'right' as const : 'left' as const }, ...proof.steps.slice(1)],
    };
    expect(verifyMerkleInclusionProofV2(values[1], sideTamper, root)).toBe(false);
    expect(verifyMerkleInclusionProofV2(values[1], proof, 'f'.repeat(64))).toBe(false);
  });

  it('keeps historical Merkle v1 roots distinct rather than silently rewriting them', () => {
    const value = { protocolVersion: 1, beneficiaryId: 'creator:merkle', amountMinor: 123n, currency: 'USD' };
    expect(buildMerkleCommitmentV2([value]).root).not.toBe(buildMerkleCommitment([value]).root);
  });

  it('inherits scalar-string rejection from accounting-wire v1', () => {
    expect(() => hashAccountingLeafV2('\uD800')).toThrow(/lone UTF-16 surrogates/);
  });
});
